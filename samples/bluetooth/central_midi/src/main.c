#include <errno.h>
#include <sys/byteorder.h>

#include <drivers/uart.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>

#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <sdc_hci_vs.h>

#include <bluetooth/services/midi.h>
#include <bluetooth/services/midi_client.h>

#include <midi/midi_parser.h>
#include <midi/midi_types.h>

#include <dk_buttons_and_leds.h>

#include <mpsl_radio_notification.h>

#include <logging/log.h>

#define LOG_MODULE_NAME central_midi
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define RADIO_NOTIF_PRIORITY 1

#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define UART_RX_LED DK_LED3
#define RUN_LED_BLINK_INTERVAL 1

#define BLE_MIDI_TX_MAX_SIZE 73

#define INTERVAL_MIN 6 /* 80 units,  100 ms */
#define INTERVAL_MAX 6 /* 80 units,  100 ms */
#define INTERVAL_LLPM 0x0D01 /* Proprietary  1 ms */
#define INTERVAL_LLPM_US 1000

#define TIMESTAMP(time) (uint16_t)((time)&8191)

static uint8_t ble_midi_pck[BLE_MIDI_TX_MAX_SIZE];
static uint8_t ble_midi_pck_len;
static uint8_t mtu_size = BLE_MIDI_TX_MAX_SIZE + 3;

static K_SEM_DEFINE(ble_init_ok, 0, 1);
static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);
static K_FIFO_DEFINE(fifo_uart_midi_write);

static uint8_t uart_rx_buf[2];

uint16_t conn_time;

uint64_t rx_time;

static bool radio_notif_flag;
static bool ble_midi_ready;

static struct bt_midi_client midi_client;

static struct bt_le_conn_param *conn_param =
	BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, 400);
static struct bt_conn *current_conn;

static const struct device *uart;
static struct k_work uart_empty_tx_buffer_work;
static struct k_work ble_tx_work;
static struct k_work ble_midi_encode_work;
static struct k_work_delayable uart_rx_enable_work;
static struct k_work_delayable active_sense_tx_work;
static struct k_work_delayable active_sense_rx_work;
static struct k_work_delayable uart_midi_write_work;

static K_THREAD_STACK_DEFINE(ble_tx_work_q_stack_area, 1024);
static struct k_work_q ble_tx_work_q;

void uart_midi_write(struct midi_msg_t *msg)
{
	int32_t msg_delay;

	msg_delay =
		(int32_t)((msg->timestamp * 1000) -
			  (k_ticks_to_us_near32((uint32_t)k_uptime_ticks()) %
			   8192000));
	k_fifo_put(&fifo_uart_midi_write, msg);
	if (msg_delay < 0) {
		k_work_schedule(&uart_midi_write_work, K_NO_WAIT);
	} else {
		k_work_schedule(&uart_midi_write_work, K_USEC(msg_delay));
	}
}

void ble_midi_write(struct midi_msg_t *msg)
{
	k_fifo_put(&fifo_uart_rx_data, msg);

	k_work_submit_to_queue(&ble_tx_work_q, &ble_midi_encode_work);
}

static void uart_cb(const struct device *dev, struct uart_event *evt,
		    void *user_data)
{
	ARG_UNUSED(dev);
	static uint8_t *released_buf;
	struct midi_msg_t *rx_msg;
	static struct midi_parser_t parser;

	uint16_t timestamp;

	static uint8_t blink_status = 0;

	switch (evt->type) {
	case UART_TX_DONE:
		/** UART has been transmitted, checking if more data is available. */
		if ((evt->data.tx.len == 0) || (!evt->data.tx.buf)) {
			/** No data was sent, and no allocated pointer */
			return;
		}

		k_work_submit(&uart_empty_tx_buffer_work);
		break;
	case UART_RX_RDY:
		dk_set_led(UART_RX_LED, (++blink_status) % 2);

		if (*evt->data.rx.buf == 0xFE ||
		    k_work_delayable_is_pending(&active_sense_rx_work)) {
			k_work_reschedule(&active_sense_rx_work, K_MSEC(330));
			if (*evt->data.rx.buf == 0xFE) {
				break;
			}
		}

		timestamp = TIMESTAMP(k_ticks_to_ms_near64(k_uptime_ticks()));

		rx_msg = midi_parse_byte(timestamp, *evt->data.rx.buf, &parser);

		if (rx_msg) {
			/** Message complete */
			LOG_HEXDUMP_INF(rx_msg->data, rx_msg->len, "rx:");
			if (!ble_midi_ready) {
				uart_midi_write(rx_msg);
			} else {
				ble_midi_write(rx_msg);
			}
		}
		break;
	case UART_RX_DISABLED:
		LOG_WRN("UART RX-disabled");
		break;
	case UART_RX_BUF_REQUEST:
		if (released_buf == &uart_rx_buf[0]) {
			uart_rx_buf_rsp(uart, &uart_rx_buf[0], 1);
		} else {
			uart_rx_buf_rsp(uart, &uart_rx_buf[1], 1);
		}
		break;
	case UART_RX_BUF_RELEASED:
		released_buf = evt->data.rx_buf.buf;
		break;
	case UART_RX_STOPPED:
		LOG_ERR("UART Rx stopped, reason %d", evt->data.rx_stop.reason);
		if ((evt->data.rx_stop.reason == 4) ||
		    (evt->data.rx_stop.reason == 8)) {
			/** UART might be disconnected, 
			 * disable until rx-port is connected again. */
			uart_rx_disable(uart);
			k_work_reschedule(&uart_rx_enable_work, K_MSEC(500));
		}
		break;
	default:
		break;
	}
}

static void uart_midi_write_work_handler(struct k_work *item)
{
	struct midi_msg_t *msg;
	static uint8_t running_status = 0;
	int32_t msg_delay;

	msg = k_fifo_get(&fifo_uart_midi_write, K_NO_WAIT);

	if (msg) {
		k_work_reschedule(&active_sense_tx_work, K_MSEC(270));

		/** Enforce Running status */
		if (msg->data[0] == running_status) {
			/** Running status */
			msg->len--;
			msg->data[0] = msg->data[1];
			msg->data[1] = msg->data[2];
		} else if (msg->data[0] < 0xF8) {
			/** Running status cancelled, new? */
			if (msg->data[0] < 0xF0) {
				running_status = msg->data[0];
			} else {
				running_status = 0;
			}
		} else {
			/** System RTM: Unchanged */
		}

		/** Send msg if possible */
		if (!k_fifo_is_empty(&fifo_uart_tx_data)) {
			/** There are msg queued before the current msg */
			k_fifo_put(&fifo_uart_tx_data, msg);
		} else if (uart_tx(uart, msg->data, msg->len, SYS_FOREVER_MS)) {
			/** UART Tx busy, send later */
			k_fifo_put(&fifo_uart_tx_data, msg);
		} else {
			/** Sent */
			k_free(msg);
		}

		/** Check if new messages are available */
		msg = k_fifo_peek_head(&fifo_uart_midi_write);
		if (msg) {
			msg_delay =
				(int32_t)((msg->timestamp * 1000) -
					  (k_ticks_to_us_near32(
						   (uint32_t)k_uptime_ticks()) %
					   8192000));

			if (msg_delay < 0) {
				k_work_schedule(&uart_midi_write_work,
						K_NO_WAIT);
			} else {
				k_work_schedule(&uart_midi_write_work,
						K_USEC(msg_delay));
			}
		}
	}
}

static void uart_empty_tx_buffer_work_handler(struct k_work *item)
{
	struct midi_msg_t *tx_buf;
	int err;

	tx_buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
	if (!tx_buf) {
		return;
	}

	err = uart_tx(uart, (const uint8_t *)tx_buf->data, tx_buf->len,
		      SYS_FOREVER_MS);
	if (err) {
		LOG_WRN("%s:%d Failed to send data over UART (err:%d). TX-buffer: %p",
			__func__, __LINE__, err, tx_buf);
	} else {
		k_free(tx_buf);
	}
}

static void uart_rx_enable_work_handler(struct k_work *item)
{
	int err;

	err = uart_rx_enable(uart, &uart_rx_buf[0], 1, SYS_FOREVER_MS);
	if (err) {
		LOG_ERR("Unable to enable UART RX");
		k_work_reschedule(&uart_rx_enable_work, K_MSEC(500));
	}
}

static void active_sense_tx_work_handler(struct k_work *item)
{
	struct midi_msg_t *msg;

	msg = k_malloc(sizeof(*msg));

	msg->data[0] = 0xFE;
	msg->len = 1;

	if (uart_tx(uart, msg->data, msg->len, SYS_FOREVER_MS)) {
		k_fifo_put(&fifo_uart_tx_data, msg);
	} else {
		k_free(msg);
	}

	k_work_reschedule(&active_sense_tx_work, K_MSEC(270));
}

static void active_sense_rx_work_handler(struct k_work *item)
{
	LOG_WRN("Active sensing lost, possible disconnect of serial.");
	/** All notes off */
}

static int uart_init(void)
{
	int err;

	uart = device_get_binding(DT_LABEL(DT_NODELABEL(arduino_serial)));
	if (!uart) {
		return -ENXIO;
	}

	k_work_init(&uart_empty_tx_buffer_work,
		    uart_empty_tx_buffer_work_handler);

	k_work_init_delayable(&uart_rx_enable_work,
			      uart_rx_enable_work_handler);
	k_work_init_delayable(&active_sense_tx_work,
			      active_sense_tx_work_handler);
	k_work_init_delayable(&active_sense_rx_work,
			      active_sense_rx_work_handler);
	k_work_init_delayable(&uart_midi_write_work,
			      uart_midi_write_work_handler);

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		return err;
	}

	k_work_schedule(&active_sense_tx_work, K_MSEC(270));

	return uart_rx_enable(uart, &uart_rx_buf[0], 1, SYS_FOREVER_MS);
}

static void ble_tx_work_handler(struct k_work *item)
{
	if (ble_midi_pck_len != 0) {
		bt_midi_client_send(&midi_client, ble_midi_pck,
				    ble_midi_pck_len);
		ble_midi_pck_len = 0;
	}
}

static void ble_midi_encode_work_handler(struct k_work *item)
{
	struct midi_msg_t *msg;
	static uint8_t running_status = 0;

	msg = k_fifo_get(&fifo_uart_rx_data, K_NO_WAIT);

	/** Process received MIDI message and prepare BLE packet */
	if (msg) {
		if ((msg->len + 1) > (mtu_size - 3) - ble_midi_pck_len) {
			/** BLE Packet is full, packet is ready to send */
			LOG_INF("Packet is full! %d", ble_midi_pck_len);
			bt_midi_client_send(&midi_client, ble_midi_pck,
					    ble_midi_pck_len);
			ble_midi_pck_len = 0;
		}

		if (ble_midi_pck_len == 0) {
			/** New packet */
			ble_midi_pck[0] = ((msg->timestamp >> 7) | 0x80) & 0xBF;
			ble_midi_pck_len = 1;
			running_status = 0;
		}

		/** Enforce Running status */
		if (msg->data[0] == running_status) {
			/** Running status */
			msg->len--;
			msg->data[0] = msg->data[1];
			msg->data[1] = msg->data[2];
		} else if (msg->data[0] < 0xF8) {
			/** Running status cancelled, new? */
			if (msg->data[0] < 0xF0) {
				running_status = msg->data[0];
			} else {
				running_status = 0;
			}
		} else {
			/** System RTM: Unchanged */
		}

		/** Message Timestamp */
		ble_midi_pck[ble_midi_pck_len] = (msg->timestamp & 0x7F) | 0x80;
		ble_midi_pck_len++;

		/** Add MIDI message to packet and free memory */
		memcpy((ble_midi_pck + ble_midi_pck_len), msg->data, msg->len);
		ble_midi_pck_len += msg->len;
		k_free(msg);
		k_work_submit_to_queue(&ble_tx_work_q, &ble_midi_encode_work);
	}
}

static int enable_llpm_short_connection_interval(void)
{
	int err;
	struct net_buf *buf;

	sdc_hci_cmd_vs_conn_update_t *cmd_conn_update;

	buf = bt_hci_cmd_create(SDC_HCI_OPCODE_CMD_VS_CONN_UPDATE,
				sizeof(*cmd_conn_update));
	if (!buf) {
		LOG_INF("Could not allocate command buffer\n");
		return -ENOMEM;
	}

	uint16_t conn_handle;

	err = bt_hci_get_conn_handle(current_conn, &conn_handle);
	if (err) {
		LOG_INF("Failed obtaining conn_handle (err %d)\n", err);
		return err;
	}

	cmd_conn_update = net_buf_add(buf, sizeof(*cmd_conn_update));
	cmd_conn_update->connection_handle = conn_handle;
	cmd_conn_update->conn_interval_us = INTERVAL_LLPM_US;
	cmd_conn_update->conn_latency = 0;
	cmd_conn_update->supervision_timeout = 300;

	err = bt_hci_cmd_send_sync(SDC_HCI_OPCODE_CMD_VS_CONN_UPDATE, buf,
				   NULL);
	if (err) {
		LOG_INF("Update connection parameters failed (err %d)\n", err);
		return err;
	}

	return 0;
}

static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
	struct bt_midi_client *midi = context;
	LOG_INF("Service discovery completed");

	bt_gatt_dm_data_print(dm);
	bt_midi_client_handles_assign(dm, midi);
	bt_midi_client_notif_enable(midi);
	ble_midi_ready = true;
	bt_gatt_dm_data_release(dm);

	k_work_init(&ble_tx_work, ble_tx_work_handler);
	k_work_init(&ble_midi_encode_work, ble_midi_encode_work_handler);

	if (enable_llpm_short_connection_interval()) {
		LOG_INF("Enable LLPM short connection interval failed");
	}

	irq_enable(TEMP_IRQn);

	dk_set_led_on(CON_STATUS_LED);
}

static void discovery_service_not_found(struct bt_conn *conn, void *context)
{
	LOG_INF("Service not found");
}

static void discovery_error(struct bt_conn *conn, int err, void *context)
{
	LOG_WRN("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found = discovery_error,
};

static void exchange_func(struct bt_conn *conn, uint8_t err,
			  struct bt_gatt_exchange_params *params)
{
	if (!err) {
		mtu_size = bt_gatt_get_mtu(conn);
		LOG_INF("MTU exchange done: %d", mtu_size);
	} else {
		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;
	struct bt_conn_info conn_info;
	bt_conn_get_info(conn, &conn_info);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Connected: %s", log_strdup(addr));

	if (conn_err) {
		LOG_INF("Failed to connect to %s (%d)", log_strdup(addr),
			conn_err);

		if (current_conn == conn) {
			bt_conn_unref(current_conn);
			current_conn = NULL;

			err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
			if (err) {
				LOG_ERR("Scanning failed to start (err %d)",
					err);
			}
		}
		return;
	}

	err = bt_scan_stop();
	if ((!err) && (err != -EALREADY)) {
		LOG_ERR("Stop LE scan failed (err %d)", err);
	}

	if (conn == current_conn) {
		err = bt_gatt_dm_start(conn, BT_UUID_MIDI_SERVICE,
				       &discovery_cb, &midi_client);
		if (err) {
			LOG_ERR("could not start the discovery procedure, error code: %d",
				err);
		}
	}

	static struct bt_gatt_exchange_params exchange_params;

	exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_WRN("MTU exchange failed (err %d)", err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	ble_midi_ready = false;
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s (reason %u)", log_strdup(addr), reason);

	if (current_conn != conn) {
		return;
	}

	bt_conn_unref(current_conn);
	current_conn = NULL;
	irq_disable(TEMP_IRQn);
	dk_set_led_off(CON_STATUS_LED);

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
	}
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	LOG_INF("Param requested");
	return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	if (interval == INTERVAL_LLPM) {
		LOG_INF("Connection interval updated: LLPM (1 ms)");
	} else {
		LOG_INF("Params updated interval: %d, latency: %d, timeout %d: ",
			interval, latency, timeout);
	}
}


static struct bt_conn_cb conn_callbacks = { 
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated
};

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	LOG_INF("Filters matched. Address: %s connectable: %d",
		log_strdup(addr), connectable);
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	current_conn = bt_conn_ref(conn);
}

static uint16_t convert_timestamp(uint16_t timestamp, uint16_t conn_interval,
				  uint16_t conn_time)
{
	static uint16_t correction;

	uint16_t interval_start = TIMESTAMP(conn_time + correction);
	uint16_t interval_end = interval_start + conn_interval;

	if (timestamp > interval_end) {
		correction = TIMESTAMP(timestamp - conn_time - conn_interval);
	} else if ((timestamp < (interval_start)) &&
		   (interval_start < TIMESTAMP(interval_end))) {
		correction = TIMESTAMP(timestamp - conn_time);
	}

	return TIMESTAMP(timestamp - correction);
}

uint8_t bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
		      uint16_t len)
{
	struct midi_msg_t *msg;
	static struct midi_parser_t parser;
	static uint32_t conn_time;

	uint8_t current_byte;
	uint16_t timestamp_ble;
	uint16_t timestamp;

	uint16_t interval;

	rx_time = k_uptime_get();

	struct bt_conn_info conn_info;
	bt_conn_get_info(conn, &conn_info);
	if (conn_info.le.interval == INTERVAL_LLPM) {
		interval = 2;
	} else {
		interval = (conn_info.le.interval * 1.25) + 1;
	}

	bool next_is_new_timestamp = false;

	if (radio_notif_flag) {
		radio_notif_flag = false;
		conn_time = TIMESTAMP(k_ticks_to_ms_near64(k_uptime_ticks()));
	};

	char addr[BT_ADDR_LE_STR_LEN] = { 0 };
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	/** Collecting first Timestamp */
	timestamp_ble = (*(data + 1) & 0x7F) + ((*data & 0x3F) << 7);
	timestamp = convert_timestamp(timestamp_ble, interval, conn_time);

	/** Decoding rest of data */
	for (uint8_t pos = 2; pos < len; pos++) {
		current_byte = *(data + pos);

		/** Check MSB and whether current byte is Timestamp, Statusbyte or
		 * databyte */
		if ((current_byte >> 7) && (next_is_new_timestamp)) {
			/** New Timestamp means last message is complete */
			next_is_new_timestamp = false;

			if ((current_byte & 0x7F) < (timestamp_ble & 0x7F)) {
				/** Timestamp overflow, increment Timestamp High */
				timestamp_ble += 1 << 7;
			}

			/** Storing newest timestamp for later reference, and translating
			 * timestamp to local time */
			timestamp_ble = ((current_byte & 0x7F) +
					 (timestamp_ble & 0x1F80));
			timestamp = convert_timestamp(timestamp_ble, interval,
						      conn_time);

			if (!msg) {
				/** Previous message was not complete */
				LOG_ERR("Incomplete message: pos %d.", pos);
			}
		} else {
			/** Statusbytes and databytes */
			next_is_new_timestamp = true;
			msg = midi_parse_byte(timestamp, current_byte, &parser);

			if (msg) {
				/** Message completed */
				uart_midi_write(msg);
			}
		}
	}
	return BT_GATT_ITER_CONTINUE;
}

static int midi_client_init(void)
{
	int err;
	static struct bt_midi_client_cb midi_client_cb = {
		.received = bt_receive_cb,
	};
	const struct bt_midi_client_init_param midi_client_init = {
		.cb = midi_client_cb,
	};

	err = bt_midi_client_init(&midi_client, &midi_client_init);
	if (err) {
		LOG_ERR("MIDI Client initialization failed (err %d)", err);
		return err;
	}

	LOG_INF("MIDI Client module initialized");
	return err;
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, NULL, scan_connecting);

static int scan_init(void)
{
	int err;

	struct bt_scan_init_param scan_init = { .connect_if_match = 1,
						.conn_param = conn_param };

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID,
				 BT_UUID_MIDI_SERVICE);
	if (err) {
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		return err;
	}

	LOG_INF("Scan module initialized");
	return err;
}

static void radio_notif_handler(void)
{
	radio_notif_flag = true;
	k_work_submit_to_queue(&ble_tx_work_q, &ble_tx_work);
}

static void radio_notif_setup()
{
	mpsl_radio_notification_cfg_set(
		MPSL_RADIO_NOTIFICATION_TYPE_INT_ON_ACTIVE,
		MPSL_RADIO_NOTIFICATION_DISTANCE_420US, TEMP_IRQn);
	IRQ_CONNECT(TEMP_IRQn, RADIO_NOTIF_PRIORITY, radio_notif_handler, NULL,
		    0);
}

static int enable_llpm_mode(void)
{
	int err;
	struct net_buf *buf;
	sdc_hci_cmd_vs_llpm_mode_set_t *cmd_enable;

	buf = bt_hci_cmd_create(SDC_HCI_OPCODE_CMD_VS_LLPM_MODE_SET,
				sizeof(*cmd_enable));
	if (!buf) {
		LOG_INF("Could not allocate LLPM command buffer\n");
		return -ENOMEM;
	}

	cmd_enable = net_buf_add(buf, sizeof(*cmd_enable));
	cmd_enable->enable = true;

	err = bt_hci_cmd_send_sync(SDC_HCI_OPCODE_CMD_VS_LLPM_MODE_SET, buf,
				   NULL);
	if (err) {
		LOG_INF("Error enabling LLPM %d\n", err);
		return err;
	}

	LOG_INF("LLPM mode enabled\n");
	return 0;
}

void main(void)
{
	int blink_status = 0;
	int err;

	k_work_queue_start(&ble_tx_work_q, ble_tx_work_q_stack_area,
			   K_THREAD_STACK_SIZEOF(ble_tx_work_q_stack_area), -2,
			   NULL);

	radio_notif_setup();
	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}

	err = uart_init();
	if (err) {
		LOG_ERR("UART unable to initialize (err: %d)", err);
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}
	bt_conn_cb_register(&conn_callbacks);

	err = midi_client_init();
	if (err) {
		LOG_ERR("Failed to initialize MIDI service (err: %d)", err);
		return;
	}

	err = scan_init();
	if (err) {
		LOG_ERR("Failed to initialize Scan (err: %d)", err);
		return;
	}

	if (enable_llpm_mode()) {
		LOG_ERR("Enable LLPM mode failed.\n");
		return;
	}

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return;
	}
	LOG_INF("Scanning successfully started");

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}