#include <zephyr/types.h>
#include <zephyr.h>
#include <drivers/uart.h>

#include <device.h>
#include <soc.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <sdc_hci_vs.h>

#include <bluetooth/services/midi.h>

#include <dk_buttons_and_leds.h>

#include <mpsl_radio_notification.h>

#include <logging/log.h>

#define LOG_MODULE_NAME peripheral_midi
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define RADIO_NOTIF_PRIORITY 1

#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define UART_RX_LED DK_LED3
#define RUN_LED_BLINK_INTERVAL 1000

#define BLE_MIDI_TX_MAX_SIZE 73

#define INTERVAL_LLPM 0x0D01 /* Proprietary  1 ms */
#define INTERVAL_LLPM_US 1000

#define TIMESTAMP(time) (uint16_t)((time)&8191)

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);
static K_FIFO_DEFINE(fifo_uart_midi_write);

static K_THREAD_STACK_DEFINE(ble_tx_work_q_stack_area, 512);

static struct bt_conn *current_conn;

static const struct device *uart;
static struct k_work uart_empty_tx_buffer_work;
static struct k_work ble_tx_work;
static struct k_work ble_midi_encode_work;
static struct k_work_delayable uart_rx_enable_work;
static struct k_work_delayable active_sense_tx_work;
static struct k_work_delayable active_sense_rx_work;
static struct k_work_delayable uart_midi_write_work;

static struct k_work_q ble_tx_work_q;

static uint8_t ble_midi_pck[BLE_MIDI_TX_MAX_SIZE];
static uint8_t ble_midi_pck_len;
static uint8_t mtu_size = BLE_MIDI_TX_MAX_SIZE + 3;

static uint8_t uart_rx_buf[2];

static bool radio_notif_flag;

struct midi_msg_t {
	void *fifo_reserved;
	uint16_t timestamp;
	uint8_t data[3];
	uint8_t len;
};

struct midi_parser_t {
	struct midi_msg_t *msg;
	uint8_t running_status;
	bool third_byte_flag;
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MIDI_VAL),
};

void uart_midi_write(struct midi_msg_t *msg)
{
	int32_t msg_delay;

	msg_delay =
		(int32_t)((msg->timestamp * 1000) -
			  (k_ticks_to_us_near32((uint32_t)k_uptime_ticks()) %
			   8192000));
	k_fifo_put(&fifo_uart_midi_write, msg);
	k_work_schedule(&uart_midi_write_work, K_USEC(msg_delay));
}

void ble_midi_write(struct midi_msg_t *msg)
{
	k_fifo_put(&fifo_uart_rx_data, msg);
	k_work_submit_to_queue(&ble_tx_work_q, &ble_midi_encode_work);
}

static bool parse_midi_byte(uint16_t timestamp, uint8_t byte,
			    struct midi_parser_t *parser)
{
	struct midi_msg_t *msg;
	uint8_t running_status;

	if (!parser->msg) {
		/** Byte is part of new message */
		parser->msg = k_malloc(sizeof(*msg));
		if (!parser->msg) {
			LOG_WRN("Not able to allocate midi message buffer");
			return false;
		}
		parser->msg->len = 0;
		parser->msg->timestamp = 0;
	}

	msg = parser->msg;
	running_status = parser->running_status;

	/** Setting timestamp */
	if (parser->msg->timestamp == 0) {
		msg->timestamp = timestamp;
	}

	if ((byte >> 7) == 1) {
		/** Current byte is statusbyte */
		parser->running_status = byte;
		parser->third_byte_flag = false;

		/** Message with only one byte */
		if ((byte >> 2) == 0b111101) {
			if (byte == 0xF7) {
				/** End of exclusive, not supported. Discarded for now.  */
				return false;
			}

			msg->data[msg->len] = byte;
			msg->len = 1;

			return true;
		}
		return false;
	}

	if (parser->third_byte_flag == true) {
		/** Expected third, and last, byte of message */
		parser->third_byte_flag = false;
		msg->data[2] = byte;
		msg->len = 3;
		return true;
	}

	if (running_status == 0) {
		/** System Exclusive (SysEx) databytes, from 3rd byte until EoX, or
		 * orphaned databytes. */
		return false;
	}

	/** Channel Voice Messages */
	switch (running_status >> 4) {
	case 0x8:
	case 0x9:
	case 0xA:
	case 0xB:
	case 0xE:
		parser->third_byte_flag = true;
		msg->data[0] = running_status;
		msg->data[1] = byte;
		msg->len = 2;
		return false;
	case 0xC:
	case 0xD:
		msg->data[0] = running_status;
		msg->data[1] = byte;
		msg->len = 2;
		return true;
	}

	/** System Common Message */
	switch (running_status) {
	case 0xF2:
		parser->third_byte_flag = true;
		msg->data[0] = running_status;
		msg->data[1] = byte;
		msg->len = 2;
		parser->running_status = 0;
		return false;
	case 0xF1:
	case 0xF3:
		parser->third_byte_flag = false;
		msg->data[0] = running_status;
		msg->data[1] = byte;
		msg->len = 2;
		parser->running_status = 0;
		return true;
	case 0xF0:
		break;
	}

	parser->running_status = 0;
	return false;
}

struct midi_msg_t *parse_real_time_message(uint16_t timestamp, uint8_t byte)
{
	struct midi_msg_t *msg;

	if (((byte >> 7) == 1) && ((byte >> 3) == 0b11111)) {
		/** System Real-Time Messages */
		msg = k_malloc(sizeof(*msg));

		if (!msg) {
			LOG_WRN("Not able to allocate midi message buffer");
			return NULL;
		}

		msg->data[0] = byte;
		msg->len = 1;
		msg->timestamp = timestamp;

		if (byte == 0xFF) {
			/** MIDI Reset message, reset device to initial conditions */
		}
		return msg;
	}

	return NULL;
}

static void uart_cb(const struct device *dev, struct uart_event *evt,
		    void *user_data)
{
	ARG_UNUSED(dev);
	static uint8_t *released_buf;
	struct midi_msg_t *midi_rx_buf;
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

		/** Check if the Received byte is a System Real-Time message. */
		midi_rx_buf =
			parse_real_time_message(timestamp, *evt->data.rx.buf);
		if (!midi_rx_buf) {
			/** Received byte is System Common- or Channel Voice message. */
			if (parse_midi_byte(timestamp, *evt->data.rx.buf,
					    &parser)) {
				midi_rx_buf = parser.msg;
				parser.msg = NULL;
			}
		} else if (parser.msg && (parser.msg->timestamp < timestamp)) {
			/** RTM has interrupted a message */
			LOG_INF("Interrupting Cow!");
			parser.msg->timestamp = timestamp;
		}

		if (midi_rx_buf) {
			/** Message complete, add to FIFO */
			if (!current_conn) {
				uart_midi_write(midi_rx_buf);
			} else {
				ble_midi_write(midi_rx_buf);
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
			k_work_schedule(&uart_rx_enable_work, K_MSEC(500));
		}
		break;
	default:
		break;
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
		LOG_WRN("Failed to send data over UART (err:%d)", err);
	} else {
		k_free(tx_buf);
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

			k_work_schedule(&uart_midi_write_work,
					K_USEC(msg_delay));
		}
	}
}

static void uart_rx_enable_work_handler(struct k_work *item)
{
	int err;

	err = uart_rx_enable(uart, &uart_rx_buf[0], 1, SYS_FOREVER_MS);
	if (err) {
		LOG_ERR("Unable to enable UART RX");
		k_work_schedule(&uart_rx_enable_work, K_MSEC(500));
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

/** @brief UART Initializer */
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
		bt_midi_send(current_conn, ble_midi_pck, ble_midi_pck_len);
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
			bt_midi_send(current_conn, ble_midi_pck,
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

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn_err) {
		LOG_ERR("Connection failed (err %u)", conn_err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected to %s", log_strdup(addr));

	current_conn = bt_conn_ref(conn);

	/** Return uart_write_thread from k_fifo_get, disable uart echo.  */
	k_fifo_cancel_wait(&fifo_uart_rx_data);

	k_work_init(&ble_tx_work, ble_tx_work_handler);
	k_work_init(&ble_midi_encode_work, ble_midi_encode_work_handler);

	irq_enable(TEMP_IRQn);
	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s (reason %u)", log_strdup(addr), reason);

	/** Return uart_write_thread from k_fifo_get, enable uart echo.  */
	k_fifo_cancel_wait(&fifo_uart_midi_write);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		irq_disable(TEMP_IRQn);
		dk_set_led_off(CON_STATUS_LED);
	}
}


static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param) {
	LOG_INF("Param requested");
	return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	struct bt_conn_info info;

	bt_conn_get_info(conn, &info);

	mtu_size = bt_gatt_get_mtu(conn);

	if (interval == INTERVAL_LLPM) {
		LOG_INF("Connection interval updated: LLPM (1 ms), MTU: %d\n",
			mtu_size);
	} else {
		if (info.le.interval > 12) {
			const struct bt_le_conn_param param = {
				.interval_min = 6,
				.interval_max = 6,
				.latency = 0,
				.timeout = info.le.timeout
			};

			bt_conn_le_param_update(conn, &param);
		}

		LOG_INF("Params updated interval: %d, latency: %d, timeout %d: MTU: %d",
			interval, latency, timeout, mtu_size);
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
};

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

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	struct midi_msg_t *msg;
	static struct midi_parser_t parser;
	static uint32_t conn_time;

	uint8_t current_byte;
	uint16_t timestamp_ble;
	uint16_t timestamp;
	uint16_t interval;

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

		/** Check MSB and expectations to ID current byte as timestamp,
		 * statusbyte or databyte */
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
			msg = parse_real_time_message(timestamp, current_byte);

			if (!msg) {
				if (parse_midi_byte(timestamp, current_byte,
						    &parser)) {
					msg = parser.msg;
					parser.msg = NULL;
				}
			}

			if (msg) {
				/** Message completed */
				uart_midi_write(msg);
			}
		}
	}
}

static struct bt_midi_cb midi_cb = {
	.received = bt_receive_cb,
};

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
		LOG_ERR("Bluetooth unable to initialize (err: %d)", err);
	}
	bt_conn_cb_register(&conn_callbacks);

	err = bt_midi_init(&midi_cb);
	if (err) {
		LOG_ERR("Failed to initialize MIDI service (err: %d)", err);
		return;
	}

	if (enable_llpm_mode()) {
		LOG_ERR("Enable LLPM mode failed.\n");
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
	}
	LOG_INF("Advertising successfully started");

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}