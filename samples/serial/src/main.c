#include <zephyr/types.h>
#include <zephyr.h>
#include <drivers/uart.h>

#include <device.h>
#include <soc.h>

#include <midi/midi_parser.h>
#include <midi/midi_types.h>

#include <dk_buttons_and_leds.h>

#include <logging/log.h>

#define LOG_MODULE_NAME peripheral_midi
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define RUN_STATUS_LED DK_LED1
#define UART_RX_LED DK_LED2
#define RUN_LED_BLINK_INTERVAL 1000

#define STACKSIZE 1024
#define PRIORITY 7

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);
static K_FIFO_DEFINE(fifo_uart_midi_write);

static const struct device *uart;
static struct k_work uart_empty_tx_buffer_work;
static struct k_work_delayable uart_rx_enable_work;
static struct k_work_delayable uart_midi_write_work;

static uint8_t uart_rx_buf[2];

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

		timestamp = TIMESTAMP(k_ticks_to_ms_near64(k_uptime_ticks()));

		rx_msg = midi_parse_byte(timestamp, *evt->data.rx.buf, &parser);

		if (rx_msg) {
			k_fifo_put(&fifo_uart_rx_data, rx_msg);
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
	k_work_init_delayable(&uart_midi_write_work,
			uart_midi_write_work_handler);

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		return err;
	}

	return uart_rx_enable(uart, &uart_rx_buf[0], 1, SYS_FOREVER_MS);
}



void main(void)
{
	int blink_status = 0;
	int err;

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}

	err = uart_init();
	if (err) {
		LOG_ERR("UART unable to initialize (err: %d)", err);
	}

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

void midi_rx_thread(void)
{

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct midi_msg_t *msg = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		/* Handle MIDI data here */

		uart_midi_write(msg);
	}
}

K_THREAD_DEFINE(midi_rx_thread_id, STACKSIZE, midi_rx_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);