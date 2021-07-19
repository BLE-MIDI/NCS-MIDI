
#include "midi/midi_parser.h"

static bool parse_byte(uint16_t timestamp, uint8_t byte,
			    struct midi_parser_t *parser)
{
	struct midi_msg_t *msg;
	uint8_t running_status;

	if (!parser->msg) {
		/** Byte is part of new message */
		parser->msg = k_malloc(sizeof(*msg));
		if (!parser->msg) {
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
		parser->third_byte_flag = true;
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

struct midi_msg_t *parse_rtm(uint16_t timestamp, uint8_t byte)
{
	struct midi_msg_t *msg;

	if (((byte >> 7) == 1) && ((byte >> 3) == 0b11111)) {
		/** System Real-Time Messages */
		msg = k_malloc(sizeof(*msg));

		if (!msg) {
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

struct midi_msg_t *midi_parse_byte(uint16_t timestamp, uint8_t byte,
			    struct midi_parser_t *parser)
{
    struct midi_msg_t *ret = NULL;

    ret = parse_rtm(timestamp, byte);
		if (!ret) {
			/** Received byte is System Common- or Channel Voice message. */
			if (parse_byte(timestamp, byte, parser)) {
				ret = parser->msg;
				parser->msg = NULL;
			}
		} else if (parser->msg && (parser->msg->timestamp < timestamp)) {
			/** RTM has interrupted a message */
			parser->msg->timestamp = timestamp;
		}

		return ret;
}

void midi_parser_reset(struct midi_parser_t *parser)
{
    if (parser->msg != NULL) {
        k_free(parser->msg);
    }
    parser->running_status = 0;
    parser->third_byte_flag = false;
}