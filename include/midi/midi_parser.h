/**
 * @file midi_parser.h
 *
 * @defgroup midi_parser MIDI parser
 * @{
 * @brief Basic parser for MIDI streams.
 */
#ifndef MIDI_PARSER_H__
#define MIDI_PARSER_H__

#include <stdbool.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <midi/midi_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Struct holding the state of the MIDI parser instance. */
struct midi_parser_t {
	struct midi_msg_t *msg;
	uint8_t running_status;
	bool third_byte_flag;
};

/**
 * @brief Parse a byte of a MIDI stream.
 *
 * This function parses a byte from a MIDI stream. Function should be called
 * per byte of the stream, and the @p parser state has to be maintained 
 * between each call. When the function returns true the message is completed.
 *
 * @param timestamp        13-bit timestamp with ms resolution.
 *
 * @param byte             The byte to be parsed.
 *
 * @param parser           Pointer to the parser of the MIDI stream
 *
 * @retval pointer to the parsed message if message is complete.
 * @retval NULL if the message is not completed.
 *
 */
struct midi_msg_t *midi_parse_byte(uint16_t timestamp, uint8_t byte,
			    struct midi_parser_t *parser);


/**
 * @brief Reset a parser to initial conditions
 */
void midi_parser_reset(struct midi_parser_t *parser);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MIDI_PARSER_H__ */
