

/**
 * @file midi_types.h
 *
 * @defgroup midi_types MIDI types
 * @{
 * @brief Types used for MIDI applications
 */
#ifndef MIDI_TYPES_H__
#define MIDI_TYPES_H__

#include <stdbool.h>
#include <zephyr/types.h>


#ifdef __cplusplus
extern "C" {
#endif

/** @brief Struct holding a MIDI message. */
struct midi_msg_t {
    /** reserved for FIFO use. */
	void *fifo_reserved;
    /** timestamp, 13 bits are used with ms resolution */
	uint16_t timestamp;
    /** MIDI data */
	uint8_t data[3];
    /** length of MIDI data */
	uint8_t len;
};



/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MIDI_TYPES_H__ */
