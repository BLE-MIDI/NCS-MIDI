#ifndef BT_MIDI_H_
#define BT_MIDI_H_

/**
 * @file
 * @defgroup bt_nus Nordic MIDI GATT Service
 * @{
 * @brief Nordic MIDI GATT Service API.
 */

#include <zephyr/types.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief UUID of the MIDI Service. **/
#define BT_UUID_MIDI_VAL \
	BT_UUID_128_ENCODE(0x03b80e5a, 0xede8, 0x4b33, 0xa751, 0x6ce34ec4c700)

/** @brief UUID of the Data IO Characteristic. **/
#define BT_UUID_MIDI_IO_VAL \
	BT_UUID_128_ENCODE(0x7772e5db, 0x3868, 0x4112, 0xa1a9, 0xf2669d106bf3)

#define BT_UUID_MIDI_SERVICE   		BT_UUID_DECLARE_128(BT_UUID_MIDI_VAL)
#define BT_UUID_MIDI_SERVICE_SHORT 	BT_UUID_DECLARE_16(0x0E5A)
#define BT_UUID_MIDI_IO 			BT_UUID_DECLARE_128(BT_UUID_MIDI_IO_VAL)

/** @brief MIDI send status. */
enum bt_midi_send_status {
	/** Send notification enabled. */
	BT_MIDI_SEND_STATUS_ENABLED,
	/** Send notification disabled. */
	BT_MIDI_SEND_STATUS_DISABLED,
};

/** @brief Pointers to the callback functions for service events. */
struct bt_midi_cb {
	/** @brief Data received callback.
	 *
	 * The data has been received on the MIDI I/O Characteristic.
	 *
	 * @param[in] conn  Pointer to connection object that has received data.
	 * @param[in] data  Received data.
	 * @param[in] len   Length of received data.
	 */
 	void (*received)(struct bt_conn *conn,
				  const uint8_t *const data, uint16_t len);

    /** @brief Data sent callback.
	 *
	 * The data has been sent as a notification and written on the MIDI I/O
	 * Characteristic.
	 *
	 * @param[in] conn Pointer to connection object, or NULL if sent to all
	 *                 connected peers.
	 */
	void (*sent)(struct bt_conn *conn);

	/** @brief Send state callback.
	 *
	 * Indicate the CCCD descriptor status of the MIDI I/O characteristic.
	 *
	 * @param[in] status Send notification status.
	 */
	void (*send_enabled)(enum bt_midi_send_status status);
};

/** @brief Initialize the service.
 *
 * @details This function registers a GATT service with one I/O characteristic.
 *          A remote device that is connected to this service can
 *          send data to the Characteristic. When the remote enables
 *          notifications, it is notified when data is sent to the
 *          Characteristic.
 *
 * @param[in] callbacks  Struct with function pointers to callbacks for service
 *                       events. If no callbacks are needed, this parameter can
 *                       be NULL.
 *
 * @retval 0 If initialization is successful.
 *           Otherwise, a negative value is returned.
 */
int bt_midi_init(struct bt_midi_cb *callbacks);

/** @brief Send data.
 *
 * @details This function sends data to a connected peer.
 *
 * @param[in] conn Pointer to connection Object.
 * @param[in] data Pointer to a data buffer.
 * @param[in] len  Length of the data in the buffer.
 *
 * @retval 0 If the data is sent.
 *           Otherwise, a negative value is returned.
 */
int bt_midi_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);

/** @brief Get maximum data length that can be used for @ref bt_midi_send.
 *
 * @param[in] conn Pointer to connection Object.
 *
 * @return Maximum data length.
 */
static inline uint32_t bt_nus_get_mtu(struct bt_conn *conn)
{
	/* According to 3.4.7.1 Handle Value Notification off the ATT protocol.
	 * Maximum supported notification is ATT_MTU - 3 */
	return bt_gatt_get_mtu(conn) - 3;
}

#ifdef __cplusplus
}
#endif

/**
 *@}
 */

#endif /* BT_MIDI_H_ */
