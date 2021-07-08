#ifndef BT_MIDI_CLIENT_H_
#define BT_MIDI_CLIENT_H_

/**
 * @file
 * @defgroup bt_midi_client Bluetooth LE GATT MIDI Client API
 * @{
 * @brief API for the Bluetooth LE GATT MIDI Service Client.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <bluetooth/gatt.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt_dm.h>

/** @brief Handles on the connected peer device that are needed to interact with
 * the device.
 */
struct bt_midi_client_handles {

 	/** Handle of the IO characteristic, as provided by
	 *  a discovery.
 	 */
	uint16_t io;

 	/** Handle of the CCC descriptor of the IO characteristic,
	 *  as provided by a discovery.
	 */
	uint16_t io_ccc;
};

/** @brief MIDI Client callback structure. */
struct bt_midi_client_cb {
	/** @brief Data received callback.
	 *
	 * The data has been received as a notification of the MIDI I/O
	 * Characteristic.
	 *
	 * @param[in] data Received data.
	 * @param[in] len Length of received data.
	 *
	 * @retval BT_GATT_ITER_CONTINUE To keep notifications enabled.
	 * @retval BT_GATT_ITER_STOP To disable notifications.
	 */
	uint8_t (*received)(struct bt_conn *conn, const uint8_t *data, uint16_t len);

	/** @brief Data sent callback.
	 *
	 * The data has been sent and written to the MIDI I/O Characteristic.
	 *
	 * @param[in] err ATT error code.
	 * @param[in] data Transmitted data.
	 * @param[in] len Length of transmitted data.
	 */
	void (*sent)(uint8_t err, const uint8_t *data, uint16_t len);

	/** @brief TX notifications disabled callback.
	 *
	 * TX notifications have been disabled.
	 */
	void (*unsubscribed)(void);
};

/** @brief MIDI Client structure. */
struct bt_midi_client {

    /** Connection object. */
	struct bt_conn *conn;

    /** Internal state. */
	atomic_t state;

	/** Handles on the connected peer device that are needed
	 * to interact with the device.
	 */
	struct bt_midi_client_handles handles;

	/** GATT subscribe parameters for MIDI I/O Characteristic. */
	struct bt_gatt_subscribe_params io_notif_params;

	/** GATT write parameters for MIDI I/O Characteristic. */
	struct bt_gatt_write_params io_write_params;

	/** Application callbacks. */
	struct bt_midi_client_cb cb;
};

/** @brief MIDI Client initialization structure. */
struct bt_midi_client_init_param {

    /** Callbacks provided by the user. */
	struct bt_midi_client_cb cb;
};

/** @brief Initialize the MIDI Client module.
 *
 * This function initializes the MIDI Client module with callbacks provided by
 * the user.
 *
 * @param[in,out] midi MIDI Client instance.
 * @param[in] init_param bms Client initialization instance.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int bt_midi_client_init(struct bt_midi_client *midi,
		       const struct bt_midi_client_init_param *init_param);

/** @brief Send data to the server.
 *
 * This function writes to the I/O Characteristic of the server.
 *
 * @note This procedure is asynchronous. Therefore, the data to be sent must
 * remain valid while the function is active.
 *
 * @param[in,out] midi MIDI Client instance.
 * @param[in] data Data to be transmitted.
 * @param[in] len Length of data.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int bt_midi_client_send(struct bt_midi_client *midi, const uint8_t *data,
		       uint16_t len);

/** @brief Assign handles to the bms Client instance.
 *
 * This function should be called when a link with a peer has been established
 * to associate the link to this instance of the module. This makes it
 * possible to handle several links and associate each link to a particular
 * instance of this module. The GATT attribute handles are provided by the
 * GATT DB discovery module.
 *
 * @param[in] dm Discovery object.
 * @param[in,out] midi MIDI Client instance.
 *
 * @retval 0 If the operation was successful.
 * @retval (-ENOTSUP) Special error code used when UUID
 *         of the service does not match the expected UUID.
 * @retval Otherwise, a negative error code is returned.
 */
int bt_midi_client_handles_assign(struct bt_gatt_dm *dm,
				 struct bt_midi_client *midi);

/** @brief Request the peer to start sending notifications for the I/O
 *	   Characteristic.
 *
 * This function enables notifications for the MIDI I/O Characteristic at the peer
 * by writing to the CCC descriptor of the bms I/O Characteristic.
 *
 * @param[in,out] midi MIDI Client instance.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int bt_midi_client_notif_enable(struct bt_midi_client *midi);

#ifdef __cplusplus
}
#endif

/**
 *@}
 */

#endif /* BT_MIDI_CLIENT_H_ */
