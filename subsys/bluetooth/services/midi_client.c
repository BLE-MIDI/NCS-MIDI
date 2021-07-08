#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <bluetooth/services/midi.h>
#include <bluetooth/services/midi_client.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(bt_midi_client, CONFIG_BT_MIDI_CLIENT_LOG_LEVEL);

enum {
	MIDI_CLIENT_INITIALIZED,
	MIDI_CLIENT_IO_NOTIF_ENABLED,
	MIDI_CLIENT_IO_WRITE_PENDING
};

static uint8_t on_received(struct bt_conn *conn,
			struct bt_gatt_subscribe_params *params,
			const void *data, uint16_t length)
{
	struct bt_midi_client *midi_client;

	/* Retrieve MIDI Client module context. */
	midi_client = CONTAINER_OF(params, struct bt_midi_client, io_notif_params);

	if (!data) {
		LOG_DBG("[UNSUBSCRIBED]");
		params->value_handle = 0;
		atomic_clear_bit(&midi_client->state, MIDI_CLIENT_IO_NOTIF_ENABLED);
		if (midi_client->cb.unsubscribed) {
			midi_client->cb.unsubscribed();
		}
		return BT_GATT_ITER_STOP;
	}

	LOG_DBG("[NOTIFICATION] data %p length %u", data, length);
	if (midi_client->cb.received) {
		return midi_client->cb.received(conn, data, length);
		
	}

	return BT_GATT_ITER_CONTINUE;
}


int bt_midi_client_init(struct bt_midi_client *midi_client,
		       const struct bt_midi_client_init_param *midi_client_init)
{
	if (!midi_client || !midi_client_init) {
		return -EINVAL;
	}

	if (atomic_test_and_set_bit(&midi_client->state, MIDI_CLIENT_INITIALIZED)) {
		return -EALREADY;
	}

	memcpy(&midi_client->cb, &midi_client_init->cb, sizeof(midi_client->cb));

	return 0;
}

int bt_midi_client_send(struct bt_midi_client *midi_client, const uint8_t *data,
		       uint16_t len)
{
	int err;

	if (!midi_client || !midi_client->conn) {
		return -EINVAL;
	}

	err = bt_gatt_write_without_response(midi_client->conn,
					     midi_client->handles.io, data, len, false);

	return err;
}

int bt_midi_client_handles_assign(struct bt_gatt_dm *dm,
								 struct bt_midi_client *midi_client) {
	const struct bt_gatt_dm_attr *gatt_service_attr =
		bt_gatt_dm_service_get(dm);
	const struct bt_gatt_service_val *gatt_service =
		bt_gatt_dm_attr_service_val(gatt_service_attr);
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_desc;

	if (bt_uuid_cmp(gatt_service->uuid, BT_UUID_MIDI_SERVICE)) {
		return -ENOTSUP;
	}
	LOG_DBG("Getting handles from MIDI service.");

	memset(&midi_client->handles, 0xFF, sizeof(midi_client->handles));

	/* MIDI I/O Characteristic */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_MIDI_IO);
	if (!gatt_chrc) {
		LOG_ERR("Missing MIDI IO characteristic.");
		return -EINVAL;
	}
	/* MIDI I/O */
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_MIDI_IO);
	if (!gatt_desc) {
		LOG_ERR("Missing MIDI IO value descriptor in characteristic.");
		return -EINVAL;
	}
	LOG_DBG("Found handle for MIDI IO characteristic.");
	midi_client->handles.io = gatt_desc->handle;
	/* MIDI IO CCC */
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_GATT_CCC);
	if (!gatt_desc) {
		LOG_ERR("Missing MIDI IO CCC in characteristic.");
		return -EINVAL;
	}
	LOG_DBG("Found handle for CCC of MIDI IO characteristic.");
	midi_client->handles.io_ccc = gatt_desc->handle;

	/* Assign connection instance. */
	midi_client->conn = bt_gatt_dm_conn_get(dm);
	return 0;
}


int bt_midi_client_notif_enable(struct bt_midi_client *midi_client)
{
	int err;

	if (atomic_test_and_set_bit(&midi_client->state, MIDI_CLIENT_IO_NOTIF_ENABLED)) {
		return -EALREADY;
	}

	midi_client->io_notif_params.notify = on_received;
	midi_client->io_notif_params.value = BT_GATT_CCC_NOTIFY;
	midi_client->io_notif_params.value_handle = midi_client->handles.io;
	midi_client->io_notif_params.ccc_handle = midi_client->handles.io_ccc;
	atomic_set_bit(midi_client->io_notif_params.flags,
		       BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

	err = bt_gatt_subscribe(midi_client->conn, &midi_client->io_notif_params);
	if (err) {
		LOG_ERR("Subscribe failed (err %d)", err);
		atomic_clear_bit(&midi_client->state, MIDI_CLIENT_IO_NOTIF_ENABLED);
	} else {
		LOG_DBG("[SUBSCRIBED]");
	}

	return err;
}