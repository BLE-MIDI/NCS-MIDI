#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <bluetooth/services/midi.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(bt_midi, CONFIG_BT_MIDI_LOG_LEVEL);

static struct bt_midi_cb midi_cb;

static void midi_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	if (midi_cb.send_enabled) {
		LOG_DBG("Notification has been turned %s",
			value == BT_GATT_CCC_NOTIFY ? "on" : "off");
		midi_cb.send_enabled(value == BT_GATT_CCC_NOTIFY ?
			BT_MIDI_SEND_STATUS_ENABLED : BT_MIDI_SEND_STATUS_DISABLED);
	}
}

static ssize_t on_receive(struct bt_conn *conn, 
			const struct bt_gatt_attr *attr,
			const void *buf, 
			uint16_t len, 
			uint16_t offset, 
			uint8_t flags)
{
	LOG_DBG("Received data, handle %d, conn %p",
		attr->handle, (void *)conn);

	if (midi_cb.received) {
		midi_cb.received(conn, buf, len);
    }
    return len;
}

static void on_sent(struct bt_conn *conn, void *user_data)
{
	ARG_UNUSED(user_data);

	LOG_DBG("Data send, conn %p", (void *)conn);

	if (midi_cb.sent) {
		midi_cb.sent(conn);
	}
}

/* BLE MIDI Service Declaration */
BT_GATT_SERVICE_DEFINE(midi_svc,                                               
BT_GATT_PRIMARY_SERVICE(BT_UUID_MIDI_SERVICE),                          
	BT_GATT_CHARACTERISTIC(BT_UUID_MIDI_IO,                        
							BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ | 
							BT_GATT_CHRC_WRITE_WITHOUT_RESP,            
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,     
							NULL, on_receive, NULL),                 
	BT_GATT_CCC(midi_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);         

int bt_midi_init(struct bt_midi_cb *callbacks)
{
	if (callbacks) {
		midi_cb.received = callbacks->received;
		midi_cb.sent = callbacks->sent;
		midi_cb.send_enabled = callbacks->send_enabled;
	}

	return 0;
}

int bt_midi_send(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
	if (!conn) {
		return -EINVAL;
	}
	struct bt_gatt_notify_params params = {0};
	const struct bt_gatt_attr *attr = &midi_svc.attrs[1];

	params.attr = attr;
	params.data = data;
	params.len = len;
	params.func = on_sent;

	if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)){
		return bt_gatt_notify_cb(conn, &params);
	} else {
		return -EINVAL;
	}
}