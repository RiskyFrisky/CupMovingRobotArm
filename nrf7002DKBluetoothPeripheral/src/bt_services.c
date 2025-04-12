#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>
#include "bt_services.h"

static bool angle_notify_enabled;
static uint16_t angle_values[5] = {0, 0, 0, 0, 0};
static uint8_t angle_state = 0;
static struct bt_my_service_cb my_service_cb;

static void angle_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                uint16_t value)
{
    angle_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t read_angle(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         void *buf,
                         uint16_t len,
                         uint16_t offset)
{
    uint8_t data[13]; // 5 angles (2 bytes each) + 1 state byte
    sys_put_le16(angle_values[0], &data[0]);
    sys_put_le16(angle_values[1], &data[2]);
    sys_put_le16(angle_values[2], &data[4]);
    sys_put_le16(angle_values[3], &data[6]);
    sys_put_le16(angle_values[4], &data[8]);
    data[10] = angle_state;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(data));
}

static ssize_t write_angle(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf,
                         uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != 13) { // 5 angles (2 bytes each) + 1 state byte
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (my_service_cb.angle_cb) {
        const uint8_t *data = buf;
        uint16_t new_angles[5];
        uint8_t new_state;

        new_angles[0] = sys_get_le16(&data[0]);
        new_angles[1] = sys_get_le16(&data[2]);
        new_angles[2] = sys_get_le16(&data[4]);
        new_angles[3] = sys_get_le16(&data[6]);
        new_angles[4] = sys_get_le16(&data[8]);
        new_state = data[10];

        // Validate angles are within 0-360 range
        for (int i = 0; i < 5; i++) {
            if (new_angles[i] > 360) {
                return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
            }
        }

        // Validate state is 0 or 1
        if (new_state > 1) {
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
        }

        memcpy(angle_values, new_angles, sizeof(angle_values));
        angle_state = new_state;
        my_service_cb.angle_cb(new_angles[0], new_angles[1], new_angles[2],
                              new_angles[3], new_angles[4], new_state);
    }

    return len;
}

/* Service Declaration */
BT_GATT_SERVICE_DEFINE(angle_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_ANGLE_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_ANGLE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_angle, write_angle, NULL),
    BT_GATT_CCC(angle_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

int bt_my_service_init(struct bt_my_service_cb *callbacks)
{
    if (callbacks) {
        my_service_cb.angle_cb = callbacks->angle_cb;
    }

    return 0;
}

int bt_my_service_send_angles(uint16_t angle1, uint16_t angle2, uint16_t angle3,
                             uint16_t angle4, uint16_t angle5, uint8_t state)
{
    if (!angle_notify_enabled) {
        return -EACCES;
    }

    // Validate angles are within 0-360 range
    if (angle1 > 360 || angle2 > 360 || angle3 > 360 ||
        angle4 > 360 || angle5 > 360) {
        return -EINVAL;
    }

    // Validate state is 0 or 1
    if (state > 1) {
        return -EINVAL;
    }

    angle_values[0] = angle1;
    angle_values[1] = angle2;
    angle_values[2] = angle3;
    angle_values[3] = angle4;
    angle_values[4] = angle5;
    angle_state = state;

    uint8_t data[13];
    sys_put_le16(angle1, &data[0]);
    sys_put_le16(angle2, &data[2]);
    sys_put_le16(angle3, &data[4]);
    sys_put_le16(angle4, &data[6]);
    sys_put_le16(angle5, &data[8]);
    data[10] = state;

    return bt_gatt_notify(NULL, &angle_service.attrs[1],
                         data, sizeof(data));
}

void bt_my_service_get_angles(uint16_t *angle1, uint16_t *angle2, uint16_t *angle3,
                             uint16_t *angle4, uint16_t *angle5, uint8_t *state)
{
    if (angle1) *angle1 = angle_values[0];
    if (angle2) *angle2 = angle_values[1];
    if (angle3) *angle3 = angle_values[2];
    if (angle4) *angle4 = angle_values[3];
    if (angle5) *angle5 = angle_values[4];
    if (state) *state = angle_state;
}