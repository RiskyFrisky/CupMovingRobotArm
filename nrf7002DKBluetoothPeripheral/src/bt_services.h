#ifndef BT_SERVICES_H_
#define BT_SERVICES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

/** @brief Angle Service UUID. */
#define BT_UUID_ANGLE_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x0000A000, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

/** @brief Angle Characteristic UUID. */
#define BT_UUID_ANGLE_VAL \
	BT_UUID_128_ENCODE(0x0000A001, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

/** @brief Service Name Characteristic UUID. */
#define BT_UUID_SERVICE_NAME_VAL \
	BT_UUID_128_ENCODE(0x0000A002, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_ANGLE_SERVICE BT_UUID_DECLARE_128(BT_UUID_ANGLE_SERVICE_VAL)
#define BT_UUID_ANGLE BT_UUID_DECLARE_128(BT_UUID_ANGLE_VAL)
#define BT_UUID_SERVICE_NAME BT_UUID_DECLARE_128(BT_UUID_SERVICE_NAME_VAL)

/** @brief Callback type for angle value change. */
typedef void (*angle_cb_t)(uint16_t angle1, uint16_t angle2, uint16_t angle3,
                          uint16_t angle4, uint16_t angle5, uint8_t state);

/** @brief Callback struct used by Angle Service. */
struct bt_my_service_cb {
	/** Angle value change callback. */
	angle_cb_t angle_cb;
};

/** @brief Initialize Angle Service.
 *
 * This function registers a GATT service with one characteristic:
 * - Angle: Read, Write and Notify
 *
 * @param[in] callbacks Struct containing pointers to callback functions
 *			used by the service. This pointer can be NULL
 *			if no callback functions are defined.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_my_service_init(struct bt_my_service_cb *callbacks);

/** @brief Send angle values.
 *
 * This function sends the angle values to all connected peers.
 *
 * @param[in] angle1 First angle value (0-360)
 * @param[in] angle2 Second angle value (0-360)
 * @param[in] angle3 Third angle value (0-360)
 * @param[in] angle4 Fourth angle value (0-360)
 * @param[in] angle5 Fifth angle value (0-360)
 * @param[in] state Binary state (0 or 1)
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_my_service_send_angles(uint16_t angle1, uint16_t angle2, uint16_t angle3,
                             uint16_t angle4, uint16_t angle5, uint8_t state);

/** @brief Get current angle values.
 *
 * @param[out] angle1 First angle value
 * @param[out] angle2 Second angle value
 * @param[out] angle3 Third angle value
 * @param[out] angle4 Fourth angle value
 * @param[out] angle5 Fifth angle value
 * @param[out] state Binary state
 */
void bt_my_service_get_angles(uint16_t *angle1, uint16_t *angle2, uint16_t *angle3,
                             uint16_t *angle4, uint16_t *angle5, uint8_t *state);

#ifdef __cplusplus
}
#endif

#endif /* BT_SERVICES_H_ */