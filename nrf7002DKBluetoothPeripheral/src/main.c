#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/settings/settings.h>

#include <dk_buttons_and_leds.h>

#include "bt_services.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#define DEVICE_NAME "RobotArm"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED1 // Connection status LED
#define RUN_LED_BLINK_INTERVAL 1000

#define INCREMENT_BUTTON DK_BTN1_MSK
#define DECREMENT_BUTTON DK_BTN2_MSK
#define GRIPPER_LED DK_LED2 // Gripper state indicator LED

#define RECEIVE_BUFF_SIZE 10
#define RECEIVE_TIMEOUT 100

static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};

static uint8_t gripperOpen = 0; // State variable for gripper (0=closed, 1=opened)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ANGLE_SERVICE_VAL),
};

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err)
	{
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		return;
	}

	printk("Connected\n");

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	dk_set_led_off(CON_STATUS_LED);
}

#ifdef CONFIG_BT_MY_SERVICE_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
							 enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err)
	{
		printk("Security changed: %s level %u\n", addr, level);
	}
	else
	{
		printk("Security failed: %s level %u err %d %s\n", addr, level, err,
			   bt_security_err_to_str(err));
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;

static void update_gripper_open_led(void)
{
	// LED state is controlled by the gripper state
	dk_set_led(GRIPPER_LED, gripperOpen);
}

static void app_angle_cb(uint16_t angle1, uint16_t angle2, uint16_t angle3,
						 uint16_t angle4, uint16_t angle5, uint8_t state)
{
	printk("Angles updated: %d, %d, %d, %d, %d, State: %d\n",
		   angle1, angle2, angle3, angle4, angle5, state);
	gripperOpen = state; // Update gripper state based on received state value
	update_gripper_open_led();
}

static struct bt_my_service_cb my_service_callbacks = {
	.angle_cb = app_angle_cb,
};

// Shared function to send angles to Bluetooth
static void send_angles_bt(uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4, uint16_t angle5, uint8_t state)
{
	int err = bt_my_service_send_angles(angle1, angle2, angle3, angle4, angle5, state);
	if (err)
	{
		printk("Failed to send bt angles (err: %d)\n", err);
	}
}

// Shared function to send angles to UART
static void send_angles_uart(uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4, uint16_t angle5, uint8_t state)
{
	char tx_buf[32];
	int len = snprintf(tx_buf, sizeof(tx_buf), "%u,%u,%u,%u,%u,%u\n", angle1, angle2, angle3, angle4, angle5, state);
	if (len > 0 && len < sizeof(tx_buf))
	{
		int ret = uart_tx(uart, tx_buf, len, SYS_FOREVER_US);
		if (ret)
		{
			printk("Failed to send uart data (err: %d)\n", ret);
		}
	}
}

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;
	uint16_t angle1 = 0, angle2 = 0, angle3 = 0, angle4 = 0, angle5 = 0;
	uint8_t state = 0;

	if (buttons & (INCREMENT_BUTTON | DECREMENT_BUTTON))
	{
		bt_my_service_get_angles(&angle1, &angle2, &angle3, &angle4, &angle5, &state);

		// Determine if we're incrementing or decrementing
		int delta = (buttons & INCREMENT_BUTTON) ? 1 : -1;

		// Update angles with wrapping between 0 and 360
		angle1 = (angle1 + delta + 360) % 360;
		angle2 = (angle2 + delta + 360) % 360;
		angle3 = (angle3 + delta + 360) % 360;
		angle4 = (angle4 + delta + 360) % 360;
		angle5 = (angle5 + delta + 360) % 360;

		printk("Button pressed - New values:\n");
		printk("Angle1: %d, Angle2: %d, Angle3: %d, Angle4: %d, Angle5: %d, State: %d\n",
			   angle1, angle2, angle3, angle4, angle5, state);

		send_angles_bt(angle1, angle2, angle3, angle4, angle5, state);
		send_angles_uart(angle1, angle2, angle3, angle4, angle5, state);
	}
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type)
	{
	case UART_RX_RDY:
		if (evt->data.rx.len > 0)
		{
			// get string from buffer
			char *str = (char *)evt->data.rx.buf;
			// format is "0,0,0,0,0,0\n"
			uint16_t angle1 = 0, angle2 = 0, angle3 = 0, angle4 = 0, angle5 = 0;
			uint8_t state = 0;
			int parsed = sscanf(str, "%hu,%hu,%hu,%hu,%hu,%hhu", &angle1, &angle2, &angle3, &angle4, &angle5, &state);
			if (parsed == 6)
			{
				printk("UART RX parsed: %d,%d,%d,%d,%d,%d\n", angle1, angle2, angle3, angle4, angle5, state);
				send_angles_bt(angle1, angle2, angle3, angle4, angle5, state);
			}
			else
			{
				printk("UART RX parse failed: %s\n", str);
			}
		}
		break;
	case UART_RX_DISABLED:
		uart_rx_enable(dev, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
		break;
	default:
		break;
	}
}

int main(void)
{
	int err;

	printk("Starting RobotArm program\n");

	err = dk_leds_init();
	if (err)
	{
		printk("LEDs init failed (err %d)\n", err);
		return 1;
	}

	err = dk_buttons_init(button_changed);
	if (err)
	{
		printk("Button init failed (err %d)\n", err);
		return 1;
	}

	if (!device_is_ready(uart))
	{
		printk("UART device not ready\n");
		return 1;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err)
	{
		printk("Failed to set UART callback (err %d)\n", err);
		return 1;
	}

	err = uart_rx_enable(uart, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
	if (err)
	{
		printk("Failed to enable UART RX (err %d)\n", err);
		return 1;
	}

	// Initialize gripper LED state
	update_gripper_open_led();

	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	err = bt_my_service_init(&my_service_callbacks);
	if (err)
	{
		printk("Failed to init my service (err: %d)\n", err);
		return 1;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
						  ARRAY_SIZE(sd));
	if (err)
	{
		printk("Advertising failed to start (err %d)\n", err);
		return 1;
	}

	printk("Advertising successfully started\n");

	for (;;)
	{
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}
