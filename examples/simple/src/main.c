
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <app_version.h>
#include <stdio.h>
#include <string.h>

#include <argos_smd.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);


BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

void array_to_string(uint8_t *buf, char *str, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        str[i] = (char) buf[i];
    }
    str[len] = '\0'; // Null-terminate the string
}

void read_callback(const struct device *dev_smd, void *user_data)
{
	const struct device *argos_smd_dev = user_data;
	struct argos_smd_data *drv_data = argos_smd_dev->data;

	char new_tag_str[ARGOS_SMD_BUF_SIZE + 1];
	array_to_string(drv_data->response.data, new_tag_str, drv_data->response.len);
	LOG_INF("Received response: %s", new_tag_str);
}

int main(void)

{
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	const struct device *dev_smd = DEVICE_DT_GET_ONE(arribada_argossmd);

	if (usb_enable(NULL)) {
		return 0;
	}

	uint32_t dtr = 0;
	LOG_DBG("UART initialization complete. Waiting for data...\n");
	/* Poll if the DTR flag was set */
	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		/* Give CPU resources to low priority threads. */
		k_sleep(K_MSEC(100));
	}
	k_sleep(K_MSEC(1000));
	argos_smd_set_callback(dev_smd, read_callback, NULL);

	argos_read_ping(dev_smd);
	
	k_sleep(K_MSEC(1000));
	argos_read_firmware_version(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_address(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_serial_number(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_id(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_configuration(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_prepass_enable(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_udate(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_repetition_configured(dev_smd);

	k_sleep(K_MSEC(3000));
	char msg[9] = "FFFFFFFF";
	argos_send_message(dev_smd, msg);

	k_sleep(K_MSEC(3000));

	return 0;
}