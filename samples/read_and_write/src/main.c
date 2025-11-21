
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

#include <argos-smd/argos_smd.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

void array_to_string(uint8_t *buf, char *str, uint8_t len)
{
	for (int i = 0; i < len; i++) {
		str[i] = (char)buf[i];
	}
	str[len-1] = '\0'; // Remove renew line and null-terminate the string
}


void read_callback(uint8_t *buf, size_t len, void *user_data)
{
	char response[ARGOS_SMD_BUF_SIZE];
	array_to_string(buf, response, len);
	LOG_INF("Received response: %s", response);
}

int main(void)

{
	const struct device *dev_smd = DEVICE_DT_GET_ONE(arribada_argossmd);

	LOG_INF("Booting");
	k_sleep(K_MSEC(1000));

	k_sleep(K_MSEC(1000));
	argos_smd_set_callback(dev_smd, read_callback, NULL);

	argos_read_ping(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_set_address(dev_smd, "ABCDEF01");
	argos_read_address(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_id(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_firmware_version(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_serial_number(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_radioconf(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_prepass_enable(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_udate(dev_smd);

	k_sleep(K_MSEC(1000));
	argos_read_kmac(dev_smd);

	k_sleep(K_MSEC(3000));
	char msg[9] = "FFFFFFFF";
	argos_send_payload(dev_smd, msg);

	k_sleep(K_MSEC(3000));

	return 0;
}
