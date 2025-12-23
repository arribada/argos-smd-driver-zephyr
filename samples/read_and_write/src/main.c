
#include "zephyr/sys/__assert.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

#include <argos-smd/argos_smd.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

#define RESPONSE_BUFFER_SIZE 128
K_MSGQ_DEFINE(response_msgq, RESPONSE_BUFFER_SIZE, 10, 1);

void read_callback(const char *response, void *user_data)
{
	ARG_UNUSED(user_data);
	LOG_INF("Received response: %s, adding to queue", response);

	char out[RESPONSE_BUFFER_SIZE];
	strcpy(out, response);
	k_msgq_put(&response_msgq, out, K_NO_WAIT);
}

int main(void)

{
	char out[RESPONSE_BUFFER_SIZE];

	const struct device *dev_smd = DEVICE_DT_GET_ONE(arribada_argossmd);

	LOG_INF("Zephyr SMD Read and Write Sample");

	argos_smd_set_callback(dev_smd, read_callback, NULL);

	/* Enable wakeup pin to communicate with the module */
	int ret = argos_smd_wakeup_enable(dev_smd);
	if (ret == 0) {
		LOG_INF("Wakeup pin enabled");
	} else if (ret == -ENOTSUP) {
		LOG_INF("No wakeup pin configured, continuing...");
	}

	/////////////////////////////////////////////////

	argos_read_ping(dev_smd);

	k_msgq_get(&response_msgq, out, K_FOREVER);
	if (strcmp(out, "+OK") != 0) {
		LOG_ERR("Invalid Response: %s", out);
	}

	/////////////////////////////////////////////////

	argos_set_address(dev_smd, "ABCDEF01");

	k_msgq_get(&response_msgq, out, K_FOREVER);
	if (strcmp(out, "+OK") != 0) {
		LOG_ERR("Invalid Response: %s", out);
	}

	argos_read_address(dev_smd);

	/* First response: +ADDR=abcdef01 */
	k_msgq_get(&response_msgq, out, K_FOREVER);
	if (strcmp(out, "+ADDR=abcdef01") != 0) {
		LOG_ERR("Invalid Response: %s", out);
	}

	/* Second response: +OK */
	k_msgq_get(&response_msgq, out, K_FOREVER);
	if (strcmp(out, "+OK") != 0) {
		LOG_ERR("Invalid Response: %s", out);
	}

	/////////////////////////////////////////////////

	char msg[9] = "FFFFFFFF";
	argos_send_payload(dev_smd, msg);

	/* First response: +OK */
	k_msgq_get(&response_msgq, out, K_FOREVER);
	if (strcmp(out, "+OK") != 0) {
		LOG_ERR("Invalid Response: %s", out);
	}

	/* Second response: +TX=0,FFFFFFFF => +TX=0 = success*/
	k_msgq_get(&response_msgq, out, K_FOREVER);
	if (strncmp(out, "+TX=0", 5) != 0) {
		LOG_ERR("Invalid TX Response: %s", out);
	}

	/////////////////////////////////////////////////

	/* Disable wakeup pin when done communicating */
	ret = argos_smd_wakeup_disable(dev_smd);
	if (ret == 0) {
		LOG_INF("Wakeup pin disabled, module can enter low power mode");
	}

	LOG_INF("Done");

	return 0;
}
