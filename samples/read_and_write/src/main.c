
#include "zephyr/sys/__assert.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <argos-smd/argos_smd.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

#define RESPONSE_BUFFER_SIZE 128
K_MSGQ_DEFINE(response_msgq, RESPONSE_BUFFER_SIZE, 10, 1);

void read_callback(const char *response, void *user_data)
{
	ARG_UNUSED(user_data);

	/* Driver filters non-AT lines, so we only receive AT responses here */
	char out[RESPONSE_BUFFER_SIZE];
	strcpy(out, response);

	int ret = k_msgq_put(&response_msgq, out, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Queue full! Lost response: %s", response);
	}
	/* Note: Driver already logs "Driver RX complete: [...]" at DBG level */
}

/**
 * @brief Get next response from queue, optionally skipping unwanted responses
 * @param out Output buffer for response
 * @param expected_prefix Expected prefix (e.g., "+RCONF="), or NULL to accept any
 * @param timeout Timeout for waiting
 * @return 0 on success, negative on error
 */
static int get_response(char *out, const char *expected_prefix, k_timeout_t timeout)
{
	while (1) {
		int ret = k_msgq_get(&response_msgq, out, timeout);
		if (ret != 0) {
			return ret;
		}

		size_t len = strlen(out);

		/* Skip echo responses ending with '?' (e.g., +PING=?, +ADDR=?) */
		if (len > 0 && out[len - 1] == '?') {
			LOG_WRN("Skipping echo response: %s", out);
			continue;
		}

		/* If specific prefix expected, check for match FIRST (before +OK).
		 * This prevents +OK from being returned as "data" when the real
		 * data response was lost due to debug log interleaving on STM32.
		 */
		if (expected_prefix != NULL) {
			size_t prefix_len = strlen(expected_prefix);
			/* If prefix ends with '=', also accept response without '=' */
			if (prefix_len > 0 && expected_prefix[prefix_len - 1] == '=') {
				/* Check with '=' (e.g., "+ADDR=xxx") */
				if (strncmp(out, expected_prefix, prefix_len) == 0) {
					return 0;
				}
				/* Check without '=' (e.g., "+ADDR") */
				if (strncmp(out, expected_prefix, prefix_len - 1) == 0 &&
				    (out[prefix_len - 1] == '\0' || out[prefix_len - 1] == '\r' || out[prefix_len - 1] == '\n')) {
					return 0;
				}
			} else {
				/* No '=' in prefix, exact prefix match */
				if (strncmp(out, expected_prefix, prefix_len) == 0) {
					return 0;
				}
			}
		}

		/* Accept +OK and +ERROR as valid terminal responses */
		if (strcmp(out, "+OK") == 0 || strncmp(out, "+ERROR=", 7) == 0) {
			return 0;
		}

		/* Handle asynchronous TX notifications (can arrive anytime) */
		if (strncmp(out, "+TX=", 4) == 0) {
			/* If we're specifically waiting for +TX, return it */
			if (expected_prefix != NULL && strncmp(expected_prefix, "+TX=", 4) == 0) {
				return 0;
			}
			/* Otherwise, log it and continue waiting */
			LOG_INF("Async TX notification: %s", out);
			continue;
		}

		/* If no specific prefix expected, accept any response starting with '+' */
		if (expected_prefix == NULL && out[0] == '+') {
			return 0;
		}

		/* Skip unexpected responses */
		LOG_WRN("Skipping unexpected response: %s", out);
	}
}

/**
 * @brief Wait for +OK response, consuming any intermediate responses
 */
static int wait_for_ok(void)
{
	char out[RESPONSE_BUFFER_SIZE];

	while (1) {
		int ret = k_msgq_get(&response_msgq, out, K_FOREVER);
		if (ret != 0) {
			return ret;
		}

		if (strcmp(out, "+OK") == 0) {
			return 0;
		}

		if (strncmp(out, "+ERROR=", 7) == 0) {
			LOG_ERR("Device returned error: %s", out);
			return -EIO;
		}

		/* Handle asynchronous TX notifications */
		if (strncmp(out, "+TX=", 4) == 0) {
			LOG_INF("Async TX notification: %s", out);
			continue;  /* Continue waiting for +OK */
		}

		/* Log but continue waiting for +OK */
		LOG_DBG("Skipping intermediate response: %s", out);
	}
}

int main(void)

{
	char out[RESPONSE_BUFFER_SIZE];

	const struct device *dev_smd = DEVICE_DT_GET_ONE(arribada_argos_smd_uart);

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
	/* Test 1: Ping */
	/////////////////////////////////////////////////

	LOG_INF("Test 1: Ping device");
	argos_read_ping(dev_smd);

	/* Wait for +OK (auto-skip echo like +PING=?) */
	if (wait_for_ok() != 0) {
		LOG_ERR("Ping failed");
	} else {
		LOG_INF("Ping: OK");
	}

	/////////////////////////////////////////////////
	/* Test 2: Read radio configuration */
	/////////////////////////////////////////////////

	LOG_INF("Test 2: Read radio configuration");
	argos_read_radioconf(dev_smd);

	/* Get +RCONF=<data> response */
	if (get_response(out, "+RCONF=", K_FOREVER) == 0) {
		LOG_INF("RCONF: %s", out);
	} else {
		LOG_ERR("Failed to get RCONF response");
	}

	/* Wait for +OK */
	if (wait_for_ok() != 0) {
		LOG_ERR("RCONF command failed");
	}

	/////////////////////////////////////////////////
	/* Test 3: Read raw radio configuration */
	/////////////////////////////////////////////////

	LOG_INF("Test 3: Read raw radio configuration");
	argos_read_radioconf_raw(dev_smd);

	/* Get +RCONFRAW=<data> response or +ERROR if not supported */
	if (get_response(out, "+RCONFRAW=", K_FOREVER) == 0) {
		LOG_INF("RCONFRAW: %s", out);
	} else  {
		LOG_WRN("RCONFRAW failed: %s", out);
	}

	/* Wait for +OK */
	if (wait_for_ok() != 0) {
		LOG_ERR("RCONF command failed");
	}

	/////////////////////////////////////////////////
	/* Test 4: Set and read address */
	/////////////////////////////////////////////////

	LOG_INF("Test 4: Set address to ABCDEF01");
	argos_set_address(dev_smd, "ABCDEF01");

	if (wait_for_ok() != 0) {
		LOG_ERR("Set address failed");
	}

	LOG_INF("Test 5: Read address back");
	argos_read_address(dev_smd);

	/* Get +ADDR=<data> response (auto-skip echo like +ADDR=?) */
	if (get_response(out, "+ADDR=", K_FOREVER) == 0) {
		LOG_INF("Address: %s", out);
		/* Note: Address might be in different case, don't do strict comparison */
		if (strstr(out, "ABCDEF01") != NULL || strstr(out, "abcdef01") != NULL) {
			LOG_INF("Address matches!");
		} else {
			LOG_WRN("Address mismatch (might need manual verification): %s", out);
		}
	}

	/* Wait for +OK */
	wait_for_ok();

	/////////////////////////////////////////////////
	/* Test 6: Send payload */
	/////////////////////////////////////////////////

	LOG_INF("Test 6: Send payload FFFFFFFF");
	char msg[9] = "FFFFFFFF";
	argos_send_payload(dev_smd, msg);

	/* Wait for +OK */
	if (wait_for_ok() != 0) {
		LOG_ERR("Send payload failed");
	}

	/* Get +TX=<status>,<data> notification */
	if (get_response(out, "+TX=", K_FOREVER) == 0) {
		LOG_INF("TX result: %s", out);
		if (strncmp(out, "+TX=0", 5) == 0) {
			LOG_INF("TX success!");
		} else {
			LOG_WRN("TX status: %s", out);
		}
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
