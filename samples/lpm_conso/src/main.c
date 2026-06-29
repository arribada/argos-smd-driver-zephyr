/*
 * Copyright (c) 2025 Arribada Initiative
 * SPDX-License-Identifier: Apache-2.0
 *
 * LPM current-consumption helper for the Argos SMD module over UART.
 *
 * Holds the module in each state for a fixed window so an ammeter on the module
 * supply can be read per step: first ACTIVE (LPM=NONE, awake) as a reference,
 * then SLEEP / STOP / STANDBY / SHUTDOWN with the wake line LOW so the module is
 * actually sleeping. Clear START/END markers bracket each 30 s measurement
 * window. AT+LPM is set in HEX (AT+LPM=0x3); the module is NRST-reset at startup.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <argos-smd/argos_smd.h>

LOG_MODULE_REGISTER(lpm_conso, LOG_LEVEL_INF);

#define HOLD_MS        60000
#define WAKE_BOOT_MS   700

/* Build with -DHOLD_MODE=<1..4> to hold ONE mode (user-paced, ~150 s) instead of
 * the full auto-cycle; default 0 = full cycle. */
#ifndef HOLD_MODE
#define HOLD_MODE 0
#endif

#define RESP_SZ 128
K_MSGQ_DEFINE(response_msgq, RESP_SZ, 16, 1);

static const char *const names[] = { "NONE/ACTIVE", "SLEEP", "STOP", "STANDBY", "SHUTDOWN" };

/* AT+LPM uses a BITMAP, not a mode number: SLEEP=0x01, STOP=0x02, STANDBY=0x04,
 * SHUTDOWN=0x08 (NONE=0x00). Sending 0x3 for STANDBY (wrong) = SLEEP|STOP and is
 * ignored; 0x4 = STANDBY, not SHUTDOWN. */
static const uint8_t lpm_bits[] = { 0x00, 0x01, 0x02, 0x04, 0x08 };

static void read_callback(const char *response, void *ud)
{
	ARG_UNUSED(ud);
	char out[RESP_SZ];
	strncpy(out, response, sizeof(out) - 1);
	out[sizeof(out) - 1] = '\0';
	k_msgq_put(&response_msgq, out, K_NO_WAIT);
}

static void flush_responses(void)
{
	char out[RESP_SZ];
	while (k_msgq_get(&response_msgq, out, K_NO_WAIT) == 0) {
	}
}

static int wait_for_ok(k_timeout_t timeout)
{
	char out[RESP_SZ];
	while (1) {
		if (k_msgq_get(&response_msgq, out, timeout) != 0) {
			return -EAGAIN;
		}
		if (strcmp(out, "+OK") == 0) {
			return 0;
		}
		if (strncmp(out, "+ERROR=", 7) == 0) {
			return -EIO;
		}
	}
}

static void wake_module(const struct device *dev)
{
	argos_smd_wakeup_enable(dev);
	k_msleep(WAKE_BOOT_MS);
}

static bool module_responds(const struct device *dev)
{
	flush_responses();
	argos_read_ping(dev);
	return wait_for_ok(K_MSEC(3000)) == 0;
}

/* FORCE the mode: AT+LPM=0x<bitmap>,0x<forced>. Sending only the bitmap
 * (AT+LPM=0x3) clears the forced mode, so the module never actually enters the
 * deep mode. Here bitmap and forced are both the mode so it is allowed AND
 * forced; mode 0 -> "0x0,0x0" clears the force (back to NONE). */
static bool set_lpm(const struct device *dev, int mode)
{
	char s[12];
	uint8_t bits = lpm_bits[mode];

	snprintf(s, sizeof(s), "0x%X,0x%X", bits, bits);
	flush_responses();
	argos_set_lpm(dev, s);
	return wait_for_ok(K_MSEC(2000)) == 0;
}

/* Log a 30 s measurement window with a 10 s countdown. */
static void hold_window(const char *label, int code)
{
	LOG_INF("================================================");
	LOG_INF(">>> MEASURE: %-10s (LPM=0x%02X) — read the meter NOW", label, code);
	LOG_INF(">>> holding %d s ...", HOLD_MS / 1000);
	LOG_INF("================================================");
	for (int left = HOLD_MS / 1000; left > 0; left -= 10) {
		k_msleep(10000);
		if (left - 10 > 0) {
			LOG_INF("    %s: %d s left", label, left - 10);
		}
	}
	LOG_INF("<<< END: %s", label);
	LOG_INF(" ");
}

int main(void)
{
	const struct device *dev = DEVICE_DT_GET_ONE(arribada_argos_smd_uart);

	LOG_INF("===== Argos SMD LPM current-consumption run (UART) =====");

	if (!device_is_ready(dev)) {
		LOG_ERR("Argos SMD UART device not ready");
		return -ENODEV;
	}
	argos_smd_set_callback(dev, read_callback, NULL);

	/* NRST reset so the module is responsive. */
	const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (device_is_ready(gpio0)) {
		gpio_pin_configure(gpio0, 7, GPIO_OUTPUT_HIGH);
		k_msleep(10);
		gpio_pin_set_raw(gpio0, 7, 0);
		k_msleep(20);
		gpio_pin_set_raw(gpio0, 7, 1);
		k_msleep(600);
	}

	wake_module(dev);
	if (!module_responds(dev)) {
		LOG_ERR("Module not responding — aborting");
		return -EIO;
	}
	LOG_INF("Module responsive. Starting measurement in 3 s...");
	k_msleep(3000);

	/* --- ACTIVE baseline: LPM=NONE, module awake (wake line HIGH) --- */
	set_lpm(dev, 0);
	argos_smd_wakeup_enable(dev);     /* keep awake */
	hold_window(names[0], lpm_bits[0]);

	/* --- Each low-power mode: set (hex), drop wake -> module sleeps --- */
	for (int mode = 1; mode <= 4; mode++) {
		wake_module(dev);
		if (!module_responds(dev)) {
			LOG_WRN("re-wake before %s failed, continuing", names[mode]);
			wake_module(dev);
		}
		/* Drop WKUP3 LOW BEFORE forcing the mode. If WKUP3 is HIGH when the
		 * module enters STANDBY/SHUTDOWN, the active level wakes it immediately
		 * (cold boot) -> it never sleeps and draws the full active current.
		 * The module is in NONE here (awake), so it still receives the cmd. */
		argos_smd_wakeup_disable(dev);
		k_msleep(100);
		bool ack = set_lpm(dev, mode);
		LOG_INF("  AT+LPM=0x%02X,0x%02X (%s, forced): %s", lpm_bits[mode],
			lpm_bits[mode], names[mode],
			ack ? "+OK" : "no ack (forced into deep sleep)");

		hold_window(names[mode], lpm_bits[mode]);
	}

	/* Restore NONE and leave the module awake/responsive. */
	wake_module(dev);
	set_lpm(dev, 0);
	LOG_INF("===== Conso run done — LPM restored to NONE =====");
	return 0;
}
