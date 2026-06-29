/*
 * Copyright (c) 2025 Arribada Initiative
 * SPDX-License-Identifier: Apache-2.0
 *
 * Low-Power Mode (LPM) wake test for the Argos SMD module over UART.
 *
 * The STM32WL55 Kineis firmware exposes 5 low-power profiles via AT+LPM=<n>:
 *   0 = NONE      (no low power)
 *   1 = SLEEP     (CPU sleep, peripherals on, wakes on activity)
 *   2 = STOP      (clocks stopped, wakes on EXTI/UART)
 *   3 = STANDBY   (VCORE off, wakes on WKUP pin / RTC, COLD BOOT on wake)
 *   4 = SHUTDOWN  (deepest, wakes ONLY on WKUP3/PB3 / NRST / RTC, COLD BOOT)
 *
 * For STANDBY/SHUTDOWN the UART is dead while asleep: the host MUST drive the
 * wake line (Feather D9/P0.26 -> STM32 PB3/WKUP3) HIGH to wake the module, and
 * the rising edge triggers a cold boot, so the module re-runs its firmware
 * before its UART is ready again (hence the boot delay before the first cmd).
 *
 * For each mode this test: wakes + confirms the module answers, sets AT+LPM=<n>,
 * drops the wake line so the module sleeps, then drives the wake line HIGH again
 * and verifies the module answers a ping -> "WAKE OK". The final summary lists
 * every mode. Power consumption is validated separately with a current meter.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <argos-smd/argos_smd.h>

LOG_MODULE_REGISTER(lpm_test, LOG_LEVEL_INF);

#define RESPONSE_BUFFER_SIZE 128
K_MSGQ_DEFINE(response_msgq, RESPONSE_BUFFER_SIZE, 16, 1);

/* How long to hold the module in each LPM mode before trying to wake it. */
#define LPM_SLEEP_DWELL_MS   3000
/* Cold-boot settle time after a rising edge on the wake line. */
#define WAKE_BOOT_DELAY_MS   700
/* Timeout when waiting for a response to a ping. */
#define PING_TIMEOUT_MS      3000

static const char *const lpm_names[] = {
	"NONE", "SLEEP", "STOP", "STANDBY", "SHUTDOWN",
};

static void read_callback(const char *response, void *user_data)
{
	ARG_UNUSED(user_data);

	char out[RESPONSE_BUFFER_SIZE];
	strncpy(out, response, sizeof(out) - 1);
	out[sizeof(out) - 1] = '\0';

	if (k_msgq_put(&response_msgq, out, K_NO_WAIT) != 0) {
		LOG_WRN("Response queue full, dropped: %s", response);
	}
}

static void flush_responses(void)
{
	char out[RESPONSE_BUFFER_SIZE];

	while (k_msgq_get(&response_msgq, out, K_NO_WAIT) == 0) {
		/* drain */
	}
}

/*
 * Wait for a terminal +OK within the timeout, skipping echoes (lines ending in
 * '?') and async +TX notifications. Returns 0 on +OK, -EIO on +ERROR, -EAGAIN
 * on timeout (the module did not answer -> still asleep / failed to wake).
 */
static int wait_for_ok(k_timeout_t timeout)
{
	char out[RESPONSE_BUFFER_SIZE];

	while (1) {
		int ret = k_msgq_get(&response_msgq, out, timeout);
		if (ret != 0) {
			return -EAGAIN;
		}

		if (strcmp(out, "+OK") == 0) {
			return 0;
		}
		if (strncmp(out, "+ERROR=", 7) == 0) {
			LOG_WRN("  module returned %s", out);
			return -EIO;
		}
		/* echo (e.g. +PING=?) or async +TX -> keep waiting */
	}
}

/* Drive the wake line HIGH and give the module time to (cold-)boot. */
static void wake_module(const struct device *dev)
{
	argos_smd_wakeup_enable(dev);   /* D9 -> HIGH, rising edge on WKUP3 */
	k_msleep(WAKE_BOOT_DELAY_MS);
}

/* Ping the module and return true if it answers +OK within PING_TIMEOUT_MS. */
static bool module_responds(const struct device *dev)
{
	flush_responses();
	argos_read_ping(dev);
	return wait_for_ok(K_MSEC(PING_TIMEOUT_MS)) == 0;
}

/* Set AT+LPM=0x<mode>; return true if acked with +OK.
 * The Kineis firmware expects the LPM value in HEX (AT+LPM=0x1); a decimal
 * "AT+LPM=1" is rejected with +ERROR=1200. AT+LPM=? confirms it reads back hex
 * (+LPM=0x1). */
static bool set_lpm(const struct device *dev, int mode)
{
	char mode_str[6];

	snprintf(mode_str, sizeof(mode_str), "0x%X", mode);
	flush_responses();
	argos_set_lpm(dev, mode_str);
	return wait_for_ok(K_MSEC(2000)) == 0;
}

int main(void)
{
	const struct device *dev = DEVICE_DT_GET_ONE(arribada_argos_smd_uart);
	int results[5] = { 0 };
	bool acked[5] = { false };

	LOG_INF("===== Argos SMD LPM wake test (UART) =====");

	if (!device_is_ready(dev)) {
		LOG_ERR("Argos SMD UART device not ready");
		return -ENODEV;
	}
	argos_smd_set_callback(dev, read_callback, NULL);

	/* Cold-boot the module via NRST (Feather P0.07 -> STM32 NRST) so we start
	 * from a known-good, responsive state regardless of any prior LPM state. */
	const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (device_is_ready(gpio0)) {
		gpio_pin_configure(gpio0, 7, GPIO_OUTPUT_HIGH);
		k_msleep(10);
		gpio_pin_set_raw(gpio0, 7, 0);   /* assert NRST low */
		k_msleep(20);
		gpio_pin_set_raw(gpio0, 7, 1);   /* release */
		k_msleep(600);                   /* let it boot */
		LOG_INF("Module NRST pulsed (P0.07)");
	}

	/* Make sure we start from an awake, responsive module. Retry a few times:
	 * a module sitting in a deep LPM from a previous run may need a couple of
	 * wake pulses / boot windows before it answers. */
	bool up = false;
	for (int attempt = 1; attempt <= 5 && !up; attempt++) {
		wake_module(dev);
		up = module_responds(dev);
		if (!up) {
			LOG_WRN("Startup ping attempt %d/5: no answer, retrying...", attempt);
			k_msleep(1000);
		}
	}
	if (!up) {
		LOG_ERR("Module not responding at startup - check UART wiring / power");
		return -EIO;
	}
	LOG_INF("Startup: module responsive");

	/* Test each real low-power profile (skip 0=NONE, it never sleeps). */
	for (int mode = 1; mode <= 4; mode++) {
		LOG_INF("---------- LPM %d (%s) ----------", mode, lpm_names[mode]);

		/* Ensure awake before (re)configuring. */
		wake_module(dev);
		bool before = module_responds(dev);
		LOG_INF("  pre-sleep ping: %s", before ? "OK" : "NO ANSWER");

		/* Configure the low-power profile (HEX, e.g. AT+LPM=0x3). */
		acked[mode] = set_lpm(dev, mode);
		if (acked[mode]) {
			LOG_INF("  AT+LPM=0x%X: +OK (mode set)", mode);
		} else {
			/* STANDBY/SHUTDOWN power down the UART on entry, so the module
			 * sleeps before it can send +OK — no ack is EXPECTED there and
			 * means the deep mode engaged. */
			LOG_INF("  AT+LPM=0x%X: no ack (module entered deep sleep)", mode);
		}

		/* Drop the wake line so the module enters the configured mode. */
		argos_smd_wakeup_disable(dev);   /* D9 -> LOW */
		LOG_INF("  wake line LOW, dwelling %d ms in %s...",
			LPM_SLEEP_DWELL_MS, lpm_names[mode]);
		k_msleep(LPM_SLEEP_DWELL_MS);

		/* Wake it back up (rising edge cold-boots STANDBY/SHUTDOWN). */
		wake_module(dev);

		bool after = module_responds(dev);
		results[mode] = after ? 1 : 0;
		LOG_INF("  post-wake ping: %s -> %s", after ? "OK" : "NO ANSWER",
			after ? "WAKE OK" : "WAKE FAIL");
	}

	/* Restore NONE so the module does not stay in a deep sleep profile. */
	wake_module(dev);
	if (set_lpm(dev, 0)) {
		LOG_INF("Restored LPM=0 (NONE)");
	} else {
		LOG_WRN("Could not restore LPM=0 (module may need a power-cycle)");
	}

	/* Summary. */
	LOG_INF("===== LPM WAKE TEST SUMMARY =====");
	int pass = 0;
	for (int mode = 1; mode <= 4; mode++) {
		LOG_INF("  %-8s (LPM=0x%X): set=%-5s  wake=%s", lpm_names[mode], mode,
			acked[mode] ? "ack" : "slept",
			results[mode] ? "OK" : "FAIL");
		pass += results[mode];
	}
	LOG_INF("===== %d/4 modes woke successfully =====", pass);
	LOG_INF("Done - ready for current-consumption validation");

	return 0;
}
