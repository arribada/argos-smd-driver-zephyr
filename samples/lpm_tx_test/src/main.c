/*
 * Copyright (c) 2025 Arribada Initiative
 * SPDX-License-Identifier: Apache-2.0
 *
 * Does a TX still work after waking from STANDBY/SHUTDOWN WITHOUT re-sending the
 * KMAC? Waking those modes is a cold boot (RAM lost). If the KMAC / radio config
 * lives in NVM it survives and AT+TX returns +TX=0; if it was only in RAM the TX
 * fails and the host must re-send AT+KMAC. Flow: baseline TX while awake, then
 * for STANDBY then SHUTDOWN: force the mode, wake (cold boot), AT+TX (no KMAC).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <argos-smd/argos_smd.h>

LOG_MODULE_REGISTER(lpm_tx, LOG_LEVEL_INF);

#define RESP_SZ 128
K_MSGQ_DEFINE(rq, RESP_SZ, 16, 1);

static const uint8_t lpm_bits[] = { 0x00, 0x01, 0x02, 0x04, 0x08 };
static const char *const names[] = { "NONE", "SLEEP", "STOP", "STANDBY", "SHUTDOWN" };

static void cb(const char *r, void *u)
{
	ARG_UNUSED(u);
	char o[RESP_SZ];
	strncpy(o, r, RESP_SZ - 1);
	o[RESP_SZ - 1] = '\0';
	k_msgq_put(&rq, o, K_NO_WAIT);
}

static void flush(void)
{
	char o[RESP_SZ];
	while (k_msgq_get(&rq, o, K_NO_WAIT) == 0) {
	}
}

static int wait_ok(k_timeout_t t)
{
	char o[RESP_SZ];
	while (1) {
		if (k_msgq_get(&rq, o, t) != 0) {
			return -EAGAIN;
		}
		if (strcmp(o, "+OK") == 0) {
			return 0;
		}
		if (strncmp(o, "+ERROR", 6) == 0) {
			return -EIO;
		}
	}
}

static void wake(const struct device *d)
{
	argos_smd_wakeup_enable(d);
	k_msleep(700);
}

static bool responds(const struct device *d)
{
	flush();
	argos_read_ping(d);
	return wait_ok(K_MSEC(3000)) == 0;
}

static bool setlpm(const struct device *d, int m)
{
	char s[12];
	snprintf(s, sizeof(s), "0x%X,0x%X", lpm_bits[m], lpm_bits[m]);
	flush();
	argos_set_lpm(d, s);
	return wait_ok(K_MSEC(2000)) == 0;
}

/* Send a TX and report what comes back (collect for ~12s: +OK then +TX=<status>,
 * or +ERROR). +TX=0 means success. */
static void do_tx(const struct device *d, const char *tag)
{
	char o[RESP_SZ];
	bool accepted = false;
	char result[RESP_SZ] = "(no +TX)";

	flush();
	argos_send_payload(d, "FFFFFFFF");   /* AT+TX=FFFFFFFF */

	int64_t end = k_uptime_get() + 12000;
	while (k_uptime_get() < end) {
		int rem = (int)(end - k_uptime_get());
		if (rem <= 0 || k_msgq_get(&rq, o, K_MSEC(rem)) != 0) {
			break;
		}
		size_t L = strlen(o);
		if (L > 0 && o[L - 1] == '?') {
			continue;                /* echo like +TX=? */
		}
		if (strcmp(o, "+OK") == 0) {
			accepted = true;
			continue;
		}
		if (strncmp(o, "+ERROR", 6) == 0) {
			strncpy(result, o, RESP_SZ - 1);
			break;
		}
		if (strncmp(o, "+TX=", 4) == 0) {
			strncpy(result, o, RESP_SZ - 1);
			break;
		}
	}
	LOG_INF("  [%s] AT+TX accepted=%s  result=[%s]", tag,
		accepted ? "+OK" : "NO/ERROR", result);
}

int main(void)
{
	const struct device *dev = DEVICE_DT_GET_ONE(arribada_argos_smd_uart);

	LOG_INF("===== LPM wake -> TX without re-sending KMAC =====");
	if (!device_is_ready(dev)) {
		LOG_ERR("device not ready");
		return -ENODEV;
	}
	argos_smd_set_callback(dev, cb, NULL);

	/* NRST reset for a clean, responsive start. */
	const struct device *g = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (device_is_ready(g)) {
		gpio_pin_configure(g, 7, GPIO_OUTPUT_HIGH);
		k_msleep(10);
		gpio_pin_set_raw(g, 7, 0);
		k_msleep(20);
		gpio_pin_set_raw(g, 7, 1);
		k_msleep(600);
	}

	wake(dev);
	if (!responds(dev)) {
		LOG_ERR("module not responding - aborting");
		return -EIO;
	}
	LOG_INF("Module responsive");

	/* Baseline TX (awake) — confirms the module has a working config/KMAC. */
	do_tx(dev, "BASELINE awake");

	/* STANDBY (0x04) then SHUTDOWN (0x08): force, wake (cold boot), TX. */
	int modes[] = { 3, 4 };
	for (int i = 0; i < 2; i++) {
		int m = modes[i];
		LOG_INF("---------- %s ----------", names[m]);

		wake(dev);
		responds(dev);
		argos_smd_wakeup_disable(dev);
		k_msleep(100);
		bool ack = setlpm(dev, m);
		LOG_INF("  forced %s: %s", names[m], ack ? "+OK" : "no ack (deep sleep)");

		k_msleep(3000);                 /* dwell in deep sleep */

		wake(dev);                      /* rising edge -> cold boot */
		bool up = responds(dev);
		LOG_INF("  post-wake responsive: %s", up ? "yes" : "NO");
		if (up) {
			do_tx(dev, names[m]);   /* TX WITHOUT re-sending KMAC */
		}
	}

	wake(dev);
	setlpm(dev, 0);
	LOG_INF("===== test done =====");
	return 0;
}
