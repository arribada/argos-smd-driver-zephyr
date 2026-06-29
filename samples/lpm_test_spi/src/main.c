/*
 * Copyright (c) 2025 Arribada Initiative
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI LPM validation — CLEAN/BOUNDED. Order: WRITE-PATH, STANDBY, STOP, then
 * SHUTDOWN LAST (SHUTDOWN doesn't wake on WKUP3 over SPI -> it bricks the module,
 * so it goes last after the other results are logged). Recovery is bounded (one
 * WKUP3 rising edge or a short grace-window retry, no flooding loops) so the
 * result lines survive in the RTT buffer. Relies on the driver's per-transaction
 * spacing fix (no degradation).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include <argos-smd/argos_smd_spi.h>

LOG_MODULE_REGISTER(lpm_spi, LOG_LEVEL_INF);

#define WAKE_PIN 26
static const struct device *dev, *gpio0;

static void wkup3_low(void)  { gpio_pin_set_raw(gpio0, WAKE_PIN, 0); }
static void wkup3_high(void) { gpio_pin_set_raw(gpio0, WAKE_PIN, 1); }

static bool wait_alive(int max_ms)
{
	int64_t end = k_uptime_get() + max_ms;
	do {
		argos_spi_sync(dev);
		if (argos_spi_ping(dev) == 0) return true;
		k_msleep(120);
	} while (k_uptime_get() < end);
	return false;
}

/* One WKUP3 rising edge + warmup-tolerant wait. Leaves WKUP3 HIGH. */
static bool wkup3_wake(int settle_ms)
{
	wkup3_low();
	k_msleep(800);
	wkup3_high();
	k_msleep(settle_ms);
	return wait_alive(6000);
}

/* Deep mode: forced entry, prove slept, single WKUP3 rising-edge wake. */
static int test_deep(uint8_t bitmap, const char *name, int settle_ms)
{
	LOG_INF("---- %s (0x%02X) ----", name, bitmap);
	wkup3_low();
	k_msleep(100);
	int r = argos_spi_set_lpm_forced(dev, bitmap, bitmap);
	k_msleep(2000);
	bool slept = !wait_alive(1200);
	LOG_INF("  forced ret=%d, slept(SPI-silent)=%s", r, slept ? "yes" : "NO");
	bool wk = wkup3_wake(settle_ms);
	LOG_INF("  WKUP3 wake -> %s", wk ? "OK" : "FAIL");
	if (wk) { argos_spi_set_lpm(dev, 0x00); wkup3_low(); }
	if (!slept) return 1;
	return wk ? 2 : 0;
}

int main(void)
{
	dev = DEVICE_DT_GET_ONE(arribada_argos_smd_spi);
	gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (!device_is_ready(dev) || !device_is_ready(gpio0)) return -ENODEV;
	gpio_pin_configure(gpio0, WAKE_PIN, GPIO_OUTPUT_LOW | GPIO_INPUT);
	k_msleep(2500);

	LOG_INF("===== SPI LPM clean validation =====");
	if (!wait_alive(8000)) {
		LOG_ERR("muet at startup - power-cycle/reflash the module");
		return -EIO;
	}
	LOG_INF("alive");

	int r0 = argos_spi_set_lpm(dev, 0x00);
	bool wpath = (r0 == 0);
	LOG_INF("WRITE-PATH: set_lpm(0x00)=%d -> %s", r0, wpath ? "OK" : "FAIL");

	/* STANDBY (WKUP3-recoverable). */
	int standby = test_deep(0x04, "STANDBY", 2500);
	if (!wait_alive(2000)) wkup3_wake(2500);

	/* STOP (grace-window). */
	LOG_INF("---- STOP (0x02) ----");
	wkup3_low();
	k_msleep(100);
	int rs = argos_spi_set_lpm(dev, 0x02);
	k_msleep(2000);
	bool stop_ok = false;
	for (int i = 0; i < 6 && !stop_ok; i++) {
		argos_spi_set_lpm(dev, 0x00);
		if (argos_spi_set_lpm(dev, 0x00) == 0) stop_ok = wait_alive(1500);
		else k_msleep(50);
	}
	LOG_INF("  set_lpm(0x02)=%d, grace-window recover -> %s", rs, stop_ok ? "OK" : "FAIL");
	if (!wait_alive(2000)) wkup3_wake(2500);

	/* SHUTDOWN LAST (may brick - no WKUP3 wake). Generous settle to be sure. */
	int shutdown = test_deep(0x08, "SHUTDOWN", 4000);

	LOG_INF("===== SUMMARY =====");
	LOG_INF("  WRITE-path:  %s", wpath ? "OK" : "FAIL");
	LOG_INF("  STANDBY:     %s", standby == 2 ? "OK (slept+woke)" : standby == 1 ? "never slept" : "FAIL(no wake)");
	LOG_INF("  STOP:        %s", stop_ok ? "OK (grace-window)" : "FAIL");
	LOG_INF("  SHUTDOWN:    %s", shutdown == 2 ? "OK (slept+woke)" : shutdown == 1 ? "never slept" : "slept, NO WKUP3 wake (NRST only)");
	return 0;
}
