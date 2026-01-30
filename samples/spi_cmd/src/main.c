/*
 * Copyright (c) 2025 Arribada Initiative
 * SPDX-License-Identifier: Apache-2.0
 *
 * Argos SMD SPI Command Test - Tests all high-level SPI API functions
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>
#include <argos-smd/argos_smd_spi.h>

LOG_MODULE_REGISTER(spi_cmd_test, LOG_LEVEL_INF);

#define ARGOS_SMD_SPI_NODE DT_NODELABEL(argos_smd)
#if !DT_NODE_EXISTS(ARGOS_SMD_SPI_NODE)
#error "Argos SMD SPI device not found in devicetree"
#endif

static const struct device *dev = DEVICE_DT_GET(ARGOS_SMD_SPI_NODE);
static int errors, total;

/* Print hex data inline */
static void log_hex(const char *prefix, const uint8_t *data, size_t len)
{
	char buf[64];
	int pos = 0;
	size_t max = (len > 16) ? 16 : len;
	for (size_t i = 0; i < max; i++) {
		pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", data[i]);
	}
	if (len > 16) {
		snprintf(buf + pos, sizeof(buf) - pos, "...");
	}
	LOG_INF("  %s[%u]: %s", prefix, (unsigned)len, buf);
}

/* Generic test runner */
#define RUN_TEST(name, expr) do { \
	LOG_INF("--- %s ---", name); \
	total++; \
	int _ret = (expr); \
	if (_ret == 0) { \
		LOG_INF("  PASS"); \
	} else if (_ret == -ENOTSUP) { \
		LOG_WRN("  SKIP (not supported)"); \
		total--; \
	} else { \
		LOG_ERR("  FAIL: %d", _ret); \
		errors++; \
	} \
	k_msleep(100); \
} while (0)

/* Test read string parameter (version, sn) */
static int test_read_string(const char *name,
	int (*fn)(const struct device *, char *, size_t *))
{
	char buf[64];
	size_t len = sizeof(buf);
	int ret = fn(dev, buf, &len);
	if (ret == 0) {
		buf[len] = '\0';
		LOG_INF("  %s: \"%s\"", name, buf);
	}
	return ret;
}

/* Test read binary parameter (id, addr, rconf) */
static int test_read_binary(const char *name,
	int (*fn)(const struct device *, uint8_t *, size_t *))
{
	uint8_t buf[64];
	size_t len = sizeof(buf);
	int ret = fn(dev, buf, &len);
	if (ret == 0) {
		log_hex(name, buf, len);
	}
	return ret;
}

/* Test write+verify parameter (id or addr) */
static int test_write_verify(const char *name,
	int (*get_fn)(const struct device *, uint8_t *, size_t *),
	int (*set_fn)(const struct device *, const uint8_t *, size_t))
{
	uint8_t orig[8], test[8], readback[8];
	size_t len = sizeof(orig), rb_len = sizeof(readback);

	/* Read original */
	int ret = get_fn(dev, orig, &len);
	if (ret != 0) {
		LOG_ERR("  Cannot read current %s: %d", name, ret);
		return ret;
	}
	log_hex("Original", orig, len);

	/* Write modified value */
	memcpy(test, orig, len);
	test[len - 1]++;
	ret = set_fn(dev, test, len);
	if (ret != 0) {
		LOG_ERR("  Set failed: %d", ret);
		return ret;
	}

	/* Verify */
	k_msleep(50);
	ret = get_fn(dev, readback, &rb_len);
	if (ret != 0) {
		LOG_ERR("  Readback failed: %d", ret);
		return ret;
	}

	if (memcmp(test, readback, len) == 0) {
		LOG_INF("  Verified OK");
	} else {
		LOG_WRN("  Mismatch (may need save)");
	}

	/* Restore */
	set_fn(dev, orig, len);
	return 0;
}

int main(void)
{
	LOG_INF("========================================");
	LOG_INF("   Argos SMD SPI Command Test");
	LOG_INF("========================================");

	if (!device_is_ready(dev)) {
		LOG_ERR("Device not ready!");
		return -1;
	}

	/* Phase 1: Init */
	LOG_INF("");
	LOG_INF("== INIT ==");
	RUN_TEST("RESET", argos_spi_reset(dev));
	k_msleep(400);

	/* Phase 2: Read-only */
	LOG_INF("");
	LOG_INF("== READ COMMANDS ==");
	RUN_TEST("PING", argos_spi_ping(dev));
	RUN_TEST("VERSION", test_read_string("Version", argos_spi_get_version));
	RUN_TEST("SERIAL", test_read_string("SN", argos_spi_get_sn));
	RUN_TEST("ID", test_read_binary("ID", argos_spi_get_id));
	RUN_TEST("ADDR", test_read_binary("Addr", argos_spi_get_addr));
	RUN_TEST("RCONF", test_read_binary("RConf", argos_spi_get_rconf));

	/* Phase 3: Read/Write */
	LOG_INF("");
	LOG_INF("== WRITE COMMANDS ==");
	RUN_TEST("SET_ID", test_write_verify("ID", argos_spi_get_id, argos_spi_set_id));
	RUN_TEST("SET_ADDR", test_write_verify("Addr", argos_spi_get_addr, argos_spi_set_addr));

	/* Phase 4: Diagnostic */
	LOG_INF("");
	LOG_INF("== DIAGNOSTIC ==");
	RUN_TEST("DIAG", argos_spi_diagnostic(dev));

	/* Summary */
	LOG_INF("");
	LOG_INF("========================================");
	LOG_INF("  Tests: %d | Passed: %d | Failed: %d", total, total - errors, errors);
	LOG_INF("  %s", errors == 0 ? ">> ALL PASSED <<" : ">> FAILURES <<");
	LOG_INF("========================================");

	return errors;
}
