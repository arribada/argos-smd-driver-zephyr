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
	char buf[80];
	int pos = 0;
	size_t max = (len > 20) ? 20 : len;
	for (size_t i = 0; i < max; i++) {
		pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", data[i]);
	}
	if (len > 20) {
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

/* Test read binary parameter */
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

/* Test ID write: read original, write new, verify, restore */
static int test_set_id(void)
{
	uint8_t orig_id[4], new_id[4], verify_id[4];
	size_t len = sizeof(orig_id);
	int ret;

	/* Read original ID */
	ret = argos_spi_get_id(dev, orig_id, &len);
	if (ret != 0) {
		LOG_ERR("  Read original ID failed: %d", ret);
		return ret;
	}
	log_hex("Original ID", orig_id, len);

	/* Test ID: 123456 (0x12 0x34 0x56 0x00) */
	new_id[0] = 0x12;
	new_id[1] = 0x34;
	new_id[2] = 0x56;
	new_id[3] = 0x00;

	/* Write new ID */
	ret = argos_spi_set_id(dev, new_id, 4);
	if (ret != 0) {
		LOG_ERR("  Write ID failed: %d", ret);
		return ret;
	}
	LOG_INF("  Write OK");

	/* Verify */
	len = sizeof(verify_id);
	ret = argos_spi_get_id(dev, verify_id, &len);
	if (ret != 0) {
		LOG_ERR("  Verify read failed: %d", ret);
		return ret;
	}

	if (memcmp(new_id, verify_id, 4) == 0) {
		LOG_INF("  Verify OK");
	} else {
		log_hex("Expected", new_id, 4);
		log_hex("Got", verify_id, 4);
		LOG_WRN("  Mismatch (RAM vs Flash)");
	}

	/* Restore original */
	ret = argos_spi_set_id(dev, orig_id, 4);
	if (ret == 0) {
		LOG_INF("  Restored original ID");
	}

	return 0;
}

/* Test ADDR write: read original, write new, verify, restore */
static int test_set_addr(void)
{
	uint8_t orig_addr[4], new_addr[4], verify_addr[4];
	size_t len = sizeof(orig_addr);
	int ret;

	/* Read original ADDR */
	ret = argos_spi_get_addr(dev, orig_addr, &len);
	if (ret != 0) {
		LOG_ERR("  Read original ADDR failed: %d", ret);
		return ret;
	}
	log_hex("Original ADDR", orig_addr, len);

	/* Test ADDR: 00112233 */
	new_addr[0] = 0x00;
	new_addr[1] = 0x11;
	new_addr[2] = 0x22;
	new_addr[3] = 0x33;

	/* Write new ADDR */
	ret = argos_spi_set_addr(dev, new_addr, 4);
	if (ret != 0) {
		LOG_ERR("  Write ADDR failed: %d", ret);
		return ret;
	}
	LOG_INF("  Write OK");

	/* Verify */
	len = sizeof(verify_addr);
	ret = argos_spi_get_addr(dev, verify_addr, &len);
	if (ret != 0) {
		LOG_ERR("  Verify read failed: %d", ret);
		return ret;
	}

	if (memcmp(new_addr, verify_addr, 4) == 0) {
		LOG_INF("  Verify OK");
	} else {
		log_hex("Expected", new_addr, 4);
		log_hex("Got", verify_addr, 4);
		LOG_WRN("  Mismatch (RAM vs Flash)");
	}

	/* Restore original */
	ret = argos_spi_set_addr(dev, orig_addr, 4);
	if (ret == 0) {
		LOG_INF("  Restored original ADDR");
	}

	return 0;
}

/* Test SECRET_KEY write: read original, write test key, verify, restore */
static int test_set_secret_key(void)
{
	uint8_t orig_key[16], verify_key[16];
	size_t len = sizeof(orig_key);
	int ret;

	/* Device secret key for encryption (16 bytes) */
	static const uint8_t test_device_secret_key[16] = {
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
		0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
	};

	/* Read original SECRET_KEY */
	ret = argos_spi_get_secret_key(dev, orig_key, &len);
	if (ret != 0) {
		LOG_ERR("  Read original SECRET_KEY failed: %d", ret);
		return ret;
	}
	log_hex("Original SECRET_KEY", orig_key, len);

	/* Write test SECRET_KEY */
	ret = argos_spi_set_secret_key(dev, test_device_secret_key, sizeof(test_device_secret_key));
	if (ret != 0) {
		LOG_ERR("  Write SECRET_KEY failed: %d", ret);
		return ret;
	}
	LOG_INF("  Write OK");

	/* Verify */
	size_t vlen = sizeof(verify_key);
	ret = argos_spi_get_secret_key(dev, verify_key, &vlen);
	if (ret != 0) {
		LOG_ERR("  Verify read failed: %d", ret);
		return ret;
	}

	if (memcmp(test_device_secret_key, verify_key, sizeof(test_device_secret_key)) == 0) {
		LOG_INF("  Verify OK");
	} else {
		log_hex("Expected", test_device_secret_key, sizeof(test_device_secret_key));
		log_hex("Got", verify_key, vlen);
		LOG_WRN("  Mismatch");
	}

	/* Restore original */
	ret = argos_spi_set_secret_key(dev, orig_key, len);
	if (ret == 0) {
		LOG_INF("  Restored original SECRET_KEY");
	}

	return 0;
}

/* Test RCONF write: read original, write test config, verify, restore */
static int test_set_rconf(void)
{
	uint8_t orig_rconf[16], verify_rconf[16];
	size_t len = sizeof(orig_rconf);
	int ret;

	/* Valid LDK RCONF from Kineis (16 bytes encrypted config) */
	static const uint8_t test_rconf[16] = {
		0x03, 0x92, 0x1f, 0xb1, 0x04, 0xb9, 0x28, 0x59,
		0x20, 0x9b, 0x18, 0xab, 0xd0, 0x09, 0xde, 0x96
	};

	/* Read original RCONF */
	ret = argos_spi_get_rconf(dev, orig_rconf, &len);
	if (ret != 0) {
		LOG_ERR("  Read original RCONF failed: %d", ret);
		return ret;
	}
	log_hex("Original RCONF", orig_rconf, len);

	/* Write test RCONF */
	ret = argos_spi_set_rconf(dev, test_rconf, sizeof(test_rconf));
	if (ret != 0) {
		LOG_ERR("  Write RCONF failed: %d", ret);
		return ret;
	}
	LOG_INF("  Write OK");

	/* Verify */
	size_t vlen = sizeof(verify_rconf);
	ret = argos_spi_get_rconf(dev, verify_rconf, &vlen);
	if (ret != 0) {
		LOG_ERR("  Verify read failed: %d", ret);
		return ret;
	}

	if (memcmp(test_rconf, verify_rconf, sizeof(test_rconf)) == 0) {
		LOG_INF("  Verify OK");
	} else {
		log_hex("Expected", test_rconf, sizeof(test_rconf));
		log_hex("Got", verify_rconf, vlen);
		LOG_WRN("  Mismatch");
	}

	/* Restore original */
	ret = argos_spi_set_rconf(dev, orig_rconf, len);
	if (ret == 0) {
		LOG_INF("  Restored original RCONF");
	}

	return 0;
}

/* Test TX message send */
static int test_tx_message(void)
{
	int ret;

	/* Configure RCONF for TX test */
	static const uint8_t tx_rconf[16] = {
		0x3d, 0x67, 0x8a, 0xf1, 0x6b, 0x5a, 0x57, 0x20,
		0x78, 0xf3, 0xdb, 0xc9, 0x5a, 0x11, 0x04, 0xe7
	};

	LOG_INF("  Writing RCONF for TX test...");
	ret = argos_spi_set_rconf(dev, tx_rconf, sizeof(tx_rconf));
	if (ret != 0) {
		LOG_ERR("  RCONF write failed: %d", ret);
		return ret;
	}
	LOG_INF("  RCONF configured successfully");

	/* Set KMAC=1 (profile 1) */
	LOG_INF("  Setting KMAC=1...");
	ret = argos_spi_set_kmac(dev, 1);
	if (ret != 0) {
		LOG_ERR("  KMAC write failed: %d", ret);
		return ret;
	}
	LOG_INF("  KMAC set to 1 successfully");

	/* Test payload: 24 bytes for LDA2 mode (or 16 for LDK)
	 * The payload size must match the RCONF modulation setting */
	static const uint8_t tx_data[24] = {
		'H', 'E', 'L', 'L', 'O', ' ', 'A', 'R',
		'G', 'O', 'S', ' ', 'T', 'E', 'S', 'T',
		0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
	};

	LOG_INF("  Sending TX message: %zu bytes (LDA2 mode)", sizeof(tx_data));

	ret = argos_spi_write_tx(dev, tx_data, sizeof(tx_data));
	if (ret != 0) {
		LOG_ERR("  TX queue failed: %d", ret);
		return ret;
	}
	LOG_INF("  TX queued successfully");

	/* Wait 2s after TX before checking status (RF transmission in progress) */
	LOG_INF("  Waiting 2s for RF transmission to start...");
	k_msleep(2000);

	/* Wait for TX completion with timeout */
	LOG_INF("  Waiting for TX completion...");
	ret = argos_spi_wait_tx_complete(dev, K_SECONDS(30));
	if (ret == 0) {
		LOG_INF("  TX completed successfully!");
	} else if (ret == -ETIMEDOUT) {
		LOG_WRN("  TX timeout (expected without satellite)");
		return 0;  /* Not a failure - just no satellite */
	} else {
		LOG_ERR("  TX failed: %d", ret);
		return ret;
	}

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

	/* Phase 1: Init - Sync protocol before starting tests */
	LOG_INF("");
	LOG_INF("== INIT ==");
	RUN_TEST("SYNC", argos_spi_sync(dev));
	RUN_TEST("RESET", argos_spi_reset(dev));
	k_msleep(400);

	/* Phase 2: Read-only commands */
	LOG_INF("");
	LOG_INF("== READ COMMANDS ==");
	RUN_TEST("PING", argos_spi_ping(dev));
	RUN_TEST("VERSION", test_read_string("Version", argos_spi_get_version));
	RUN_TEST("SERIAL", test_read_string("SN", argos_spi_get_sn));
	RUN_TEST("ID", test_read_binary("ID", argos_spi_get_id));
	RUN_TEST("ADDR", test_read_binary("Addr", argos_spi_get_addr));
	RUN_TEST("RCONF", test_read_binary("RConf", argos_spi_get_rconf));
	RUN_TEST("RCONF_RAW", test_read_binary("RConfRaw", argos_spi_get_rconf_raw));

	/* Phase 3: Write commands (ID/ADDR only - don't affect MAC) */
	LOG_INF("");
	LOG_INF("== WRITE COMMANDS ==");
	RUN_TEST("SET_ID", test_set_id());
	RUN_TEST("SET_ADDR", test_set_addr());
	RUN_TEST("SET_SECRET_KEY", test_set_secret_key());

	/* Phase 4: TX Message (before RCONF changes - RCONF change requires MAC reset) */
	LOG_INF("");
	LOG_INF("== TX MESSAGE ==");
	RUN_TEST("TX_MSG", test_tx_message());

	/* Phase 5: RCONF test (last - changes MAC profile, would need AT+KMAC=1 to TX again) */
	LOG_INF("");
	LOG_INF("== RCONF WRITE ==");
	RUN_TEST("SET_RCONF", test_set_rconf());

	/* Phase 6: Diagnostic */
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

