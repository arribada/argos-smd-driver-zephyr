/*
 * Copyright (c) 2025 Arribada Initiative
 * SPDX-License-Identifier: Apache-2.0
 *
 * Kineis SPI DFU Test Application
 *
 * This sample demonstrates firmware update over SPI using Protocol A+.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include <argos-smd/kineis_spi.h>
#include <argos-smd/kineis_dfu_spi.h>

LOG_MODULE_REGISTER(spi_dfu_test, LOG_LEVEL_INF);

/* Get Kineis SPI device from devicetree */
#define KINEIS_SPI_NODE DT_NODELABEL(kineis)

#if !DT_NODE_EXISTS(KINEIS_SPI_NODE)
#error "Kineis SPI device not found in devicetree. Check your overlay."
#endif

static const struct device *kineis_dev = DEVICE_DT_GET(KINEIS_SPI_NODE);

/* External firmware image (if provided via CMake) */
#ifdef USE_EXTERNAL_FIRMWARE
#include "firmware_image.h"
#define TEST_FW_IMAGE    firmware_image
#define TEST_FW_SIZE     FIRMWARE_IMAGE_SIZE
#else
/* Small test pattern for basic testing */
static const uint8_t test_firmware[] = {
	0x00, 0x20, 0x00, 0x08, /* Stack pointer */
	0x41, 0x81, 0x00, 0x08, /* Reset vector */
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	/* ... more test data ... */
};
#define TEST_FW_IMAGE    test_firmware
#define TEST_FW_SIZE     sizeof(test_firmware)
#endif

/* DFU chunk size for testing */
#define TEST_CHUNK_SIZE  KINEIS_DFU_CHUNK_SIZE

/* Progress callback */
static void dfu_progress_callback(uint32_t current, uint32_t total, void *user_data)
{
	static uint32_t last_percent = 0;
	uint32_t percent = (current * 100) / total;

	if (percent != last_percent && (percent % 5 == 0)) {
		LOG_INF("  >> Transfer progress: %u%% (%u/%u bytes)", percent, current, total);
		last_percent = percent;
	}
}

/* Test: Hardware reset */
static int test_hardware_reset(const struct device *dev)
{
	LOG_INF("--- Test: HARDWARE RESET ---");

	int ret = kineis_spi_reset(dev);
	if (ret == 0) {
		LOG_INF("RESULT: Hardware reset completed");
	} else if (ret == -ENOTSUP) {
		LOG_WRN("RESULT: Reset GPIO not configured (skipped)");
		return 0;  /* Not a failure */
	} else {
		LOG_ERR("RESULT: Reset failed: %d", ret);
	}

	return ret;
}

/* Test: SPI diagnostic (raw communication check) */
static int test_spi_diagnostic(const struct device *dev)
{
	LOG_INF("--- Test: SPI DIAGNOSTIC ---");

	int ret = kineis_spi_diagnostic(dev);
	if (ret == 0) {
		LOG_INF("RESULT: Diagnostic completed (check output above)");
	} else {
		LOG_ERR("RESULT: Diagnostic failed: %d", ret);
	}

	return ret;
}

/* Test: Basic SPI communication (ping) */
static int test_spi_ping(const struct device *dev)
{
	LOG_INF("--- Test: SPI PING (app mode) ---");

	int ret = kineis_spi_ping(dev);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Device responding to PING");
	} else {
		LOG_ERR("RESULT: FAIL - PING failed: %d", ret);
	}

	return ret;
}

/* Test: Get version */
static int test_spi_get_version(const struct device *dev)
{
	char version[32];
	size_t version_len = sizeof(version);

	LOG_INF("--- Test: GET VERSION ---");

	int ret = kineis_spi_get_version(dev, version, &version_len);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Version: %s", version);
	} else {
		LOG_ERR("RESULT: FAIL - Get version failed: %d", ret);
	}

	return ret;
}

/* Test: Enter bootloader mode */
static int test_dfu_enter(const struct device *dev)
{
	LOG_INF("--- Test: DFU ENTER ---");

	int ret = kineis_dfu_enter(dev);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - DFU enter command sent");
	} else {
		LOG_ERR("RESULT: FAIL - DFU enter failed: %d", ret);
	}

	return ret;
}

/* Test: DFU Ping (bootloader mode) */
static int test_dfu_ping(const struct device *dev)
{
	LOG_INF("--- Test: DFU PING (bootloader mode) ---");

	int ret = kineis_dfu_ping(dev);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Bootloader responding");
	} else {
		LOG_ERR("RESULT: FAIL - Bootloader not responding: %d", ret);
	}

	return ret;
}

/* Test: Get bootloader info */
static int test_dfu_get_info(const struct device *dev)
{
	struct kineis_bl_info info;

	LOG_INF("--- Test: DFU GET INFO ---");

	int ret = kineis_dfu_get_info(dev, &info);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Bootloader info retrieved");
		LOG_INF("  Version: %u.%u.%u", info.version_major,
			info.version_minor, info.version_patch);
		LOG_INF("  App start: 0x%08X", info.app_start_addr);
		LOG_INF("  Max size: %u bytes", info.app_max_size);
		LOG_INF("  Page size: %u bytes", info.page_size);
	} else {
		LOG_ERR("RESULT: FAIL - Get info failed: %d", ret);
	}

	return ret;
}

/* Test: DFU Erase */
static int test_dfu_erase(const struct device *dev)
{
	LOG_INF("--- Test: DFU ERASE ---");

	int ret = kineis_dfu_erase(dev);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Flash erased successfully");
	} else {
		LOG_ERR("RESULT: FAIL - Erase failed: %d", ret);
	}

	return ret;
}

/* Test: Write a single chunk */
static int test_dfu_write_chunk(const struct device *dev)
{
	uint8_t test_data[64];
	uint32_t addr = KINEIS_FLASH_APPLICATION;

	LOG_INF("--- Test: DFU WRITE CHUNK ---");

	/* Fill with test pattern */
	for (int i = 0; i < sizeof(test_data); i++) {
		test_data[i] = (uint8_t)i;
	}

	int ret = kineis_dfu_write_chunk(dev, addr, test_data, sizeof(test_data));
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Chunk written at 0x%08X", addr);
	} else {
		LOG_ERR("RESULT: FAIL - Write chunk failed: %d", ret);
	}

	return ret;
}

/* Test: DFU Abort */
static int test_dfu_abort(const struct device *dev)
{
	LOG_INF("--- Test: DFU ABORT ---");

	int ret = kineis_dfu_abort(dev);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - DFU aborted");
	} else {
		LOG_ERR("RESULT: FAIL - Abort failed: %d", ret);
	}

	return ret;
}

/* Test: Jump to application */
static int test_dfu_jump(const struct device *dev)
{
	LOG_INF("--- Test: DFU JUMP ---");

	int ret = kineis_dfu_jump(dev);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Jump command sent");
	} else {
		LOG_ERR("RESULT: FAIL - Jump failed: %d", ret);
	}

	return ret;
}

/* Test: Full firmware update */
static int test_full_firmware_update(const struct device *dev)
{
	LOG_INF("--- Test: FULL FIRMWARE UPDATE ---");
	LOG_INF("Firmware size: %u bytes", TEST_FW_SIZE);

	int ret = kineis_firmware_update(dev, TEST_FW_IMAGE, TEST_FW_SIZE,
					 dfu_progress_callback, NULL);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Firmware update successful");
	} else {
		LOG_ERR("RESULT: FAIL - Firmware update failed: %d", ret);
	}

	return ret;
}

int main(void)
{
	int errors = 0;

	LOG_INF("========================================");
	LOG_INF("   Kineis SPI DFU Test Application");
	LOG_INF("========================================");

#ifdef USE_EXTERNAL_FIRMWARE
	LOG_INF("Using EXTERNAL firmware file");
	LOG_INF("Firmware size: %u bytes", TEST_FW_SIZE);
#else
	LOG_INF("Using TEST firmware pattern");
	LOG_INF("Firmware size: %u bytes", TEST_FW_SIZE);
#endif
	LOG_INF("DFU chunk size: %d bytes", TEST_CHUNK_SIZE);
	LOG_INF("");

	/* Check device is ready */
	if (!device_is_ready(kineis_dev)) {
		LOG_ERR("Kineis SPI device not ready!");
		return -1;
	}

	LOG_INF("Kineis SPI device ready");
	LOG_INF("");

	/* ===== PHASE 0: SPI DIAGNOSTIC ===== */
	LOG_INF("======================================");
	LOG_INF("  PHASE 0: SPI BUS DIAGNOSTIC");
	LOG_INF("======================================");
	LOG_INF("");

	/* Try hardware reset first */
	test_hardware_reset(kineis_dev);
	k_msleep(100);

	/* Run SPI diagnostic */
	test_spi_diagnostic(kineis_dev);
	k_msleep(500);

	LOG_INF("");

	/* ===== PHASE 1: BASIC COMMUNICATION TESTS ===== */
	LOG_INF("======================================");
	LOG_INF("  PHASE 1: BASIC COMMUNICATION TESTS");
	LOG_INF("======================================");
	LOG_INF("");

	errors += (test_spi_ping(kineis_dev) != 0) ? 1 : 0;
	k_msleep(500);

	errors += (test_spi_get_version(kineis_dev) != 0) ? 1 : 0;
	k_msleep(500);

	LOG_INF("");

	/* ===== PHASE 2: BOOTLOADER MODE TESTS ===== */
	LOG_INF("======================================");
	LOG_INF("  PHASE 2: BOOTLOADER MODE TESTS");
	LOG_INF("======================================");
	LOG_INF("");

	errors += (test_dfu_enter(kineis_dev) != 0) ? 1 : 0;
	k_msleep(200);

	/* Wait for bootloader to be ready */
	LOG_INF("Waiting for bootloader...");
	int ret = kineis_dfu_wait_ready(kineis_dev, K_SECONDS(5));
	if (ret != 0) {
		LOG_ERR("Bootloader not ready: %d", ret);
		errors++;
	} else {
		errors += (test_dfu_ping(kineis_dev) != 0) ? 1 : 0;
		k_msleep(500);

		errors += (test_dfu_get_info(kineis_dev) != 0) ? 1 : 0;
		k_msleep(500);
	}

	LOG_INF("");

	/* ===== PHASE 3: DFU OPERATIONS TESTS ===== */
	LOG_INF("======================================");
	LOG_INF("  PHASE 3: DFU OPERATIONS TESTS");
	LOG_INF("======================================");
	LOG_INF("");

	/* Make sure we're in bootloader mode */
	kineis_dfu_wait_ready(kineis_dev, K_SECONDS(2));

	errors += (test_dfu_erase(kineis_dev) != 0) ? 1 : 0;
	k_msleep(500);

	errors += (test_dfu_write_chunk(kineis_dev) != 0) ? 1 : 0;
	k_msleep(500);

	errors += (test_dfu_abort(kineis_dev) != 0) ? 1 : 0;
	k_msleep(500);

	LOG_INF("");

	/* ===== PHASE 4: FULL OTA UPDATE (OPTIONAL) ===== */
#ifdef CONFIG_KINEIS_SPI_DFU_TEST_FULL_UPDATE
	LOG_INF("======================================");
	LOG_INF("  PHASE 4: FULL OTA UPDATE TEST");
	LOG_INF("======================================");
	LOG_INF("");
	LOG_WRN("WARNING: This will perform a real firmware update!");
	LOG_INF("");

	/* Re-enter bootloader mode (abort may have exited) */
	kineis_dfu_enter(kineis_dev);
	k_msleep(200);
	kineis_dfu_wait_ready(kineis_dev, K_SECONDS(5));

	errors += (test_full_firmware_update(kineis_dev) != 0) ? 1 : 0;
#else
	LOG_INF("======================================");
	LOG_INF("  PHASE 4: FULL OTA UPDATE - SKIPPED");
	LOG_INF("======================================");
	LOG_INF("(Enable CONFIG_KINEIS_SPI_DFU_TEST_FULL_UPDATE to run)");

	/* Jump back to application */
	test_dfu_jump(kineis_dev);
#endif

	LOG_INF("");

	/* ===== TEST SUMMARY ===== */
	LOG_INF("========================================");
	LOG_INF("         TEST RESULTS SUMMARY");
	LOG_INF("========================================");

	if (errors == 0) {
		LOG_INF("   All tests PASSED!");
	} else {
		LOG_ERR("   %d test(s) FAILED", errors);
	}

	LOG_INF("========================================");

	return errors;
}
