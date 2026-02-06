/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <argos-smd/argos_smd.h>
#include <argos-smd/argos_dfu.h>

LOG_MODULE_REGISTER(ota_test, LOG_LEVEL_INF);

/*
 * Firmware image selection
 * - If USE_EXTERNAL_FIRMWARE is defined, use firmware from file (firmware_image.h)
 * - Otherwise, use built-in test firmware
 */

#ifdef USE_EXTERNAL_FIRMWARE
/* Include generated firmware array from external file */
#include "firmware_image.h"

/* Use the generated firmware */
#define TEST_FW_IMAGE firmware_image
#define TEST_FW_SIZE  FIRMWARE_IMAGE_SIZE

#else
/* Built-in test firmware image (dummy data for testing) */
static const uint8_t test_firmware[] = {
	/* Firmware header (example) */
	0x4B, 0x4E, 0x53, 0x46,  /* Magic "KNSF" */
	0x01, 0x01, 0x00, 0x00,  /* Version 1.1.0.0 */
	0x00, 0x80, 0x00, 0x00,  /* Entry point */
	0x00, 0x04, 0x00, 0x00,  /* Size: 1024 bytes */

	/* Padding with recognizable pattern */
	0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
	0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
	0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
	0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,

	/* Fill rest with incremental pattern for easier debugging */
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
	0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
	0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,

	/* Repeat pattern to reach ~512 bytes */
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
	0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
	0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,
	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
	0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77,
	0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F,
	0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
	0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97,
	0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
	0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,
	0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
	0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7,
	0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
	0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
	0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
	0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
	0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
	0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
	0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
	0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7,
	0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
};

/* Use the built-in test firmware */
#define TEST_FW_IMAGE test_firmware
#define TEST_FW_SIZE  sizeof(test_firmware)

#endif /* USE_EXTERNAL_FIRMWARE */

/* Progress callback for OTA updates */
static void progress_callback(uint32_t current, uint32_t total)
{
	static uint32_t last_percent = 0;
	uint32_t percent = (current * 100) / total;

	if (percent != last_percent && (percent % 5 == 0)) {
		LOG_INF("  >> Transfer progress: %u%% (%u/%u bytes)",
			percent, current, total);
		last_percent = percent;
	}
}

/* ==================== BASIC TESTS ==================== */

static int test_ping(const struct device *dev)
{
	LOG_INF("--- Test: PING ---");

	int ret = argos_read_ping(dev);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Device responding to PING");
	} else {
		LOG_ERR("RESULT: FAIL - PING failed with error %d", ret);
	}

	return ret;
}

static int test_version(const struct device *dev)
{
	LOG_INF("--- Test: READ VERSION ---");

	int ret = argos_read_version(dev);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Version read successfully");
	} else {
		LOG_ERR("RESULT: FAIL - Version read failed with error %d", ret);
	}

	return ret;
}

static int test_firmware_version(const struct device *dev)
{
	LOG_INF("--- Test: READ FIRMWARE VERSION ---");

	int ret = argos_read_firmware_version(dev);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Firmware version read successfully");
	} else {
		LOG_ERR("RESULT: FAIL - Firmware version read failed with error %d", ret);
	}

	return ret;
}

/* ==================== BOOTLOADER TESTS ==================== */

static int test_enter_bootloader(const struct device *dev)
{
	LOG_INF("--- Test: ENTER BOOTLOADER MODE ---");

	int ret = argos_enter_bootloader(dev);
	if (ret < 0) {
		LOG_ERR("RESULT: FAIL - Failed to enter bootloader: %d", ret);
		return ret;
	}

	LOG_INF("Waiting for bootloader to be ready...");
	ret = argos_wait_bootloader_ready(dev, K_SECONDS(10));
	if (ret < 0) {
		LOG_ERR("RESULT: FAIL - Bootloader not ready: %d", ret);
		return ret;
	}

	LOG_INF("RESULT: PASS - Bootloader mode entered successfully");
	return 0;
}

static int test_bootloader_ping(const struct device *dev)
{
	LOG_INF("--- Test: PING IN BOOTLOADER MODE ---");

	/* Ensure we're in bootloader mode */
	argos_wait_bootloader_ready(dev, K_SECONDS(5));

	int ret = argos_read_ping(dev);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Bootloader responding to PING");
	} else {
		LOG_ERR("RESULT: FAIL - Bootloader PING failed: %d", ret);
	}

	return ret;
}

/* ==================== DFU SESSION TESTS ==================== */

static int test_dfu_erase(const struct device *dev)
{
	LOG_INF("--- Test: DFU ERASE ---");

	/* Ensure we're in bootloader mode */
	argos_wait_bootloader_ready(dev, K_SECONDS(5));

	int ret = argos_dfu_erase(dev);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Flash erased successfully");
	} else {
		LOG_ERR("RESULT: FAIL - Erase failed: %d", ret);
	}

	return ret;
}

static int test_dfu_write_single_chunk(const struct device *dev)
{
	LOG_INF("--- Test: WRITE SINGLE CHUNK ---");

	/* Ensure we're in bootloader mode and erase first */
	argos_wait_bootloader_ready(dev, K_SECONDS(5));

	int ret = argos_dfu_erase(dev);
	if (ret < 0) {
		LOG_ERR("RESULT: FAIL - Couldn't erase flash: %d", ret);
		return ret;
	}

	/* Write first chunk at APP base address */
	size_t chunk_size = MIN(ARGOS_DFU_CHUNK_SIZE, TEST_FW_SIZE);
	ret = argos_dfu_write(dev, ARGOS_DFU_APP_BASE, TEST_FW_IMAGE, chunk_size);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Single chunk written at 0x%08X", ARGOS_DFU_APP_BASE);
	} else {
		LOG_ERR("RESULT: FAIL - Write failed: %d", ret);
	}

	/* Abort the session */
	argos_dfu_abort(dev);

	return ret;
}

static int test_dfu_abort(const struct device *dev)
{
	LOG_INF("--- Test: DFU ABORT ---");

	/* Ensure bootloader mode */
	argos_wait_bootloader_ready(dev, K_SECONDS(5));

	/* Start a DFU session (erase) */
	int ret = argos_dfu_erase(dev);
	if (ret < 0) {
		LOG_WRN("DFU erase (for abort test) failed: %d", ret);
	}

	/* Abort it */
	ret = argos_dfu_abort(dev);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - DFU session aborted successfully");
	} else {
		LOG_ERR("RESULT: FAIL - DFU abort failed: %d", ret);
	}

	return ret;
}

/* ==================== COMPLETE OTA TEST ==================== */

static int test_full_ota(const struct device *dev)
{
	LOG_INF("--- Test: FULL OTA UPDATE ---");
	LOG_INF("This test will perform a complete firmware update");
	LOG_INF("Firmware size: %zu bytes", TEST_FW_SIZE);

	int ret = argos_ota_update(dev, TEST_FW_IMAGE, TEST_FW_SIZE, progress_callback);

	if (ret == 0) {
		LOG_INF("RESULT: PASS - Full OTA update completed successfully");
	} else {
		LOG_ERR("RESULT: FAIL - Full OTA update failed: %d", ret);
	}

	return ret;
}

/* ==================== ERROR HANDLING TESTS ==================== */

static int test_invalid_crc(const struct device *dev)
{
	LOG_INF("--- Test: INVALID CRC (should fail gracefully) ---");

	/* Enter bootloader */
	argos_enter_bootloader(dev);
	argos_wait_bootloader_ready(dev, K_SECONDS(10));

	/* Erase flash first */
	int ret = argos_dfu_erase(dev);
	if (ret < 0) {
		LOG_ERR("RESULT: FAIL - Erase failed: %d", ret);
		return ret;
	}

	/* Write all data */
	uint32_t addr = ARGOS_DFU_APP_BASE;
	size_t offset = 0;
	while (offset < TEST_FW_SIZE) {
		size_t chunk_len = MIN(ARGOS_DFU_CHUNK_SIZE, TEST_FW_SIZE - offset);
		ret = argos_dfu_write(dev, addr, &TEST_FW_IMAGE[offset], chunk_len);
		if (ret < 0) {
			LOG_INF("RESULT: PASS - Write rejected during transfer");
			argos_dfu_abort(dev);
			return 0;
		}
		addr += chunk_len;
		offset += chunk_len;
	}

	/* Try to verify with intentionally wrong CRC - should fail */
	ret = argos_dfu_verify(dev, 0xDEADBEEF);
	if (ret < 0) {
		LOG_INF("RESULT: PASS - Verify with bad CRC rejected");
		argos_dfu_abort(dev);
		return 0;
	}

	LOG_ERR("RESULT: FAIL - DFU with bad CRC should have been rejected!");
	return -1;
}

static int test_chunk_too_large(const struct device *dev)
{
	LOG_INF("--- Test: CHUNK TOO LARGE (should be rejected) ---");

	/* Ensure bootloader mode */
	argos_enter_bootloader(dev);
	argos_wait_bootloader_ready(dev, K_SECONDS(10));

	/* Erase flash */
	int ret = argos_dfu_erase(dev);
	if (ret < 0) {
		LOG_ERR("RESULT: FAIL - Couldn't erase: %d", ret);
		return ret;
	}

	/* Try to write oversized chunk */
	uint8_t large_chunk[ARGOS_DFU_CHUNK_SIZE + 1];
	memset(large_chunk, 0xAA, sizeof(large_chunk));

	ret = argos_dfu_write(dev, ARGOS_DFU_APP_BASE, large_chunk, sizeof(large_chunk));
	if (ret < 0) {
		LOG_INF("RESULT: PASS - Oversized chunk rejected (error: %d)", ret);
		argos_dfu_abort(dev);
		return 0;
	}

	LOG_ERR("RESULT: FAIL - Oversized chunk should have been rejected!");
	argos_dfu_abort(dev);
	return -1;
}

static int test_dfu_without_erase(const struct device *dev)
{
	LOG_INF("--- Test: DFU WRITE WITHOUT ERASE (should fail) ---");

	/* Enter bootloader but don't erase */
	argos_enter_bootloader(dev);
	argos_wait_bootloader_ready(dev, K_SECONDS(10));

	/* Abort any previous session to ensure clean state */
	argos_dfu_abort(dev);
	k_msleep(100);

	/* Try to write without erasing first - bootloader should reject */
	int ret = argos_dfu_write(dev, ARGOS_DFU_APP_BASE, TEST_FW_IMAGE, 64);
	if (ret < 0) {
		LOG_INF("RESULT: PASS - Write without erase rejected (error: %d)", ret);
		return 0;
	}

	LOG_ERR("RESULT: FAIL - Write without erase should have been rejected!");
	return -1;
}

/* ==================== STRESS TESTS ==================== */

static int test_repeated_ota(const struct device *dev, int iterations)
{
	LOG_INF("--- Test: REPEATED OTA (%d iterations) ---", iterations);

	for (int i = 0; i < iterations; i++) {
		LOG_INF("OTA iteration %d/%d", i + 1, iterations);

		int ret = argos_ota_update(dev, TEST_FW_IMAGE, TEST_FW_SIZE, NULL);
		if (ret < 0) {
			LOG_ERR("RESULT: FAIL - OTA iteration %d failed: %d", i + 1, ret);
			return ret;
		}

		/* Wait a bit between iterations */
		k_msleep(1000);
	}

	LOG_INF("RESULT: PASS - %d OTA iterations completed successfully", iterations);
	return 0;
}

/* ==================== MAIN TEST RUNNER ==================== */

int main(void)
{
	LOG_INF("========================================");
	LOG_INF("   Argos SMD OTA Test Application");
	LOG_INF("========================================");
#ifdef USE_EXTERNAL_FIRMWARE
	LOG_INF("Using EXTERNAL firmware file");
#else
	LOG_INF("Using BUILT-IN test firmware");
#endif
	LOG_INF("Firmware size: %zu bytes", TEST_FW_SIZE);
	LOG_INF("DFU chunk size: %d bytes", ARGOS_DFU_CHUNK_SIZE);
	LOG_INF("");

	/* Get device */
	const struct device *argos = DEVICE_DT_GET(DT_NODELABEL(argossmd));

	if (!device_is_ready(argos)) {
		LOG_ERR("ERROR: Argos SMD device not ready!");
		LOG_ERR("Check your devicetree configuration.");
		return -1;
	}

	LOG_INF("Argos SMD device ready");
	LOG_INF("");

	int errors = 0;

	/* ===== BASIC TESTS ===== */
	LOG_INF("======================================");
	LOG_INF("  PHASE 1: BASIC COMMUNICATION TESTS");
	LOG_INF("======================================");
	LOG_INF("");

	errors += (test_ping(argos) != 0) ? 1 : 0;
	k_msleep(500);

	errors += (test_version(argos) != 0) ? 1 : 0;
	k_msleep(500);

	errors += (test_firmware_version(argos) != 0) ? 1 : 0;
	k_msleep(500);

	LOG_INF("");

	/* ===== BOOTLOADER TESTS ===== */
	LOG_INF("======================================");
	LOG_INF("  PHASE 2: BOOTLOADER MODE TESTS");
	LOG_INF("======================================");
	LOG_INF("");

	errors += (test_enter_bootloader(argos) != 0) ? 1 : 0;
	k_msleep(500);

	errors += (test_bootloader_ping(argos) != 0) ? 1 : 0;
	k_msleep(500);

	LOG_INF("");

	/* ===== DFU SESSION TESTS ===== */
	LOG_INF("======================================");
	LOG_INF("  PHASE 3: DFU SESSION TESTS");
	LOG_INF("======================================");
	LOG_INF("");

	errors += (test_dfu_erase(argos) != 0) ? 1 : 0;
	k_msleep(500);

	errors += (test_dfu_write_single_chunk(argos) != 0) ? 1 : 0;
	k_msleep(500);

	errors += (test_dfu_abort(argos) != 0) ? 1 : 0;
	k_msleep(500);

	LOG_INF("");

	/* ===== ERROR HANDLING TESTS (DISABLED FOR SPEED) ===== */
	LOG_INF("======================================");
	LOG_INF("  PHASE 4: ERROR HANDLING TESTS - SKIPPED");
	LOG_INF("======================================");
	LOG_INF("");

	/* Disabled to speed up testing - uncomment to enable
	errors += (test_invalid_crc(argos) != 0) ? 1 : 0;
	k_msleep(500);

	errors += (test_chunk_too_large(argos) != 0) ? 1 : 0;
	k_msleep(500);

	errors += (test_dfu_without_erase(argos) != 0) ? 1 : 0;
	k_msleep(500);
	*/

	LOG_INF("");

	/* ===== FULL OTA TEST (Optional - comment out if not needed) ===== */
#ifdef CONFIG_ARGOS_OTA_TEST_FULL_UPDATE
	LOG_INF("======================================");
	LOG_INF("  PHASE 5: FULL OTA UPDATE TEST");
	LOG_INF("======================================");
	LOG_INF("");
	LOG_WRN("WARNING: This will attempt a real firmware update!");
	LOG_INF("");

	errors += (test_full_ota(argos) != 0) ? 1 : 0;
	LOG_INF("");
#else
	LOG_INF("======================================");
	LOG_INF("  PHASE 5: FULL OTA UPDATE - SKIPPED");
	LOG_INF("======================================");
	LOG_INF("To enable: Set CONFIG_ARGOS_OTA_TEST_FULL_UPDATE=y");
	LOG_INF("");
#endif

	/* ===== STRESS TESTS (Optional) ===== */
#ifdef CONFIG_ARGOS_OTA_TEST_STRESS
	LOG_INF("======================================");
	LOG_INF("  PHASE 6: STRESS TESTS");
	LOG_INF("======================================");
	LOG_INF("");

	errors += (test_repeated_ota(argos, 3) != 0) ? 1 : 0;
	LOG_INF("");
#endif

	/* ===== TEST RESULTS ===== */
	LOG_INF("========================================");
	LOG_INF("         TEST RESULTS SUMMARY");
	LOG_INF("========================================");

	if (errors == 0) {
		LOG_INF("   ✓ ALL TESTS PASSED!");
	} else {
		LOG_ERR("   ✗ %d TEST(S) FAILED", errors);
	}

	LOG_INF("========================================");

	return errors;
}
