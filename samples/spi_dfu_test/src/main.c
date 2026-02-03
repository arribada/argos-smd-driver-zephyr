/*
 * Copyright (c) 2025 Arribada Initiative
 * SPDX-License-Identifier: Apache-2.0
 *
 * Argos SMD SPI DFU Test Application
 *
 * This sample demonstrates firmware update over SPI using Protocol A+.
 *
 * Protocol A+ Frame Format:
 *   Request:  [0xAA][SEQ][CMD][LEN][DATA...][CRC8]
 *   Response: [0x55][SEQ][STATUS][LEN][DATA...][CRC8]
 *
 * The response arrives in the NEXT SPI transaction!
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include <argos-smd/argos_smd_spi.h>
#include <argos-smd/argos_smd_dfu_spi.h>

LOG_MODULE_REGISTER(spi_dfu_test, LOG_LEVEL_INF);

/* Get Argos SMD SPI device from devicetree */
#define ARGOS_SMD_SPI_NODE DT_NODELABEL(argos_smd)

#if !DT_NODE_EXISTS(ARGOS_SMD_SPI_NODE)
#error "Argos SMD SPI device not found in devicetree. Check your overlay."
#endif

static const struct device *argos_dev = DEVICE_DT_GET(ARGOS_SMD_SPI_NODE);

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
#define TEST_CHUNK_SIZE  ARGOS_DFU_CHUNK_SIZE

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

	int ret = argos_spi_reset(dev);
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

	int ret = argos_spi_diagnostic(dev);
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

	int ret = argos_spi_ping(dev);
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

	int ret = argos_spi_get_version(dev, version, &version_len);
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

	int ret = argos_dfu_enter(dev);
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

	int ret = argos_dfu_ping(dev);
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
	struct argos_bl_info info;

	LOG_INF("--- Test: DFU GET INFO ---");

	int ret = argos_dfu_get_info(dev, &info);
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
	LOG_INF("(Note: This takes ~2-3 seconds - using 3000ms delay)");

	int ret = argos_dfu_erase(dev);
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
	uint32_t addr = ARGOS_FLASH_APPLICATION;

	LOG_INF("--- Test: DFU WRITE CHUNK ---");
	LOG_INF("Protocol: WRITE_REQ (0x33) + WRITE_DATA (0x34)");
	LOG_INF("  WRITE_REQ payload: [addr:4B LE][len:2B LE]");

	/* Fill with test pattern */
	for (int i = 0; i < sizeof(test_data); i++) {
		test_data[i] = (uint8_t)i;
	}

	int ret = argos_dfu_write_chunk(dev, addr, test_data, sizeof(test_data));
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Chunk written at 0x%08X", addr);
	} else {
		LOG_ERR("RESULT: FAIL - Write chunk failed: %d", ret);
	}

	return ret;
}

/* Test: Read flash data (2-transaction protocol) */
static int test_dfu_read(const struct device *dev)
{
	uint8_t read_data[64];
	uint32_t addr = ARGOS_FLASH_APPLICATION;

	LOG_INF("--- Test: DFU READ ---");
	LOG_INF("Protocol: READ_REQ (0x35) + READ_DATA (0x36)");
	LOG_INF("  READ_REQ payload: [addr:4B LE][len:2B LE]");

	int ret = argos_dfu_read(dev, addr, read_data, sizeof(read_data));
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Read %zu bytes from 0x%08X", sizeof(read_data), addr);
		LOG_INF("  Data: %02X %02X %02X %02X %02X %02X %02X %02X ...",
			read_data[0], read_data[1], read_data[2], read_data[3],
			read_data[4], read_data[5], read_data[6], read_data[7]);

		/* Verify pattern matches what was written */
		bool match = true;
		for (int i = 0; i < sizeof(read_data); i++) {
			if (read_data[i] != (uint8_t)i) {
				match = false;
				break;
			}
		}
		if (match) {
			LOG_INF("  Pattern verification: MATCH");
		} else {
			LOG_WRN("  Pattern verification: MISMATCH (data may not have been written)");
		}
	} else {
		LOG_ERR("RESULT: FAIL - Read failed: %d", ret);
	}

	return ret;
}

/* Test: DFU Get Extended Status */
static int test_dfu_get_extended_status(const struct device *dev)
{
	struct argos_dfu_extended_status status;

	LOG_INF("--- Test: DFU GET EXTENDED STATUS (32 bytes) ---");

	int ret = argos_dfu_get_extended_status(dev, &status);
	if (ret == 0) {
		LOG_INF("RESULT: PASS - Extended status retrieved");
		LOG_INF("  Protocol version: %u", status.protocol_version);
		LOG_INF("  Bootloader state: %u", status.bootloader_state);
		LOG_INF("  DFU op state: %u", status.dfu_op_state);
		LOG_INF("  Last error: 0x%02X", status.last_error);
		LOG_INF("  Session active: %u", status.session_active);
		LOG_INF("  Erase done: %u", status.erase_done);
		LOG_INF("  Verify passed: %u", status.verify_passed);
		LOG_INF("  Received bytes: %u", status.received_bytes);
		LOG_INF("  Write address: 0x%08X", status.write_address);
		LOG_INF("  Frame count: %u", status.frame_count);
		LOG_INF("  CRC error count: %u", status.crc_error_count);
	} else {
		LOG_ERR("RESULT: FAIL - Get extended status failed: %d", ret);
	}

	return ret;
}

/* Test: DFU Abort */
static int test_dfu_abort(const struct device *dev)
{
	LOG_INF("--- Test: DFU ABORT ---");

	int ret = argos_dfu_abort(dev);
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

	int ret = argos_dfu_jump(dev);
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

	int ret = argos_spi_firmware_update(dev, TEST_FW_IMAGE, TEST_FW_SIZE,
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
	LOG_INF("   Argos SMD SPI DFU Test Application");
	LOG_INF("   Protocol A+ Implementation");
	LOG_INF("========================================");
	LOG_INF("");
	LOG_INF("Protocol A+ uses two-transaction model:");
	LOG_INF("  TX: [0xAA][SEQ][CMD][LEN][DATA][CRC8]");
	LOG_INF("  RX: [0x55][SEQ][STAT][LEN][DATA][CRC8]");
	LOG_INF("  (response arrives in NEXT transaction)");
	LOG_INF("");

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
	if (!device_is_ready(argos_dev)) {
		LOG_ERR("Argos SMD SPI device not ready!");
		return -1;
	}

	LOG_INF("Argos SMD SPI device ready");
	LOG_INF("");

	/* ===== PHASE 0: SPI DIAGNOSTIC ===== */
	LOG_INF("======================================");
	LOG_INF("  PHASE 0: SPI BUS DIAGNOSTIC");
	LOG_INF("======================================");
	LOG_INF("");

	/* Try hardware reset first */
	test_hardware_reset(argos_dev);
	k_msleep(100);

	/* Run SPI diagnostic */
	test_spi_diagnostic(argos_dev);
	k_msleep(500);

	LOG_INF("");

	/* ===== PHASE 1: DETECT APP OR BOOTLOADER ===== */
	LOG_INF("======================================");
	LOG_INF("  PHASE 1: DETECT MODE (APP/BOOTLOADER)");
	LOG_INF("======================================");
	LOG_INF("");

	bool in_bootloader = false;
	int ret;

	/* Try to ping application first */
	LOG_INF("Attempting to communicate with application...");
	ret = argos_spi_ping(argos_dev);
	if (ret == 0) {
		LOG_INF(">> APPLICATION MODE detected");
		LOG_INF("   Testing application communication...");
		k_msleep(200);

		errors += (test_spi_get_version(argos_dev) != 0) ? 1 : 0;
		k_msleep(500);
	} else {
		LOG_WRN("Application not responding, checking if already in bootloader...");

		/* Try to ping bootloader */
		ret = argos_dfu_ping(argos_dev);
		if (ret == 0) {
			LOG_INF(">> BOOTLOADER MODE detected (already active)");
			in_bootloader = true;
		} else {
			LOG_ERR(">> No response from application or bootloader!");
			LOG_ERR("   Check hardware connections (SPI, power)");
			errors++;
		}
	}

	LOG_INF("");

	/* ===== PHASE 2: ENTER BOOTLOADER MODE ===== */
	LOG_INF("======================================");
	LOG_INF("  PHASE 2: ENTER BOOTLOADER MODE");
	LOG_INF("======================================");
	LOG_INF("");

	if (!in_bootloader) {
		LOG_INF("Sending DFU_ENTER command to application...");
		errors += (test_dfu_enter(argos_dev) != 0) ? 1 : 0;
		k_msleep(200);  /* Wait for app reset */

		/* Wait for bootloader to be ready */
		LOG_INF("Waiting for bootloader to start...");
		ret = argos_dfu_wait_ready(argos_dev, K_SECONDS(5));
		if (ret != 0) {
			LOG_ERR("Bootloader not ready after DFU_ENTER: %d", ret);
			errors++;
		} else {
			LOG_INF(">> Bootloader started successfully");
			in_bootloader = true;
		}
	} else {
		LOG_INF("Already in bootloader mode, skipping DFU_ENTER");
	}

	/* Verify bootloader communication */
	if (in_bootloader) {
		errors += (test_dfu_ping(argos_dev) != 0) ? 1 : 0;
		k_msleep(500);

		errors += (test_dfu_get_info(argos_dev) != 0) ? 1 : 0;
		k_msleep(500);
	}

	LOG_INF("");

	/* ===== PHASE 3: DFU OPERATIONS TESTS ===== */
	LOG_INF("======================================");
	LOG_INF("  PHASE 3: DFU OPERATIONS TESTS");
	LOG_INF("  (Protocol A+ two-transaction model)");
	LOG_INF("======================================");
	LOG_INF("");

	/* Skip DFU tests if not in bootloader mode */
	if (!in_bootloader) {
		LOG_ERR("Cannot run DFU tests - not in bootloader mode");
		LOG_INF("Skipping DFU operations...");
		goto skip_dfu_tests;
	}

	/* Make sure we're in bootloader mode */
	argos_dfu_wait_ready(argos_dev, K_SECONDS(2));

	/* Test GET_EXTENDED_STATUS (32 bytes) */
	errors += (test_dfu_get_extended_status(argos_dev) != 0) ? 1 : 0;
	k_msleep(500);

	/* Test ERASE (takes ~2-3 seconds) */
	errors += (test_dfu_erase(argos_dev) != 0) ? 1 : 0;
	k_msleep(500);

	/* Test extended status after erase */
	LOG_INF("--- Checking status after erase ---");
	test_dfu_get_extended_status(argos_dev);
	k_msleep(200);

	/* Test WRITE_REQ + WRITE_DATA */
	errors += (test_dfu_write_chunk(argos_dev) != 0) ? 1 : 0;
	k_msleep(500);

	/* Test READ_REQ + READ_DATA to verify 2-transaction protocol */
	errors += (test_dfu_read(argos_dev) != 0) ? 1 : 0;
	k_msleep(500);

	errors += (test_dfu_abort(argos_dev) != 0) ? 1 : 0;
	k_msleep(500);

	LOG_INF("");

	/* ===== PHASE 4: FULL OTA UPDATE (OPTIONAL) ===== */
#ifdef CONFIG_ARGOS_SMD_DFU_TEST_FULL_UPDATE
	LOG_INF("======================================");
	LOG_INF("  PHASE 4: FULL OTA UPDATE TEST");
	LOG_INF("======================================");
	LOG_INF("");
	LOG_WRN("WARNING: This will perform a real firmware update!");
	LOG_INF("");

	/* Re-enter bootloader mode (abort may have exited) */
	argos_dfu_enter(argos_dev);
	k_msleep(200);
	argos_dfu_wait_ready(argos_dev, K_SECONDS(5));

	errors += (test_full_firmware_update(argos_dev) != 0) ? 1 : 0;
#else
	LOG_INF("======================================");
	LOG_INF("  PHASE 4: FULL OTA UPDATE - SKIPPED");
	LOG_INF("======================================");
	LOG_INF("(Enable CONFIG_ARGOS_SMD_DFU_TEST_FULL_UPDATE to run)");

	/* Jump back to application */
	test_dfu_jump(argos_dev);
#endif

	LOG_INF("");

skip_dfu_tests:
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
