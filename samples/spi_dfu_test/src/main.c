/*
 * Copyright (c) 2025 Arribada Initiative
 * SPDX-License-Identifier: Apache-2.0
 *
 * Argos SMD SPI DFU Test - Simplified version
 * Only tests: PING app -> DFU_ENTER -> Wait -> PING bootloader
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include <argos-smd/argos_smd_spi.h>
#include <argos-smd/argos_smd_dfu_spi.h>

LOG_MODULE_REGISTER(spi_dfu_test, LOG_LEVEL_DBG);

#define ARGOS_SMD_SPI_NODE DT_NODELABEL(argos_smd)
#if !DT_NODE_EXISTS(ARGOS_SMD_SPI_NODE)
#error "Argos SMD SPI device not found in devicetree"
#endif

static const struct device *dev = DEVICE_DT_GET(ARGOS_SMD_SPI_NODE);

int main(void)
{
	int ret;

	LOG_INF("========================================");
	LOG_INF("  Simplified DFU Test");
	LOG_INF("  PING -> DFU_ENTER -> PING bootloader");
	LOG_INF("========================================");
	LOG_INF("");

	if (!device_is_ready(dev)) {
		LOG_ERR("Device not ready!");
		return -1;
	}
	LOG_INF("Device ready");

	/* ===== STEP 1: SYNC ===== */
	LOG_INF("");
	LOG_INF("=== STEP 1: SYNC ===");
	ret = argos_spi_sync(dev);
	LOG_INF("Sync returned: %d", ret);
	k_msleep(100);

	/* ===== STEP 2: PING APP ===== */
	LOG_INF("");
	LOG_INF("=== STEP 2: PING APP (cmd 0x02) ===");
	ret = argos_spi_ping(dev);
	if (ret == 0) {
		LOG_INF(">> APP PING OK!");
	} else {
		LOG_ERR(">> APP PING FAILED: %d", ret);
		/* Check if already in bootloader */
		LOG_INF("Checking if in bootloader...");
		ret = argos_dfu_ping(dev);
		if (ret == 0) {
			LOG_INF(">> Already in BOOTLOADER mode!");
			goto done;
		}
		LOG_ERR("No response from app or bootloader");
		return -1;
	}
	k_msleep(200);

	/* ===== STEP 3: DFU_ENTER (0x3F) ===== */
	LOG_INF("");
	LOG_INF("=== STEP 3: DFU_ENTER (cmd 0x3F) ===");
	LOG_INF("Calling argos_dfu_enter()...");
	ret = argos_dfu_enter(dev);
	LOG_INF("argos_dfu_enter() returned: %d", ret);

	/* ===== STEP 4: WAIT FOR RESET ===== */
	LOG_INF("");
	LOG_INF("=== STEP 4: WAIT 2s FOR STM32 RESET ===");
	k_msleep(2000);

	/* ===== STEP 5: SYNC WITH BOOTLOADER ===== */
	LOG_INF("");
	LOG_INF("=== STEP 5: SYNC WITH BOOTLOADER ===");
	ret = argos_spi_sync(dev);
	LOG_INF("Sync returned: %d", ret);
	k_msleep(100);

	/* ===== STEP 6: PING BOOTLOADER ===== */
	LOG_INF("");
	LOG_INF("=== STEP 6: PING BOOTLOADER (DFU cmd 0x30) ===");
	ret = argos_dfu_ping(dev);
	if (ret == 0) {
		LOG_INF(">> BOOTLOADER PING OK!");
	} else {
		LOG_ERR(">> BOOTLOADER PING FAILED: %d", ret);
	}

	/* ===== STEP 7: GET BOOTLOADER INFO ===== */
	if (ret == 0) {
		struct argos_bl_info info;
		LOG_INF("");
		LOG_INF("=== STEP 7: GET BOOTLOADER INFO ===");
		ret = argos_dfu_get_info(dev, &info);
		if (ret == 0) {
			LOG_INF("Bootloader v%u.%u.%u",
				info.version_major, info.version_minor, info.version_patch);
			LOG_INF("App start: 0x%08X, max size: %u",
				info.app_start_addr, info.app_max_size);
		} else {
			LOG_ERR("Get info failed: %d", ret);
		}
	}

done:
	LOG_INF("");
	LOG_INF("========================================");
	LOG_INF("  Test Complete");
	LOG_INF("========================================");

	return 0;
}

#if 0
/* ============================================================
 * ADVANCED TESTS - COMMENTED OUT FOR NOW
 * ============================================================ */

/* External firmware image (if provided via CMake) */
#ifdef USE_EXTERNAL_FIRMWARE
#include "firmware_image.h"
#define TEST_FW_IMAGE    firmware_image
#define TEST_FW_SIZE     FIRMWARE_IMAGE_SIZE
#else
static const uint8_t test_firmware[] = {
	0x00, 0x20, 0x00, 0x08,
	0x41, 0x81, 0x00, 0x08,
};
#define TEST_FW_IMAGE    test_firmware
#define TEST_FW_SIZE     sizeof(test_firmware)
#endif

#define TEST_CHUNK_SIZE  ARGOS_DFU_CHUNK_SIZE

static void dfu_progress_callback(uint32_t current, uint32_t total, void *user_data)
{
	uint32_t percent = (current * 100) / total;
	if (percent % 10 == 0) {
		LOG_INF("Progress: %u%%", percent);
	}
}

static int test_dfu_erase(const struct device *dev)
{
	LOG_INF("--- DFU ERASE ---");
	int ret = argos_dfu_erase(dev);
	LOG_INF("Erase result: %d", ret);
	return ret;
}

static int test_dfu_write_chunk(const struct device *dev)
{
	uint8_t test_data[64];
	for (int i = 0; i < sizeof(test_data); i++) {
		test_data[i] = (uint8_t)i;
	}
	int ret = argos_dfu_write_chunk(dev, ARGOS_FLASH_APPLICATION, test_data, sizeof(test_data));
	LOG_INF("Write chunk result: %d", ret);
	return ret;
}

static int test_full_firmware_update(const struct device *dev)
{
	int ret = argos_spi_firmware_update(dev, TEST_FW_IMAGE, TEST_FW_SIZE,
					 dfu_progress_callback, NULL);
	LOG_INF("Firmware update result: %d", ret);
	return ret;
}

#endif /* #if 0 - Advanced tests */
