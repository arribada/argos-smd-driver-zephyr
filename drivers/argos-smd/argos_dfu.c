/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <argos-smd/argos_dfu.h>
#include <argos-smd/argos_smd.h>

LOG_MODULE_REGISTER(argos_dfu, CONFIG_ARGOS_SMD_DFU_LOG_LEVEL);

/* CRC32 calculation using standard polynomial (reflected form) */
uint32_t argos_dfu_crc32(const uint8_t *data, size_t len)
{
	uint32_t crc = 0xFFFFFFFF;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++) {
			crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
		}
	}

	return ~crc;
}

/* Convert binary data to hexadecimal ASCII string */
static void bin_to_hex(const uint8_t *bin, size_t bin_len, char *hex)
{
	static const char hex_chars[] = "0123456789ABCDEF";

	for (size_t i = 0; i < bin_len; i++) {
		hex[i * 2]     = hex_chars[(bin[i] >> 4) & 0x0F];
		hex[i * 2 + 1] = hex_chars[bin[i] & 0x0F];
	}
	hex[bin_len * 2] = '\0';
}

int argos_enter_bootloader(const struct device *dev)
{
	LOG_INF("Entering bootloader mode...");

	int ret = argos_send_raw(dev, "AT+BOOT");
	if (ret < 0) {
		LOG_ERR("Failed to send BOOT command: %d", ret);
		return ret;
	}

	/* Device will reboot - give it time */
	k_msleep(500);

	return 0;
}

int argos_wait_bootloader_ready(const struct device *dev, k_timeout_t timeout)
{
	LOG_INF("Waiting for bootloader to be ready...");

	int64_t start = k_uptime_get();
	int64_t timeout_ms = k_ticks_to_ms_floor64(timeout.ticks);

	while ((k_uptime_get() - start) < timeout_ms) {
		/* Try to ping the device */
		int ret = argos_read_ping(dev);
		if (ret == 0) {
			LOG_INF("Bootloader is ready!");
			return 0;
		}

		/* Wait a bit before retrying */
		k_msleep(100);
	}

	LOG_ERR("Bootloader not responding after timeout");
	return -ETIMEDOUT;
}

int argos_dfu_start(const struct device *dev, uint32_t fw_size, uint32_t fw_crc)
{
	char cmd[64];

	LOG_INF("Starting DFU session: size=%u bytes, crc32=0x%08X", fw_size, fw_crc);

	/* Format command: AT+DFUSTART=<size>,<crc32> */
	snprintf(cmd, sizeof(cmd), "AT+DFUSTART=%u,%08X", fw_size, fw_crc);

	int ret = argos_send_raw(dev, cmd);
	if (ret < 0) {
		LOG_ERR("Failed to start DFU session: %d", ret);
		return ret;
	}

	LOG_DBG("DFU session started successfully");
	return 0;
}

int argos_dfu_send_chunk(const struct device *dev, const uint8_t *data, size_t len)
{
	/* Buffer for command: "AT+DFUDATA=" + hex data + null terminator */
	char cmd[16 + ARGOS_DFU_CHUNK_SIZE * 2 + 1];
	char hex_data[ARGOS_DFU_CHUNK_SIZE * 2 + 1];

	if (len > ARGOS_DFU_CHUNK_SIZE) {
		LOG_ERR("Chunk size %zu exceeds maximum %d", len, ARGOS_DFU_CHUNK_SIZE);
		return -EINVAL;
	}

	if (len == 0) {
		LOG_ERR("Cannot send empty chunk");
		return -EINVAL;
	}

	/* Convert binary data to hex string */
	bin_to_hex(data, len, hex_data);

	/* Build command */
	snprintf(cmd, sizeof(cmd), "AT+DFUDATA=%s", hex_data);

	LOG_DBG("Sending chunk: %zu bytes", len);

	int ret = argos_send_raw(dev, cmd);
	if (ret < 0) {
		LOG_ERR("Failed to send data chunk: %d", ret);
		return ret;
	}

	return 0;
}

int argos_dfu_finish(const struct device *dev)
{
	LOG_INF("Finalizing DFU session...");

	int ret = argos_send_raw(dev, "AT+DFUEND");
	if (ret < 0) {
		LOG_ERR("Failed to finalize DFU: %d", ret);
		return ret;
	}

	LOG_INF("DFU finalized successfully, device will reboot...");

	/* Give device time to reboot */
	k_msleep(500);

	return 0;
}

int argos_dfu_abort(const struct device *dev)
{
	LOG_WRN("Aborting DFU session...");

	int ret = argos_send_raw(dev, "AT+DFUABORT");
	if (ret < 0) {
		LOG_ERR("Failed to abort DFU: %d", ret);
		return ret;
	}

	LOG_INF("DFU session aborted");
	return 0;
}

int argos_dfu_get_status(const struct device *dev,
			 uint32_t *progress, enum argos_dfu_state *state)
{
	/* Note: This function would need to parse the response from AT+DFUSTATUS=?
	 * The current argos_smd driver doesn't provide a way to get the response string,
	 * so this is a placeholder implementation.
	 * In a real implementation, you would need to extend the driver API or use callbacks.
	 */
	LOG_WRN("argos_dfu_get_status not fully implemented - needs response parsing");
	return -ENOSYS;
}

int argos_ota_update(const struct device *dev,
		     const uint8_t *fw_image, size_t fw_size,
		     argos_dfu_progress_cb_t progress_cb)
{
	int ret;

	LOG_INF("========================================");
	LOG_INF("     OTA Update Started");
	LOG_INF("========================================");
	LOG_INF("Firmware size: %zu bytes", fw_size);

	if (!fw_image || fw_size == 0) {
		LOG_ERR("Invalid firmware image");
		return -EINVAL;
	}

	/* Step 1: Calculate CRC32 of the firmware image */
	uint32_t fw_crc = argos_dfu_crc32(fw_image, fw_size);
	LOG_INF("Firmware CRC32: 0x%08X", fw_crc);

	/* Step 2: Enter bootloader mode */
	LOG_INF("Step 1/6: Entering bootloader mode...");
	ret = argos_enter_bootloader(dev);
	if (ret < 0) {
		LOG_ERR("Failed to enter bootloader mode: %d", ret);
		return ret;
	}

	/* Step 3: Wait for bootloader to be ready */
	LOG_INF("Step 2/6: Waiting for bootloader...");
	ret = argos_wait_bootloader_ready(dev, K_SECONDS(10));
	if (ret < 0) {
		LOG_ERR("Bootloader not ready: %d", ret);
		return ret;
	}

	/* Step 4: Start DFU session */
	LOG_INF("Step 3/6: Starting DFU session...");
	ret = argos_dfu_start(dev, fw_size, fw_crc);
	if (ret < 0) {
		LOG_ERR("Failed to start DFU session: %d", ret);
		return ret;
	}

	/* Step 5: Send all firmware chunks */
	LOG_INF("Step 4/6: Transferring firmware data...");
	size_t offset = 0;
	uint32_t chunk_num = 0;
	uint32_t last_progress_percent = 0;

	while (offset < fw_size) {
		size_t chunk_len = MIN(ARGOS_DFU_CHUNK_SIZE, fw_size - offset);

		ret = argos_dfu_send_chunk(dev, &fw_image[offset], chunk_len);
		if (ret < 0) {
			LOG_ERR("Failed to send chunk %u at offset %zu: %d",
				chunk_num, offset, ret);
			LOG_WRN("Attempting to abort DFU session...");
			argos_dfu_abort(dev);
			return ret;
		}

		offset += chunk_len;
		chunk_num++;

		/* Call progress callback if provided */
		if (progress_cb) {
			progress_cb(offset, fw_size);
		}

		/* Log progress every 10% */
		uint32_t progress_percent = (offset * 100) / fw_size;
		if (progress_percent != last_progress_percent &&
		    (progress_percent % 10 == 0)) {
			LOG_INF("Progress: %u%% (%zu/%zu bytes)",
				progress_percent, offset, fw_size);
			last_progress_percent = progress_percent;
		}
	}

	LOG_INF("Successfully sent %u chunks (%zu bytes total)", chunk_num, offset);

	/* Step 6: Finalize DFU */
	LOG_INF("Step 5/6: Finalizing DFU...");
	ret = argos_dfu_finish(dev);
	if (ret < 0) {
		LOG_ERR("Failed to finalize DFU: %d", ret);
		return ret;
	}

	/* Step 7: Wait for device to reboot and verify it's responsive */
	LOG_INF("Step 6/6: Verifying device after reboot...");
	k_msleep(2000);  /* Wait for device to fully reboot */

	ret = argos_read_ping(dev);
	if (ret < 0) {
		LOG_ERR("Device not responding after update: %d", ret);
		return ret;
	}

	LOG_INF("========================================");
	LOG_INF("     OTA Update Successful!");
	LOG_INF("========================================");

	return 0;
}
