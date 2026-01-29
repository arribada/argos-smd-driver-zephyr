/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

#include <argos-smd/argos_smd_spi.h>
#include <argos-smd/argos_smd_dfu_spi.h>

LOG_MODULE_REGISTER(argos_smd_dfu_spi, CONFIG_ARGOS_SMD_DFU_LOG_LEVEL);

/* CRC32 calculation using standard polynomial (same as UART DFU) */
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

int argos_dfu_enter(const struct device *dev)
{
	uint8_t status;
	int ret;

	LOG_INF("Entering DFU bootloader mode...");

	/* Send CMD_DFU_ENTER (0x3F) - device will ACK then reset */
	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_DFU_ENTER, NULL, 0, NULL, NULL, &status);
	if (ret < 0) {
		LOG_ERR("Failed to send DFU enter command: %d", ret);
		return ret;
	}

	if (status != ARGOS_SPI_RSP_OK) {
		LOG_ERR("DFU enter rejected: status 0x%02X", status);
		return -EIO;
	}

	LOG_INF("DFU enter acknowledged, device will reset to bootloader");

	/* Wait for device to reset */
	k_msleep(ARGOS_DFU_RESET_WAIT_MS);

	return 0;
}

int argos_dfu_ping(const struct device *dev)
{
	uint8_t status;
	uint8_t rx_data[16];
	size_t rx_len = sizeof(rx_data);
	int ret;

	LOG_DBG("Pinging DFU bootloader...");

	ret = argos_spi_transact(dev, ARGOS_SPI_DFU_CMD_PING, NULL, 0,
				  rx_data, &rx_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_DFU_RSP_OK) {
		LOG_ERR("DFU ping failed: status 0x%02X", status);
		return -EIO;
	}

	if (rx_len >= 3) {
		LOG_INF("Bootloader version: %u.%u.%u", rx_data[0], rx_data[1], rx_data[2]);
	}

	return 0;
}

int argos_dfu_wait_ready(const struct device *dev, k_timeout_t timeout)
{
	LOG_INF("Waiting for DFU bootloader to be ready...");

	int64_t start = k_uptime_get();
	int64_t timeout_ms = k_ticks_to_ms_floor64(timeout.ticks);

	while ((k_uptime_get() - start) < timeout_ms) {
		int ret = argos_dfu_ping(dev);
		if (ret == 0) {
			LOG_INF("DFU bootloader is ready!");
			return 0;
		}

		k_msleep(50);
	}

	LOG_ERR("DFU bootloader not responding after timeout");
	return -ETIMEDOUT;
}

int argos_dfu_get_info(const struct device *dev, struct argos_bl_info *info)
{
	uint8_t status;
	uint8_t rx_data[32];
	size_t rx_len = sizeof(rx_data);
	int ret;

	if (!info) {
		return -EINVAL;
	}

	LOG_DBG("Getting bootloader info...");

	ret = argos_spi_transact(dev, ARGOS_SPI_DFU_CMD_GET_INFO, NULL, 0,
				  rx_data, &rx_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_DFU_RSP_OK) {
		LOG_ERR("Get info failed: status 0x%02X", status);
		return -EIO;
	}

	/* Parse response */
	if (rx_len >= 15) {
		info->version_major = rx_data[0];
		info->version_minor = rx_data[1];
		info->version_patch = rx_data[2];
		info->app_start_addr = (rx_data[3] << 24) | (rx_data[4] << 16) |
				       (rx_data[5] << 8) | rx_data[6];
		info->app_max_size = (rx_data[7] << 24) | (rx_data[8] << 16) |
				     (rx_data[9] << 8) | rx_data[10];
		info->page_size = (rx_data[11] << 24) | (rx_data[12] << 16) |
				  (rx_data[13] << 8) | rx_data[14];

		LOG_INF("Bootloader v%u.%u.%u, app_start=0x%08X, max_size=%u, page_size=%u",
			info->version_major, info->version_minor, info->version_patch,
			info->app_start_addr, info->app_max_size, info->page_size);
	} else {
		LOG_WRN("Bootloader info response too short: %zu bytes", rx_len);
		memset(info, 0, sizeof(*info));
	}

	return 0;
}

int argos_dfu_erase(const struct device *dev)
{
	uint8_t status;
	int ret;

	LOG_INF("Erasing application flash (this may take a few seconds)...");

	/* Erase can take 2-3 seconds - use longer timeout */
	ret = argos_spi_transact(dev, ARGOS_SPI_DFU_CMD_ERASE, NULL, 0,
				  NULL, NULL, &status);
	if (ret < 0) {
		LOG_ERR("Erase command failed: %d", ret);
		return ret;
	}

	if (status != ARGOS_DFU_RSP_OK) {
		LOG_ERR("Flash erase failed: status 0x%02X", status);
		return -EIO;
	}

	LOG_INF("Flash erased successfully");
	return 0;
}

int argos_dfu_write_chunk(const struct device *dev, uint32_t addr,
			   const uint8_t *data, size_t len)
{
	uint8_t status;
	uint8_t req_data[6];
	int ret;

	if (!data || len == 0 || len > ARGOS_DFU_CHUNK_SIZE) {
		return -EINVAL;
	}

	LOG_DBG("Writing %zu bytes at 0x%08X", len, addr);

	/* Step 1: Send WRITE_REQ with address and length */
	req_data[0] = (addr >> 24) & 0xFF;
	req_data[1] = (addr >> 16) & 0xFF;
	req_data[2] = (addr >> 8) & 0xFF;
	req_data[3] = addr & 0xFF;
	req_data[4] = (len >> 8) & 0xFF;
	req_data[5] = len & 0xFF;

	ret = argos_spi_transact(dev, ARGOS_SPI_DFU_CMD_WRITE_REQ, req_data, 6,
				  NULL, NULL, &status);
	if (ret < 0) {
		LOG_ERR("Write request failed: %d", ret);
		return ret;
	}

	if (status != ARGOS_DFU_RSP_OK) {
		LOG_ERR("Write request rejected: status 0x%02X", status);
		return -EIO;
	}

	/* Step 2: Send WRITE_DATA with actual data */
	ret = argos_spi_transact(dev, ARGOS_SPI_DFU_CMD_WRITE_DATA, data, len,
				  NULL, NULL, &status);
	if (ret < 0) {
		LOG_ERR("Write data failed: %d", ret);
		return ret;
	}

	if (status != ARGOS_DFU_RSP_OK) {
		LOG_ERR("Write data rejected: status 0x%02X", status);
		if (status == ARGOS_DFU_RSP_FLASH_ERROR) {
			return -EIO;
		} else if (status == ARGOS_DFU_RSP_ADDR_ERROR) {
			return -EFAULT;
		} else if (status == ARGOS_DFU_RSP_SIZE_ERROR) {
			return -EINVAL;
		}
		return -EIO;
	}

	return 0;
}

int argos_dfu_read(const struct device *dev, uint32_t addr, uint8_t *data, size_t len)
{
	uint8_t status;
	uint8_t req_data[6];
	size_t rx_len;
	int ret;

	if (!data || len == 0 || len > ARGOS_SPI_MAX_PAYLOAD) {
		return -EINVAL;
	}

	LOG_DBG("Reading %zu bytes from 0x%08X", len, addr);

	/* Step 1: Send READ_REQ with address and length */
	req_data[0] = (addr >> 24) & 0xFF;
	req_data[1] = (addr >> 16) & 0xFF;
	req_data[2] = (addr >> 8) & 0xFF;
	req_data[3] = addr & 0xFF;
	req_data[4] = (len >> 8) & 0xFF;
	req_data[5] = len & 0xFF;

	ret = argos_spi_transact(dev, ARGOS_SPI_DFU_CMD_READ_REQ, req_data, 6,
				  NULL, NULL, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_DFU_RSP_OK) {
		LOG_ERR("Read request rejected: status 0x%02X", status);
		return -EIO;
	}

	/* Step 2: Send READ_DATA to get the data */
	rx_len = len;
	ret = argos_spi_transact(dev, ARGOS_SPI_DFU_CMD_READ_DATA, NULL, 0,
				  data, &rx_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_DFU_RSP_OK) {
		LOG_ERR("Read data failed: status 0x%02X", status);
		return -EIO;
	}

	return 0;
}

int argos_dfu_verify(const struct device *dev, uint32_t crc32)
{
	uint8_t status;
	uint8_t crc_data[4];
	int ret;

	LOG_INF("Verifying firmware CRC32: 0x%08X", crc32);

	/* Send CRC32 in big-endian format */
	crc_data[0] = (crc32 >> 24) & 0xFF;
	crc_data[1] = (crc32 >> 16) & 0xFF;
	crc_data[2] = (crc32 >> 8) & 0xFF;
	crc_data[3] = crc32 & 0xFF;

	ret = argos_spi_transact(dev, ARGOS_SPI_DFU_CMD_VERIFY, crc_data, 4,
				  NULL, NULL, &status);
	if (ret < 0) {
		LOG_ERR("Verify command failed: %d", ret);
		return ret;
	}

	if (status == ARGOS_DFU_RSP_OK) {
		LOG_INF("CRC verification passed!");
		return 0;
	} else if (status == ARGOS_DFU_RSP_CRC_ERROR ||
		   status == ARGOS_DFU_RSP_VERIFY_ERROR) {
		LOG_ERR("CRC verification failed - mismatch");
		return -EILSEQ;
	} else {
		LOG_ERR("Verify failed: status 0x%02X", status);
		return -EIO;
	}
}

int argos_dfu_reset(const struct device *dev)
{
	int ret;

	LOG_INF("Resetting device...");

	/* Reset command - no response expected */
	ret = argos_spi_send_only(dev, ARGOS_SPI_DFU_CMD_RESET, NULL, 0);
	if (ret < 0) {
		LOG_ERR("Reset command failed: %d", ret);
		return ret;
	}

	/* Wait for reset to complete */
	k_msleep(ARGOS_DFU_RESET_WAIT_MS);

	return 0;
}

int argos_dfu_jump(const struct device *dev)
{
	int ret;

	LOG_INF("Jumping to application...");

	/* Jump command - device will reset to application */
	ret = argos_spi_send_only(dev, ARGOS_SPI_DFU_CMD_JUMP, NULL, 0);
	if (ret < 0) {
		LOG_ERR("Jump command failed: %d", ret);
		return ret;
	}

	/* Wait for jump/reset to complete */
	k_msleep(ARGOS_DFU_RESET_WAIT_MS * 2);

	return 0;
}

int argos_dfu_get_status_spi(const struct device *dev, struct argos_dfu_status *status_out)
{
	uint8_t status;
	uint8_t rx_data[16];
	size_t rx_len = sizeof(rx_data);
	int ret;

	if (!status_out) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_DFU_CMD_GET_STATUS, NULL, 0,
				  rx_data, &rx_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_DFU_RSP_OK) {
		LOG_ERR("Get status failed: status 0x%02X", status);
		return -EIO;
	}

	/* Parse response */
	if (rx_len >= 10) {
		status_out->state = rx_data[0];
		status_out->bytes_written = (rx_data[1] << 24) | (rx_data[2] << 16) |
					    (rx_data[3] << 8) | rx_data[4];
		status_out->total_bytes = (rx_data[5] << 24) | (rx_data[6] << 16) |
					  (rx_data[7] << 8) | rx_data[8];
		status_out->last_error = rx_data[9];
	}

	return 0;
}

int argos_dfu_abort(const struct device *dev)
{
	uint8_t status;
	int ret;

	LOG_WRN("Aborting DFU operation...");

	ret = argos_spi_transact(dev, ARGOS_SPI_DFU_CMD_ABORT, NULL, 0,
				  NULL, NULL, &status);
	if (ret < 0) {
		LOG_ERR("Abort command failed: %d", ret);
		return ret;
	}

	if (status != ARGOS_DFU_RSP_OK) {
		LOG_ERR("Abort failed: status 0x%02X", status);
		return -EIO;
	}

	LOG_INF("DFU aborted");
	return 0;
}

int argos_dfu_set_header(const struct device *dev, const uint8_t *header)
{
	uint8_t status;
	int ret;

	if (!header) {
		return -EINVAL;
	}

	LOG_DBG("Setting application header (%d bytes)", ARGOS_DFU_HEADER_SIZE);

	ret = argos_spi_transact(dev, ARGOS_SPI_DFU_CMD_SET_HEADER,
				  header, ARGOS_DFU_HEADER_SIZE,
				  NULL, NULL, &status);
	if (ret < 0) {
		LOG_ERR("Set header failed: %d", ret);
		return ret;
	}

	if (status != ARGOS_DFU_RSP_OK) {
		LOG_ERR("Set header rejected: status 0x%02X", status);
		return -EIO;
	}

	return 0;
}

int argos_spi_firmware_update(const struct device *dev,
			      const uint8_t *firmware, size_t size,
			      argos_dfu_progress_cb_t progress_cb, void *user_data)
{
	int ret;
	uint32_t crc32;

	LOG_INF("========================================");
	LOG_INF("     SPI DFU Update Started");
	LOG_INF("========================================");
	LOG_INF("Firmware size: %zu bytes", size);

	if (!firmware || size == 0) {
		LOG_ERR("Invalid firmware image");
		return -EINVAL;
	}

	if (size > ARGOS_MAX_APP_SIZE) {
		LOG_ERR("Firmware too large: %zu bytes (max %u)", size, ARGOS_MAX_APP_SIZE);
		return -EINVAL;
	}

	/* Step 1: Calculate CRC32 */
	LOG_INF("Step 1/7: Calculating firmware CRC32...");
	crc32 = argos_dfu_crc32(firmware, size);
	LOG_INF("Firmware CRC32: 0x%08X", crc32);

	/* Step 2: Enter bootloader mode */
	LOG_INF("Step 2/7: Entering bootloader mode...");
	ret = argos_dfu_enter(dev);
	if (ret < 0) {
		LOG_ERR("Failed to enter bootloader: %d", ret);
		return ret;
	}

	/* Step 3: Wait for bootloader to be ready */
	LOG_INF("Step 3/7: Waiting for bootloader...");
	ret = argos_dfu_wait_ready(dev, K_SECONDS(5));
	if (ret < 0) {
		LOG_ERR("Bootloader not ready: %d", ret);
		return ret;
	}

	/* Step 4: Get bootloader info (optional but useful) */
	struct argos_bl_info bl_info;
	ret = argos_dfu_get_info(dev, &bl_info);
	if (ret < 0) {
		LOG_WRN("Could not get bootloader info: %d", ret);
		/* Continue anyway */
	}

	/* Step 5: Erase flash */
	LOG_INF("Step 4/7: Erasing flash...");
	ret = argos_dfu_erase(dev);
	if (ret < 0) {
		LOG_ERR("Flash erase failed: %d", ret);
		argos_dfu_abort(dev);
		return ret;
	}

	/* Step 6: Write firmware in chunks */
	LOG_INF("Step 5/7: Writing firmware (%zu bytes)...", size);

	uint32_t addr = ARGOS_FLASH_APPLICATION;
	size_t offset = 0;
	uint32_t last_progress = 0;

	while (offset < size) {
		size_t chunk_len = MIN(ARGOS_DFU_CHUNK_SIZE, size - offset);

		ret = argos_dfu_write_chunk(dev, addr, &firmware[offset], chunk_len);
		if (ret < 0) {
			LOG_ERR("Write failed at offset %zu: %d", offset, ret);
			argos_dfu_abort(dev);
			return ret;
		}

		addr += chunk_len;
		offset += chunk_len;

		/* Progress callback */
		if (progress_cb) {
			progress_cb(offset, size, user_data);
		}

		/* Log progress every 10% */
		uint32_t progress = (offset * 100) / size;
		if (progress != last_progress && (progress % 10 == 0)) {
			LOG_INF("Progress: %u%% (%zu/%zu bytes)", progress, offset, size);
			last_progress = progress;
		}
	}

	LOG_INF("Firmware written: %zu bytes", offset);

	/* Step 7: Verify CRC */
	LOG_INF("Step 6/7: Verifying CRC...");
	ret = argos_dfu_verify(dev, crc32);
	if (ret < 0) {
		LOG_ERR("CRC verification failed: %d", ret);
		argos_dfu_abort(dev);
		return ret;
	}

	/* Step 8: Jump to application */
	LOG_INF("Step 7/7: Starting application...");
	ret = argos_dfu_jump(dev);
	if (ret < 0) {
		LOG_ERR("Jump to application failed: %d", ret);
		return ret;
	}

	/* Verify device is responding in app mode */
	k_msleep(500);
	ret = argos_spi_ping(dev);
	if (ret < 0) {
		LOG_WRN("Device not responding after update (may need manual reset)");
	}

	LOG_INF("========================================");
	LOG_INF("     SPI DFU Update Successful!");
	LOG_INF("========================================");

	return 0;
}
