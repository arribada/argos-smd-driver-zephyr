/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Argos SMD DFU over SPI - Protocol A+ Implementation
 *
 * Protocol A+ Frame Format:
 * Request:  [0xAA][SEQ][CMD][LEN][DATA...][CRC8]
 * Response: [0x55][SEQ][STATUS][LEN][DATA...][CRC8]
 *
 * The response arrives in the NEXT SPI transaction!
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

#include <argos-smd/argos_smd_spi.h>
#include <argos-smd/argos_smd_dfu_spi.h>

LOG_MODULE_REGISTER(argos_smd_dfu_spi, CONFIG_ARGOS_SMD_DFU_LOG_LEVEL);

/* Protocol A+ Magic bytes */
#define DFU_MAGIC_REQUEST     0xAA
#define DFU_MAGIC_RESPONSE    0x55
#define DFU_IDLE_PATTERN      0xAA

/* Maximum frame sizes */
#define DFU_MAX_PAYLOAD       250
#define DFU_HEADER_SIZE       4    /* magic + seq + cmd/status + len */
#define DFU_CRC_SIZE          1
#define DFU_MAX_FRAME_SIZE    (DFU_HEADER_SIZE + DFU_MAX_PAYLOAD + DFU_CRC_SIZE)

/* Transaction buffer size (use 64 bytes for short commands, larger for data) */
#define DFU_TRANSACTION_SIZE  64
#define DFU_LARGE_TX_SIZE     280

/* Static sequence number for DFU session */
static uint8_t dfu_seq_num = 0;

/**
 * @brief Calculate CRC-8 CCITT checksum (polynomial 0x07)
 */
static uint8_t crc8_ccitt(const uint8_t *data, uint16_t len)
{
	uint8_t crc = 0x00;

	for (uint16_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
		}
	}

	return crc;
}

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

/**
 * @brief Low-level SPI transceive wrapper
 */
static int spi_transceive_dfu(const struct device *dev,
			      uint8_t *tx_buf, uint8_t *rx_buf, size_t len)
{
	const struct argos_spi_config *cfg = dev->config;

	struct spi_buf spi_tx = { .buf = tx_buf, .len = len };
	struct spi_buf spi_rx = { .buf = rx_buf, .len = len };
	struct spi_buf_set tx_set = { .buffers = &spi_tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &spi_rx, .count = 1 };

	return spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
}

/**
 * @brief Build and send a Protocol A+ command, then read response
 *
 * Protocol A+ uses a two-transaction model:
 * 1. Transaction 1: Send [0xAA][SEQ][CMD][LEN][DATA][CRC8]
 *    - RX during this transaction contains idle pattern (ignore)
 * 2. Wait the appropriate delay for the command
 * 3. Transaction 2: Send idle pattern [0xAA][0xAA]...
 *    - RX contains response: [0x55][SEQ][STATUS][LEN][DATA][CRC8]
 *
 * @param dev Device structure
 * @param cmd Command byte (0x30-0x3C)
 * @param payload TX payload (can be NULL if payload_len is 0)
 * @param payload_len Payload length (0-250)
 * @param response Buffer to store response data (can be NULL)
 * @param resp_len Pointer to response length (in: max size, out: actual size)
 * @param delay_ms Delay between TX and RX transactions
 * @return Status code (0x00 = OK) or negative errno on failure
 */
static int dfu_send_cmd(const struct device *dev, uint8_t cmd,
			const uint8_t *payload, uint8_t payload_len,
			uint8_t *response, uint8_t *resp_len, uint32_t delay_ms)
{
	struct argos_spi_data *data = dev->data;
	uint8_t tx_buf[DFU_LARGE_TX_SIZE];
	uint8_t rx_buf[DFU_LARGE_TX_SIZE];
	uint16_t idx = 0;
	int ret;

	if (payload_len > DFU_MAX_PAYLOAD) {
		LOG_ERR("Payload too large: %u", payload_len);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* ═══════════════════════════════════════════════════════════════
	 * Step 1: Build Protocol A+ request frame
	 * Format: [0xAA][SEQ][CMD][LEN][DATA...][CRC8]
	 * ═══════════════════════════════════════════════════════════════ */
	tx_buf[idx++] = DFU_MAGIC_REQUEST;     /* Magic */
	tx_buf[idx++] = dfu_seq_num;           /* Sequence number */
	tx_buf[idx++] = cmd;                   /* Command */
	tx_buf[idx++] = payload_len;           /* Length */

	if (payload && payload_len > 0) {
		memcpy(&tx_buf[idx], payload, payload_len);
		idx += payload_len;
	}

	/* CRC calculated over: MAGIC + SEQ + CMD + LEN + DATA (include magic) */
	tx_buf[idx] = crc8_ccitt(tx_buf, idx);
	idx++;

	/* Pad to transaction size */
	size_t tx_size = (idx < DFU_TRANSACTION_SIZE) ? DFU_TRANSACTION_SIZE : idx;
	if (idx < tx_size) {
		memset(&tx_buf[idx], DFU_IDLE_PATTERN, tx_size - idx);
	}

	LOG_DBG("TX[cmd=0x%02X seq=%u len=%u]: %02X %02X %02X %02X %02X ...",
		cmd, dfu_seq_num, payload_len,
		tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4]);

	/* ═══════════════════════════════════════════════════════════════
	 * Step 2: Transaction 1 - Send command
	 * (RX contains idle pattern, ignore it)
	 * ═══════════════════════════════════════════════════════════════ */
	memset(rx_buf, 0, sizeof(rx_buf));
	ret = spi_transceive_dfu(dev, tx_buf, rx_buf, tx_size);
	if (ret < 0) {
		LOG_ERR("SPI TX failed: %d", ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	LOG_DBG("RX1 (idle): %02X %02X %02X %02X",
		rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

	/* ═══════════════════════════════════════════════════════════════
	 * Step 3: Wait for slave to process command
	 * ═══════════════════════════════════════════════════════════════ */
	k_msleep(delay_ms);

	/* ═══════════════════════════════════════════════════════════════
	 * Step 4: Transaction 2 - Send idle pattern to read response
	 * ═══════════════════════════════════════════════════════════════ */
	memset(tx_buf, DFU_IDLE_PATTERN, DFU_TRANSACTION_SIZE);
	memset(rx_buf, 0, sizeof(rx_buf));
	ret = spi_transceive_dfu(dev, tx_buf, rx_buf, DFU_TRANSACTION_SIZE);
	if (ret < 0) {
		LOG_ERR("SPI RX failed: %d", ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	LOG_DBG("RX2: %02X %02X %02X %02X %02X %02X %02X %02X",
		rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
		rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

	/* ═══════════════════════════════════════════════════════════════
	 * Step 5: Parse response frame
	 * Format: [0x55][SEQ][STATUS][LEN][DATA...][CRC8]
	 * ═══════════════════════════════════════════════════════════════ */

	/* Find response magic (skip any idle bytes) */
	size_t offset = 0;
	while (offset < DFU_TRANSACTION_SIZE &&
	       rx_buf[offset] != DFU_MAGIC_RESPONSE) {
		if (rx_buf[offset] != DFU_IDLE_PATTERN && rx_buf[offset] != 0xFF) {
			LOG_WRN("Unexpected byte 0x%02X at offset %zu", rx_buf[offset], offset);
		}
		offset++;
	}

	if (offset >= DFU_TRANSACTION_SIZE || rx_buf[offset] != DFU_MAGIC_RESPONSE) {
		LOG_ERR("No response magic (0x55) found");
		k_mutex_unlock(&data->lock);
		return -ENODATA;
	}

	/* Parse response header */
	uint8_t rsp_magic = rx_buf[offset];
	uint8_t rsp_seq = rx_buf[offset + 1];
	uint8_t rsp_status = rx_buf[offset + 2];
	uint8_t rsp_len = rx_buf[offset + 3];

	(void)rsp_magic;  /* Verified above */
	(void)rsp_seq;    /* Could verify matches dfu_seq_num */

	LOG_DBG("Response: magic=0x%02X seq=%u status=0x%02X len=%u",
		rsp_magic, rsp_seq, rsp_status, rsp_len);

	/* Validate response length */
	if (rsp_len > DFU_MAX_PAYLOAD) {
		LOG_ERR("Response payload too large: %u", rsp_len);
		k_mutex_unlock(&data->lock);
		return -EMSGSIZE;
	}

	/* Verify CRC (calculated over MAGIC + SEQ + STATUS + LEN + DATA, include magic) */
	size_t crc_len = DFU_HEADER_SIZE + rsp_len;  /* Header (4 bytes) + data */
	uint8_t expected_crc = crc8_ccitt(&rx_buf[offset], crc_len);  /* Start from magic */
	uint8_t received_crc = rx_buf[offset + DFU_HEADER_SIZE + rsp_len];

	if (expected_crc != received_crc) {
		LOG_ERR("CRC mismatch: calc=0x%02X recv=0x%02X", expected_crc, received_crc);
		k_mutex_unlock(&data->lock);
		return -EILSEQ;
	}

	/* Copy response data */
	if (response && rsp_len > 0) {
		size_t copy_len = resp_len ? MIN(rsp_len, *resp_len) : rsp_len;
		memcpy(response, &rx_buf[offset + 4], copy_len);
	}
	if (resp_len) {
		*resp_len = rsp_len;
	}

	/* Increment sequence number for next command */
	dfu_seq_num++;

	k_mutex_unlock(&data->lock);

	return rsp_status;  /* 0x00 = OK */
}

/**
 * @brief Send command with automatic retry on recoverable errors
 *
 * Uses argos_is_recoverable() to determine if error should trigger retry.
 * Recoverable errors: PROT_BUSY (0x06) and PROT_FRAME_CRC_ERROR (0x10)
 */
static int dfu_send_with_retry(const struct device *dev, uint8_t cmd,
			       const uint8_t *payload, uint8_t payload_len,
			       uint8_t *response, uint8_t *resp_len,
			       uint32_t delay_ms)
{
	for (int retry = 0; retry < ARGOS_DFU_MAX_RETRIES; retry++) {
		int ret = dfu_send_cmd(dev, cmd, payload, payload_len,
				       response, resp_len, delay_ms);

		if (ret == PROT_OK) {
			return 0;  /* Success */
		}

		/* Check for recoverable errors using unified helper */
		if (argos_is_recoverable((uint8_t)ret) || ret == -EILSEQ) {
			LOG_WRN("Recoverable error (0x%02X), retry %d/%d",
				ret, retry + 1, ARGOS_DFU_MAX_RETRIES);
			k_msleep(50);
			continue;
		}

		/* Non-recoverable error - return immediately */
		if (ret < 0) {
			return ret;
		}

		/* Non-OK status that's not recoverable - return the status */
		return ret;
	}

	return -ETIMEDOUT;
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

	/* Reset DFU sequence number for new session */
	dfu_seq_num = 0;

	/* Wait for device to reset */
	k_msleep(ARGOS_DFU_RESET_WAIT_MS);

	return 0;
}

int argos_dfu_ping(const struct device *dev)
{
	uint8_t resp[16];
	uint8_t resp_len = sizeof(resp);
	int ret;

	LOG_DBG("Pinging DFU bootloader...");

	ret = dfu_send_cmd(dev, ARGOS_SPI_DFU_CMD_PING, NULL, 0,
			   resp, &resp_len, ARGOS_DFU_DELAY_PING_MS);
	if (ret < 0) {
		return ret;
	}

	if (ret != PROT_OK) {
		LOG_ERR("DFU ping failed: status 0x%02X", ret);
		return -EIO;
	}

	if (resp_len > 0) {
		/* Response contains version string (e.g., "BL_V1.0.0") */
		resp[MIN(resp_len, sizeof(resp) - 1)] = '\0';
		LOG_INF("Bootloader: %s", resp);
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
	uint8_t resp[32];
	uint8_t resp_len = sizeof(resp);
	int ret;

	if (!info) {
		return -EINVAL;
	}

	LOG_DBG("Getting bootloader info...");

	ret = dfu_send_cmd(dev, ARGOS_SPI_DFU_CMD_GET_INFO, NULL, 0,
			   resp, &resp_len, ARGOS_DFU_DELAY_GET_INFO_MS);
	if (ret < 0) {
		return ret;
	}

	if (ret != PROT_OK) {
		LOG_ERR("Get info failed: status 0x%02X", ret);
		return -EIO;
	}

	/* Parse response (17 bytes expected) */
	if (resp_len >= 15) {
		info->version_major = resp[0];
		info->version_minor = resp[1];
		info->version_patch = resp[2];
		/* Little-endian parsing */
		info->app_start_addr = resp[3] | (resp[4] << 8) |
				       (resp[5] << 16) | (resp[6] << 24);
		info->app_max_size = resp[7] | (resp[8] << 8) |
				     (resp[9] << 16) | (resp[10] << 24);
		info->page_size = resp[11] | (resp[12] << 8) |
				  (resp[13] << 16) | (resp[14] << 24);

		LOG_INF("Bootloader v%u.%u.%u, app_start=0x%08X, max_size=%u, page_size=%u",
			info->version_major, info->version_minor, info->version_patch,
			info->app_start_addr, info->app_max_size, info->page_size);
	} else {
		LOG_WRN("Bootloader info response too short: %u bytes", resp_len);
		memset(info, 0, sizeof(*info));
	}

	return 0;
}

int argos_dfu_erase(const struct device *dev)
{
	uint8_t resp[4];
	uint8_t resp_len = sizeof(resp);
	int ret;

	LOG_INF("Erasing application flash (this takes ~2-3 seconds)...");

	/*
	 * ERASE command takes ~2-3 seconds to complete.
	 * We use a long delay before reading the response.
	 */
	ret = dfu_send_cmd(dev, ARGOS_SPI_DFU_CMD_ERASE, NULL, 0,
			   resp, &resp_len, ARGOS_DFU_DELAY_ERASE_MS);
	if (ret < 0) {
		LOG_ERR("Erase failed: %d", ret);
		return ret;
	}

	if (ret != PROT_OK) {
		LOG_ERR("Erase failed: status 0x%02X", ret);
		return -EIO;
	}

	LOG_INF("Flash erased successfully");
	return 0;
}

int argos_dfu_erase_with_polling(const struct device *dev)
{
	uint8_t resp[4];
	uint8_t resp_len = sizeof(resp);
	struct argos_dfu_extended_status status;
	int ret;

	LOG_INF("Erasing flash with polling...");

	/* Send ERASE command with short initial delay */
	ret = dfu_send_cmd(dev, ARGOS_SPI_DFU_CMD_ERASE, NULL, 0,
			   resp, &resp_len, 100);

	/* Poll GET_STATUS every 100ms until complete */
	for (int i = 0; i < 50; i++) {  /* Max 5 seconds */
		k_msleep(100);

		ret = argos_dfu_get_extended_status(dev, &status);
		if (ret != 0) {
			continue;
		}

		switch (status.dfu_op_state) {
		case ARGOS_DFU_OP_ERASING:
			LOG_DBG("Still erasing...");
			continue;
		case ARGOS_DFU_OP_READY:
		case ARGOS_DFU_OP_IDLE:
			LOG_INF("Erase complete");
			return 0;
		case ARGOS_DFU_OP_ERROR:
			LOG_ERR("Erase error: 0x%02X", status.last_error);
			return -EIO;
		default:
			continue;
		}
	}

	return -ETIMEDOUT;
}

int argos_dfu_write_chunk(const struct device *dev, uint32_t addr,
			  const uint8_t *data, size_t len)
{
	uint8_t req_data[6];
	uint8_t resp[4];
	uint8_t resp_len;
	int ret;

	if (!data || len == 0 || len > ARGOS_DFU_CHUNK_SIZE) {
		return -EINVAL;
	}

	LOG_DBG("Writing %zu bytes at 0x%08X", len, addr);

	/* ═══════════════════════════════════════════════════════════════
	 * Step 1: WRITE_REQ - Send address and length (little-endian)
	 * Payload: [addr:4B][len:2B]
	 * ═══════════════════════════════════════════════════════════════ */
	req_data[0] = addr & 0xFF;
	req_data[1] = (addr >> 8) & 0xFF;
	req_data[2] = (addr >> 16) & 0xFF;
	req_data[3] = (addr >> 24) & 0xFF;
	req_data[4] = len & 0xFF;
	req_data[5] = (len >> 8) & 0xFF;

	resp_len = sizeof(resp);
	ret = dfu_send_with_retry(dev, ARGOS_SPI_DFU_CMD_WRITE_REQ, req_data, 6,
				  resp, &resp_len, ARGOS_DFU_DELAY_WRITE_REQ_MS);
	if (ret != 0) {
		LOG_ERR("WRITE_REQ failed at 0x%08X: %d", addr, ret);
		return (ret < 0) ? ret : -EIO;
	}

	/* ═══════════════════════════════════════════════════════════════
	 * Step 2: WRITE_DATA - Send actual data
	 * ═══════════════════════════════════════════════════════════════ */
	resp_len = sizeof(resp);
	ret = dfu_send_with_retry(dev, ARGOS_SPI_DFU_CMD_WRITE_DATA, data, len,
				  resp, &resp_len, ARGOS_DFU_DELAY_WRITE_DATA_MS);
	if (ret != 0) {
		LOG_ERR("WRITE_DATA failed at 0x%08X: %d", addr, ret);
		if (ret == PROT_FLASH_ERROR) {
			return -EIO;
		} else if (ret == PROT_ADDR_ERROR) {
			return -EFAULT;
		}
		return (ret < 0) ? ret : -EIO;
	}

	return 0;
}

int argos_dfu_read(const struct device *dev, uint32_t addr, uint8_t *data, size_t len)
{
	uint8_t req_data[6];
	uint8_t resp[4];
	uint8_t resp_len;
	int ret;

	if (!data || len == 0 || len > ARGOS_SPI_MAX_PAYLOAD) {
		return -EINVAL;
	}

	LOG_DBG("Reading %zu bytes from 0x%08X", len, addr);

	/* ═══════════════════════════════════════════════════════════════
	 * Step 1: READ_REQ - Send address and length (little-endian)
	 * ═══════════════════════════════════════════════════════════════ */
	req_data[0] = addr & 0xFF;
	req_data[1] = (addr >> 8) & 0xFF;
	req_data[2] = (addr >> 16) & 0xFF;
	req_data[3] = (addr >> 24) & 0xFF;
	req_data[4] = len & 0xFF;
	req_data[5] = (len >> 8) & 0xFF;

	resp_len = sizeof(resp);
	ret = dfu_send_with_retry(dev, ARGOS_SPI_DFU_CMD_READ_REQ, req_data, 6,
				  resp, &resp_len, ARGOS_DFU_DELAY_READ_REQ_MS);
	if (ret != 0) {
		LOG_ERR("READ_REQ failed: %d", ret);
		return (ret < 0) ? ret : -EIO;
	}

	/* ═══════════════════════════════════════════════════════════════
	 * Step 2: READ_DATA - Read the data
	 * ═══════════════════════════════════════════════════════════════ */
	resp_len = len;
	ret = dfu_send_with_retry(dev, ARGOS_SPI_DFU_CMD_READ_DATA, NULL, 0,
				  data, &resp_len, ARGOS_DFU_DELAY_READ_DATA_MS);
	if (ret != 0) {
		LOG_ERR("READ_DATA failed: %d", ret);
		return (ret < 0) ? ret : -EIO;
	}

	return 0;
}

int argos_dfu_verify(const struct device *dev, uint32_t crc32)
{
	uint8_t crc_data[4];
	uint8_t resp[4];
	uint8_t resp_len = sizeof(resp);
	int ret;

	LOG_INF("Verifying firmware CRC32: 0x%08X", crc32);

	/* Send CRC32 in little-endian format */
	crc_data[0] = crc32 & 0xFF;
	crc_data[1] = (crc32 >> 8) & 0xFF;
	crc_data[2] = (crc32 >> 16) & 0xFF;
	crc_data[3] = (crc32 >> 24) & 0xFF;

	ret = dfu_send_cmd(dev, ARGOS_SPI_DFU_CMD_VERIFY, crc_data, 4,
			   resp, &resp_len, ARGOS_DFU_DELAY_VERIFY_MS);
	if (ret < 0) {
		LOG_ERR("Verify command failed: %d", ret);
		return ret;
	}

	if (ret == PROT_OK) {
		LOG_INF("CRC verification passed!");
		return 0;
	} else if (ret == PROT_CRC_ERROR || ret == PROT_VERIFY_ERROR) {
		LOG_ERR("CRC verification failed - mismatch");
		return -EILSEQ;
	} else {
		LOG_ERR("Verify failed: status 0x%02X", ret);
		return -EIO;
	}
}

int argos_dfu_reset(const struct device *dev)
{
	uint8_t resp[4];
	uint8_t resp_len = sizeof(resp);
	int ret;

	LOG_INF("Resetting device...");

	ret = dfu_send_cmd(dev, ARGOS_SPI_DFU_CMD_RESET, NULL, 0,
			   resp, &resp_len, ARGOS_DFU_DELAY_RESET_MS);
	/* Device may not respond after reset - that's OK */
	if (ret < 0 && ret != -ENODATA) {
		LOG_WRN("Reset response: %d (device may have reset)", ret);
	}

	/* Wait for reset to complete */
	k_msleep(ARGOS_DFU_RESET_WAIT_MS);

	return 0;
}

int argos_dfu_jump(const struct device *dev)
{
	uint8_t resp[4];
	uint8_t resp_len = sizeof(resp);
	int ret;

	LOG_INF("Jumping to application...");

	ret = dfu_send_cmd(dev, ARGOS_SPI_DFU_CMD_JUMP, NULL, 0,
			   resp, &resp_len, ARGOS_DFU_DELAY_JUMP_MS);
	/* Device may not respond after jump - that's OK */
	if (ret < 0 && ret != -ENODATA) {
		LOG_WRN("Jump response: %d (device may have jumped)", ret);
	}

	/* Wait for jump/reset to complete */
	k_msleep(ARGOS_DFU_RESET_WAIT_MS * 2);

	return 0;
}

int argos_dfu_get_extended_status(const struct device *dev,
				  struct argos_dfu_extended_status *status)
{
	uint8_t resp[32];
	uint8_t resp_len = sizeof(resp);
	int ret;

	if (!status) {
		return -EINVAL;
	}

	ret = dfu_send_cmd(dev, ARGOS_SPI_DFU_CMD_GET_STATUS, NULL, 0,
			   resp, &resp_len, ARGOS_DFU_DELAY_GET_STATUS_MS);
	if (ret < 0) {
		return ret;
	}

	if (ret != PROT_OK) {
		LOG_ERR("Get extended status failed: 0x%02X", ret);
		return -EIO;
	}

	/* Parse 32-byte response */
	if (resp_len >= 32) {
		memcpy(status, resp, sizeof(*status));
	} else if (resp_len > 0) {
		memset(status, 0, sizeof(*status));
		memcpy(status, resp, resp_len);
	} else {
		memset(status, 0, sizeof(*status));
	}

	return 0;
}

int argos_dfu_get_status_spi(const struct device *dev, struct argos_dfu_status *status_out)
{
	struct argos_dfu_extended_status ext_status;
	int ret;

	if (!status_out) {
		return -EINVAL;
	}

	ret = argos_dfu_get_extended_status(dev, &ext_status);
	if (ret != 0) {
		return ret;
	}

	/* Map extended status to legacy format */
	switch (ext_status.dfu_op_state) {
	case ARGOS_DFU_OP_IDLE:
		status_out->state = ext_status.session_active ?
				    ARGOS_DFU_STATE_BOOTLOADER : ARGOS_DFU_STATE_IDLE;
		break;
	case ARGOS_DFU_OP_ERASING:
		status_out->state = ARGOS_DFU_STATE_ERASING;
		break;
	case ARGOS_DFU_OP_WRITING:
		status_out->state = ARGOS_DFU_STATE_WRITING;
		break;
	case ARGOS_DFU_OP_VERIFYING:
		status_out->state = ARGOS_DFU_STATE_VERIFYING;
		break;
	case ARGOS_DFU_OP_READY:
		status_out->state = ARGOS_DFU_STATE_BOOTLOADER;
		break;
	case ARGOS_DFU_OP_COMPLETE:
		status_out->state = ARGOS_DFU_STATE_COMPLETE;
		break;
	case ARGOS_DFU_OP_ERROR:
		status_out->state = ARGOS_DFU_STATE_ERROR;
		break;
	default:
		status_out->state = ARGOS_DFU_STATE_IDLE;
		break;
	}

	status_out->bytes_written = ext_status.received_bytes;
	status_out->total_bytes = 0;  /* Not available in extended status */
	status_out->last_error = ext_status.last_error;

	return 0;
}

int argos_dfu_abort(const struct device *dev)
{
	uint8_t resp[4];
	uint8_t resp_len = sizeof(resp);
	int ret;

	LOG_WRN("Aborting DFU operation...");

	ret = dfu_send_cmd(dev, ARGOS_SPI_DFU_CMD_ABORT, NULL, 0,
			   resp, &resp_len, ARGOS_DFU_DELAY_ABORT_MS);
	if (ret < 0) {
		LOG_ERR("Abort command failed: %d", ret);
		return ret;
	}

	if (ret != PROT_OK) {
		LOG_ERR("Abort failed: status 0x%02X", ret);
		return -EIO;
	}

	LOG_INF("DFU aborted");
	return 0;
}

int argos_dfu_set_header(const struct device *dev, const uint8_t *header)
{
	uint8_t resp[4];
	uint8_t resp_len = sizeof(resp);
	int ret;

	if (!header) {
		return -EINVAL;
	}

	LOG_DBG("Setting application header (%d bytes in 2 chunks)", ARGOS_DFU_HEADER_SIZE);

	/* Send header in two 128-byte chunks (256 > DFU_MAX_PAYLOAD of 250) */
	const size_t chunk_size = 128;

	/* Chunk 1: First 128 bytes */
	ret = dfu_send_with_retry(dev, ARGOS_SPI_DFU_CMD_SET_HEADER,
				  header, chunk_size,
				  resp, &resp_len, ARGOS_DFU_DELAY_SET_HEADER_MS);
	if (ret != 0) {
		LOG_ERR("Set header chunk 1 failed: %d", ret);
		return (ret < 0) ? ret : -EIO;
	}

	/* Chunk 2: Second 128 bytes */
	resp_len = sizeof(resp);
	ret = dfu_send_with_retry(dev, ARGOS_SPI_DFU_CMD_SET_HEADER,
				  header + chunk_size, chunk_size,
				  resp, &resp_len, ARGOS_DFU_DELAY_SET_HEADER_MS);
	if (ret != 0) {
		LOG_ERR("Set header chunk 2 failed: %d", ret);
		return (ret < 0) ? ret : -EIO;
	}

	LOG_DBG("Header set successfully (256 bytes)");
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
	LOG_INF("     (Protocol A+)");
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

	/* Step 6: Write application header (256 bytes) */
	LOG_INF("Step 5/7: Writing application header (256 bytes)...");
	if (size >= ARGOS_DFU_HEADER_SIZE) {
		/* First 256 bytes of firmware are the application header */
		ret = argos_dfu_set_header(dev, firmware);
		if (ret < 0) {
			LOG_ERR("Set header failed: %d", ret);
			argos_dfu_abort(dev);
			return ret;
		}
	} else {
		LOG_WRN("Firmware too small for header (%zu < 256), skipping header", size);
	}

	/* Step 7: Write firmware (starting from byte 256) */
	LOG_INF("Step 6/7: Writing firmware application code...");

	uint32_t addr = ARGOS_FLASH_APPLICATION;
	size_t offset = ARGOS_DFU_HEADER_SIZE;  /* Start after header */
	uint32_t last_progress = 0;

	if (size <= ARGOS_DFU_HEADER_SIZE) {
		LOG_WRN("Firmware is only header, no application code to write");
		offset = size;  /* Skip write loop */
	}

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

	size_t app_code_written = offset - ARGOS_DFU_HEADER_SIZE;
	LOG_INF("Application code written: %zu bytes (total with header: %zu bytes)",
		app_code_written, offset);

	/* Step 8: Verify CRC */
	LOG_INF("Step 7/8: Verifying CRC...");
	ret = argos_dfu_verify(dev, crc32);
	if (ret < 0) {
		LOG_ERR("CRC verification failed: %d", ret);
		argos_dfu_abort(dev);
		return ret;
	}

	/* Step 9: Jump to application */
	LOG_INF("Step 8/8: Starting application...");
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
