/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Argos SMD SPI Driver - Pipelined Single-Transaction Protocol
 *
 * This driver implements a pipelined protocol where:
 * - Each transaction is a fixed 64 bytes
 * - Master sends command, slave sends response to PREVIOUS command
 * - For immediate response: send CMD, then send NOP to get response
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

#include <argos-smd/argos_smd_spi.h>

LOG_MODULE_REGISTER(argos_smd_spi, CONFIG_ARGOS_SMD_LOG_LEVEL);

/**
 * @brief Calculate CRC-8 CCITT checksum
 */
uint8_t argos_spi_crc8_ccitt(const uint8_t *data, size_t len)
{
	uint8_t crc = 0x00;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

/**
 * @brief Build a Protocol A+ request frame into a 64-byte transaction buffer
 */
static size_t build_request_frame(uint8_t *buf, uint8_t seq, uint8_t cmd,
				  const uint8_t *data, size_t data_len)
{
	size_t idx = 0;

	/* Magic byte */
	buf[idx++] = ARGOS_SPI_MAGIC_REQUEST;

	/* Sequence number */
	buf[idx++] = seq;

	/* Command */
	buf[idx++] = cmd;

	/* Length */
	buf[idx++] = (uint8_t)data_len;

	/* Data payload */
	if (data && data_len > 0) {
		memcpy(&buf[idx], data, data_len);
		idx += data_len;
	}

	/* Calculate CRC over magic, seq, cmd, len, and data */
	buf[idx] = argos_spi_crc8_ccitt(buf, idx);
	idx++;

	/* Pad rest with 0xFF to fill transaction size */
	if (idx < ARGOS_SPI_TRANSACTION_SIZE) {
		memset(&buf[idx], 0xFF, ARGOS_SPI_TRANSACTION_SIZE - idx);
	}

	return idx;  /* Return actual frame length (not padded size) */
}

/**
 * @brief Parse a Protocol A+ response from RX buffer
 *
 * Handles finding the response magic byte, skipping idle patterns.
 */
static int parse_response_frame(const uint8_t *buf, size_t buf_len,
				struct argos_spi_response *resp)
{
	/* Find response magic byte, skip idle pattern (0xAA) and 0xFF */
	size_t offset = 0;
	while (offset < buf_len) {
		if (buf[offset] == ARGOS_SPI_MAGIC_RESPONSE) {
			break;
		}
		if (buf[offset] != ARGOS_SPI_IDLE_PATTERN && buf[offset] != 0xFF) {
			/* Unknown pattern - not a valid response */
			break;
		}
		offset++;
	}

	if (offset >= buf_len || buf[offset] != ARGOS_SPI_MAGIC_RESPONSE) {
		LOG_DBG("No response magic found (first byte: 0x%02X at offset %zu)",
			buf[0], offset);
		return -ENODATA;
	}

	/* Check minimum size for header */
	if (buf_len - offset < ARGOS_SPI_HEADER_SIZE + ARGOS_SPI_CRC_SIZE) {
		LOG_ERR("Response too short after offset %zu", offset);
		return -EINVAL;
	}

	resp->magic = buf[offset];
	resp->seq = buf[offset + 1];
	resp->status = buf[offset + 2];
	resp->len = buf[offset + 3];

	/* Validate length */
	if (resp->len > ARGOS_SPI_MAX_PAYLOAD) {
		LOG_ERR("Response payload too large: %u", resp->len);
		return -EMSGSIZE;
	}

	/* Check if we have enough data for payload + CRC */
	size_t expected_len = ARGOS_SPI_HEADER_SIZE + resp->len + ARGOS_SPI_CRC_SIZE;
	if (buf_len - offset < expected_len) {
		LOG_ERR("Response incomplete: got %zu, expected %zu",
			buf_len - offset, expected_len);
		return -EINVAL;
	}

	/* Copy payload */
	if (resp->len > 0) {
		memcpy(resp->data, &buf[offset + 4], resp->len);
	}

	/* Get and verify CRC */
	resp->crc = buf[offset + 4 + resp->len];
	uint8_t calc_crc = argos_spi_crc8_ccitt(&buf[offset], 4 + resp->len);
	if (calc_crc != resp->crc) {
		LOG_ERR("CRC mismatch: calculated 0x%02X, received 0x%02X",
			calc_crc, resp->crc);
		return -EILSEQ;
	}

	return 0;
}

/**
 * @brief Perform a single 64-byte SPI transaction
 */
static int spi_transaction_64(const struct spi_dt_spec *spi,
			      uint8_t *tx_buf, uint8_t *rx_buf)
{
	struct spi_buf spi_tx = { .buf = tx_buf, .len = ARGOS_SPI_TRANSACTION_SIZE };
	struct spi_buf spi_rx = { .buf = rx_buf, .len = ARGOS_SPI_TRANSACTION_SIZE };
	struct spi_buf_set tx_set = { .buffers = &spi_tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &spi_rx, .count = 1 };

	return spi_transceive_dt(spi, &tx_set, &rx_set);
}

int argos_spi_init(const struct device *dev)
{
	const struct argos_spi_config *cfg = dev->config;
	struct argos_spi_data *data = dev->data;

	/* Verify SPI bus is ready */
	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Initialize IRQ GPIO if configured */
	if (cfg->irq_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
			LOG_ERR("IRQ GPIO not ready");
			return -ENODEV;
		}
		int ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure IRQ GPIO: %d", ret);
			return ret;
		}
	}

	/* Initialize reset GPIO if configured */
	if (cfg->reset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}
		int ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure reset GPIO: %d", ret);
			return ret;
		}
	}

	/* Initialize mutex */
	k_mutex_init(&data->lock);

	/* Initialize sequence number */
	data->seq_num = 0;

	LOG_INF("Argos SMD SPI initialized (pipelined protocol, %d-byte transactions)",
		ARGOS_SPI_TRANSACTION_SIZE);

	return 0;
}

/**
 * @brief Check if RX buffer contains only idle pattern (no response ready)
 */
static bool is_rx_all_idle(const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (buf[i] != ARGOS_SPI_IDLE_PATTERN) {
			return false;
		}
	}
	return true;
}

/**
 * @brief Check if RX buffer contains only busy pattern (still processing)
 */
static bool is_rx_all_busy(const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (buf[i] != ARGOS_SPI_BUSY_PATTERN) {
			return false;
		}
	}
	return true;
}

/**
 * @brief Check if slave is not ready (idle or busy pattern)
 */
static bool is_slave_not_ready(const uint8_t *buf, size_t len)
{
	return is_rx_all_idle(buf, len) || is_rx_all_busy(buf, len);
}

/* Maximum retries when slave returns idle (still processing) */
#define FLASH_WRITE_MAX_RETRIES  10
#define FLASH_WRITE_RETRY_DELAY_MS  50

/**
 * @brief Internal transaction function with configurable delay
 */
static int argos_spi_transact_internal(const struct device *dev, uint8_t cmd,
				       const uint8_t *tx_data, size_t tx_len,
				       uint8_t *rx_data, size_t *rx_len,
				       uint8_t *status, uint32_t delay_ms)
{
	const struct argos_spi_config *cfg = dev->config;
	struct argos_spi_data *data = dev->data;
	int ret;
	int retries = 0;
	bool is_flash_write = (delay_ms >= ARGOS_SPI_FLASH_DELAY_MS);

	if (tx_len > ARGOS_SPI_MAX_PAYLOAD) {
		LOG_ERR("TX payload too large: %zu", tx_len);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/*
	 * Wait for STM32 to be ready from previous transaction.
	 * STM32 needs: detection timeout (10ms) + processing + DMA re-arm
	 */
	k_msleep(ARGOS_SPI_INTER_TX_DELAY_MS);

	/*
	 * Pipelined Protocol:
	 * Transaction 1: Send CMD, receive previous response (discard)
	 * Transaction 2+: Send NOP, receive response to CMD (retry if idle)
	 */

	/* Build command frame */
	build_request_frame(data->tx_buf, data->seq_num, cmd, tx_data, tx_len);

	LOG_DBG("TX1 [CMD 0x%02X seq %u]: %02X %02X %02X %02X %02X",
		cmd, data->seq_num,
		data->tx_buf[0], data->tx_buf[1], data->tx_buf[2],
		data->tx_buf[3], data->tx_buf[4]);

	/* Transaction 1: Send command */
	memset(data->rx_buf, 0, sizeof(data->rx_buf));
	ret = spi_transaction_64(&cfg->spi, data->tx_buf, data->rx_buf);
	if (ret < 0) {
		LOG_ERR("Transaction 1 failed: %d", ret);
		goto unlock;
	}

	LOG_DBG("RX1: %02X %02X %02X %02X %02X %02X %02X %02X",
		data->rx_buf[0], data->rx_buf[1], data->rx_buf[2], data->rx_buf[3],
		data->rx_buf[4], data->rx_buf[5], data->rx_buf[6], data->rx_buf[7]);

	/* Delay for STM32 to process command */
	k_msleep(delay_ms);

	/* Build NOP frame to retrieve response */
	data->seq_num++;

	/*
	 * Transaction 2+: Send NOP, receive response
	 * Retry loop for flash write operations when slave is not ready
	 */
	do {
		build_request_frame(data->tx_buf, data->seq_num, ARGOS_SPI_CMD_NOP, NULL, 0);

		LOG_DBG("TX2 [NOP seq %u]: %02X %02X %02X %02X %02X",
			data->seq_num,
			data->tx_buf[0], data->tx_buf[1], data->tx_buf[2],
			data->tx_buf[3], data->tx_buf[4]);

		/* Transaction 2: Send NOP, receive response */
		memset(data->rx_buf, 0, sizeof(data->rx_buf));
		ret = spi_transaction_64(&cfg->spi, data->tx_buf, data->rx_buf);
		if (ret < 0) {
			LOG_ERR("Transaction 2 failed: %d", ret);
			goto unlock;
		}

		LOG_DBG("RX2: %02X %02X %02X %02X %02X %02X %02X %02X",
			data->rx_buf[0], data->rx_buf[1], data->rx_buf[2], data->rx_buf[3],
			data->rx_buf[4], data->rx_buf[5], data->rx_buf[6], data->rx_buf[7]);

		/* For flash write operations, retry if slave is not ready (IDLE or BUSY) */
		if (is_flash_write && is_slave_not_ready(data->rx_buf, 8) &&
		    retries < FLASH_WRITE_MAX_RETRIES) {
			retries++;
			if (is_rx_all_busy(data->rx_buf, 8)) {
				LOG_DBG("Slave BUSY (processing), retry %d/%d",
					retries, FLASH_WRITE_MAX_RETRIES);
			} else {
				LOG_DBG("Slave IDLE (not ready), retry %d/%d",
					retries, FLASH_WRITE_MAX_RETRIES);
			}
			k_msleep(FLASH_WRITE_RETRY_DELAY_MS);
			data->seq_num++;
			continue;  /* Retry NOP transaction */
		}
		break;  /* Got valid response or max retries reached */
	} while (retries <= FLASH_WRITE_MAX_RETRIES);

	/* Parse response */
	struct argos_spi_response resp;
	ret = parse_response_frame(data->rx_buf, ARGOS_SPI_TRANSACTION_SIZE, &resp);
	if (ret < 0) {
		LOG_ERR("Failed to parse response: %d", ret);
		goto unlock;
	}

	LOG_DBG("Response: status 0x%02X, len %u", resp.status, resp.len);

	/* Return status */
	if (status) {
		*status = resp.status;
	}

	/* Return data */
	if (rx_data && rx_len && resp.len > 0) {
		size_t copy_len = MIN(resp.len, *rx_len);
		memcpy(rx_data, resp.data, copy_len);
		*rx_len = copy_len;
	} else if (rx_len) {
		*rx_len = resp.len;
	}

	/* Increment sequence number for next transaction */
	data->seq_num++;

	ret = 0;

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

int argos_spi_transact(const struct device *dev, uint8_t cmd,
		       const uint8_t *tx_data, size_t tx_len,
		       uint8_t *rx_data, size_t *rx_len, uint8_t *status)
{
	return argos_spi_transact_internal(dev, cmd, tx_data, tx_len,
					   rx_data, rx_len, status,
					   ARGOS_SPI_PIPELINE_DELAY_MS);
}

/**
 * @brief 2-phase write helper for STM32 flash write operations
 *
 * The STM32 firmware requires a 2-phase protocol for write operations:
 * 1. Transaction 1: Send *_REQ command to prepare STM32 RX buffer
 * 2. Transaction 2: Send WRITE_* command with actual data
 *
 * @param dev Pointer to device structure
 * @param req_cmd Request command code (e.g., ARGOS_SPI_CMD_WRITE_LPM_REQ)
 * @param write_cmd Write command code (e.g., ARGOS_SPI_CMD_WRITE_LPM)
 * @param data Data to write
 * @param len Length of data
 * @return 0 on success, negative errno on failure
 */
static int argos_spi_write_2phase(const struct device *dev,
				  uint8_t req_cmd, uint8_t write_cmd,
				  const uint8_t *data, size_t len)
{
	uint8_t status;
	int ret;

	LOG_DBG("2-phase write: REQ=0x%02X, WRITE=0x%02X, len=%zu",
		req_cmd, write_cmd, len);

	/* Phase 1: Send REQ command to prepare STM32 for data reception */
	ret = argos_spi_transact(dev, req_cmd, NULL, 0, NULL, NULL, &status);
	if (ret < 0) {
		LOG_ERR("Phase 1 (REQ 0x%02X) transaction failed: %d", req_cmd, ret);
		return ret;
	}
	if (status != PROT_OK) {
		LOG_ERR("Phase 1 (REQ 0x%02X) rejected: status 0x%02X", req_cmd, status);
		return -EIO;
	}

	/* Inter-transaction delay for STM32 to prepare RX buffer */
	k_msleep(1);

	/* Phase 2: Send WRITE command with data (use flash delay for NVM write) */
	ret = argos_spi_transact_internal(dev, write_cmd, data, len,
					  NULL, NULL, &status,
					  ARGOS_SPI_FLASH_DELAY_MS);
	if (ret < 0) {
		LOG_ERR("Phase 2 (WRITE 0x%02X) transaction failed: %d", write_cmd, ret);
		return ret;
	}
	if (status != PROT_OK) {
		LOG_ERR("Phase 2 (WRITE 0x%02X) failed: status 0x%02X", write_cmd, status);
		return -EIO;
	}

	LOG_DBG("2-phase write completed successfully");
	return 0;
}

int argos_spi_send_only(const struct device *dev, uint8_t cmd,
			 const uint8_t *tx_data, size_t tx_len)
{
	const struct argos_spi_config *cfg = dev->config;
	struct argos_spi_data *data = dev->data;
	int ret;

	if (tx_len > ARGOS_SPI_MAX_PAYLOAD) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Build and send request frame */
	build_request_frame(data->tx_buf, data->seq_num, cmd, tx_data, tx_len);

	LOG_DBG("Sending cmd 0x%02X (no response expected)", cmd);

	/* Single transaction - no response expected */
	memset(data->rx_buf, 0, sizeof(data->rx_buf));
	ret = spi_transaction_64(&cfg->spi, data->tx_buf, data->rx_buf);
	if (ret < 0) {
		LOG_ERR("SPI transaction failed: %d", ret);
	} else {
		data->seq_num++;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

int argos_spi_ping(const struct device *dev)
{
	uint8_t status;
	int ret;

	LOG_DBG("Pinging device");

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_PING, NULL, 0, NULL, NULL, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_SPI_RSP_OK) {
		LOG_ERR("Ping failed: status 0x%02X", status);
		return -EIO;
	}

	return 0;
}

int argos_spi_get_version(const struct device *dev, char *version, size_t *version_len)
{
	uint8_t status;
	int ret;

	if (!version || !version_len || *version_len == 0) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_READ_VERSION, NULL, 0,
				  (uint8_t *)version, version_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_SPI_RSP_OK) {
		LOG_ERR("Get version failed: status 0x%02X", status);
		return -EIO;
	}

	/* Null-terminate the string */
	if (*version_len < 32) {
		version[*version_len] = '\0';
	}

	return 0;
}

int argos_spi_reset(const struct device *dev)
{
	const struct argos_spi_config *cfg = dev->config;
	int ret;

	if (cfg->reset_gpio.port == NULL) {
		LOG_WRN("Reset GPIO not configured");
		return -ENOTSUP;
	}

	LOG_INF("Performing hardware reset...");

	/* Assert reset (active low) */
	ret = gpio_pin_set_dt(&cfg->reset_gpio, 1);
	if (ret < 0) {
		LOG_ERR("Failed to assert reset: %d", ret);
		return ret;
	}

	/* Hold reset for minimum pulse width */
	k_msleep(ARGOS_SPI_RETRY_DELAY_MS);

	/* Release reset */
	ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);
	if (ret < 0) {
		LOG_ERR("Failed to release reset: %d", ret);
		return ret;
	}

	/* Wait for module to boot */
	k_msleep(ARGOS_SPI_BOOT_DELAY_MS);

	LOG_INF("Hardware reset complete");
	return 0;
}

int argos_spi_diagnostic(const struct device *dev)
{
	const struct argos_spi_config *cfg = dev->config;
	struct argos_spi_data *data = dev->data;
	int ret;

	LOG_INF("=== SPI DIAGNOSTIC (Pipelined Protocol) ===");
	LOG_INF("Transaction size: %d bytes", ARGOS_SPI_TRANSACTION_SIZE);
	LOG_INF("SPI frequency: %u Hz", cfg->spi.config.frequency);

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Wait for STM32 to be ready from previous transaction */
	k_msleep(ARGOS_SPI_INTER_TX_DELAY_MS);

	/* Test 1: Basic transaction */
	LOG_INF("");
	LOG_INF("--- TEST 1: Basic 64-byte transaction ---");

	memset(data->tx_buf, 0xFF, ARGOS_SPI_TRANSACTION_SIZE);
	memset(data->rx_buf, 0xAB, ARGOS_SPI_TRANSACTION_SIZE);

	ret = spi_transaction_64(&cfg->spi, data->tx_buf, data->rx_buf);
	if (ret < 0) {
		LOG_ERR("Transaction failed: %d", ret);
	} else {
		LOG_INF("RX: %02X %02X %02X %02X %02X %02X %02X %02X",
			data->rx_buf[0], data->rx_buf[1], data->rx_buf[2], data->rx_buf[3],
			data->rx_buf[4], data->rx_buf[5], data->rx_buf[6], data->rx_buf[7]);

		if (data->rx_buf[0] == ARGOS_SPI_IDLE_PATTERN) {
			LOG_INF("Got idle pattern (0xAA) - STM32 responding!");
		} else if (data->rx_buf[0] == 0xAB) {
			LOG_ERR("RX unchanged - SPI not working");
		}
	}

	/* Wait before next transaction */
	k_msleep(ARGOS_SPI_INTER_TX_DELAY_MS);

	/* Test 2: PING with pipelined protocol - use actual sequence numbers */
	LOG_INF("");
	LOG_INF("--- TEST 2: PING (Pipelined) ---");

	/* Build PING frame with current sequence number */
	uint8_t ping_seq = data->seq_num++;
	build_request_frame(data->tx_buf, ping_seq, ARGOS_SPI_CMD_PING, NULL, 0);
	LOG_INF("TX1 [PING seq %u]: %02X %02X %02X %02X %02X",
		ping_seq,
		data->tx_buf[0], data->tx_buf[1], data->tx_buf[2],
		data->tx_buf[3], data->tx_buf[4]);

	memset(data->rx_buf, 0, sizeof(data->rx_buf));
	ret = spi_transaction_64(&cfg->spi, data->tx_buf, data->rx_buf);
	if (ret < 0) {
		LOG_ERR("TX1 failed: %d", ret);
		goto done;
	}

	LOG_INF("RX1: %02X %02X %02X %02X %02X %02X %02X %02X",
		data->rx_buf[0], data->rx_buf[1], data->rx_buf[2], data->rx_buf[3],
		data->rx_buf[4], data->rx_buf[5], data->rx_buf[6], data->rx_buf[7]);

	k_msleep(ARGOS_SPI_PIPELINE_DELAY_MS);

	/* Build NOP frame with next sequence number */
	uint8_t nop_seq = data->seq_num++;
	build_request_frame(data->tx_buf, nop_seq, ARGOS_SPI_CMD_NOP, NULL, 0);
	LOG_INF("TX2 [NOP seq %u]: %02X %02X %02X %02X %02X",
		nop_seq,
		data->tx_buf[0], data->tx_buf[1], data->tx_buf[2],
		data->tx_buf[3], data->tx_buf[4]);

	memset(data->rx_buf, 0, sizeof(data->rx_buf));
	ret = spi_transaction_64(&cfg->spi, data->tx_buf, data->rx_buf);
	if (ret < 0) {
		LOG_ERR("TX2 failed: %d", ret);
		goto done;
	}

	LOG_INF("RX2: %02X %02X %02X %02X %02X %02X %02X %02X",
		data->rx_buf[0], data->rx_buf[1], data->rx_buf[2], data->rx_buf[3],
		data->rx_buf[4], data->rx_buf[5], data->rx_buf[6], data->rx_buf[7]);

	/* Check for response */
	struct argos_spi_response resp;
	ret = parse_response_frame(data->rx_buf, ARGOS_SPI_TRANSACTION_SIZE, &resp);
	if (ret == 0) {
		LOG_INF("SUCCESS! Got response: magic=0x%02X status=0x%02X len=%u",
			resp.magic, resp.status, resp.len);
	} else {
		LOG_WRN("No valid response found (err=%d)", ret);
	}

done:
	k_mutex_unlock(&data->lock);
	return ret;
}

/*
 * High-level API functions
 */

int argos_spi_get_sn(const struct device *dev, char *sn, size_t *sn_len)
{
	uint8_t status;
	int ret;

	if (!sn || !sn_len || *sn_len == 0) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_READ_SN, NULL, 0,
				 (uint8_t *)sn, sn_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_SPI_RSP_OK) {
		LOG_ERR("GET_SN failed: status 0x%02X", status);
		return -EIO;
	}

	/* Null-terminate the string */
	if (*sn_len < 32) {
		sn[*sn_len] = '\0';
	}

	LOG_DBG("SN: %s (len=%zu)", sn, *sn_len);
	return 0;
}

int argos_spi_get_id(const struct device *dev, uint8_t *id, size_t *id_len)
{
	uint8_t status;
	int ret;

	if (!id || !id_len || *id_len == 0) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_READ_ID, NULL, 0,
				 id, id_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_SPI_RSP_OK) {
		LOG_ERR("GET_ID failed: status 0x%02X", status);
		return -EIO;
	}

	LOG_DBG("ID: %02X %02X %02X %02X (len=%zu)",
		id[0], id[1], id[2], id[3], *id_len);
	return 0;
}

int argos_spi_set_id(const struct device *dev, const uint8_t *id, size_t id_len)
{
	if (!id || id_len == 0) {
		return -EINVAL;
	}

	LOG_DBG("Setting ID (%zu bytes)", id_len);

	/* 2-phase write: REQ (0x20) then WRITE (0x21) */
	int ret = argos_spi_write_2phase(dev,
					 ARGOS_SPI_CMD_WRITE_ID_REQ,
					 ARGOS_SPI_CMD_WRITE_ID,
					 id, id_len);
	if (ret < 0) {
		LOG_ERR("SET_ID failed: %d", ret);
		return ret;
	}

	LOG_DBG("ID set successfully");
	return 0;
}

int argos_spi_get_addr(const struct device *dev, uint8_t *addr, size_t *addr_len)
{
	uint8_t status;
	int ret;

	if (!addr || !addr_len || *addr_len == 0) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_READ_ADDR, NULL, 0,
				 addr, addr_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_SPI_RSP_OK) {
		LOG_ERR("GET_ADDR failed: status 0x%02X", status);
		return -EIO;
	}

	LOG_DBG("ADDR: %02X %02X %02X %02X (len=%zu)",
		addr[0], addr[1], addr[2], addr[3], *addr_len);
	return 0;
}

int argos_spi_set_addr(const struct device *dev, const uint8_t *addr, size_t addr_len)
{
	if (!addr || addr_len == 0) {
		return -EINVAL;
	}

	LOG_DBG("Setting ADDR (%zu bytes)", addr_len);

	/* 2-phase write: REQ (0x22) then WRITE (0x23) */
	int ret = argos_spi_write_2phase(dev,
					 ARGOS_SPI_CMD_WRITE_ADDR_REQ,
					 ARGOS_SPI_CMD_WRITE_ADDR,
					 addr, addr_len);
	if (ret < 0) {
		LOG_ERR("SET_ADDR failed: %d", ret);
		return ret;
	}

	LOG_DBG("ADDR set successfully");
	return 0;
}

int argos_spi_get_secret_key(const struct device *dev, uint8_t *key, size_t *key_len)
{
	uint8_t status;
	int ret;

	if (!key || !key_len || *key_len == 0) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_READ_SECKEY, NULL, 0,
				 key, key_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_SPI_RSP_OK) {
		LOG_ERR("GET_SECRET_KEY failed with status: 0x%02X", status);
		/* When status != OK, the payload contains error code, not valid data */
		return -EIO;
	}

	LOG_DBG("SECRET_KEY received (len=%zu)", *key_len);
	return 0;
}

int argos_spi_set_secret_key(const struct device *dev, const uint8_t *key, size_t key_len)
{
	if (!key || key_len == 0) {
		return -EINVAL;
	}

	LOG_DBG("Setting SECRET_KEY (%zu bytes)", key_len);

	/* 2-phase write: REQ (0x25) then WRITE (0x26) */
	int ret = argos_spi_write_2phase(dev,
					 ARGOS_SPI_CMD_WRITE_SECKEY_REQ,
					 ARGOS_SPI_CMD_WRITE_SECKEY,
					 key, key_len);
	if (ret < 0) {
		LOG_ERR("SET_SECRET_KEY failed: %d", ret);
		return ret;
	}

	LOG_DBG("SECRET_KEY set successfully");
	return 0;
}

int argos_spi_get_rconf(const struct device *dev, uint8_t *rconf, size_t *rconf_len)
{
	uint8_t status;
	int ret;

	if (!rconf || !rconf_len || *rconf_len == 0) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_READ_RCONF, NULL, 0,
				 rconf, rconf_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_SPI_RSP_OK) {
		LOG_ERR("GET_RCONF failed with status: 0x%02X", status);
		/* When status != OK, the payload contains error code, not valid data */
		return -EIO;
	}

	LOG_DBG("RCONF received (len=%zu)", *rconf_len);
	return 0;
}

int argos_spi_set_rconf(const struct device *dev, const uint8_t *rconf, size_t rconf_len)
{
	if (!rconf || rconf_len == 0) {
		return -EINVAL;
	}

	LOG_DBG("Setting RCONF (%zu bytes)", rconf_len);

	/* 2-phase write: REQ (0x0B) then WRITE (0x0C) */
	int ret = argos_spi_write_2phase(dev,
					 ARGOS_SPI_CMD_WRITE_RCONF_REQ,
					 ARGOS_SPI_CMD_WRITE_RCONF,
					 rconf, rconf_len);
	if (ret < 0) {
		LOG_ERR("SET_RCONF failed: %d", ret);
		return ret;
	}

	LOG_DBG("RCONF set successfully");
	return 0;
}

int argos_spi_get_kmac(const struct device *dev, uint8_t *kmac)
{
	uint8_t status;
	uint8_t resp[4];
	size_t resp_len = sizeof(resp);
	int ret;

	if (!kmac) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_READ_KMAC, NULL, 0,
				 resp, &resp_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_SPI_RSP_OK) {
		LOG_ERR("GET_KMAC failed with status: 0x%02X", status);
		return -EIO;
	}

	/* KMAC value is in first byte of response */
	*kmac = (resp_len > 0) ? resp[0] : 0;
	LOG_DBG("KMAC received: %u", *kmac);
	return 0;
}

int argos_spi_set_kmac(const struct device *dev, uint8_t kmac)
{
	LOG_DBG("Setting KMAC to %u", kmac);

	/* 2-phase write: REQ (0x0F) then WRITE (0x10) */
	int ret = argos_spi_write_2phase(dev,
					 ARGOS_SPI_CMD_WRITE_KMAC_REQ,
					 ARGOS_SPI_CMD_WRITE_KMAC,
					 &kmac, 1);
	if (ret < 0) {
		LOG_ERR("SET_KMAC failed: %d", ret);
		return ret;
	}

	LOG_DBG("KMAC set successfully to %u", kmac);
	return 0;
}

int argos_spi_get_mac_status(const struct device *dev, uint8_t *mac_status)
{
	uint8_t status;
	uint8_t resp[4];
	size_t resp_len = sizeof(resp);
	int ret;

	if (!mac_status) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_MAC_STATUS, NULL, 0,
				 resp, &resp_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != PROT_OK) {
		LOG_ERR("GET_MAC_STATUS failed: status 0x%02X", status);
		return -EIO;
	}

	/* MAC status is in first byte of response */
	*mac_status = (resp_len > 0) ? resp[0] : MAC_UNKNOWN;
	LOG_DBG("MAC status: 0x%02X", *mac_status);
	return 0;
}

int argos_spi_get_spimac_state(const struct device *dev, uint8_t *state, size_t *state_len)
{
	uint8_t status;
	int ret;

	if (!state || !state_len || *state_len < 2) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_READ_SPIMAC_STATE, NULL, 0,
				 state, state_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != PROT_OK) {
		LOG_ERR("GET_SPIMAC_STATE failed: status 0x%02X", status);
		return -EIO;
	}

	LOG_DBG("SPIMAC state: 0x%02X 0x%02X (len=%zu)", state[0], state[1], *state_len);
	return 0;
}

int argos_spi_wait_tx_complete(const struct device *dev, k_timeout_t timeout)
{
	uint8_t mac_status;
	int64_t start = k_uptime_get();
	int64_t timeout_ms = k_ticks_to_ms_floor64(timeout.ticks);
	int ret;
	int consecutive_errors = 0;
	const int max_consecutive_errors = 3;
	uint32_t backoff_delay = ARGOS_TIMING_POLL_MS;

	LOG_DBG("Waiting for TX complete (timeout=%lld ms)", timeout_ms);

	while ((k_uptime_get() - start) < timeout_ms) {
		ret = argos_spi_get_mac_status(dev, &mac_status);
		if (ret < 0) {
			consecutive_errors++;
			LOG_WRN("MAC status read error: %d (consecutive: %d)", ret, consecutive_errors);

			/* If multiple consecutive errors, try SPI resync */
			if (consecutive_errors >= max_consecutive_errors) {
				LOG_WRN("Multiple consecutive errors, attempting SPI resync");
				argos_spi_sync(dev);
				consecutive_errors = 0;
				backoff_delay = ARGOS_TIMING_POLL_MS; /* Reset backoff */
			} else {
				/* Exponential backoff for transient errors */
				backoff_delay = MIN(backoff_delay * 2, 2000);
			}

			k_msleep(backoff_delay);
			continue;
		}

		/* Success - reset error counter and backoff */
		consecutive_errors = 0;
		backoff_delay = ARGOS_TIMING_POLL_MS;

		/* Check TX result using unified helpers */
		if (argos_is_tx_complete(mac_status)) {
			LOG_INF("TX complete: MAC status 0x%02X", mac_status);
			return 0;
		}

		if (argos_is_tx_failed(mac_status)) {
			LOG_ERR("TX failed: MAC status 0x%02X", mac_status);
			return -EIO;
		}

		if (argos_is_tx_pending(mac_status)) {
			/* Still in progress - use full polling interval during RF TX */
			k_msleep(ARGOS_TIMING_POLL_MS);
			continue;
		}

		/* Other status (MAC_OK, MAC_RX_RECEIVED, etc.) - continue polling */
		k_msleep(ARGOS_TIMING_POLL_MS);
	}

	LOG_ERR("TX timeout after %lld ms", timeout_ms);
	return -ETIMEDOUT;
}

int argos_spi_sync(const struct device *dev)
{
	const struct argos_spi_config *cfg = dev->config;
	struct argos_spi_data *data = dev->data;
	int ret;
	int sync_count = 0;
	const int max_sync_attempts = 5;

	LOG_INF("Synchronizing SPI protocol...");

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Reset sequence number to start fresh */
	data->seq_num = 0;

	/*
	 * Send multiple dummy (0xFF) transactions to:
	 * 1. Complete any pending DMA transaction on STM32
	 * 2. Flush old data from TX buffer
	 * 3. Reset protocol state on slave
	 *
	 * The STM32 will see these as invalid frames (no magic byte)
	 * and should reset its protocol state after a few errors.
	 */
	for (int i = 0; i < max_sync_attempts; i++) {
		/* Fill TX with 0xFF (invalid pattern - not a valid frame) */
		memset(data->tx_buf, 0xFF, ARGOS_SPI_TRANSACTION_SIZE);
		memset(data->rx_buf, 0, sizeof(data->rx_buf));

		ret = spi_transaction_64(&cfg->spi, data->tx_buf, data->rx_buf);
		if (ret < 0) {
			LOG_ERR("Sync transaction %d failed: %d", i, ret);
			continue;
		}

		LOG_DBG("Sync %d RX: %02X %02X %02X %02X %02X %02X %02X %02X",
			i,
			data->rx_buf[0], data->rx_buf[1], data->rx_buf[2], data->rx_buf[3],
			data->rx_buf[4], data->rx_buf[5], data->rx_buf[6], data->rx_buf[7]);

		/* Check if we're getting consistent idle pattern (0xAA) */
		if (is_rx_all_idle(data->rx_buf, 8)) {
			sync_count++;
			if (sync_count >= 2) {
				LOG_INF("SPI synchronized after %d transactions", i + 1);
				break;
			}
		} else {
			/* Got something else - still flushing old data */
			sync_count = 0;
		}

		/* Small delay between sync attempts */
		k_msleep(ARGOS_SPI_DETECT_TIMEOUT_MS);
	}

	k_mutex_unlock(&data->lock);

	if (sync_count < 2) {
		LOG_WRN("SPI sync incomplete, proceeding anyway");
	}

	return 0;
}

int argos_spi_write_tx(const struct device *dev, const uint8_t *data, size_t len)
{
	uint8_t status;
	uint8_t size_data[2];
	int ret;

	if (!data || len == 0 || len > ARGOS_SPI_MAX_PAYLOAD) {
		return -EINVAL;
	}

	LOG_DBG("Sending TX data (%zu bytes)", len);

	/* Step 1: WRITE_TX_REQ (0x14) - Prepare for TX */
	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_WRITE_TX_REQ, NULL, 0,
				 NULL, NULL, &status);
	if (ret < 0) {
		LOG_ERR("WRITE_TX_REQ failed: %d", ret);
		return ret;
	}
	if (status != PROT_OK) {
		LOG_ERR("WRITE_TX_REQ rejected: status 0x%02X", status);
		return -EIO;
	}

	/* Step 2: WRITE_TX_SIZE (0x15) - Send size (little-endian) */
	size_data[0] = len & 0xFF;
	size_data[1] = (len >> 8) & 0xFF;
	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_WRITE_TX_SIZE, size_data, 2,
				 NULL, NULL, &status);
	if (ret < 0) {
		LOG_ERR("WRITE_TX_SIZE failed: %d", ret);
		return ret;
	}
	if (status != PROT_OK) {
		LOG_ERR("WRITE_TX_SIZE rejected: status 0x%02X", status);
		return -EIO;
	}

	/* Step 3: WRITE_TX (0x16) - Send actual data */
	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_WRITE_TX, data, len,
				 NULL, NULL, &status);
	if (ret < 0) {
		LOG_ERR("WRITE_TX failed: %d", ret);
		return ret;
	}
	if (status != PROT_OK) {
		LOG_ERR("WRITE_TX rejected: status 0x%02X", status);
		return -EIO;
	}

	/*
	 * CRITICAL: Wait for STM32 to finish processing TX data.
	 * The WRITE_TX handler does async work (FIFO, MAC queue) after sending ACK.
	 * DMA is re-armed only after handler returns. Give STM32 time to complete.
	 */
	k_msleep(ARGOS_SPI_POST_TX_DELAY_MS);

	LOG_INF("TX data queued (%zu bytes), use wait_tx_complete() to poll", len);
	return 0;
}

/*
 * Additional API functions - matching UART driver functionality
 */

int argos_spi_get_lpm(const struct device *dev, uint8_t *lpm)
{
	uint8_t status;
	uint8_t resp[4];
	size_t resp_len = sizeof(resp);
	int ret;

	if (!lpm) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_READ_LPM, NULL, 0,
				 resp, &resp_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != PROT_OK) {
		LOG_ERR("GET_LPM failed: status 0x%02X", status);
		return -EIO;
	}

	*lpm = (resp_len > 0) ? resp[0] : 0;
	LOG_DBG("LPM: %u", *lpm);
	return 0;
}

int argos_spi_set_lpm(const struct device *dev, uint8_t lpm)
{
	LOG_DBG("Setting LPM to %u", lpm);

	/* 2-phase write: REQ (0x12) then WRITE (0x13) */
	int ret = argos_spi_write_2phase(dev,
					 ARGOS_SPI_CMD_WRITE_LPM_REQ,
					 ARGOS_SPI_CMD_WRITE_LPM,
					 &lpm, 1);
	if (ret < 0) {
		LOG_ERR("SET_LPM failed: %d", ret);
		return ret;
	}

	LOG_DBG("LPM set to %u", lpm);
	return 0;
}

int argos_spi_get_tcxo_wu(const struct device *dev, uint8_t *tcxo_wu, size_t *tcxo_len)
{
	uint8_t status;
	int ret;

	if (!tcxo_wu || !tcxo_len || *tcxo_len == 0) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_READ_TCXO_WU, NULL, 0,
				 tcxo_wu, tcxo_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != PROT_OK) {
		LOG_ERR("GET_TCXO_WU failed: status 0x%02X", status);
		return -EIO;
	}

	LOG_DBG("TCXO warmup received (len=%zu)", *tcxo_len);
	return 0;
}

int argos_spi_set_tcxo_wu(const struct device *dev, const uint8_t *tcxo_wu, size_t tcxo_len)
{
	if (!tcxo_wu || tcxo_len == 0) {
		return -EINVAL;
	}

	LOG_DBG("Setting TCXO_WU (%zu bytes)", tcxo_len);

	/* 2-phase write: REQ (0x29) then WRITE (0x2A) */
	int ret = argos_spi_write_2phase(dev,
					 ARGOS_SPI_CMD_WRITE_TCXOWU_REQ,
					 ARGOS_SPI_CMD_WRITE_TCXOWU,
					 tcxo_wu, tcxo_len);
	if (ret < 0) {
		LOG_ERR("SET_TCXO_WU failed: %d", ret);
		return ret;
	}

	LOG_DBG("TCXO warmup set successfully");
	return 0;
}

int argos_spi_save_rconf(const struct device *dev)
{
	uint8_t status;
	int ret;

	LOG_DBG("Saving radio config to NVM...");

	/* Save operation writes to flash */
	ret = argos_spi_transact_internal(dev, ARGOS_SPI_CMD_SAVE_RCONF, NULL, 0,
					  NULL, NULL, &status,
					  ARGOS_SPI_FLASH_DELAY_MS);
	if (ret < 0) {
		return ret;
	}

	if (status != PROT_OK) {
		LOG_ERR("SAVE_RCONF failed: status 0x%02X", status);
		return -EIO;
	}

	LOG_INF("Radio config saved to NVM");
	return 0;
}

int argos_spi_get_rconf_raw(const struct device *dev, uint8_t *rconf_raw, size_t *rconf_len)
{
	uint8_t status;
	int ret;

	if (!rconf_raw || !rconf_len || *rconf_len < 16) {
		return -EINVAL;
	}

	ret = argos_spi_transact(dev, ARGOS_SPI_CMD_READ_RCONF_RAW, NULL, 0,
				 rconf_raw, rconf_len, &status);
	if (ret < 0) {
		return ret;
	}

	if (status != PROT_OK) {
		LOG_ERR("GET_RCONF_RAW failed: status 0x%02X", status);
		return -EIO;
	}

	LOG_DBG("RCONF_RAW received (len=%zu): %02X%02X%02X%02X%02X%02X%02X%02X...",
		*rconf_len,
		rconf_raw[0], rconf_raw[1], rconf_raw[2], rconf_raw[3],
		rconf_raw[4], rconf_raw[5], rconf_raw[6], rconf_raw[7]);
	return 0;
}

/* Device tree instantiation macros */
#define DT_DRV_COMPAT arribada_argos_smd_spi

/* Default SPI frequency if not specified in devicetree */
#define ARGOS_SPI_DEFAULT_FREQ  125000  /* 125 kHz for reliable slave operation */

/* Get SPI max frequency - use property if exists, otherwise default */
#define ARGOS_SPI_FREQ(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), spi_max_frequency, ARGOS_SPI_DEFAULT_FREQ)

/*
 * Get CS GPIO from parent SPI bus node.
 * The cs-gpios is defined on the SPI bus (e.g., &spi1).
 * We use index 0 since only one argos-smd device per SPI bus is supported.
 */
#define ARGOS_SPI_CS_GPIO(inst) \
	GPIO_DT_SPEC_GET_BY_IDX(DT_INST_BUS(inst), cs_gpios, 0)

#define ARGOS_SPI_INIT(inst)                                                    \
	static struct argos_spi_data argos_spi_data_##inst;                     \
	static const struct argos_spi_config argos_spi_config_##inst = {        \
		.spi = {                                                        \
			.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),                \
			.config = {                                             \
				.frequency = ARGOS_SPI_FREQ(inst),              \
				.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB \
					   | SPI_OP_MODE_MASTER,                \
				.slave = DT_INST_REG_ADDR(inst),                \
				.cs = {                                         \
					.gpio = ARGOS_SPI_CS_GPIO(inst),        \
					.delay = 0,                             \
				},                                              \
			},                                                      \
		},                                                              \
		.irq_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, {0}),     \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}), \
	};                                                                      \
	DEVICE_DT_INST_DEFINE(inst, argos_spi_init, NULL,                       \
			      &argos_spi_data_##inst,                           \
			      &argos_spi_config_##inst,                         \
			      POST_KERNEL, CONFIG_ARGOS_SMD_SPI_INIT_PRIORITY,  \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(ARGOS_SPI_INIT)
