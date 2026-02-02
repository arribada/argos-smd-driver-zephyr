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
	k_msleep(15);

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

retry_nop:
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
		goto retry_nop;
	}

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

int argos_spi_transact_raw(const struct device *dev, uint8_t cmd,
			   const uint8_t *tx_data, size_t tx_len,
			   uint8_t *rx_data, size_t *rx_len, uint8_t *status)
{
	const struct argos_spi_config *cfg = dev->config;
	struct argos_spi_data *data = dev->data;
	int ret;

	if (tx_len > ARGOS_SPI_MAX_PAYLOAD) {
		LOG_ERR("TX payload too large: %zu", tx_len);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Build raw TX frame: [CMD] [PAYLOAD...] padded to 64 bytes */
	memset(data->tx_buf, 0xFF, ARGOS_SPI_TRANSACTION_SIZE);
	data->tx_buf[0] = cmd;
	if (tx_data && tx_len > 0) {
		memcpy(&data->tx_buf[1], tx_data, tx_len);
	}

	LOG_DBG("RAW TX: cmd=0x%02X, len=%zu", cmd, tx_len);
	LOG_HEXDUMP_DBG(data->tx_buf, MIN(16, 1 + tx_len), "RAW TX frame");

	/* Transaction 1: Send command */
	memset(data->rx_buf, 0, sizeof(data->rx_buf));
	ret = spi_transaction_64(&cfg->spi, data->tx_buf, data->rx_buf);
	if (ret < 0) {
		LOG_ERR("SPI transaction 1 failed: %d", ret);
		goto unlock;
	}

	/* Delay for processing */
	k_msleep(ARGOS_SPI_PIPELINE_DELAY_MS);

	/* Transaction 2: Send dummy to get response */
	memset(data->tx_buf, 0xFF, ARGOS_SPI_TRANSACTION_SIZE);
	memset(data->rx_buf, 0, sizeof(data->rx_buf));
	ret = spi_transaction_64(&cfg->spi, data->tx_buf, data->rx_buf);
	if (ret < 0) {
		LOG_ERR("SPI transaction 2 failed: %d", ret);
		goto unlock;
	}

	LOG_HEXDUMP_DBG(data->rx_buf, 16, "RAW RX");

	/* Parse raw response: [STATUS] [DATA...] */
	/* Skip leading 0xAA (idle), 0xBB (busy), or 0xFF (padding) */
	size_t offset = 0;
	while (offset < ARGOS_SPI_TRANSACTION_SIZE &&
	       (data->rx_buf[offset] == ARGOS_SPI_IDLE_PATTERN ||
	        data->rx_buf[offset] == ARGOS_SPI_BUSY_PATTERN ||
	        data->rx_buf[offset] == 0xFF)) {
		offset++;
	}

	if (offset >= ARGOS_SPI_TRANSACTION_SIZE) {
		if (is_rx_all_busy(data->rx_buf, 8)) {
			LOG_ERR("Slave still BUSY - timeout too short");
		} else {
			LOG_ERR("No valid response in raw transaction");
		}
		ret = -ENODATA;
		goto unlock;
	}

	*status = data->rx_buf[offset];
	LOG_DBG("RAW RX: status=0x%02X at offset %zu", *status, offset);

	if (rx_data && rx_len && *rx_len > 0) {
		size_t avail = ARGOS_SPI_TRANSACTION_SIZE - offset - 1;
		size_t copy_len = MIN(*rx_len, avail);
		memcpy(rx_data, &data->rx_buf[offset + 1], copy_len);
		*rx_len = copy_len;
	}

	ret = 0;

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

int argos_spi_transact_raw_timeout(const struct device *dev, uint8_t cmd,
				   const uint8_t *tx_data, size_t tx_len,
				   uint8_t *rx_data, size_t *rx_len, uint8_t *status,
				   uint32_t timeout_ms)
{
	const struct argos_spi_config *cfg = dev->config;
	struct argos_spi_data *data = dev->data;
	int ret;

	if (tx_len > ARGOS_SPI_MAX_PAYLOAD) {
		LOG_ERR("TX payload too large: %zu", tx_len);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Build raw TX frame: [CMD] [PAYLOAD...] padded to 64 bytes */
	memset(data->tx_buf, 0xFF, ARGOS_SPI_TRANSACTION_SIZE);
	data->tx_buf[0] = cmd;
	if (tx_data && tx_len > 0) {
		memcpy(&data->tx_buf[1], tx_data, tx_len);
	}

	LOG_DBG("RAW TX (timeout=%ums): cmd=0x%02X, len=%zu", timeout_ms, cmd, tx_len);
	LOG_HEXDUMP_DBG(data->tx_buf, MIN(16, 1 + tx_len), "RAW TX frame");

	/* Transaction 1: Send command */
	memset(data->rx_buf, 0, sizeof(data->rx_buf));
	ret = spi_transaction_64(&cfg->spi, data->tx_buf, data->rx_buf);
	if (ret < 0) {
		LOG_ERR("SPI transaction 1 failed: %d", ret);
		goto unlock;
	}

	/* Wait for operation to complete (custom timeout for long operations) */
	k_msleep(timeout_ms);

	/* Transaction 2: Send dummy to get response */
	memset(data->tx_buf, 0xFF, ARGOS_SPI_TRANSACTION_SIZE);
	memset(data->rx_buf, 0, sizeof(data->rx_buf));
	ret = spi_transaction_64(&cfg->spi, data->tx_buf, data->rx_buf);
	if (ret < 0) {
		LOG_ERR("SPI transaction 2 failed: %d", ret);
		goto unlock;
	}

	LOG_HEXDUMP_DBG(data->rx_buf, 16, "RAW RX");

	/* Parse raw response: [STATUS] [DATA...] */
	/* Skip leading 0xAA (idle), 0xBB (busy), or 0xFF (padding) */
	size_t offset = 0;
	while (offset < ARGOS_SPI_TRANSACTION_SIZE &&
	       (data->rx_buf[offset] == ARGOS_SPI_IDLE_PATTERN ||
	        data->rx_buf[offset] == ARGOS_SPI_BUSY_PATTERN ||
	        data->rx_buf[offset] == 0xFF)) {
		offset++;
	}

	if (offset >= ARGOS_SPI_TRANSACTION_SIZE) {
		if (is_rx_all_busy(data->rx_buf, 8)) {
			LOG_ERR("Slave still BUSY - timeout may be too short");
		} else {
			LOG_ERR("No valid response in raw transaction (timeout may be too short)");
		}
		ret = -ENODATA;
		goto unlock;
	}

	*status = data->rx_buf[offset];
	LOG_DBG("RAW RX: status=0x%02X at offset %zu", *status, offset);

	if (rx_data && rx_len && *rx_len > 0) {
		size_t avail = ARGOS_SPI_TRANSACTION_SIZE - offset - 1;
		size_t copy_len = MIN(*rx_len, avail);
		memcpy(rx_data, &data->rx_buf[offset + 1], copy_len);
		*rx_len = copy_len;
	}

	ret = 0;

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

int argos_spi_send_only_raw(const struct device *dev, uint8_t cmd,
			    const uint8_t *tx_data, size_t tx_len)
{
	const struct argos_spi_config *cfg = dev->config;
	struct argos_spi_data *data = dev->data;
	int ret;

	if (tx_len > ARGOS_SPI_MAX_PAYLOAD) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Build raw TX frame padded to 64 bytes */
	memset(data->tx_buf, 0xFF, ARGOS_SPI_TRANSACTION_SIZE);
	data->tx_buf[0] = cmd;
	if (tx_data && tx_len > 0) {
		memcpy(&data->tx_buf[1], tx_data, tx_len);
	}

	LOG_DBG("RAW TX (no response): cmd=0x%02X", cmd);

	memset(data->rx_buf, 0, sizeof(data->rx_buf));
	ret = spi_transaction_64(&cfg->spi, data->tx_buf, data->rx_buf);
	if (ret < 0) {
		LOG_ERR("SPI transaction failed: %d", ret);
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

	/* Hold reset for 50ms */
	k_msleep(50);

	/* Release reset */
	ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);
	if (ret < 0) {
		LOG_ERR("Failed to release reset: %d", ret);
		return ret;
	}

	/* Wait for module to boot (500ms typical) */
	k_msleep(500);

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
	k_msleep(15);

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
	k_msleep(15);

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
	uint8_t status;
	int ret;

	if (!id || id_len == 0) {
		return -EINVAL;
	}

	/* Use longer delay for flash write operations */
	ret = argos_spi_transact_internal(dev, ARGOS_SPI_CMD_WRITE_ID, id, id_len,
					  NULL, NULL, &status,
					  ARGOS_SPI_FLASH_DELAY_MS);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_SPI_RSP_OK) {
		LOG_ERR("SET_ID failed: status 0x%02X", status);
		return -EIO;
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
	uint8_t status;
	int ret;

	if (!addr || addr_len == 0) {
		return -EINVAL;
	}

	/* Use longer delay for flash write operations */
	ret = argos_spi_transact_internal(dev, ARGOS_SPI_CMD_WRITE_ADDR, addr, addr_len,
					  NULL, NULL, &status,
					  ARGOS_SPI_FLASH_DELAY_MS);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_SPI_RSP_OK) {
		LOG_ERR("SET_ADDR failed: status 0x%02X", status);
		return -EIO;
	}

	LOG_DBG("ADDR set successfully");
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
		LOG_WRN("GET_RCONF status: 0x%02X", status);
		/* Don't fail - some devices return non-OK for unconfigured */
	}

	LOG_DBG("RCONF received (len=%zu)", *rconf_len);
	return 0;
}

int argos_spi_set_rconf(const struct device *dev, const uint8_t *rconf, size_t rconf_len)
{
	uint8_t status;
	int ret;

	if (!rconf || rconf_len == 0) {
		return -EINVAL;
	}

	/* Use longer delay for flash write operations */
	ret = argos_spi_transact_internal(dev, ARGOS_SPI_CMD_WRITE_RCONF, rconf, rconf_len,
					  NULL, NULL, &status,
					  ARGOS_SPI_FLASH_DELAY_MS);
	if (ret < 0) {
		return ret;
	}

	if (status != ARGOS_SPI_RSP_OK) {
		LOG_ERR("SET_RCONF failed: status 0x%02X", status);
		return -EIO;
	}

	LOG_DBG("RCONF set successfully");
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

int argos_spi_wait_tx_complete(const struct device *dev, k_timeout_t timeout)
{
	uint8_t mac_status;
	int64_t start = k_uptime_get();
	int64_t timeout_ms = k_ticks_to_ms_floor64(timeout.ticks);
	int ret;

	LOG_DBG("Waiting for TX complete (timeout=%lld ms)", timeout_ms);

	while ((k_uptime_get() - start) < timeout_ms) {
		ret = argos_spi_get_mac_status(dev, &mac_status);
		if (ret < 0) {
			LOG_WRN("MAC status read error: %d", ret);
			k_msleep(ARGOS_TIMING_POLL_MS);
			continue;
		}

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
			/* Still in progress - continue polling */
			k_msleep(ARGOS_TIMING_POLL_MS);
			continue;
		}

		/* Other status (MAC_OK, MAC_RX_RECEIVED, etc.) - continue polling */
		k_msleep(ARGOS_TIMING_POLL_MS / 2);
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
		k_msleep(10);
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
	k_msleep(100);

	LOG_INF("TX data queued (%zu bytes), use wait_tx_complete() to poll", len);
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
