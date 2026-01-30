/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
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
 *
 * Polynomial: x^8 + x^2 + x + 1 (0x07)
 * Initial value: 0x00
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
 * @brief Build a Protocol A+ request frame
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

	return idx;
}

/**
 * @brief Parse a Protocol A+ response frame
 */
static int parse_response_frame(const uint8_t *buf, size_t buf_len,
				struct argos_spi_response *resp)
{
	if (buf_len < ARGOS_SPI_HEADER_SIZE + ARGOS_SPI_CRC_SIZE) {
		LOG_ERR("Response too short: %zu bytes", buf_len);
		return -EINVAL;
	}

	/* Check magic byte */
	if (buf[0] != ARGOS_SPI_MAGIC_RESPONSE) {
		LOG_ERR("Invalid magic byte: 0x%02X (expected 0x%02X)",
			buf[0], ARGOS_SPI_MAGIC_RESPONSE);
		return -EPROTO;
	}

	resp->magic = buf[0];
	resp->seq = buf[1];
	resp->status = buf[2];
	resp->len = buf[3];

	/* Validate length */
	if (resp->len > ARGOS_SPI_MAX_PAYLOAD) {
		LOG_ERR("Response payload too large: %u", resp->len);
		return -EMSGSIZE;
	}

	/* Check if we have enough data */
	size_t expected_len = ARGOS_SPI_HEADER_SIZE + resp->len + ARGOS_SPI_CRC_SIZE;
	if (buf_len < expected_len) {
		LOG_ERR("Response incomplete: got %zu, expected %zu", buf_len, expected_len);
		return -EINVAL;
	}

	/* Copy payload */
	if (resp->len > 0) {
		memcpy(resp->data, &buf[4], resp->len);
	}

	/* Get CRC */
	resp->crc = buf[4 + resp->len];

	/* Verify CRC */
	uint8_t calc_crc = argos_spi_crc8_ccitt(buf, 4 + resp->len);
	if (calc_crc != resp->crc) {
		LOG_ERR("CRC mismatch: calculated 0x%02X, received 0x%02X",
			calc_crc, resp->crc);
		return -EILSEQ;
	}

	return 0;
}

/**
 * @brief Wait for device to be ready (check IRQ pin if available)
 */
static int wait_device_ready(const struct argos_spi_config *cfg, k_timeout_t timeout)
{
	/* If IRQ/ready pin is configured, wait for it */
	if (cfg->irq_gpio.port != NULL) {
		int64_t start = k_uptime_get();
		int64_t timeout_ms = k_ticks_to_ms_floor64(timeout.ticks);

		while ((k_uptime_get() - start) < timeout_ms) {
			if (gpio_pin_get_dt(&cfg->irq_gpio) == 1) {
				return 0;
			}
			k_msleep(1);
		}
		return -ETIMEDOUT;
	}

	/* No IRQ pin - just use a small delay */
	k_msleep(1);
	return 0;
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

	LOG_INF("Argos SMD SPI initialized");

	return 0;
}

int argos_spi_transact(const struct device *dev, uint8_t cmd,
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

	/* Build request frame */
	size_t frame_len = build_request_frame(data->tx_buf, data->seq_num, cmd,
					       tx_data, tx_len);

	LOG_DBG("Sending cmd 0x%02X, seq %u, len %zu", cmd, data->seq_num, tx_len);
	LOG_HEXDUMP_DBG(data->tx_buf, frame_len, "TX frame");

	/* Prepare SPI buffers */
	struct spi_buf tx_spi_buf = {
		.buf = data->tx_buf,
		.len = frame_len,
	};
	struct spi_buf_set tx_set = {
		.buffers = &tx_spi_buf,
		.count = 1,
	};

	/* Send request */
	ret = spi_write_dt(&cfg->spi, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		goto unlock;
	}

	/* Wait for device to be ready */
	ret = wait_device_ready(cfg, K_MSEC(ARGOS_SPI_TIMEOUT_MS));
	if (ret < 0) {
		LOG_ERR("Device not ready: %d", ret);
		goto unlock;
	}

	/*
	 * IMPORTANT: Delay between TX and RX
	 * The STM32 slave processes commands in its main loop.
	 * It needs time to:
	 * 1. Detect end of SPI transaction (CS rising)
	 * 2. Process the command in main loop
	 * 3. Prepare the response buffer
	 * 4. Be ready for the next SPI transaction
	 *
	 * 10ms should be enough for simple commands.
	 * Longer commands (erase) may need more time.
	 */
	k_msleep(10);

	/*
	 * Read response using transceive with 0xFF dummy bytes
	 * Protocol A+ requires:
	 * - Transaction 1: TX=[CMD] → RX=[0xFF...] (slave idle)
	 * - Transaction 2: TX=[0xFF...] → RX=[RESPONSE] (actual response)
	 *
	 * The slave uses TransmitReceive, so we must send 0xFF while reading.
	 */
	memset(data->tx_buf, 0xFF, ARGOS_SPI_MAX_FRAME_SIZE);  /* Dummy TX bytes */
	memset(data->rx_buf, 0xAB, sizeof(data->rx_buf));       /* Clear RX buffer */

	struct spi_buf tx_dummy_buf = {
		.buf = data->tx_buf,
		.len = ARGOS_SPI_MAX_FRAME_SIZE,
	};
	struct spi_buf_set tx_dummy_set = {
		.buffers = &tx_dummy_buf,
		.count = 1,
	};

	struct spi_buf rx_spi_buf = {
		.buf = data->rx_buf,
		.len = ARGOS_SPI_MAX_FRAME_SIZE,
	};
	struct spi_buf_set rx_set = {
		.buffers = &rx_spi_buf,
		.count = 1,
	};

	/* Use transceive to send 0xFF while reading response */
	ret = spi_transceive_dt(&cfg->spi, &tx_dummy_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI transceive failed: %d", ret);
		goto unlock;
	}

	LOG_HEXDUMP_DBG(data->rx_buf, 16, "RX raw");

	/* Find response magic byte (skip any leading 0xFF padding) */
	size_t rx_start = 0;
	while (rx_start < ARGOS_SPI_MAX_FRAME_SIZE && data->rx_buf[rx_start] == 0xFF) {
		rx_start++;
	}

	if (rx_start >= ARGOS_SPI_MAX_FRAME_SIZE) {
		LOG_ERR("No response received");
		ret = -ETIMEDOUT;
		goto unlock;
	}

	/* Parse response */
	struct argos_spi_response resp;
	ret = parse_response_frame(&data->rx_buf[rx_start],
				   ARGOS_SPI_MAX_FRAME_SIZE - rx_start, &resp);
	if (ret < 0) {
		LOG_ERR("Failed to parse response: %d", ret);
		goto unlock;
	}

	/* Verify sequence number matches */
	if (resp.seq != data->seq_num) {
		LOG_WRN("Sequence mismatch: sent %u, got %u", data->seq_num, resp.seq);
		/* Continue anyway - some devices may not track sequence */
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
	size_t frame_len = build_request_frame(data->tx_buf, data->seq_num, cmd,
					       tx_data, tx_len);

	LOG_DBG("Sending cmd 0x%02X (no response expected)", cmd);

	struct spi_buf tx_spi_buf = {
		.buf = data->tx_buf,
		.len = frame_len,
	};
	struct spi_buf_set tx_set = {
		.buffers = &tx_spi_buf,
		.count = 1,
	};

	ret = spi_write_dt(&cfg->spi, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
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

	/* Build raw TX frame: [CMD] [PAYLOAD...] */
	data->tx_buf[0] = cmd;
	if (tx_data && tx_len > 0) {
		memcpy(&data->tx_buf[1], tx_data, tx_len);
	}
	size_t frame_len = 1 + tx_len;

	LOG_DBG("RAW TX: cmd=0x%02X, len=%zu", cmd, tx_len);
	LOG_HEXDUMP_DBG(data->tx_buf, frame_len, "RAW TX frame");

	/* TX: Send command */
	struct spi_buf tx_spi_buf = {
		.buf = data->tx_buf,
		.len = frame_len,
	};
	struct spi_buf_set tx_set = {
		.buffers = &tx_spi_buf,
		.count = 1,
	};

	ret = spi_write_dt(&cfg->spi, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		goto unlock;
	}

	/*
	 * Wait for bootloader to process command (10-20ms).
	 * The bootloader needs time to:
	 * 1. Detect end of SPI transaction (CS rising)
	 * 2. Process the command
	 * 3. Prepare the response buffer
	 */
	k_msleep(15);

	/* Determine expected RX length: status byte + optional response data */
	size_t rx_expect = 1;  /* At minimum, status byte */
	if (rx_len && *rx_len > 0) {
		rx_expect += *rx_len;
	} else {
		/* Default: read up to 32 bytes for variable-length responses */
		rx_expect += 32;
	}

	/* Cap at buffer size */
	if (rx_expect > ARGOS_SPI_MAX_FRAME_SIZE) {
		rx_expect = ARGOS_SPI_MAX_FRAME_SIZE;
	}

	/* RX: Clock out response using dummy 0xFF bytes on MOSI */
	memset(data->tx_buf, 0xFF, rx_expect);
	memset(data->rx_buf, 0xFF, sizeof(data->rx_buf));

	struct spi_buf tx_dummy_buf = {
		.buf = data->tx_buf,
		.len = rx_expect,
	};
	struct spi_buf_set tx_dummy_set = {
		.buffers = &tx_dummy_buf,
		.count = 1,
	};

	struct spi_buf rx_spi_buf = {
		.buf = data->rx_buf,
		.len = rx_expect,
	};
	struct spi_buf_set rx_set = {
		.buffers = &rx_spi_buf,
		.count = 1,
	};

	ret = spi_transceive_dt(&cfg->spi, &tx_dummy_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI transceive failed: %d", ret);
		goto unlock;
	}

	LOG_HEXDUMP_DBG(data->rx_buf, rx_expect, "RAW RX");

	/* Parse response: [STATUS] [DATA...] */
	*status = data->rx_buf[0];
	LOG_DBG("RAW RX: status=0x%02X", *status);

	if (rx_data && rx_len && *rx_len > 0) {
		size_t copy_len = MIN(*rx_len, rx_expect - 1);
		memcpy(rx_data, &data->rx_buf[1], copy_len);
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

	/* Build raw TX frame: [CMD] [PAYLOAD...] */
	data->tx_buf[0] = cmd;
	if (tx_data && tx_len > 0) {
		memcpy(&data->tx_buf[1], tx_data, tx_len);
	}
	size_t frame_len = 1 + tx_len;

	LOG_DBG("RAW TX (no response): cmd=0x%02X", cmd);

	struct spi_buf tx_spi_buf = {
		.buf = data->tx_buf,
		.len = frame_len,
	};
	struct spi_buf_set tx_set = {
		.buffers = &tx_spi_buf,
		.count = 1,
	};

	ret = spi_write_dt(&cfg->spi, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

int argos_spi_ping(const struct device *dev)
{
	uint8_t status;
	int ret;

	LOG_DBG("Pinging device (app mode)");

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
	uint8_t tx_buf[32];
	uint8_t rx_buf[32];

	LOG_INF("=== SPI DIAGNOSTIC ===");

	/* Log SPI configuration */
	LOG_INF("SPI config:");
	LOG_INF("  Frequency: %u Hz", cfg->spi.config.frequency);
	LOG_INF("  Operation: 0x%04X", cfg->spi.config.operation);
	LOG_INF("  CPOL=%d, CPHA=%d (Mode %d)",
		(cfg->spi.config.operation & SPI_MODE_CPOL) ? 1 : 0,
		(cfg->spi.config.operation & SPI_MODE_CPHA) ? 1 : 0,
		((cfg->spi.config.operation & SPI_MODE_CPOL) ? 2 : 0) |
		((cfg->spi.config.operation & SPI_MODE_CPHA) ? 1 : 0));

	/* Check GPIO status */
	if (cfg->irq_gpio.port != NULL) {
		int val = gpio_pin_get_dt(&cfg->irq_gpio);
		LOG_INF("IRQ GPIO: %s (value: %d)",
			gpio_is_ready_dt(&cfg->irq_gpio) ? "ready" : "NOT ready", val);
	} else {
		LOG_INF("IRQ GPIO: not configured");
	}

	if (cfg->reset_gpio.port != NULL) {
		LOG_INF("Reset GPIO: %s",
			gpio_is_ready_dt(&cfg->reset_gpio) ? "ready" : "NOT ready");
	} else {
		LOG_INF("Reset GPIO: not configured");
	}

	/* Check CS GPIO directly */
	LOG_INF("");
	LOG_INF("CS GPIO status:");
	if (cfg->spi.config.cs.gpio.port != NULL) {
		const struct gpio_dt_spec *cs_gpio = &cfg->spi.config.cs.gpio;
		LOG_INF("  Port: %s, Pin: %d, Flags: 0x%x",
			cs_gpio->port->name, cs_gpio->pin, cs_gpio->dt_flags);
		if (gpio_is_ready_dt(cs_gpio)) {
			LOG_INF("  Status: GPIO READY");
			/* Try to manually toggle CS for testing */
			LOG_INF("  Attempting manual CS toggle test...");
			/* Note: The SPI driver manages CS, we can't easily toggle it manually
			 * but we can at least verify the GPIO port is accessible */
			int pin_val = gpio_pin_get_raw(cs_gpio->port, cs_gpio->pin);
			LOG_INF("  Current CS pin state (raw): %d", pin_val);
		} else {
			LOG_ERR("  Status: GPIO NOT READY - P0.10 may still be in NFC mode!");
			LOG_ERR("  Make sure UICR is properly flashed with nfct-pins-as-gpios");
			LOG_ERR("  Try: 'nrfjprog --eraseall' then re-flash the firmware");
		}
	} else {
		LOG_ERR("  CS GPIO: NOT CONFIGURED - check cs-gpios in devicetree");
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* ===== TEST 1: Full-duplex transceive ===== */
	LOG_INF("");
	LOG_INF("--- TEST 1: Full-duplex transceive ---");
	LOG_INF("Send PING command and read simultaneously");

	/* Build ping frame */
	tx_buf[0] = ARGOS_SPI_MAGIC_REQUEST;  /* 0xAA */
	tx_buf[1] = 0x00;                       /* Sequence */
	tx_buf[2] = ARGOS_SPI_CMD_PING;           /* 0x01 */
	tx_buf[3] = 0x00;                       /* Length */
	tx_buf[4] = argos_spi_crc8_ccitt(tx_buf, 4);

	LOG_INF("TX: AA 00 01 00 %02X (PING)", tx_buf[4]);

	memset(rx_buf, 0xAB, sizeof(rx_buf));

	struct spi_buf spi_tx = { .buf = tx_buf, .len = 5 };
	struct spi_buf spi_rx = { .buf = rx_buf, .len = 5 };
	struct spi_buf_set tx_set = { .buffers = &spi_tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &spi_rx, .count = 1 };

	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("spi_transceive failed: %d", ret);
	} else {
		LOG_INF("RX during TX (full-duplex):");
		LOG_HEXDUMP_INF(rx_buf, 8, "  ");
		if (rx_buf[0] == 0x00) {
			LOG_INF("  -> Slave TX buffer empty during our TX (normal)");
		} else if (rx_buf[0] == 0xFF) {
			LOG_WRN("  -> MISO high/floating");
		} else if (rx_buf[0] == 0xAB) {
			LOG_WRN("  -> Buffer unchanged - transceive issue");
		} else {
			LOG_INF("  -> Got data: 0x%02X", rx_buf[0]);
		}
	}

	/* ===== TEST 2: Separate TX then RX with transceive (0xFF TX) ===== */
	LOG_INF("");
	LOG_INF("--- TEST 2: TX cmd, delay, then TX 0xFF to read response ---");

	uint8_t dummy_tx[16];
	memset(dummy_tx, 0xFF, sizeof(dummy_tx));  /* MOSI must be 0xFF during read */

	int delays_ms[] = {1, 10, 50, 100, 200, 500};
	for (int d = 0; d < ARRAY_SIZE(delays_ms); d++) {
		LOG_INF("Delay: %d ms", delays_ms[d]);

		/* TX command */
		struct spi_buf tx2 = { .buf = tx_buf, .len = 5 };
		struct spi_buf_set tx2_set = { .buffers = &tx2, .count = 1 };
		ret = spi_write_dt(&cfg->spi, &tx2_set);
		if (ret < 0) {
			LOG_ERR("  TX failed: %d", ret);
			continue;
		}

		k_msleep(delays_ms[d]);

		/* RX using transceive with 0xFF TX (STM32 requires 0xFF on MOSI) */
		memset(rx_buf, 0xAB, sizeof(rx_buf));
		struct spi_buf tx_dummy = { .buf = dummy_tx, .len = 16 };
		struct spi_buf rx2 = { .buf = rx_buf, .len = 16 };
		struct spi_buf_set tx_dummy_set = { .buffers = &tx_dummy, .count = 1 };
		struct spi_buf_set rx2_set = { .buffers = &rx2, .count = 1 };
		ret = spi_transceive_dt(&cfg->spi, &tx_dummy_set, &rx2_set);
		if (ret < 0) {
			LOG_ERR("  RX failed: %d", ret);
			continue;
		}

		LOG_HEXDUMP_INF(rx_buf, 16, "  RX");

		/* Check for magic anywhere in buffer */
		for (int i = 0; i < 16; i++) {
			if (rx_buf[i] == ARGOS_SPI_MAGIC_RESPONSE) {
				LOG_INF("  SUCCESS! Magic 0x55 at byte %d, status=0x%02X",
					i, rx_buf[i + 2]);
				k_mutex_unlock(&data->lock);
				return 0;
			}
		}
	}

	/* ===== TEST 3: Send dummy bytes to clock out response ===== */
	LOG_INF("");
	LOG_INF("--- TEST 3: TX command + clock dummy bytes ---");
	LOG_INF("Send PING, then clock out 16 dummy bytes in same transaction");

	/* Prepare: 5 bytes command + 16 dummy bytes (0xFF) */
	memset(&tx_buf[5], 0xFF, 16);  /* Dummy bytes after command */
	memset(rx_buf, 0xAB, sizeof(rx_buf));

	struct spi_buf tx3 = { .buf = tx_buf, .len = 21 };  /* 5 cmd + 16 dummy */
	struct spi_buf rx3 = { .buf = rx_buf, .len = 21 };
	struct spi_buf_set tx3_set = { .buffers = &tx3, .count = 1 };
	struct spi_buf_set rx3_set = { .buffers = &rx3, .count = 1 };

	ret = spi_transceive_dt(&cfg->spi, &tx3_set, &rx3_set);
	if (ret < 0) {
		LOG_ERR("spi_transceive failed: %d", ret);
	} else {
		LOG_INF("Full RX (21 bytes):");
		LOG_HEXDUMP_INF(rx_buf, 21, "  ");

		/* Look for magic anywhere in response */
		for (int i = 0; i < 21; i++) {
			if (rx_buf[i] == ARGOS_SPI_MAGIC_RESPONSE) {
				LOG_INF("  Magic 0x55 found at byte %d!", i);
				LOG_INF("  Response starts at offset %d", i);
				k_mutex_unlock(&data->lock);
				return 0;
			}
		}
		LOG_WRN("  No magic byte found in extended transaction");
	}

	/* ===== TEST 4: Analyze byte patterns ===== */
	LOG_INF("");
	LOG_INF("--- TEST 4: Byte pattern analysis ---");

	/* Count different byte values in last RX */
	int count_00 = 0, count_ff = 0, count_other = 0;
	for (int i = 0; i < 21; i++) {
		if (rx_buf[i] == 0x00) count_00++;
		else if (rx_buf[i] == 0xFF) count_ff++;
		else count_other++;
	}
	LOG_INF("Byte stats: 0x00=%d, 0xFF=%d, other=%d", count_00, count_ff, count_other);

	if (count_other > 0) {
		LOG_INF("Non-0x00/0xFF bytes found:");
		for (int i = 0; i < 21; i++) {
			if (rx_buf[i] != 0x00 && rx_buf[i] != 0xFF) {
				LOG_INF("  [%d] = 0x%02X (binary: %d%d%d%d%d%d%d%d)",
					i, rx_buf[i],
					(rx_buf[i] >> 7) & 1, (rx_buf[i] >> 6) & 1,
					(rx_buf[i] >> 5) & 1, (rx_buf[i] >> 4) & 1,
					(rx_buf[i] >> 3) & 1, (rx_buf[i] >> 2) & 1,
					(rx_buf[i] >> 1) & 1, rx_buf[i] & 1);
			}
		}
		LOG_WRN("These partial bytes suggest timing/sync issues between master and slave");
	}

	k_mutex_unlock(&data->lock);

	LOG_INF("");
	LOG_WRN("=== DIAGNOSTIC COMPLETE - No valid response from slave ===");
	LOG_WRN("");
	if (count_ff > count_00) {
		LOG_WRN("MISO is mostly HIGH (0xFF) - slave TX buffer has 0xFF (idle)");
		LOG_WRN("The STM32 is NOT loading its response into the TX buffer");
	} else if (count_00 > count_ff) {
		LOG_WRN("MISO is mostly LOW (0x00) - slave TX buffer empty or MISO stuck");
	} else {
		LOG_WRN("Mixed 0x00/0xFF - possible electrical/timing issues");
	}
	LOG_WRN("");
	LOG_WRN("STM32 firmware must prepare response BEFORE master clocks it out:");
	LOG_WRN("  1. Receive cmd in RX interrupt/DMA callback");
	LOG_WRN("  2. Parse command and prepare response");
	LOG_WRN("  3. Load response into TX buffer");
	LOG_WRN("  4. Call HAL_SPI_TransmitReceive_IT() with response buffer");
	LOG_WRN("  5. Wait for master to start next transaction");

	return -ENODEV;
}

/* Device tree instantiation macros */
#define DT_DRV_COMPAT arribada_argos_smd_spi

/* Default SPI frequency if not specified in devicetree */
#define ARGOS_SPI_DEFAULT_FREQ  1000000

/* Get SPI max frequency - use property if exists, otherwise default */
#define ARGOS_SPI_FREQ(inst) \
	DT_PROP_OR(DT_DRV_INST(inst), spi_max_frequency, ARGOS_SPI_DEFAULT_FREQ)

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
					.gpio = SPI_CS_GPIOS_DT_SPEC_GET(       \
						DT_INST_BUS(inst)),             \
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
