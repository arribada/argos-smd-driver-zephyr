/*
 * Copyright (c) 2025 Arribada Initiative
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI Debug Sample - Debug SPI communication with Argos SMD module
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

LOG_MODULE_REGISTER(spi_debug, LOG_LEVEL_DBG);

/* Get SPI bus from devicetree */
#define SPI_NODE DT_NODELABEL(spi1)
static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

/* CS GPIO - get from spi1 cs-gpios */
static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET_BY_IDX(SPI_NODE, cs_gpios, 0);

/* Reset GPIO (optional) */
#define ARGOS_SMD_NODE DT_NODELABEL(argos_smd)
#if DT_NODE_HAS_PROP(ARGOS_SMD_NODE, reset_gpios)
static const struct gpio_dt_spec reset_gpio = GPIO_DT_SPEC_GET(ARGOS_SMD_NODE, reset_gpios);
#define HAS_RESET_GPIO 1
#else
#define HAS_RESET_GPIO 0
#endif

/* SPI config - Mode 0, 8-bit, MSB first */
static struct spi_config spi_cfg = {
	.frequency = 125000,  /* 125 kHz - minimum for nRF52840 SPIM */
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	.slave = 0,
	.cs = {
		.gpio = GPIO_DT_SPEC_GET_BY_IDX(SPI_NODE, cs_gpios, 0),
		.delay = 0,
	},
};

/* Protocol A+ constants */
#define MAGIC_REQUEST   0xAA
#define MAGIC_RESPONSE  0x55
#define CMD_PING        0x01

/* CRC-8 CCITT */
static uint8_t crc8_ccitt(const uint8_t *data, size_t len)
{
	uint8_t crc = 0x00;
	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

/* Test 1: Check GPIO status */
static void test_gpio_status(void)
{
	LOG_INF("========================================");
	LOG_INF("TEST 1: GPIO STATUS CHECK");
	LOG_INF("========================================");

	/* Check CS GPIO */
	LOG_INF("CS GPIO (P0.10):");
	LOG_INF("  Port: %s", cs_gpio.port ? cs_gpio.port->name : "NULL");
	LOG_INF("  Pin: %d", cs_gpio.pin);
	LOG_INF("  Flags: 0x%x", cs_gpio.dt_flags);

	if (!gpio_is_ready_dt(&cs_gpio)) {
		LOG_ERR("  STATUS: NOT READY!");
		LOG_ERR("  P0.10 may still be in NFC mode.");
		LOG_ERR("  Run: nrfjprog --eraseall && west flash");
	} else {
		LOG_INF("  STATUS: READY");
		int val = gpio_pin_get_raw(cs_gpio.port, cs_gpio.pin);
		LOG_INF("  Current value (raw): %d", val);
	}

#if HAS_RESET_GPIO
	LOG_INF("Reset GPIO (P0.07):");
	LOG_INF("  Port: %s", reset_gpio.port ? reset_gpio.port->name : "NULL");
	LOG_INF("  Pin: %d", reset_gpio.pin);
	if (gpio_is_ready_dt(&reset_gpio)) {
		LOG_INF("  STATUS: READY");
	} else {
		LOG_WRN("  STATUS: NOT READY");
	}
#else
	LOG_INF("Reset GPIO: not configured");
#endif
}

/* Protocol A+ Commands */
#define CMD_GET_VERSION    0x02
#define CMD_GET_ID         0x04
#define CMD_SET_ID         0x05
#define CMD_GET_ADDR       0x06
#define CMD_SET_ADDR       0x07
#define CMD_GET_SN         0x08
#define CMD_GET_RCONF      0x0A

/* Sequence number for transactions */
static uint8_t g_seq_num = 0;

/**
 * @brief Send a command and receive response (two-transaction protocol)
 */
static int spi_send_command(uint8_t cmd, const uint8_t *tx_data, size_t tx_len,
			    uint8_t *rx_data, size_t *rx_len, uint8_t *status)
{
	if (!device_is_ready(spi_dev)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	uint8_t tx_cmd[261];  /* Max frame size */
	uint8_t tx_dummy[64];
	uint8_t rx_buf[64];
	int ret;

	/* Build command frame: MAGIC SEQ CMD LEN DATA CRC */
	size_t frame_len = 5 + tx_len;
	tx_cmd[0] = MAGIC_REQUEST;
	tx_cmd[1] = g_seq_num++;
	tx_cmd[2] = cmd;
	tx_cmd[3] = (uint8_t)tx_len;
	if (tx_len > 0 && tx_data) {
		memcpy(&tx_cmd[4], tx_data, tx_len);
	}
	tx_cmd[4 + tx_len] = crc8_ccitt(tx_cmd, 4 + tx_len);

	LOG_DBG("TX frame (%d bytes): %02X %02X %02X %02X ...",
		(int)frame_len, tx_cmd[0], tx_cmd[1], tx_cmd[2], tx_cmd[3]);

	/* Transaction 1: Send command */
	struct spi_buf spi_tx1 = { .buf = tx_cmd, .len = frame_len };
	struct spi_buf_set tx_set1 = { .buffers = &spi_tx1, .count = 1 };

	ret = spi_write(spi_dev, &spi_cfg, &tx_set1);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}

	/* Delay for STM32 to process */
	k_msleep(10);

	/* Transaction 2: Read response */
	memset(tx_dummy, 0xFF, sizeof(tx_dummy));
	memset(rx_buf, 0x00, sizeof(rx_buf));

	struct spi_buf spi_tx2 = { .buf = tx_dummy, .len = sizeof(rx_buf) };
	struct spi_buf spi_rx2 = { .buf = rx_buf, .len = sizeof(rx_buf) };
	struct spi_buf_set tx_set2 = { .buffers = &spi_tx2, .count = 1 };
	struct spi_buf_set rx_set2 = { .buffers = &spi_rx2, .count = 1 };

	ret = spi_transceive(spi_dev, &spi_cfg, &tx_set2, &rx_set2);
	if (ret < 0) {
		LOG_ERR("SPI transceive failed: %d", ret);
		return ret;
	}

	/* Find response start (skip leading 0xFF) */
	int offset = 0;
	while (offset < sizeof(rx_buf) && rx_buf[offset] == 0xFF) {
		offset++;
	}

	if (offset >= sizeof(rx_buf) || rx_buf[offset] != MAGIC_RESPONSE) {
		LOG_ERR("No valid response found (offset=%d, first=0x%02X)",
			offset, offset < sizeof(rx_buf) ? rx_buf[offset] : 0);
		return -EBADMSG;
	}

	/* Parse response: MAGIC SEQ STATUS LEN DATA CRC */
	uint8_t rsp_seq = rx_buf[offset + 1];
	uint8_t rsp_status = rx_buf[offset + 2];
	uint8_t rsp_len = rx_buf[offset + 3];

	LOG_DBG("RX frame: MAGIC=0x%02X SEQ=%d STATUS=0x%02X LEN=%d",
		rx_buf[offset], rsp_seq, rsp_status, rsp_len);

	if (status) {
		*status = rsp_status;
	}

	/* Copy response data */
	if (rx_data && rx_len && rsp_len > 0) {
		size_t copy_len = (rsp_len < *rx_len) ? rsp_len : *rx_len;
		memcpy(rx_data, &rx_buf[offset + 4], copy_len);
		*rx_len = copy_len;
	} else if (rx_len) {
		*rx_len = 0;
	}

	return 0;
}

/* Helper to print hex data */
static void print_hex(const char *label, const uint8_t *data, size_t len)
{
	char buf[128];
	size_t pos = 0;
	for (size_t i = 0; i < len && pos < sizeof(buf) - 3; i++) {
		pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", data[i]);
	}
	LOG_INF("%s: %s", label, buf);
}

/* Test 2: Raw SPI write (without driver) */
static void test_raw_spi_write(void)
{
	LOG_INF("========================================");
	LOG_INF("TEST 2: RAW SPI WRITE");
	LOG_INF("========================================");

	if (!device_is_ready(spi_dev)) {
		LOG_ERR("SPI device not ready");
		return;
	}

	/* Build PING command: AA 00 01 00 CRC */
	uint8_t tx_buf[5];
	tx_buf[0] = MAGIC_REQUEST;  /* 0xAA */
	tx_buf[1] = 0x00;           /* Sequence */
	tx_buf[2] = CMD_PING;       /* 0x01 */
	tx_buf[3] = 0x00;           /* Length */
	tx_buf[4] = crc8_ccitt(tx_buf, 4);

	LOG_INF("TX buffer: %02X %02X %02X %02X %02X",
		tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4]);

	struct spi_buf spi_tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf_set tx_set = { .buffers = &spi_tx, .count = 1 };

	LOG_INF("Sending SPI write (CS should go LOW during transfer)...");

	int ret = spi_write(spi_dev, &spi_cfg, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
	} else {
		LOG_INF("SPI write OK");
	}
}

/* Test 3: Raw SPI transceive */
static void test_raw_spi_transceive(void)
{
	LOG_INF("========================================");
	LOG_INF("TEST 3: RAW SPI TRANSCEIVE");
	LOG_INF("========================================");

	if (!device_is_ready(spi_dev)) {
		LOG_ERR("SPI device not ready");
		return;
	}

	/* Build PING command */
	uint8_t tx_buf[16];
	uint8_t rx_buf[16];

	tx_buf[0] = MAGIC_REQUEST;
	tx_buf[1] = 0x00;
	tx_buf[2] = CMD_PING;
	tx_buf[3] = 0x00;
	tx_buf[4] = crc8_ccitt(tx_buf, 4);
	/* Pad with 0xFF for reading */
	memset(&tx_buf[5], 0xFF, 11);

	memset(rx_buf, 0xAB, sizeof(rx_buf));

	LOG_INF("TX: %02X %02X %02X %02X %02X FF FF FF...",
		tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4]);

	struct spi_buf spi_tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf spi_rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	struct spi_buf_set tx_set = { .buffers = &spi_tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &spi_rx, .count = 1 };

	LOG_INF("Sending SPI transceive (full-duplex)...");

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI transceive failed: %d", ret);
	} else {
		LOG_INF("SPI transceive OK");
		LOG_INF("RX: %02X %02X %02X %02X %02X %02X %02X %02X",
			rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
			rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);
		LOG_INF("    %02X %02X %02X %02X %02X %02X %02X %02X",
			rx_buf[8], rx_buf[9], rx_buf[10], rx_buf[11],
			rx_buf[12], rx_buf[13], rx_buf[14], rx_buf[15]);

		/* Analyze */
		if (rx_buf[0] == 0xAB) {
			LOG_WRN("RX buffer unchanged - SPI RX not working");
		} else if (rx_buf[0] == 0xFF) {
			LOG_WRN("RX all 0xFF - MISO floating or slave not responding");
		} else if (rx_buf[0] == 0x00) {
			LOG_WRN("RX all 0x00 - MISO stuck LOW or slave TX empty");
		} else if (rx_buf[0] == MAGIC_RESPONSE) {
			LOG_INF("Got response magic 0x55!");
		}
	}
}

/* Test 4: Two-transaction protocol (like the real driver) */
static void test_two_transaction(void)
{
	LOG_INF("========================================");
	LOG_INF("TEST 4: TWO-TRANSACTION PROTOCOL");
	LOG_INF("========================================");

	if (!device_is_ready(spi_dev)) {
		LOG_ERR("SPI device not ready");
		return;
	}

	uint8_t tx_cmd[5];
	uint8_t tx_dummy[16];
	uint8_t rx_buf[16];

	/* Build PING command */
	tx_cmd[0] = MAGIC_REQUEST;
	tx_cmd[1] = 0x01;  /* Sequence 1 */
	tx_cmd[2] = CMD_PING;
	tx_cmd[3] = 0x00;
	tx_cmd[4] = crc8_ccitt(tx_cmd, 4);

	memset(tx_dummy, 0xFF, sizeof(tx_dummy));
	memset(rx_buf, 0xAB, sizeof(rx_buf));

	/* Transaction 1: Send command */
	LOG_INF("Transaction 1: Send PING command");
	LOG_INF("  TX: %02X %02X %02X %02X %02X",
		tx_cmd[0], tx_cmd[1], tx_cmd[2], tx_cmd[3], tx_cmd[4]);

	struct spi_buf spi_tx1 = { .buf = tx_cmd, .len = sizeof(tx_cmd) };
	struct spi_buf_set tx_set1 = { .buffers = &spi_tx1, .count = 1 };

	int ret = spi_write(spi_dev, &spi_cfg, &tx_set1);
	if (ret < 0) {
		LOG_ERR("  SPI write failed: %d", ret);
		return;
	}
	LOG_INF("  TX complete, CS back to HIGH");

	/* Delay for STM32 to process */
	int delays[] = {10, 50, 100, 200};
	for (int d = 0; d < ARRAY_SIZE(delays); d++) {
		LOG_INF("");
		LOG_INF("Delay: %d ms", delays[d]);
		k_msleep(delays[d]);

		/* Transaction 2: Read response */
		LOG_INF("Transaction 2: Read response (TX=0xFF)");

		memset(rx_buf, 0xAB, sizeof(rx_buf));
		struct spi_buf spi_tx2 = { .buf = tx_dummy, .len = sizeof(tx_dummy) };
		struct spi_buf spi_rx2 = { .buf = rx_buf, .len = sizeof(rx_buf) };
		struct spi_buf_set tx_set2 = { .buffers = &spi_tx2, .count = 1 };
		struct spi_buf_set rx_set2 = { .buffers = &spi_rx2, .count = 1 };

		ret = spi_transceive(spi_dev, &spi_cfg, &tx_set2, &rx_set2);
		if (ret < 0) {
			LOG_ERR("  SPI transceive failed: %d", ret);
			continue;
		}

		LOG_INF("  RX: %02X %02X %02X %02X %02X %02X %02X %02X",
			rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
			rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

		if (rx_buf[0] == MAGIC_RESPONSE) {
			LOG_INF("  SUCCESS! Got response magic 0x55");
			LOG_INF("  Status: 0x%02X, Len: %d", rx_buf[2], rx_buf[3]);
			return;
		} else if (rx_buf[0] == 0xFF) {
			LOG_WRN("  MISO high (0xFF) - slave not sending");
		} else if (rx_buf[0] == 0x00) {
			LOG_WRN("  MISO low (0x00) - slave TX empty");
		} else {
			LOG_WRN("  Unknown data: 0x%02X", rx_buf[0]);
		}

		/* Re-send command for next delay test */
		if (d < ARRAY_SIZE(delays) - 1) {
			tx_cmd[1]++;  /* Increment sequence */
			tx_cmd[4] = crc8_ccitt(tx_cmd, 4);
			spi_tx1.buf = tx_cmd;
			spi_write(spi_dev, &spi_cfg, &tx_set1);
		}
	}
}

/* Test 5: Hardware reset */
static void test_hardware_reset(void)
{
	LOG_INF("========================================");
	LOG_INF("TEST 5: HARDWARE RESET");
	LOG_INF("========================================");

#if HAS_RESET_GPIO
	if (!gpio_is_ready_dt(&reset_gpio)) {
		LOG_ERR("Reset GPIO not ready");
		return;
	}

	int ret = gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure reset GPIO: %d", ret);
		return;
	}

	LOG_INF("Asserting reset (LOW)...");
	gpio_pin_set_dt(&reset_gpio, 1);  /* Active LOW = assert reset */
	k_msleep(100);

	LOG_INF("Releasing reset (HIGH)...");
	gpio_pin_set_dt(&reset_gpio, 0);  /* Inactive = release reset */

	LOG_INF("Waiting 500ms for device to boot...");
	k_msleep(500);

	LOG_INF("Reset complete");
#else
	LOG_WRN("Reset GPIO not configured");
#endif
}

/* Test 6: Read device parameters (PING, SN, ID, ADDR, RCONF) */
static void test_read_parameters(void)
{
	LOG_INF("========================================");
	LOG_INF("TEST 6: READ DEVICE PARAMETERS");
	LOG_INF("========================================");

	uint8_t rx_data[64];
	size_t rx_len;
	uint8_t status;
	int ret;

	/* PING */
	LOG_INF("");
	LOG_INF("--- PING (0x01) ---");
	rx_len = sizeof(rx_data);
	ret = spi_send_command(CMD_PING, NULL, 0, rx_data, &rx_len, &status);
	if (ret == 0) {
		LOG_INF("PING OK - Status: 0x%02X", status);
	} else {
		LOG_ERR("PING FAILED: %d", ret);
	}
	k_msleep(50);

	/* GET_SN */
	LOG_INF("");
	LOG_INF("--- GET_SN (0x08) ---");
	rx_len = sizeof(rx_data);
	memset(rx_data, 0, sizeof(rx_data));
	ret = spi_send_command(CMD_GET_SN, NULL, 0, rx_data, &rx_len, &status);
	if (ret == 0) {
		LOG_INF("GET_SN OK - Status: 0x%02X, Len: %d", status, (int)rx_len);
		if (rx_len > 0) {
			print_hex("  SN", rx_data, rx_len);
		}
	} else {
		LOG_ERR("GET_SN FAILED: %d", ret);
	}
	k_msleep(50);

	/* GET_ID */
	LOG_INF("");
	LOG_INF("--- GET_ID (0x04) ---");
	rx_len = sizeof(rx_data);
	memset(rx_data, 0, sizeof(rx_data));
	ret = spi_send_command(CMD_GET_ID, NULL, 0, rx_data, &rx_len, &status);
	if (ret == 0) {
		LOG_INF("GET_ID OK - Status: 0x%02X, Len: %d", status, (int)rx_len);
		if (rx_len > 0) {
			print_hex("  ID", rx_data, rx_len);
		}
	} else {
		LOG_ERR("GET_ID FAILED: %d", ret);
	}
	k_msleep(50);

	/* GET_ADDR */
	LOG_INF("");
	LOG_INF("--- GET_ADDR (0x06) ---");
	rx_len = sizeof(rx_data);
	memset(rx_data, 0, sizeof(rx_data));
	ret = spi_send_command(CMD_GET_ADDR, NULL, 0, rx_data, &rx_len, &status);
	if (ret == 0) {
		LOG_INF("GET_ADDR OK - Status: 0x%02X, Len: %d", status, (int)rx_len);
		if (rx_len > 0) {
			print_hex("  ADDR", rx_data, rx_len);
		}
	} else {
		LOG_ERR("GET_ADDR FAILED: %d", ret);
	}
	k_msleep(50);

	/* GET_RCONF (Radio Configuration) */
	LOG_INF("");
	LOG_INF("--- GET_RCONF (0x0A) ---");
	rx_len = sizeof(rx_data);
	memset(rx_data, 0, sizeof(rx_data));
	ret = spi_send_command(CMD_GET_RCONF, NULL, 0, rx_data, &rx_len, &status);
	if (ret == 0) {
		LOG_INF("GET_RCONF OK - Status: 0x%02X, Len: %d", status, (int)rx_len);
		if (rx_len > 0) {
			print_hex("  RCONF", rx_data, rx_len);
		}
	} else {
		LOG_ERR("GET_RCONF FAILED: %d", ret);
	}
}

/* Test 7: Write and re-read ID and ADDR */
static void test_write_read_params(void)
{
	LOG_INF("========================================");
	LOG_INF("TEST 7: WRITE AND READ ID/ADDR");
	LOG_INF("========================================");

	uint8_t rx_data[64];
	uint8_t original_id[32];
	uint8_t original_addr[32];
	size_t rx_len, orig_id_len, orig_addr_len;
	uint8_t status;
	int ret;

	/* First, read current ID */
	LOG_INF("");
	LOG_INF("--- Reading current ID ---");
	orig_id_len = sizeof(original_id);
	ret = spi_send_command(CMD_GET_ID, NULL, 0, original_id, &orig_id_len, &status);
	if (ret != 0) {
		LOG_ERR("Failed to read original ID: %d", ret);
		return;
	}
	print_hex("  Original ID", original_id, orig_id_len);
	k_msleep(50);

	/* Read current ADDR */
	LOG_INF("");
	LOG_INF("--- Reading current ADDR ---");
	orig_addr_len = sizeof(original_addr);
	ret = spi_send_command(CMD_GET_ADDR, NULL, 0, original_addr, &orig_addr_len, &status);
	if (ret != 0) {
		LOG_ERR("Failed to read original ADDR: %d", ret);
		return;
	}
	print_hex("  Original ADDR", original_addr, orig_addr_len);
	k_msleep(50);

	/* Write new test ID (4 bytes example) */
	LOG_INF("");
	LOG_INF("--- Writing new ID (TEST) ---");
	uint8_t new_id[] = { 0xDE, 0xAD, 0xBE, 0xEF };
	ret = spi_send_command(CMD_SET_ID, new_id, sizeof(new_id), rx_data, &rx_len, &status);
	if (ret == 0) {
		LOG_INF("SET_ID OK - Status: 0x%02X", status);
	} else {
		LOG_ERR("SET_ID FAILED: %d", ret);
	}
	k_msleep(50);

	/* Read back ID */
	LOG_INF("");
	LOG_INF("--- Reading back ID ---");
	rx_len = sizeof(rx_data);
	memset(rx_data, 0, sizeof(rx_data));
	ret = spi_send_command(CMD_GET_ID, NULL, 0, rx_data, &rx_len, &status);
	if (ret == 0) {
		LOG_INF("GET_ID OK - Status: 0x%02X, Len: %d", status, (int)rx_len);
		print_hex("  New ID", rx_data, rx_len);
	} else {
		LOG_ERR("GET_ID FAILED: %d", ret);
	}
	k_msleep(50);

	/* Write new test ADDR (4 bytes example) */
	LOG_INF("");
	LOG_INF("--- Writing new ADDR (TEST) ---");
	uint8_t new_addr[] = { 0xCA, 0xFE, 0xBA, 0xBE };
	ret = spi_send_command(CMD_SET_ADDR, new_addr, sizeof(new_addr), rx_data, &rx_len, &status);
	if (ret == 0) {
		LOG_INF("SET_ADDR OK - Status: 0x%02X", status);
	} else {
		LOG_ERR("SET_ADDR FAILED: %d", ret);
	}
	k_msleep(50);

	/* Read back ADDR */
	LOG_INF("");
	LOG_INF("--- Reading back ADDR ---");
	rx_len = sizeof(rx_data);
	memset(rx_data, 0, sizeof(rx_data));
	ret = spi_send_command(CMD_GET_ADDR, NULL, 0, rx_data, &rx_len, &status);
	if (ret == 0) {
		LOG_INF("GET_ADDR OK - Status: 0x%02X, Len: %d", status, (int)rx_len);
		print_hex("  New ADDR", rx_data, rx_len);
	} else {
		LOG_ERR("GET_ADDR FAILED: %d", ret);
	}
	k_msleep(50);

	/* Restore original ID */
	LOG_INF("");
	LOG_INF("--- Restoring original ID ---");
	ret = spi_send_command(CMD_SET_ID, original_id, orig_id_len, rx_data, &rx_len, &status);
	if (ret == 0) {
		LOG_INF("Restored original ID - Status: 0x%02X", status);
	} else {
		LOG_ERR("Failed to restore ID: %d", ret);
	}
	k_msleep(50);

	/* Restore original ADDR */
	LOG_INF("");
	LOG_INF("--- Restoring original ADDR ---");
	ret = spi_send_command(CMD_SET_ADDR, original_addr, orig_addr_len, rx_data, &rx_len, &status);
	if (ret == 0) {
		LOG_INF("Restored original ADDR - Status: 0x%02X", status);
	} else {
		LOG_ERR("Failed to restore ADDR: %d", ret);
	}
}

int main(void)
{
	LOG_INF("");
	LOG_INF("========================================");
	LOG_INF("   SPI DEBUG SAMPLE");
	LOG_INF("========================================");
	LOG_INF("Starting in 3 seconds...");
	k_msleep(3000);  /* Delay to allow RTT connection */
	LOG_INF("SPI Config:");
	LOG_INF("  Frequency: %d Hz", spi_cfg.frequency);
	LOG_INF("  Mode: 0 (CPOL=0, CPHA=0)");
	LOG_INF("  Word: 8-bit, MSB first");
	LOG_INF("");

	/* Run all tests */
	test_gpio_status();
	LOG_INF("");
	k_msleep(500);

	test_hardware_reset();
	LOG_INF("");
	k_msleep(500);

	test_raw_spi_write();
	LOG_INF("");
	k_msleep(500);

	test_raw_spi_transceive();
	LOG_INF("");
	k_msleep(500);

	test_two_transaction();
	LOG_INF("");
	k_msleep(500);

	test_read_parameters();
	LOG_INF("");
	k_msleep(500);

	test_write_read_params();

	LOG_INF("");
	LOG_INF("========================================");
	LOG_INF("   ALL TESTS COMPLETE");
	LOG_INF("========================================");
	LOG_INF("");
	LOG_INF("If SPI communication failed:");
	LOG_INF("  - Check SCK/MOSI/MISO/CS wiring");
	LOG_INF("  - Verify STM32 SPI is initialized");
	LOG_INF("  - For P0.10 (CS): run nrfjprog --eraseall && west flash");
	LOG_INF("    to enable NFC pins as GPIO in UICR");
	LOG_INF("");
	LOG_INF("Note: SECKEY command not available in SPI mode (UART only)");

	return 0;
}
