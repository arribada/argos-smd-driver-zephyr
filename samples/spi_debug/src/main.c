/*
 * Copyright (c) 2025 Arribada Initiative
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI Debug Sample - Pipelined Single-Transaction Protocol
 *
 * This sample tests the pipelined SPI protocol:
 * - Each transaction is 64 bytes
 * - Master sends command, slave sends response to PREVIOUS command
 * - For immediate response, send command then NOP
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

/* SPI config - Mode 0, 8-bit, MSB first */
static struct spi_config spi_cfg = {
	.frequency = 125000,  /* 125 kHz */
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	.slave = 0,
	.cs = {
		.gpio = GPIO_DT_SPEC_GET_BY_IDX(SPI_NODE, cs_gpios, 0),
		.delay = 0,
	},
};

/* Protocol constants */
#define SPI_TRANSACTION_SIZE  64   /* Fixed transaction size */
#define MAGIC_REQUEST         0xAA
#define MAGIC_RESPONSE        0x55
#define IDLE_PATTERN          0xAA

/* Commands - must match argos_smd_spi.h */
#define CMD_NOP         0x00  /* No operation - just to get previous response */
#define CMD_PING        0x02  /* ARGOS_SPI_CMD_PING */
#define CMD_GET_VERSION 0x05  /* ARGOS_SPI_CMD_READ_VERSION */
#define CMD_GET_ADDR    0x07  /* ARGOS_SPI_CMD_READ_ADDR */
#define CMD_GET_ID      0x08  /* ARGOS_SPI_CMD_READ_ID */
#define CMD_GET_SN      0x09  /* ARGOS_SPI_CMD_READ_SN */
#define CMD_GET_RCONF   0x0A  /* ARGOS_SPI_CMD_READ_RCONF */

/* Timing */
#define INTER_TRANSACTION_DELAY_MS  5   /* Small delay between transactions */
#define INTER_TEST_DELAY_MS         100 /* Delay between tests */

/* Sequence number */
static uint8_t g_seq_num = 0;

/* Test statistics */
static int tests_passed = 0;
static int tests_failed = 0;

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

/**
 * @brief Do a single 64-byte SPI transaction
 *
 * @param tx_buf TX buffer (64 bytes)
 * @param rx_buf RX buffer (64 bytes)
 * @return 0 on success, negative on error
 */
static int spi_transaction(uint8_t *tx_buf, uint8_t *rx_buf)
{
	struct spi_buf spi_tx = { .buf = tx_buf, .len = SPI_TRANSACTION_SIZE };
	struct spi_buf spi_rx = { .buf = rx_buf, .len = SPI_TRANSACTION_SIZE };
	struct spi_buf_set tx_set = { .buffers = &spi_tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &spi_rx, .count = 1 };

	return spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
}

/**
 * @brief Build a command frame in buffer
 *
 * @param buf Output buffer (must be SPI_TRANSACTION_SIZE bytes)
 * @param cmd Command byte
 * @param data Optional command data
 * @param data_len Length of data
 * @return Frame length
 */
static size_t build_command_frame(uint8_t *buf, uint8_t cmd,
				  const uint8_t *data, size_t data_len)
{
	/* Build frame: MAGIC SEQ CMD LEN DATA CRC */
	buf[0] = MAGIC_REQUEST;
	buf[1] = g_seq_num++;
	buf[2] = cmd;
	buf[3] = (uint8_t)data_len;

	if (data_len > 0 && data != NULL) {
		memcpy(&buf[4], data, data_len);
	}

	buf[4 + data_len] = crc8_ccitt(buf, 4 + data_len);

	/* Fill rest with 0xFF */
	size_t frame_len = 5 + data_len;
	memset(&buf[frame_len], 0xFF, SPI_TRANSACTION_SIZE - frame_len);

	return frame_len;
}

/**
 * @brief Find response in RX buffer
 *
 * @param rx_buf RX buffer
 * @param status Output: response status
 * @param data Output: response data (optional)
 * @param data_len In/Out: data buffer size / received data length
 * @return 0 if response found, -1 if not found
 */
static int parse_response(const uint8_t *rx_buf, uint8_t *status,
			  uint8_t *data, size_t *data_len)
{
	/* Find response magic, skipping idle pattern */
	int offset = 0;
	while (offset < SPI_TRANSACTION_SIZE - 5) {
		if (rx_buf[offset] == MAGIC_RESPONSE) {
			break;
		}
		if (rx_buf[offset] != IDLE_PATTERN && rx_buf[offset] != 0xFF) {
			/* Unknown pattern - might be partial response */
			break;
		}
		offset++;
	}

	if (offset >= SPI_TRANSACTION_SIZE - 5 || rx_buf[offset] != MAGIC_RESPONSE) {
		return -1;  /* No valid response */
	}

	/* Parse response: MAGIC SEQ STATUS LEN DATA CRC */
	uint8_t rsp_status = rx_buf[offset + 2];
	uint8_t rsp_len = rx_buf[offset + 3];

	if (status) {
		*status = rsp_status;
	}

	if (data && data_len && rsp_len > 0) {
		size_t copy_len = (rsp_len < *data_len) ? rsp_len : *data_len;
		memcpy(data, &rx_buf[offset + 4], copy_len);
		*data_len = copy_len;
	} else if (data_len) {
		*data_len = 0;
	}

	return 0;
}

/**
 * @brief Send command and get response using pipelined protocol
 *
 * Sends two transactions:
 * 1. Send command, get previous response (ignore)
 * 2. Send NOP, get response to our command
 *
 * @return 0 on success, negative on error
 */
static int spi_send_command(uint8_t cmd, const uint8_t *tx_data, size_t tx_len,
			    uint8_t *rx_data, size_t *rx_len, uint8_t *status)
{
	uint8_t tx_buf[SPI_TRANSACTION_SIZE];
	uint8_t rx_buf[SPI_TRANSACTION_SIZE];
	int ret;

	if (!device_is_ready(spi_dev)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	/* Transaction 1: Send command */
	size_t frame_len = build_command_frame(tx_buf, cmd, tx_data, tx_len);
	LOG_DBG("TX1 [%s]: %02X %02X %02X %02X %02X",
		cmd == CMD_NOP ? "NOP" : "CMD",
		tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4]);

	memset(rx_buf, 0x00, sizeof(rx_buf));
	ret = spi_transaction(tx_buf, rx_buf);
	if (ret < 0) {
		LOG_ERR("Transaction 1 failed: %d", ret);
		return ret;
	}

	LOG_DBG("RX1: %02X %02X %02X %02X %02X %02X %02X %02X",
		rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
		rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

	/* Small delay for STM32 to process */
	k_msleep(INTER_TRANSACTION_DELAY_MS);

	/* Transaction 2: Send NOP to get response */
	build_command_frame(tx_buf, CMD_NOP, NULL, 0);
	LOG_DBG("TX2 [NOP]: %02X %02X %02X %02X %02X",
		tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4]);

	memset(rx_buf, 0x00, sizeof(rx_buf));
	ret = spi_transaction(tx_buf, rx_buf);
	if (ret < 0) {
		LOG_ERR("Transaction 2 failed: %d", ret);
		return ret;
	}

	LOG_DBG("RX2: %02X %02X %02X %02X %02X %02X %02X %02X",
		rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
		rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

	/* Parse response */
	ret = parse_response(rx_buf, status, rx_data, rx_len);
	if (ret < 0) {
		LOG_ERR("No valid response (RX: %02X %02X %02X %02X)",
			rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
		return -EBADMSG;
	}

	LOG_DBG("Response: STATUS=0x%02X LEN=%d", *status, rx_len ? (int)*rx_len : 0);
	return 0;
}

/* Test 1: GPIO status check */
static void test_gpio_status(void)
{
	LOG_INF("========================================");
	LOG_INF("TEST 1: GPIO STATUS CHECK");
	LOG_INF("========================================");

	LOG_INF("CS GPIO: Port=%s, Pin=%d",
		cs_gpio.port ? cs_gpio.port->name : "NULL", cs_gpio.pin);

	if (!gpio_is_ready_dt(&cs_gpio)) {
		LOG_ERR("CS GPIO NOT READY!");
		tests_failed++;
	} else {
		LOG_INF("CS GPIO: READY");
		tests_passed++;
	}
}

/* Test 2: Basic SPI communication */
static void test_basic_spi(void)
{
	LOG_INF("========================================");
	LOG_INF("TEST 2: BASIC SPI COMMUNICATION");
	LOG_INF("========================================");

	uint8_t tx_buf[SPI_TRANSACTION_SIZE];
	uint8_t rx_buf[SPI_TRANSACTION_SIZE];

	/* Send a simple transaction to check hardware */
	memset(tx_buf, 0xFF, sizeof(tx_buf));
	memset(rx_buf, 0xAB, sizeof(rx_buf));

	int ret = spi_transaction(tx_buf, rx_buf);
	if (ret < 0) {
		LOG_ERR("SPI transaction failed: %d", ret);
		tests_failed++;
		return;
	}

	LOG_INF("TX: FF FF FF FF ...");
	LOG_INF("RX: %02X %02X %02X %02X %02X %02X %02X %02X",
		rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
		rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

	/* Check if we got something back */
	if (rx_buf[0] == 0xAB) {
		LOG_ERR("RX buffer unchanged - SPI RX not working");
		tests_failed++;
	} else if (rx_buf[0] == IDLE_PATTERN) {
		LOG_INF("Got idle pattern (0xAA) - STM32 is responding!");
		tests_passed++;
	} else if (rx_buf[0] == 0xFF) {
		LOG_WRN("Got 0xFF - MISO might be floating");
		tests_passed++;  /* Hardware works, just no slave response */
	} else {
		LOG_INF("Got pattern 0x%02X - communication OK", rx_buf[0]);
		tests_passed++;
	}
}

/* Test 3: PING command */
static void test_ping(void)
{
	LOG_INF("========================================");
	LOG_INF("TEST 3: PING (Pipelined Protocol)");
	LOG_INF("========================================");

	uint8_t status;
	int ret = spi_send_command(CMD_PING, NULL, 0, NULL, NULL, &status);

	if (ret == 0) {
		LOG_INF("PING OK - Status: 0x%02X", status);
		tests_passed++;
	} else {
		LOG_ERR("PING FAILED: %d", ret);
		tests_failed++;
	}
}

/* Test 4: Read device parameters */
static void test_read_parameters(void)
{
	LOG_INF("========================================");
	LOG_INF("TEST 4: READ DEVICE PARAMETERS");
	LOG_INF("========================================");

	uint8_t rx_data[32];
	size_t rx_len;
	uint8_t status;
	int ret;
	int local_pass = 0, local_fail = 0;

	struct {
		const char *name;
		uint8_t cmd;
	} cmds[] = {
		{ "GET_SN",    CMD_GET_SN },
		{ "GET_ID",    CMD_GET_ID },
		{ "GET_ADDR",  CMD_GET_ADDR },
		{ "GET_RCONF", CMD_GET_RCONF },
	};

	for (int i = 0; i < ARRAY_SIZE(cmds); i++) {
		LOG_INF("");
		LOG_INF("--- %s (0x%02X) ---", cmds[i].name, cmds[i].cmd);

		rx_len = sizeof(rx_data);
		memset(rx_data, 0, sizeof(rx_data));

		ret = spi_send_command(cmds[i].cmd, NULL, 0, rx_data, &rx_len, &status);

		if (ret == 0) {
			LOG_INF("%s OK - Status: 0x%02X, Len: %d",
				cmds[i].name, status, (int)rx_len);
			if (rx_len > 0) {
				/* For SN, print as string */
				if (cmds[i].cmd == CMD_GET_SN) {
					rx_data[rx_len] = '\0';  /* Null terminate */
					LOG_INF("  SN: \"%s\"", rx_data);
				} else {
					/* Print hex data */
					LOG_INF("  Data[%d]: %02X %02X %02X %02X %02X %02X %02X %02X",
						(int)rx_len,
						rx_data[0], rx_data[1], rx_data[2], rx_data[3],
						rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
				}
			}
			local_pass++;
		} else {
			LOG_ERR("%s FAILED: %d", cmds[i].name, ret);
			local_fail++;
		}

		k_msleep(50);  /* Small delay between commands */
	}

	if (local_fail == 0) {
		tests_passed++;
		LOG_INF("All parameter reads passed");
	} else {
		tests_failed++;
		LOG_WRN("Parameter reads: %d passed, %d failed", local_pass, local_fail);
	}
}

/* Test 5: Continuous pipelined transactions */
static void test_continuous(void)
{
	LOG_INF("========================================");
	LOG_INF("TEST 5: CONTINUOUS TRANSACTIONS");
	LOG_INF("========================================");

	uint8_t tx_buf[SPI_TRANSACTION_SIZE];
	uint8_t rx_buf[SPI_TRANSACTION_SIZE];
	int success = 0;
	int fail = 0;

	LOG_INF("Sending 10 PING commands in sequence...");

	for (int i = 0; i < 10; i++) {
		/* Build PING command */
		build_command_frame(tx_buf, CMD_PING, NULL, 0);
		memset(rx_buf, 0, sizeof(rx_buf));

		int ret = spi_transaction(tx_buf, rx_buf);
		if (ret < 0) {
			LOG_ERR("Transaction %d failed: %d", i, ret);
			fail++;
			continue;
		}

		/* Check for valid response (from previous command) */
		if (i > 0) {  /* First response is garbage */
			uint8_t status;
			if (parse_response(rx_buf, &status, NULL, NULL) == 0) {
				success++;
				LOG_DBG("Transaction %d: got response status=0x%02X", i, status);
			} else {
				LOG_DBG("Transaction %d: no response (RX=%02X)", i, rx_buf[0]);
			}
		}

		k_msleep(5);  /* Small delay */
	}

	/* One more NOP to get last response */
	build_command_frame(tx_buf, CMD_NOP, NULL, 0);
	memset(rx_buf, 0, sizeof(rx_buf));
	spi_transaction(tx_buf, rx_buf);

	uint8_t status;
	if (parse_response(rx_buf, &status, NULL, NULL) == 0) {
		success++;
	}

	LOG_INF("Results: %d responses received out of 10 commands", success);

	if (success >= 8) {  /* Allow some margin */
		tests_passed++;
	} else {
		tests_failed++;
	}
}

int main(void)
{
	LOG_INF("");
	LOG_INF("========================================");
	LOG_INF("   SPI DEBUG - Pipelined Protocol");
	LOG_INF("========================================");
	LOG_INF("");
	LOG_INF("Protocol: Single-transaction pipelined");
	LOG_INF("Transaction size: %d bytes", SPI_TRANSACTION_SIZE);
	LOG_INF("SPI Frequency: %d Hz", spi_cfg.frequency);
	LOG_INF("Mode: 0 (CPOL=0, CPHA=0)");
	LOG_INF("");
	LOG_INF("How it works:");
	LOG_INF("  - Each transaction is 64 bytes");
	LOG_INF("  - Master sends CMD, gets response to PREVIOUS cmd");
	LOG_INF("  - For immediate response, send CMD then NOP");
	LOG_INF("");
	LOG_INF("Starting in 2 seconds...");
	k_msleep(2000);

	/* Run tests */
	test_gpio_status();
	k_msleep(INTER_TEST_DELAY_MS);

	test_basic_spi();
	k_msleep(INTER_TEST_DELAY_MS);

	test_ping();
	k_msleep(INTER_TEST_DELAY_MS);

	test_read_parameters();
	k_msleep(INTER_TEST_DELAY_MS);

	test_continuous();

	/* Summary */
	LOG_INF("");
	LOG_INF("========================================");
	LOG_INF("   TEST RESULTS");
	LOG_INF("========================================");
	LOG_INF("  Passed: %d", tests_passed);
	LOG_INF("  Failed: %d", tests_failed);
	LOG_INF("========================================");

	if (tests_failed > 0) {
		LOG_INF("");
		LOG_INF("Troubleshooting:");
		LOG_INF("  - Check wiring (SCK, MOSI, MISO, CS)");
		LOG_INF("  - Verify STM32 is running and SPI initialized");
		LOG_INF("  - Check STM32 debug log for errors");
	}

	return 0;
}
