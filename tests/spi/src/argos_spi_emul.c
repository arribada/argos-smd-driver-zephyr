/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Protocol A+ SPI Emulator for Argos SMD driver testing.
 *
 * Emulates the pipelined protocol where:
 * - Transaction 1: Master sends CMD, emulator returns idle pattern (0xAA)
 * - Transaction 2: Master sends NOP, emulator returns response to previous CMD
 */

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <argos-smd/argos_smd_spi.h>

LOG_MODULE_REGISTER(argos_spi_emul, LOG_LEVEL_DBG);

#define DT_DRV_COMPAT arribada_argos_smd_spi

struct argos_spi_emul_data {
	/* Pending command from previous transaction */
	uint8_t pending_cmd;
	uint8_t pending_seq;
	uint8_t pending_data[ARGOS_SPI_MAX_PAYLOAD];
	uint8_t pending_data_len;
	bool has_pending;

	/* Emulated device state */
	char version[32];
	char sn[32];
	uint8_t id[4];
	size_t id_len;
	uint8_t addr[4];
	size_t addr_len;
};

/**
 * @brief Build a Protocol A+ response frame into a 64-byte buffer
 */
static void build_response(uint8_t *buf, uint8_t seq, uint8_t status,
			    const uint8_t *data, uint8_t data_len)
{
	size_t idx = 0;

	buf[idx++] = ARGOS_SPI_MAGIC_RESPONSE;
	buf[idx++] = seq;
	buf[idx++] = status;
	buf[idx++] = data_len;

	if (data && data_len > 0) {
		memcpy(&buf[idx], data, data_len);
		idx += data_len;
	}

	/* CRC-8 over magic + seq + status + len + data */
	buf[idx] = argos_spi_crc8_ccitt(buf, idx);
	idx++;

	/* Pad with 0xFF */
	if (idx < ARGOS_SPI_TRANSACTION_SIZE) {
		memset(&buf[idx], 0xFF, ARGOS_SPI_TRANSACTION_SIZE - idx);
	}
}

/**
 * @brief Handle a command and build the response
 */
static void handle_command(struct argos_spi_emul_data *data,
			   uint8_t *rx_buf, uint8_t cmd, uint8_t seq,
			   const uint8_t *cmd_data, uint8_t cmd_data_len)
{
	switch (cmd) {
	case ARGOS_SPI_CMD_PING:
		build_response(rx_buf, seq, PROT_OK, NULL, 0);
		break;

	case ARGOS_SPI_CMD_READ_VERSION:
		build_response(rx_buf, seq, PROT_OK,
			       (const uint8_t *)data->version,
			       strlen(data->version));
		break;

	case ARGOS_SPI_CMD_READ_SN:
		build_response(rx_buf, seq, PROT_OK,
			       (const uint8_t *)data->sn,
			       strlen(data->sn));
		break;

	case ARGOS_SPI_CMD_READ_ID:
		build_response(rx_buf, seq, PROT_OK,
			       data->id, data->id_len);
		break;

	case ARGOS_SPI_CMD_READ_ADDR:
		build_response(rx_buf, seq, PROT_OK,
			       data->addr, data->addr_len);
		break;

	/* 2-phase write: REQ commands just return OK */
	case ARGOS_SPI_CMD_WRITE_ID_REQ:
	case ARGOS_SPI_CMD_WRITE_ADDR_REQ:
		build_response(rx_buf, seq, PROT_OK, NULL, 0);
		break;

	/* 2-phase write: WRITE commands store data and return OK */
	case ARGOS_SPI_CMD_WRITE_ID:
		if (cmd_data_len > 0 && cmd_data_len <= sizeof(data->id)) {
			memcpy(data->id, cmd_data, cmd_data_len);
			data->id_len = cmd_data_len;
		}
		build_response(rx_buf, seq, PROT_OK, NULL, 0);
		break;

	case ARGOS_SPI_CMD_WRITE_ADDR:
		if (cmd_data_len > 0 && cmd_data_len <= sizeof(data->addr)) {
			memcpy(data->addr, cmd_data, cmd_data_len);
			data->addr_len = cmd_data_len;
		}
		build_response(rx_buf, seq, PROT_OK, NULL, 0);
		break;

	default:
		LOG_WRN("Unhandled command: 0x%02X", cmd);
		build_response(rx_buf, seq, PROT_INVALID_CMD, NULL, 0);
		break;
	}
}

/**
 * @brief SPI emulator IO callback - handles each 64-byte SPI transaction
 *
 * Implements the pipelined Protocol A+:
 * - CMD transaction: store the command, return idle pattern
 * - NOP transaction: return response to previously stored command
 */
static int argos_spi_emul_io(const struct emul *target,
			     const struct spi_config *config,
			     const struct spi_buf_set *tx_bufs,
			     const struct spi_buf_set *rx_bufs)
{
	struct argos_spi_emul_data *data = target->data;

	ARG_UNUSED(config);

	if (!tx_bufs || tx_bufs->count < 1 || !rx_bufs || rx_bufs->count < 1) {
		return -EINVAL;
	}

	const uint8_t *tx = tx_bufs->buffers[0].buf;
	uint8_t *rx = rx_bufs->buffers[0].buf;
	size_t len = tx_bufs->buffers[0].len;

	/* Default: fill RX with idle pattern */
	memset(rx, ARGOS_SPI_IDLE_PATTERN, len);

	/* Parse TX frame - must have at least header + CRC */
	if (len >= (ARGOS_SPI_HEADER_SIZE + ARGOS_SPI_CRC_SIZE) &&
	    tx[0] == ARGOS_SPI_MAGIC_REQUEST) {
		uint8_t seq = tx[1];
		uint8_t cmd = tx[2];
		uint8_t data_len = tx[3];

		LOG_DBG("RX cmd=0x%02X seq=%u len=%u pending=%d",
			cmd, seq, data_len, data->has_pending);

		if (cmd == ARGOS_SPI_CMD_NOP && data->has_pending) {
			/* NOP with pending command: return the response */
			handle_command(data, rx, data->pending_cmd, seq,
				       data->pending_data,
				       data->pending_data_len);
			data->has_pending = false;
		} else if (cmd != ARGOS_SPI_CMD_NOP) {
			/* Store command for next transaction */
			data->pending_cmd = cmd;
			data->pending_seq = seq;
			data->pending_data_len = data_len;
			if (data_len > 0 &&
			    data_len <= ARGOS_SPI_MAX_PAYLOAD) {
				memcpy(data->pending_data, &tx[4], data_len);
			}
			data->has_pending = true;
		}
	}

	return 0;
}

static int argos_spi_emul_init(const struct emul *target,
			       const struct device *parent)
{
	struct argos_spi_emul_data *data = target->data;

	ARG_UNUSED(parent);

	/* Initialize emulated device state */
	strcpy(data->version, "1.0.0-test");
	strcpy(data->sn, "SN12345678");
	data->id[0] = 0xDE;
	data->id[1] = 0xAD;
	data->id[2] = 0xBE;
	data->id[3] = 0xEF;
	data->id_len = 4;
	data->addr[0] = 0x01;
	data->addr[1] = 0x02;
	data->addr[2] = 0x03;
	data->addr[3] = 0x04;
	data->addr_len = 4;
	data->has_pending = false;

	LOG_INF("Argos SMD SPI emulator initialized");
	return 0;
}

static const struct spi_emul_api argos_spi_emul_api = {
	.io = argos_spi_emul_io,
};

#define ARGOS_SPI_EMUL_DEFINE(n)                                     \
	static struct argos_spi_emul_data argos_spi_emul_data_##n;   \
	EMUL_DT_INST_DEFINE(n, argos_spi_emul_init,                  \
			    &argos_spi_emul_data_##n, NULL,           \
			    &argos_spi_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(ARGOS_SPI_EMUL_DEFINE)
