/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef KINEIS_SPI_H
#define KINEIS_SPI_H

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file kineis_spi.h
 * @brief Kineis SPI Protocol A+ Interface
 *
 * This module implements the Protocol A+ framing for SPI communication
 * with Kineis (Argos) modules. It handles magic bytes, sequence numbers,
 * CRC-8 calculation, and response parsing.
 */

/* Protocol A+ Magic Bytes */
#define KINEIS_SPI_MAGIC_REQUEST   0xAA
#define KINEIS_SPI_MAGIC_RESPONSE  0x55

/* Maximum payload size */
#define KINEIS_SPI_MAX_PAYLOAD     255

/* Frame header size (magic + seq + cmd/status + len) */
#define KINEIS_SPI_HEADER_SIZE     4

/* CRC size */
#define KINEIS_SPI_CRC_SIZE        1

/* Maximum frame size */
#define KINEIS_SPI_MAX_FRAME_SIZE  (KINEIS_SPI_HEADER_SIZE + KINEIS_SPI_MAX_PAYLOAD + KINEIS_SPI_CRC_SIZE)

/* Timeout for SPI operations (ms) */
#define KINEIS_SPI_TIMEOUT_MS      5000

/* Application Commands (0x01-0x2A) */
#define KINEIS_CMD_PING            0x01
#define KINEIS_CMD_GET_VERSION     0x02
#define KINEIS_CMD_GET_FW_VERSION  0x03
#define KINEIS_CMD_GET_ID          0x04
#define KINEIS_CMD_SET_ID          0x05
#define KINEIS_CMD_GET_ADDR        0x06
#define KINEIS_CMD_SET_ADDR        0x07
#define KINEIS_CMD_GET_SN          0x08
#define KINEIS_CMD_SET_SN          0x09
#define KINEIS_CMD_GET_RCONF       0x0A
#define KINEIS_CMD_SET_RCONF       0x0B
#define KINEIS_CMD_TX              0x0C
/* ... more application commands ... */

/* DFU Enter Command (from application mode) */
#define KINEIS_CMD_DFU_ENTER       0x3F

/* DFU Bootloader Commands (0x30-0x3E) */
#define KINEIS_DFU_CMD_PING        0x30
#define KINEIS_DFU_CMD_GET_INFO    0x31
#define KINEIS_DFU_CMD_ERASE       0x32
#define KINEIS_DFU_CMD_WRITE_REQ   0x33
#define KINEIS_DFU_CMD_WRITE_DATA  0x34
#define KINEIS_DFU_CMD_READ_REQ    0x35
#define KINEIS_DFU_CMD_READ_DATA   0x36
#define KINEIS_DFU_CMD_VERIFY      0x37
#define KINEIS_DFU_CMD_RESET       0x38
#define KINEIS_DFU_CMD_JUMP        0x39
#define KINEIS_DFU_CMD_GET_STATUS  0x3A
#define KINEIS_DFU_CMD_ABORT       0x3B
#define KINEIS_DFU_CMD_SET_HEADER  0x3C

/**
 * @brief DFU Response codes
 */
enum kineis_dfu_response {
	KINEIS_DFU_RSP_OK            = 0x00,
	KINEIS_DFU_RSP_ERROR         = 0x01,
	KINEIS_DFU_RSP_CRC_ERROR     = 0x02,
	KINEIS_DFU_RSP_ADDR_ERROR    = 0x03,
	KINEIS_DFU_RSP_SIZE_ERROR    = 0x04,
	KINEIS_DFU_RSP_FLASH_ERROR   = 0x05,
	KINEIS_DFU_RSP_BUSY          = 0x06,
	KINEIS_DFU_RSP_INVALID_CMD   = 0x07,
	KINEIS_DFU_RSP_TIMEOUT       = 0x08,
	KINEIS_DFU_RSP_NOT_READY     = 0x09,
	KINEIS_DFU_RSP_INVALID_HEADER = 0x0A,
	KINEIS_DFU_RSP_VERIFY_ERROR  = 0x0B,
};

/**
 * @brief Application response codes
 */
enum kineis_app_response {
	KINEIS_APP_RSP_ACK           = 0x01,
	KINEIS_APP_RSP_NACK          = 0x02,
	KINEIS_APP_RSP_ERROR         = 0xFF,
};

/**
 * @brief SPI Protocol A+ Request Frame
 *
 * | MAGIC (0xAA) | SEQ | CMD | LEN | DATA[0..LEN-1] | CRC8 |
 * |    1 byte    |  1  |  1  |  1  |    0-255       |   1  |
 */
struct kineis_spi_request {
	uint8_t magic;
	uint8_t seq;
	uint8_t cmd;
	uint8_t len;
	uint8_t data[KINEIS_SPI_MAX_PAYLOAD];
	uint8_t crc;
};

/**
 * @brief SPI Protocol A+ Response Frame
 *
 * | MAGIC (0x55) | SEQ | STATUS | LEN | DATA[0..LEN-1] | CRC8 |
 * |    1 byte    |  1  |   1    |  1  |    0-255       |   1  |
 */
struct kineis_spi_response {
	uint8_t magic;
	uint8_t seq;
	uint8_t status;
	uint8_t len;
	uint8_t data[KINEIS_SPI_MAX_PAYLOAD];
	uint8_t crc;
};

/**
 * @brief Kineis SPI device configuration
 */
struct kineis_spi_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec irq_gpio;    /* Optional: interrupt/ready pin */
	struct gpio_dt_spec reset_gpio;  /* Optional: reset pin */
};

/**
 * @brief Kineis SPI device data
 */
struct kineis_spi_data {
	uint8_t seq_num;                 /* Current sequence number */
	uint8_t tx_buf[KINEIS_SPI_MAX_FRAME_SIZE];
	uint8_t rx_buf[KINEIS_SPI_MAX_FRAME_SIZE];
	struct k_mutex lock;
};

/**
 * @brief Calculate CRC-8 CCITT checksum
 *
 * Uses polynomial 0x07 (x^8 + x^2 + x + 1)
 *
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return CRC-8 checksum value
 */
uint8_t kineis_crc8_ccitt(const uint8_t *data, size_t len);

/**
 * @brief Initialize Kineis SPI interface
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int kineis_spi_init(const struct device *dev);

/**
 * @brief Send command and receive response via SPI Protocol A+
 *
 * Builds a Protocol A+ frame with the given command and payload,
 * sends it via SPI, and waits for a response frame.
 *
 * @param dev Pointer to device structure
 * @param cmd Command byte (0x01-0x3F)
 * @param tx_data Pointer to payload data (can be NULL if tx_len is 0)
 * @param tx_len Length of payload data
 * @param rx_data Buffer to receive response data (can be NULL)
 * @param rx_len Pointer to receive response data length
 * @param status Pointer to receive response status code
 * @return 0 on success, negative errno on failure
 */
int kineis_spi_transact(const struct device *dev, uint8_t cmd,
			const uint8_t *tx_data, size_t tx_len,
			uint8_t *rx_data, size_t *rx_len, uint8_t *status);

/**
 * @brief Send command without waiting for response
 *
 * Useful for commands that cause a reset (e.g., DFU_ENTER)
 *
 * @param dev Pointer to device structure
 * @param cmd Command byte
 * @param tx_data Pointer to payload data (can be NULL)
 * @param tx_len Length of payload data
 * @return 0 on success, negative errno on failure
 */
int kineis_spi_send_only(const struct device *dev, uint8_t cmd,
			 const uint8_t *tx_data, size_t tx_len);

/**
 * @brief Ping the device (application mode)
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int kineis_spi_ping(const struct device *dev);

/**
 * @brief Get firmware version
 *
 * @param dev Pointer to device structure
 * @param version Buffer to receive version string (at least 32 bytes)
 * @param version_len Pointer to version buffer length
 * @return 0 on success, negative errno on failure
 */
int kineis_spi_get_version(const struct device *dev, char *version, size_t *version_len);

/**
 * @brief Perform hardware reset of the Kineis module
 *
 * Uses the reset GPIO (if configured) to perform a hardware reset.
 * The reset signal is held for 50ms, then the function waits 500ms
 * for the module to boot.
 *
 * @param dev Pointer to device structure
 * @return 0 on success, -ENOTSUP if reset GPIO not configured,
 *         negative errno on GPIO failure
 */
int kineis_spi_reset(const struct device *dev);

/**
 * @brief Run SPI diagnostic test
 *
 * Performs a diagnostic check of the SPI bus and module communication.
 * Outputs detailed information about GPIO states, TX/RX data, and
 * analysis of potential issues.
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int kineis_spi_diagnostic(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* KINEIS_SPI_H */
