/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARGOS_SMD_SPI_H
#define ARGOS_SMD_SPI_H

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file argos_smd_spi.h
 * @brief Argos SMD SPI Protocol A+ Interface
 *
 * This module implements the Protocol A+ framing for SPI communication
 * with Argos SMD modules. It handles magic bytes, sequence numbers,
 * CRC-8 calculation, and response parsing.
 *
 * Frame format:
 * [MAGIC(1)] [SEQ(1)] [CMD/STATUS(1)] [LEN(1)] [DATA(0-250)] [CRC(1)]
 */

/* Protocol A+ Magic Bytes */
#define ARGOS_SPI_MAGIC_REQUEST   0xAA
#define ARGOS_SPI_MAGIC_RESPONSE  0x55

/* Maximum payload size */
#define ARGOS_SPI_MAX_PAYLOAD     250

/* Frame header size (magic + seq + cmd/status + len) */
#define ARGOS_SPI_HEADER_SIZE     4

/* CRC size */
#define ARGOS_SPI_CRC_SIZE        1

/* Maximum frame size */
#define ARGOS_SPI_MAX_FRAME_SIZE  (ARGOS_SPI_HEADER_SIZE + ARGOS_SPI_MAX_PAYLOAD + ARGOS_SPI_CRC_SIZE)

/* Timeout for SPI operations (ms) */
#define ARGOS_SPI_TIMEOUT_MS      5000

/*
 * Application Commands (0x00-0x2A)
 */
#define ARGOS_SPI_CMD_NONE              0x00  /* Unknown command */
#define ARGOS_SPI_CMD_READ              0x01  /* Generic read */
#define ARGOS_SPI_CMD_PING              0x02  /* Ping */
#define ARGOS_SPI_CMD_MAC_STATUS        0x03  /* MAC status */
#define ARGOS_SPI_CMD_SPI_STATUS        0x04  /* SPI status */
#define ARGOS_SPI_CMD_READ_VERSION      0x05  /* Firmware version */
#define ARGOS_SPI_CMD_READ_FIRMWARE     0x06  /* Firmware detailed info */
#define ARGOS_SPI_CMD_READ_ADDR         0x07  /* Device address */
#define ARGOS_SPI_CMD_READ_ID           0x08  /* Device ID */
#define ARGOS_SPI_CMD_READ_SN           0x09  /* Serial number */
#define ARGOS_SPI_CMD_READ_RCONF        0x0A  /* Radio configuration */
#define ARGOS_SPI_CMD_WRITE_RCONF_REQ   0x0B  /* Write radio config request */
#define ARGOS_SPI_CMD_WRITE_RCONF       0x0C  /* Write radio config */
#define ARGOS_SPI_CMD_SAVE_RCONF        0x0D  /* Save radio config to NVM */
#define ARGOS_SPI_CMD_READ_KMAC         0x0E  /* KMAC profile */
#define ARGOS_SPI_CMD_WRITE_KMAC_REQ    0x0F  /* Write KMAC request */
#define ARGOS_SPI_CMD_WRITE_KMAC        0x10  /* Write KMAC */
#define ARGOS_SPI_CMD_READ_LPM          0x11  /* Low power mode config */
#define ARGOS_SPI_CMD_WRITE_LPM_REQ     0x12  /* Write LPM request */
#define ARGOS_SPI_CMD_WRITE_LPM         0x13  /* Write LPM */
#define ARGOS_SPI_CMD_WRITE_TX_REQ      0x14  /* TX uplink request */
#define ARGOS_SPI_CMD_WRITE_TX_SIZE     0x15  /* TX size (uint16) */
#define ARGOS_SPI_CMD_WRITE_TX          0x16  /* TX data */
#define ARGOS_SPI_CMD_READ_CW           0x17  /* CW params */
#define ARGOS_SPI_CMD_WRITE_CW_REQ      0x18  /* Write CW request */
#define ARGOS_SPI_CMD_WRITE_CW          0x19  /* Write CW */
#define ARGOS_SPI_CMD_READ_PREPASSEN    0x1A  /* Prepass status */
#define ARGOS_SPI_CMD_WRITE_PREPASSEN_REQ 0x1B /* Write prepass request */
#define ARGOS_SPI_CMD_WRITE_PREPASSEN   0x1C  /* Write prepass */
#define ARGOS_SPI_CMD_READ_UDATE        0x1D  /* UTC date/time */
#define ARGOS_SPI_CMD_WRITE_UDATE_REQ   0x1E  /* Write date request */
#define ARGOS_SPI_CMD_WRITE_UDATE       0x1F  /* Write date */
#define ARGOS_SPI_CMD_WRITE_ID_REQ      0x20  /* Write ID request */
#define ARGOS_SPI_CMD_WRITE_ID          0x21  /* Write ID */
#define ARGOS_SPI_CMD_WRITE_ADDR_REQ    0x22  /* Write address request */
#define ARGOS_SPI_CMD_WRITE_ADDR        0x23  /* Write address */
#define ARGOS_SPI_CMD_READ_SECKEY       0x24  /* Secret key */
#define ARGOS_SPI_CMD_WRITE_SECKEY_REQ  0x25  /* Write secret key request */
#define ARGOS_SPI_CMD_WRITE_SECKEY      0x26  /* Write secret key */
#define ARGOS_SPI_CMD_READ_SPIMAC_STATE 0x27  /* SPI MAC state */
#define ARGOS_SPI_CMD_READ_TCXO_WU      0x28  /* TCXO wake-up */
#define ARGOS_SPI_CMD_WRITE_TCXOWU_REQ  0x29  /* Write TCXO request */
#define ARGOS_SPI_CMD_WRITE_TCXOWU      0x2A  /* Write TCXO */

/*
 * DFU Bootloader Commands (0x30-0x3F)
 */
#define ARGOS_SPI_DFU_CMD_PING          0x30  /* Ping bootloader */
#define ARGOS_SPI_DFU_CMD_GET_INFO      0x31  /* Bootloader info (version, memory) */
#define ARGOS_SPI_DFU_CMD_ERASE         0x32  /* Erase application zone */
#define ARGOS_SPI_DFU_CMD_WRITE_REQ     0x33  /* Write request (address+size) */
#define ARGOS_SPI_DFU_CMD_WRITE_DATA    0x34  /* Write flash chunk */
#define ARGOS_SPI_DFU_CMD_READ_REQ      0x35  /* Read request (address+size) */
#define ARGOS_SPI_DFU_CMD_READ_DATA     0x36  /* Read flash */
#define ARGOS_SPI_DFU_CMD_VERIFY        0x37  /* Verify CRC32 */
#define ARGOS_SPI_DFU_CMD_RESET         0x38  /* Reset device */
#define ARGOS_SPI_DFU_CMD_JUMP          0x39  /* Jump to application */
#define ARGOS_SPI_DFU_CMD_GET_STATUS    0x3A  /* DFU session status */
#define ARGOS_SPI_DFU_CMD_ABORT         0x3B  /* Abort DFU session */
#define ARGOS_SPI_DFU_CMD_SET_HEADER    0x3C  /* Set application header */
#define ARGOS_SPI_CMD_DFU_ENTER         0x3F  /* Enter DFU mode (from app) */

/**
 * @brief Application response status codes
 */
enum argos_spi_status {
	ARGOS_SPI_RSP_OK          = 0x00,  /* Success */
	ARGOS_SPI_RSP_ERROR       = 0x01,  /* Generic error */
	ARGOS_SPI_RSP_CRC_ERROR   = 0x02,  /* CRC error */
	ARGOS_SPI_RSP_INVALID_CMD = 0x03,  /* Unknown command */
	ARGOS_SPI_RSP_INVALID_LEN = 0x04,  /* Invalid length */
	ARGOS_SPI_RSP_BUSY        = 0x05,  /* Busy, retry later */
	ARGOS_SPI_RSP_SEQ_ERROR   = 0x06,  /* Sequence error */
};

/**
 * @brief DFU response status codes
 */
enum argos_spi_dfu_status {
	ARGOS_DFU_RSP_OK             = 0x00,  /* Success */
	ARGOS_DFU_RSP_ERROR          = 0x01,  /* Generic error */
	ARGOS_DFU_RSP_CRC_ERROR      = 0x02,  /* CRC error */
	ARGOS_DFU_RSP_ADDR_ERROR     = 0x03,  /* Invalid address */
	ARGOS_DFU_RSP_SIZE_ERROR     = 0x04,  /* Invalid size */
	ARGOS_DFU_RSP_FLASH_ERROR    = 0x05,  /* Flash error */
	ARGOS_DFU_RSP_BUSY           = 0x06,  /* Busy */
	ARGOS_DFU_RSP_INVALID_CMD    = 0x07,  /* Unknown command */
	ARGOS_DFU_RSP_TIMEOUT        = 0x08,  /* Timeout */
	ARGOS_DFU_RSP_NOT_READY      = 0x09,  /* Not ready */
	ARGOS_DFU_RSP_INVALID_HEADER = 0x0A,  /* Invalid header */
	ARGOS_DFU_RSP_VERIFY_ERROR   = 0x0B,  /* Verification error */
};

/**
 * @brief SPI Protocol A+ Request Frame
 *
 * | MAGIC (0xAA) | SEQ | CMD | LEN | DATA[0..LEN-1] | CRC8 |
 * |    1 byte    |  1  |  1  |  1  |    0-250       |   1  |
 */
struct argos_spi_request {
	uint8_t magic;
	uint8_t seq;
	uint8_t cmd;
	uint8_t len;
	uint8_t data[ARGOS_SPI_MAX_PAYLOAD];
	uint8_t crc;
};

/**
 * @brief SPI Protocol A+ Response Frame
 *
 * | MAGIC (0x55) | SEQ | STATUS | LEN | DATA[0..LEN-1] | CRC8 |
 * |    1 byte    |  1  |   1    |  1  |    0-250       |   1  |
 */
struct argos_spi_response {
	uint8_t magic;
	uint8_t seq;
	uint8_t status;
	uint8_t len;
	uint8_t data[ARGOS_SPI_MAX_PAYLOAD];
	uint8_t crc;
};

/**
 * @brief Argos SMD SPI device configuration
 */
struct argos_spi_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec irq_gpio;    /* Optional: interrupt/ready pin */
	struct gpio_dt_spec reset_gpio;  /* Optional: reset pin */
};

/**
 * @brief Argos SMD SPI device data
 */
struct argos_spi_data {
	uint8_t seq_num;                 /* Current sequence number */
	uint8_t tx_buf[ARGOS_SPI_MAX_FRAME_SIZE];
	uint8_t rx_buf[ARGOS_SPI_MAX_FRAME_SIZE];
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
uint8_t argos_spi_crc8_ccitt(const uint8_t *data, size_t len);

/**
 * @brief Initialize Argos SMD SPI interface
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int argos_spi_init(const struct device *dev);

/**
 * @brief Send command and receive response via SPI Protocol A+
 *
 * Builds a Protocol A+ frame with the given command and payload,
 * sends it via SPI, and waits for a response frame.
 *
 * @param dev Pointer to device structure
 * @param cmd Command byte (0x00-0x3F)
 * @param tx_data Pointer to payload data (can be NULL if tx_len is 0)
 * @param tx_len Length of payload data
 * @param rx_data Buffer to receive response data (can be NULL)
 * @param rx_len Pointer to receive response data length
 * @param status Pointer to receive response status code
 * @return 0 on success, negative errno on failure
 */
int argos_spi_transact(const struct device *dev, uint8_t cmd,
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
int argos_spi_send_only(const struct device *dev, uint8_t cmd,
			const uint8_t *tx_data, size_t tx_len);

/**
 * @brief Ping the device (application mode)
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int argos_spi_ping(const struct device *dev);

/**
 * @brief Get firmware version
 *
 * @param dev Pointer to device structure
 * @param version Buffer to receive version string (at least 32 bytes)
 * @param version_len Pointer to version buffer length
 * @return 0 on success, negative errno on failure
 */
int argos_spi_get_version(const struct device *dev, char *version, size_t *version_len);

/**
 * @brief Perform hardware reset of the Argos SMD module
 *
 * Uses the reset GPIO (if configured) to perform a hardware reset.
 * The reset signal is held for 50ms, then the function waits 500ms
 * for the module to boot.
 *
 * @param dev Pointer to device structure
 * @return 0 on success, -ENOTSUP if reset GPIO not configured,
 *         negative errno on GPIO failure
 */
int argos_spi_reset(const struct device *dev);

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
int argos_spi_diagnostic(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ARGOS_SMD_SPI_H */
