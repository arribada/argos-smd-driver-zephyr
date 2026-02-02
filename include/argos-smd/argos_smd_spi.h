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
#include <stdbool.h>

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
 * Pipelined Protocol Constants
 * Each SPI transaction is fixed 64 bytes (full-duplex)
 * Master sends command, receives response to PREVIOUS command
 * For immediate response, send CMD then NOP
 */
#define ARGOS_SPI_TRANSACTION_SIZE    64    /* Fixed transaction size */
#define ARGOS_SPI_IDLE_PATTERN        0xAA  /* Idle byte pattern from slave */

/*
 * Unified Timing Constants (Bootloader + Application)
 */
#define ARGOS_TIMING_STANDARD_MS      15    /* Standard commands */
#define ARGOS_TIMING_WRITE_MS         20    /* Flash write commands */
#define ARGOS_TIMING_TX_DATA_MS       100   /* TX data commands */
#define ARGOS_TIMING_ERASE_MS         3000  /* CRITICAL! Flash erase */
#define ARGOS_TIMING_RESET_MS         100   /* Reset/jump commands */
#define ARGOS_TIMING_POLL_MS          200   /* Polling interval */

/* Legacy timing aliases */
#define ARGOS_SPI_PIPELINE_DELAY_MS   ARGOS_TIMING_STANDARD_MS
#define ARGOS_SPI_FLASH_DELAY_MS      150   /* Kept for backward compat */

/*
 * Application Commands (0x00-0x2A)
 */
#define ARGOS_SPI_CMD_NOP               0x00  /* No operation (get previous response) */
#define ARGOS_SPI_CMD_NONE              0x00  /* Alias for CMD_NOP */
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
 * @brief Unified Protocol Status Codes (Bootloader + Application)
 *
 * Single enum for all protocol responses. Standard codes 0x00-0x0F,
 * protocol-specific codes 0x10+.
 */
enum argos_protocol_status {
	/* Standard status codes (0x00-0x0F) */
	PROT_OK              = 0x00,  /* Success */
	PROT_ERROR           = 0x01,  /* Generic error */
	PROT_CRC_ERROR       = 0x02,  /* CRC mismatch (data) */
	PROT_ADDR_ERROR      = 0x03,  /* Invalid address */
	PROT_SIZE_ERROR      = 0x04,  /* Invalid size */
	PROT_FLASH_ERROR     = 0x05,  /* Flash failed */
	PROT_BUSY            = 0x06,  /* Retry later */
	PROT_INVALID_CMD     = 0x07,  /* Unknown command */
	PROT_TIMEOUT         = 0x08,  /* Timeout */
	PROT_NOT_READY       = 0x09,  /* Prerequisite missing */
	PROT_INVALID_HEADER  = 0x0A,  /* Bad header */
	PROT_VERIFY_ERROR    = 0x0B,  /* Verification failed */
	/* Protocol-specific (0x10+) */
	PROT_FRAME_CRC_ERROR = 0x10,  /* Frame CRC - RESEND */
	PROT_SEQ_ERROR       = 0x11,  /* Sequence mismatch */
	PROT_FRAME_ERROR     = 0x12,  /* Malformed frame */
};

/**
 * @brief MAC Status (Application only - via CMD 0x03)
 *
 * Returned when querying MAC status after TX operations.
 * Use argos_is_tx_complete() and argos_is_tx_failed() to check result.
 *
 * Mapping from KNS_MAC events:
 * - KNS_MAC_TX_DONE      → MAC_TX_DONE (0x02)
 * - KNS_MAC_TXACK_DONE   → MAC_TXACK_DONE (0x04)
 * - KNS_MAC_TX_TIMEOUT   → MAC_TX_TIMEOUT (0x05)
 * - KNS_MAC_RX_RECEIVED/DL_BC/DL_ACK → MAC_RX_RECEIVED (0x0B)
 */
enum argos_mac_status {
	MAC_UNKNOWN       = 0x00,
	MAC_OK            = 0x01,  /* Ready */
	MAC_TX_DONE       = 0x02,  /* TX Success! */
	MAC_TX_SIZE_ERROR = 0x03,
	MAC_TXACK_DONE    = 0x04,  /* TX+ACK Success! */
	MAC_TX_TIMEOUT    = 0x05,  /* TX Failed - timeout */
	MAC_TXACK_TIMEOUT = 0x06,  /* TX ACK Failed - timeout */
	MAC_RX_ERROR      = 0x07,  /* RX Failed */
	MAC_RX_TIMEOUT    = 0x08,  /* RX Failed - timeout */
	MAC_ERROR         = 0x09,  /* Generic error */
	MAC_TX_IN_PROGRESS = 0x0A, /* TX queued, poll until done */
	MAC_RX_RECEIVED   = 0x0B,  /* RX data available (DL_BC, DL_ACK) */
	MAC_SAT_DETECTED  = 0x0C,  /* Satellite detected */
	MAC_SAT_LOST      = 0x0D,  /* Satellite lost */
	MAC_RF_ABORTED    = 0x0E,  /* RF operation aborted */
};

/**
 * @brief Check if error is recoverable (should retry)
 *
 * @param status Protocol status code
 * @return true if BUSY or FRAME_CRC_ERROR (should retry)
 */
static inline bool argos_is_recoverable(uint8_t status)
{
	return (status == PROT_BUSY || status == PROT_FRAME_CRC_ERROR);
}

/**
 * @brief Check if TX completed successfully
 *
 * @param mac_status MAC status from CMD 0x03
 * @return true if TX_DONE or TXACK_DONE
 */
static inline bool argos_is_tx_complete(uint8_t mac_status)
{
	return (mac_status == MAC_TX_DONE || mac_status == MAC_TXACK_DONE);
}

/**
 * @brief Check if TX failed
 *
 * @param mac_status MAC status from CMD 0x03
 * @return true if TX timed out or errored
 */
static inline bool argos_is_tx_failed(uint8_t mac_status)
{
	return (mac_status == MAC_TX_TIMEOUT || mac_status == MAC_TXACK_TIMEOUT ||
		mac_status == MAC_RX_ERROR || mac_status == MAC_ERROR);
}

/**
 * @brief Check if TX is in progress (should continue polling)
 *
 * @param mac_status MAC status from CMD 0x03
 * @return true if TX_IN_PROGRESS
 */
static inline bool argos_is_tx_pending(uint8_t mac_status)
{
	return (mac_status == MAC_TX_IN_PROGRESS);
}

/* Legacy aliases for backward compatibility */
#define ARGOS_SPI_RSP_OK          PROT_OK
#define ARGOS_SPI_RSP_ERROR       PROT_ERROR
#define ARGOS_SPI_RSP_CRC_ERROR   PROT_CRC_ERROR
#define ARGOS_SPI_RSP_INVALID_CMD PROT_INVALID_CMD
#define ARGOS_SPI_RSP_INVALID_LEN PROT_SIZE_ERROR
#define ARGOS_SPI_RSP_BUSY        PROT_BUSY
#define ARGOS_SPI_RSP_SEQ_ERROR   PROT_SEQ_ERROR

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
	uint8_t tx_buf[ARGOS_SPI_TRANSACTION_SIZE];
	uint8_t rx_buf[ARGOS_SPI_TRANSACTION_SIZE];
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

/**
 * @brief Raw SPI transaction for bootloader DFU (no Protocol A+ framing)
 *
 * The STM32 bootloader expects raw commands without Protocol A+ framing:
 * - TX: [CMD] [PAYLOAD...]
 * - RX: [STATUS] [RESPONSE_DATA...]
 *
 * This function handles the timing between TX and RX phases (10-20ms delay)
 * required for the bootloader to process the command.
 *
 * @param dev Pointer to device structure
 * @param cmd Command byte (0x30-0x3F for DFU commands)
 * @param tx_data Pointer to payload data (can be NULL if tx_len is 0)
 * @param tx_len Length of payload data
 * @param rx_data Buffer to receive response data (can be NULL)
 * @param rx_len Pointer to expected/received response data length
 * @param status Pointer to receive DFU status code
 * @return 0 on success, negative errno on failure
 */
int argos_spi_transact_raw(const struct device *dev, uint8_t cmd,
			   const uint8_t *tx_data, size_t tx_len,
			   uint8_t *rx_data, size_t *rx_len, uint8_t *status);

/**
 * @brief Raw SPI transaction with custom timeout for bootloader DFU
 *
 * Same as argos_spi_transact_raw() but with configurable timeout.
 * Use this for long operations like ERASE which can take 2-3 seconds.
 *
 * @param dev Pointer to device structure
 * @param cmd Command byte (0x30-0x3F for DFU commands)
 * @param tx_data Pointer to payload data (can be NULL if tx_len is 0)
 * @param tx_len Length of payload data
 * @param rx_data Buffer to receive response data (can be NULL)
 * @param rx_len Pointer to expected/received response data length
 * @param status Pointer to receive DFU status code
 * @param timeout_ms Timeout in milliseconds to wait for operation
 * @return 0 on success, negative errno on failure
 */
int argos_spi_transact_raw_timeout(const struct device *dev, uint8_t cmd,
				   const uint8_t *tx_data, size_t tx_len,
				   uint8_t *rx_data, size_t *rx_len, uint8_t *status,
				   uint32_t timeout_ms);

/**
 * @brief Raw SPI send-only for bootloader DFU (no response expected)
 *
 * Used for commands that cause immediate reset (RESET, JUMP) where
 * no response can be read.
 *
 * @param dev Pointer to device structure
 * @param cmd Command byte
 * @param tx_data Pointer to payload data (can be NULL)
 * @param tx_len Length of payload data
 * @return 0 on success, negative errno on failure
 */
int argos_spi_send_only_raw(const struct device *dev, uint8_t cmd,
			    const uint8_t *tx_data, size_t tx_len);

/*
 * High-level API functions (like UART driver)
 * These wrap the low-level SPI protocol details.
 */

/**
 * @brief Get device serial number
 *
 * @param dev Pointer to device structure
 * @param sn Buffer to receive serial number string (null-terminated)
 * @param sn_len Pointer to buffer size, updated with actual length
 * @return 0 on success, negative errno on failure
 */
int argos_spi_get_sn(const struct device *dev, char *sn, size_t *sn_len);

/**
 * @brief Get device ID
 *
 * @param dev Pointer to device structure
 * @param id Buffer to receive device ID (4 bytes)
 * @param id_len Pointer to buffer size, updated with actual length
 * @return 0 on success, negative errno on failure
 */
int argos_spi_get_id(const struct device *dev, uint8_t *id, size_t *id_len);

/**
 * @brief Set device ID
 *
 * @param dev Pointer to device structure
 * @param id Device ID to set (4 bytes)
 * @param id_len Length of ID data
 * @return 0 on success, negative errno on failure
 */
int argos_spi_set_id(const struct device *dev, const uint8_t *id, size_t id_len);

/**
 * @brief Get device address
 *
 * @param dev Pointer to device structure
 * @param addr Buffer to receive address (4 bytes)
 * @param addr_len Pointer to buffer size, updated with actual length
 * @return 0 on success, negative errno on failure
 */
int argos_spi_get_addr(const struct device *dev, uint8_t *addr, size_t *addr_len);

/**
 * @brief Set device address
 *
 * @param dev Pointer to device structure
 * @param addr Address to set (4 bytes)
 * @param addr_len Length of address data
 * @return 0 on success, negative errno on failure
 */
int argos_spi_set_addr(const struct device *dev, const uint8_t *addr, size_t addr_len);

/**
 * @brief Get radio configuration
 *
 * @param dev Pointer to device structure
 * @param rconf Buffer to receive radio config
 * @param rconf_len Pointer to buffer size, updated with actual length
 * @return 0 on success, negative errno on failure
 */
int argos_spi_get_rconf(const struct device *dev, uint8_t *rconf, size_t *rconf_len);

/**
 * @brief Get MAC status (CMD 0x03)
 *
 * @param dev Pointer to device structure
 * @param mac_status Pointer to receive MAC status (enum argos_mac_status)
 * @return 0 on success, negative errno on failure
 */
int argos_spi_get_mac_status(const struct device *dev, uint8_t *mac_status);

/**
 * @brief Wait for TX completion after CMD_WRITE_TX (0x16)
 *
 * Polls MAC status (CMD 0x03) until TX_DONE/TXACK_DONE or error.
 * Use this after sending data with argos_spi_write_tx().
 *
 * @param dev Pointer to device structure
 * @param timeout Timeout for polling
 * @return 0 on success (TX_DONE/TXACK_DONE), -EIO on TX error, -ETIMEDOUT
 */
int argos_spi_wait_tx_complete(const struct device *dev, k_timeout_t timeout);

/**
 * @brief Send TX data (uplink message)
 *
 * Sequence: CMD_WRITE_TX_REQ (0x14) → CMD_WRITE_TX_SIZE (0x15) → CMD_WRITE_TX (0x16)
 * After success, use argos_spi_wait_tx_complete() to poll for TX result.
 *
 * @param dev Pointer to device structure
 * @param data TX payload data
 * @param len Length of payload (max 250 bytes)
 * @return 0 on success (TX queued), negative errno on failure
 */
int argos_spi_write_tx(const struct device *dev, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ARGOS_SMD_SPI_H */
