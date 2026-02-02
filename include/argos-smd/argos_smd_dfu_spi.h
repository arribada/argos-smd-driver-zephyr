/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARGOS_SMD_DFU_SPI_H
#define ARGOS_SMD_DFU_SPI_H

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stddef.h>
#include <argos-smd/argos_smd_spi.h>  /* For unified protocol status codes */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file argos_smd_dfu_spi.h
 * @brief Argos SMD DFU (Device Firmware Update) via SPI
 *
 * This module provides firmware update functionality for Argos SMD modules
 * over SPI using the Protocol A+ framing and bootloader DFU commands.
 *
 * DFU Sequence:
 * 1. Enter bootloader mode (CMD_DFU_ENTER 0x3F from application)
 * 2. Wait for bootloader to be ready (~100ms)
 * 3. Ping bootloader (DFU_PING 0x30)
 * 4. Get bootloader info (DFU_GET_INFO 0x31)
 * 5. Erase flash (DFU_ERASE 0x32) - takes 2-3 seconds
 * 6. Write firmware in chunks:
 *    a. DFU_WRITE_REQ (0x33) with address and length
 *    b. DFU_WRITE_DATA (0x34) with chunk data
 * 7. Verify CRC (DFU_VERIFY 0x37)
 * 8. Jump to application (DFU_JUMP 0x39)
 */

/* DFU chunk size for writes (256 bytes recommended) */
#define ARGOS_DFU_CHUNK_SIZE      256

/* Application header size */
#define ARGOS_DFU_HEADER_SIZE     256

/* Flash memory layout */
#define ARGOS_FLASH_BOOTLOADER    0x08000000  /* 32 KB */
#define ARGOS_FLASH_APP_HEADER    0x08008000  /* 256 bytes */
#define ARGOS_FLASH_APPLICATION   0x08008100  /* ~183 KB */
#define ARGOS_FLASH_USER          0x0803B000  /* 18 KB (preserved) */
#define ARGOS_FLASH_BL_STATE      0x0803B800  /* 2 KB */
#define ARGOS_FLASH_END           0x08040000

/* Maximum application size */
#define ARGOS_MAX_APP_SIZE        (ARGOS_FLASH_USER - ARGOS_FLASH_APPLICATION)

/*
 * Protocol A+ DFU Command Timing (delay before reading response)
 * The response arrives in the NEXT SPI transaction!
 * Uses unified timing constants from argos_smd_spi.h
 */
#define ARGOS_DFU_DELAY_PING_MS       ARGOS_TIMING_STANDARD_MS
#define ARGOS_DFU_DELAY_GET_INFO_MS   ARGOS_TIMING_STANDARD_MS
#define ARGOS_DFU_DELAY_ERASE_MS      ARGOS_TIMING_ERASE_MS  /* CRITICAL! 3000ms */
#define ARGOS_DFU_DELAY_WRITE_REQ_MS  ARGOS_TIMING_STANDARD_MS
#define ARGOS_DFU_DELAY_WRITE_DATA_MS ARGOS_TIMING_WRITE_MS
#define ARGOS_DFU_DELAY_READ_REQ_MS   ARGOS_TIMING_STANDARD_MS
#define ARGOS_DFU_DELAY_READ_DATA_MS  ARGOS_TIMING_STANDARD_MS
#define ARGOS_DFU_DELAY_VERIFY_MS     ARGOS_TIMING_STANDARD_MS
#define ARGOS_DFU_DELAY_RESET_MS      ARGOS_TIMING_RESET_MS
#define ARGOS_DFU_DELAY_JUMP_MS       ARGOS_TIMING_RESET_MS
#define ARGOS_DFU_DELAY_GET_STATUS_MS ARGOS_TIMING_STANDARD_MS
#define ARGOS_DFU_DELAY_ABORT_MS      ARGOS_TIMING_STANDARD_MS
#define ARGOS_DFU_DELAY_SET_HEADER_MS ARGOS_TIMING_WRITE_MS
#define ARGOS_DFU_DELAY_DEFAULT_MS    ARGOS_TIMING_STANDARD_MS

/* DFU timeouts */
#define ARGOS_DFU_ERASE_TIMEOUT_MS   5000   /* Flash erase polling timeout */
#define ARGOS_DFU_WRITE_TIMEOUT_MS   1000
#define ARGOS_DFU_VERIFY_TIMEOUT_MS  2000
#define ARGOS_DFU_RESET_WAIT_MS      ARGOS_TIMING_RESET_MS

/* Retry configuration */
#define ARGOS_DFU_MAX_RETRIES        3

/*
 * DFU Status Codes - Use unified enum from argos_smd_spi.h
 * Legacy aliases for backward compatibility
 */
#define ARGOS_DFU_STATUS_OK            PROT_OK
#define ARGOS_DFU_STATUS_CRC_ERROR     PROT_CRC_ERROR
#define ARGOS_DFU_STATUS_ADDR_ERROR    PROT_ADDR_ERROR
#define ARGOS_DFU_STATUS_FLASH_ERROR   PROT_FLASH_ERROR
#define ARGOS_DFU_STATUS_BUSY          PROT_BUSY
#define ARGOS_DFU_STATUS_INVALID_CMD   PROT_INVALID_CMD
#define ARGOS_DFU_STATUS_NOT_READY     PROT_NOT_READY
#define ARGOS_DFU_STATUS_VERIFY_ERROR  PROT_VERIFY_ERROR
#define ARGOS_DFU_STATUS_FRAME_CRC_ERR PROT_FRAME_CRC_ERROR

/**
 * @brief DFU operation state (from GET_STATUS extended response)
 */
enum argos_dfu_op_state {
	ARGOS_DFU_OP_IDLE      = 0,  /* Idle */
	ARGOS_DFU_OP_ERASING   = 1,  /* Erasing flash */
	ARGOS_DFU_OP_WRITING   = 2,  /* Writing data */
	ARGOS_DFU_OP_VERIFYING = 3,  /* Verifying CRC */
	ARGOS_DFU_OP_READY     = 4,  /* Ready for next operation */
	ARGOS_DFU_OP_COMPLETE  = 5,  /* DFU complete */
	ARGOS_DFU_OP_ERROR     = 6,  /* Error state */
};

/**
 * @brief DFU state machine states (legacy - for compatibility)
 */
enum argos_dfu_state {
	ARGOS_DFU_STATE_IDLE,          /* Application mode */
	ARGOS_DFU_STATE_BOOTLOADER,    /* Bootloader mode, ready for DFU */
	ARGOS_DFU_STATE_ERASING,       /* Flash erase in progress */
	ARGOS_DFU_STATE_WRITING,       /* Writing firmware chunks */
	ARGOS_DFU_STATE_VERIFYING,     /* CRC verification */
	ARGOS_DFU_STATE_COMPLETE,      /* DFU successful */
	ARGOS_DFU_STATE_ERROR,         /* DFU error */
};

/**
 * @brief Bootloader information structure
 */
struct argos_bl_info {
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t version_patch;
	uint32_t app_start_addr;
	uint32_t app_max_size;
	uint32_t page_size;
};

/**
 * @brief DFU status structure (legacy - for compatibility)
 */
struct argos_dfu_status {
	enum argos_dfu_state state;
	uint32_t bytes_written;
	uint32_t total_bytes;
	uint8_t last_error;
};

/**
 * @brief DFU extended status structure (32 bytes from GET_STATUS 0x3A)
 *
 * Protocol A+ GET_STATUS response provides detailed DFU session information.
 */
struct argos_dfu_extended_status {
	uint8_t  protocol_version;   /* Protocol version (0x01) */
	uint8_t  bootloader_state;   /* Bootloader state machine */
	uint8_t  dfu_op_state;       /* DFU operation state (enum argos_dfu_op_state) */
	uint8_t  last_error;         /* Last error code */
	uint8_t  session_active;     /* 1 if DFU session active */
	uint8_t  erase_done;         /* 1 if erase completed */
	uint8_t  verify_passed;      /* 1 if CRC verified OK */
	uint8_t  reserved;
	uint32_t received_bytes;     /* Bytes received */
	uint32_t write_address;      /* Current write address */
	uint32_t expected_crc;       /* Expected CRC */
	uint32_t calculated_crc;     /* Calculated CRC */
	uint32_t frame_count;        /* Frames processed */
	uint32_t crc_error_count;    /* CRC errors count */
} __attribute__((packed));

/**
 * @brief Progress callback function type
 *
 * @param current Current bytes transferred
 * @param total Total bytes to transfer
 * @param user_data User-supplied context pointer
 */
typedef void (*argos_dfu_progress_cb_t)(uint32_t current, uint32_t total, void *user_data);

/**
 * @brief Enter DFU bootloader mode
 *
 * Sends CMD_DFU_ENTER (0x3F) to the application, which will:
 * 1. Send ACK
 * 2. Write DFU flag to flash
 * 3. Reset to bootloader
 *
 * After calling this function, wait ~100ms then use argos_dfu_ping()
 * to verify bootloader is ready.
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int argos_dfu_enter(const struct device *dev);

/**
 * @brief Ping the DFU bootloader
 *
 * Sends DFU_PING (0x30) to verify bootloader is responding.
 * Response includes bootloader version.
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int argos_dfu_ping(const struct device *dev);

/**
 * @brief Wait for bootloader to be ready
 *
 * Polls with DFU_PING until bootloader responds or timeout.
 *
 * @param dev Pointer to device structure
 * @param timeout Maximum time to wait
 * @return 0 on success, -ETIMEDOUT if bootloader doesn't respond
 */
int argos_dfu_wait_ready(const struct device *dev, k_timeout_t timeout);

/**
 * @brief Get bootloader information
 *
 * Sends DFU_GET_INFO (0x31) to get bootloader version and flash layout.
 *
 * @param dev Pointer to device structure
 * @param info Pointer to store bootloader information
 * @return 0 on success, negative errno on failure
 */
int argos_dfu_get_info(const struct device *dev, struct argos_bl_info *info);

/**
 * @brief Erase application flash area
 *
 * Sends DFU_ERASE (0x32) to erase the application area.
 * This operation takes 2-3 seconds (~92 pages at ~22ms each).
 *
 * This function uses a fixed 3-second delay before reading the response.
 * For polling-based erase, use argos_dfu_erase_with_polling().
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int argos_dfu_erase(const struct device *dev);

/**
 * @brief Erase application flash with status polling
 *
 * Alternative to argos_dfu_erase() that uses GET_STATUS polling
 * instead of a fixed delay. Polls every 100ms until complete.
 *
 * @param dev Pointer to device structure
 * @return 0 on success, -ETIMEDOUT if erase takes too long, negative errno on failure
 */
int argos_dfu_erase_with_polling(const struct device *dev);

/**
 * @brief Write firmware chunk to flash
 *
 * Writes a chunk of firmware data using a 2-transaction protocol:
 *
 * Transaction 1 - WRITE_REQ (0x33):
 *   Master: [0x33][addr 4B][len 2B]
 *   Slave:  [status]  (slave prepares for write)
 *
 * Transaction 2 - WRITE_DATA (0x34):
 *   Master: [0x34][data...]
 *   Slave:  [status]  (slave writes data to flash)
 *
 * @param dev Pointer to device structure
 * @param addr Flash address to write to
 * @param data Pointer to data buffer
 * @param len Length of data (max ARGOS_DFU_CHUNK_SIZE)
 * @return 0 on success, negative errno on failure
 */
int argos_dfu_write_chunk(const struct device *dev, uint32_t addr,
			  const uint8_t *data, size_t len);

/**
 * @brief Read flash memory
 *
 * Reads data from flash using a 2-transaction protocol:
 *
 * Transaction 1 - READ_REQ (0x35):
 *   Master: [0x35][addr 4B][len 2B]
 *   Slave:  [status]  (slave reads flash and stores data)
 *
 * Transaction 2 - READ_DATA (0x36):
 *   Master: [0x36][dummy bytes...]
 *   Slave:  [status][data...]  (slave sends stored data)
 *
 * @param dev Pointer to device structure
 * @param addr Flash address to read from
 * @param data Buffer to store read data
 * @param len Number of bytes to read (max ARGOS_SPI_MAX_PAYLOAD)
 * @return 0 on success, negative errno on failure
 */
int argos_dfu_read(const struct device *dev, uint32_t addr, uint8_t *data, size_t len);

/**
 * @brief Verify firmware CRC
 *
 * Sends DFU_VERIFY (0x37) with CRC32 of the firmware.
 * Bootloader calculates CRC of written data and compares.
 *
 * @param dev Pointer to device structure
 * @param crc32 Expected CRC32 of firmware
 * @return 0 on success (CRC matches), -EILSEQ on mismatch
 */
int argos_dfu_verify(const struct device *dev, uint32_t crc32);

/**
 * @brief Reset the device
 *
 * Sends DFU_RESET (0x38) to perform a system reset.
 *
 * @param dev Pointer to device structure
 * @return 0 on success
 */
int argos_dfu_reset(const struct device *dev);

/**
 * @brief Jump to application
 *
 * Sends DFU_JUMP (0x39) to exit bootloader and start application.
 *
 * @param dev Pointer to device structure
 * @return 0 on success
 */
int argos_dfu_jump(const struct device *dev);

/**
 * @brief Get current DFU status (legacy interface)
 *
 * Sends DFU_GET_STATUS (0x3A) to query current DFU state.
 *
 * @param dev Pointer to device structure
 * @param status Pointer to store status
 * @return 0 on success, negative errno on failure
 */
int argos_dfu_get_status_spi(const struct device *dev, struct argos_dfu_status *status);

/**
 * @brief Get extended DFU status (32 bytes)
 *
 * Sends DFU_GET_STATUS (0x3A) and returns full 32-byte status structure.
 * This provides detailed information about the DFU session state.
 *
 * @param dev Pointer to device structure
 * @param status Pointer to store extended status (32 bytes)
 * @return 0 on success, negative errno on failure
 */
int argos_dfu_get_extended_status(const struct device *dev,
				  struct argos_dfu_extended_status *status);

/**
 * @brief Abort DFU operation
 *
 * Sends DFU_ABORT (0x3B) to cancel current DFU transfer.
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int argos_dfu_abort(const struct device *dev);

/**
 * @brief Set application header
 *
 * Sends DFU_SET_HEADER (0x3C) with 256-byte application header.
 *
 * @param dev Pointer to device structure
 * @param header Pointer to 256-byte header data
 * @return 0 on success, negative errno on failure
 */
int argos_dfu_set_header(const struct device *dev, const uint8_t *header);

/**
 * @brief Perform complete firmware update via SPI
 *
 * High-level function that performs the entire DFU sequence:
 * 1. Calculate CRC32 of firmware
 * 2. Enter bootloader mode
 * 3. Wait for bootloader ready
 * 4. Erase flash
 * 5. Write all firmware chunks
 * 6. Verify CRC
 * 7. Jump to application
 *
 * @param dev Pointer to device structure
 * @param firmware Pointer to firmware image
 * @param size Size of firmware image in bytes
 * @param progress_cb Optional progress callback (can be NULL)
 * @param user_data User data passed to progress callback
 * @return 0 on success, negative errno on failure
 */
int argos_spi_firmware_update(const struct device *dev,
			      const uint8_t *firmware, size_t size,
			      argos_dfu_progress_cb_t progress_cb, void *user_data);

/**
 * @brief Calculate CRC32 checksum (same algorithm as UART DFU)
 *
 * Uses standard polynomial 0x04C11DB7 (reflected form 0xEDB88320).
 *
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return CRC32 checksum value
 */
uint32_t argos_dfu_crc32(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ARGOS_SMD_DFU_SPI_H */
