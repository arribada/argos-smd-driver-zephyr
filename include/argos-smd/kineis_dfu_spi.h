/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef KINEIS_DFU_SPI_H
#define KINEIS_DFU_SPI_H

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file kineis_dfu_spi.h
 * @brief Kineis DFU (Device Firmware Update) via SPI
 *
 * This module provides firmware update functionality for Kineis modules
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
#define KINEIS_DFU_CHUNK_SIZE      256

/* Application header size */
#define KINEIS_DFU_HEADER_SIZE     256

/* Flash memory layout */
#define KINEIS_FLASH_BOOTLOADER    0x08000000  /* 32 KB */
#define KINEIS_FLASH_APP_HEADER    0x08008000  /* 256 bytes */
#define KINEIS_FLASH_APPLICATION   0x08008100  /* ~183 KB */
#define KINEIS_FLASH_USER          0x0803B000  /* 18 KB (preserved) */
#define KINEIS_FLASH_BL_STATE      0x0803B800  /* 2 KB */
#define KINEIS_FLASH_END           0x08040000

/* Maximum application size */
#define KINEIS_MAX_APP_SIZE        (KINEIS_FLASH_USER - KINEIS_FLASH_APPLICATION)

/* DFU timeouts */
#define KINEIS_DFU_ERASE_TIMEOUT_MS   5000   /* Flash erase takes 2-3 seconds */
#define KINEIS_DFU_WRITE_TIMEOUT_MS   1000
#define KINEIS_DFU_VERIFY_TIMEOUT_MS  2000
#define KINEIS_DFU_RESET_WAIT_MS      100

/**
 * @brief DFU state machine states
 */
enum kineis_dfu_state {
	KINEIS_DFU_STATE_IDLE,          /* Application mode */
	KINEIS_DFU_STATE_BOOTLOADER,    /* Bootloader mode, ready for DFU */
	KINEIS_DFU_STATE_ERASING,       /* Flash erase in progress */
	KINEIS_DFU_STATE_WRITING,       /* Writing firmware chunks */
	KINEIS_DFU_STATE_VERIFYING,     /* CRC verification */
	KINEIS_DFU_STATE_COMPLETE,      /* DFU successful */
	KINEIS_DFU_STATE_ERROR,         /* DFU error */
};

/**
 * @brief Bootloader information structure
 */
struct kineis_bl_info {
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t version_patch;
	uint32_t app_start_addr;
	uint32_t app_max_size;
	uint32_t page_size;
};

/**
 * @brief DFU status structure
 */
struct kineis_dfu_status {
	enum kineis_dfu_state state;
	uint32_t bytes_written;
	uint32_t total_bytes;
	uint8_t last_error;
};

/**
 * @brief Progress callback function type
 *
 * @param current Current bytes transferred
 * @param total Total bytes to transfer
 * @param user_data User-supplied context pointer
 */
typedef void (*kineis_dfu_progress_cb_t)(uint32_t current, uint32_t total, void *user_data);

/**
 * @brief Enter DFU bootloader mode
 *
 * Sends CMD_DFU_ENTER (0x3F) to the application, which will:
 * 1. Send ACK
 * 2. Write DFU flag to flash
 * 3. Reset to bootloader
 *
 * After calling this function, wait ~100ms then use kineis_dfu_ping()
 * to verify bootloader is ready.
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int kineis_dfu_enter(const struct device *dev);

/**
 * @brief Ping the DFU bootloader
 *
 * Sends DFU_PING (0x30) to verify bootloader is responding.
 * Response includes bootloader version.
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int kineis_dfu_ping(const struct device *dev);

/**
 * @brief Wait for bootloader to be ready
 *
 * Polls with DFU_PING until bootloader responds or timeout.
 *
 * @param dev Pointer to device structure
 * @param timeout Maximum time to wait
 * @return 0 on success, -ETIMEDOUT if bootloader doesn't respond
 */
int kineis_dfu_wait_ready(const struct device *dev, k_timeout_t timeout);

/**
 * @brief Get bootloader information
 *
 * Sends DFU_GET_INFO (0x31) to get bootloader version and flash layout.
 *
 * @param dev Pointer to device structure
 * @param info Pointer to store bootloader information
 * @return 0 on success, negative errno on failure
 */
int kineis_dfu_get_info(const struct device *dev, struct kineis_bl_info *info);

/**
 * @brief Erase application flash area
 *
 * Sends DFU_ERASE (0x32) to erase the application area.
 * This operation takes 2-3 seconds.
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int kineis_dfu_erase(const struct device *dev);

/**
 * @brief Write firmware chunk to flash
 *
 * Writes a chunk of firmware data at the specified address.
 * Uses DFU_WRITE_REQ (0x33) followed by DFU_WRITE_DATA (0x34).
 *
 * @param dev Pointer to device structure
 * @param addr Flash address to write to
 * @param data Pointer to data buffer
 * @param len Length of data (max KINEIS_DFU_CHUNK_SIZE)
 * @return 0 on success, negative errno on failure
 */
int kineis_dfu_write_chunk(const struct device *dev, uint32_t addr,
			   const uint8_t *data, size_t len);

/**
 * @brief Read flash memory
 *
 * Reads data from flash using DFU_READ_REQ (0x35) and DFU_READ_DATA (0x36).
 *
 * @param dev Pointer to device structure
 * @param addr Flash address to read from
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return 0 on success, negative errno on failure
 */
int kineis_dfu_read(const struct device *dev, uint32_t addr, uint8_t *data, size_t len);

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
int kineis_dfu_verify(const struct device *dev, uint32_t crc32);

/**
 * @brief Reset the device
 *
 * Sends DFU_RESET (0x38) to perform a system reset.
 *
 * @param dev Pointer to device structure
 * @return 0 on success
 */
int kineis_dfu_reset(const struct device *dev);

/**
 * @brief Jump to application
 *
 * Sends DFU_JUMP (0x39) to exit bootloader and start application.
 *
 * @param dev Pointer to device structure
 * @return 0 on success
 */
int kineis_dfu_jump(const struct device *dev);

/**
 * @brief Get current DFU status
 *
 * Sends DFU_GET_STATUS (0x3A) to query current DFU state.
 *
 * @param dev Pointer to device structure
 * @param status Pointer to store status
 * @return 0 on success, negative errno on failure
 */
int kineis_dfu_get_status(const struct device *dev, struct kineis_dfu_status *status);

/**
 * @brief Abort DFU operation
 *
 * Sends DFU_ABORT (0x3B) to cancel current DFU transfer.
 *
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno on failure
 */
int kineis_dfu_abort(const struct device *dev);

/**
 * @brief Set application header
 *
 * Sends DFU_SET_HEADER (0x3C) with 256-byte application header.
 *
 * @param dev Pointer to device structure
 * @param header Pointer to 256-byte header data
 * @return 0 on success, negative errno on failure
 */
int kineis_dfu_set_header(const struct device *dev, const uint8_t *header);

/**
 * @brief Perform complete firmware update
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
int kineis_firmware_update(const struct device *dev,
			   const uint8_t *firmware, size_t size,
			   kineis_dfu_progress_cb_t progress_cb, void *user_data);

/**
 * @brief Calculate CRC32 checksum (same algorithm as UART DFU)
 *
 * Uses standard polynomial 0x04C11DB7 (reflected form 0xEDB88320).
 *
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return CRC32 checksum value
 */
uint32_t kineis_dfu_crc32(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* KINEIS_DFU_SPI_H */
