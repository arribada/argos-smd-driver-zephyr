/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARGOS_DFU_H
#define ARGOS_DFU_H

#include <zephyr/device.h>
#include <zephyr/kernel.h>

/**
 * @file argos_dfu.h
 * @brief Argos SMD Device Firmware Update (DFU) API
 *
 * This API provides functions to perform over-the-air (OTA) firmware updates
 * on Argos SMD modules via UART using AT commands.
 *
 * The DFU process involves:
 * 1. Entering bootloader mode from application mode
 * 2. Starting a DFU session with firmware size and CRC32
 * 3. Sending firmware data in chunks
 * 4. Finalizing the update and verifying CRC
 * 5. Rebooting to the new firmware
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum size of a single DFU data chunk in bytes
 *
 * Each chunk is sent as hex-encoded ASCII (2 chars per byte)
 * over AT+DFUDATA command. Maximum recommended is 64 bytes binary = 128 hex chars
 */
#define ARGOS_DFU_CHUNK_SIZE   64

/**
 * @brief Default timeout for DFU operations in milliseconds
 */
#define ARGOS_DFU_TIMEOUT_MS   5000

/**
 * @brief DFU error codes returned by the device
 */
enum argos_dfu_error {
	ARGOS_DFU_ERR_NONE = 0,         /**< No error */
	ARGOS_DFU_ERR_INVALID_SIZE = 1, /**< Invalid firmware size */
	ARGOS_DFU_ERR_INVALID_CRC = 2,  /**< CRC32 mismatch */
	ARGOS_DFU_ERR_FLASH_WRITE = 3,  /**< Flash write error */
	ARGOS_DFU_ERR_FLASH_ERASE = 4,  /**< Flash erase error */
	ARGOS_DFU_ERR_NOT_STARTED = 5,  /**< DFU session not started */
	ARGOS_DFU_ERR_OVERFLOW = 6,     /**< Data exceeds declared size */
	ARGOS_DFU_ERR_TIMEOUT = 7,      /**< Timeout between chunks */
};

/**
 * @brief DFU state machine states
 */
enum argos_dfu_state {
	ARGOS_DFU_IDLE,         /**< No active DFU session */
	ARGOS_DFU_STARTED,      /**< DFU session started */
	ARGOS_DFU_TRANSFERRING, /**< Transferring data chunks */
	ARGOS_DFU_VERIFYING,    /**< Verifying CRC */
	ARGOS_DFU_COMPLETE,     /**< DFU complete, ready to reboot */
	ARGOS_DFU_ERROR,        /**< DFU error occurred */
};

/**
 * @brief Progress callback function type
 *
 * Called periodically during DFU to report progress
 *
 * @param current Number of bytes transferred so far
 * @param total Total firmware size in bytes
 */
typedef void (*argos_dfu_progress_cb_t)(uint32_t current, uint32_t total);

/**
 * @brief Enter bootloader mode from application mode
 *
 * Sends AT+BOOT command to reboot the device into bootloader mode.
 * The device will reboot and you must wait for bootloader ready before
 * proceeding with DFU operations.
 *
 * @param dev Pointer to the Argos SMD device
 * @return 0 on success, negative errno code on failure
 */
int argos_enter_bootloader(const struct device *dev);

/**
 * @brief Wait for bootloader to be ready
 *
 * Polls the device with AT+PING commands until it responds,
 * indicating the bootloader is ready for DFU operations.
 *
 * @param dev Pointer to the Argos SMD device
 * @param timeout Maximum time to wait for bootloader
 * @return 0 on success, -ETIMEDOUT if bootloader doesn't respond
 */
int argos_wait_bootloader_ready(const struct device *dev, k_timeout_t timeout);

/**
 * @brief Start a DFU session
 *
 * Sends AT+DFUSTART=<size>,<crc32> command to initialize a firmware update session.
 * Must be called before sending any data chunks.
 *
 * @param dev Pointer to the Argos SMD device
 * @param fw_size Total firmware size in bytes
 * @param fw_crc CRC32 checksum of the entire firmware image
 * @return 0 on success, negative errno code on failure
 */
int argos_dfu_start(const struct device *dev, uint32_t fw_size, uint32_t fw_crc);

/**
 * @brief Send a chunk of firmware data
 *
 * Sends AT+DFUDATA=<hex_data> command with firmware data encoded as hex ASCII.
 * Data length must not exceed ARGOS_DFU_CHUNK_SIZE bytes.
 *
 * @param dev Pointer to the Argos SMD device
 * @param data Pointer to binary firmware data chunk
 * @param len Length of data chunk in bytes (max ARGOS_DFU_CHUNK_SIZE)
 * @return 0 on success, negative errno code on failure
 */
int argos_dfu_send_chunk(const struct device *dev, const uint8_t *data, size_t len);

/**
 * @brief Finalize the DFU session
 *
 * Sends AT+DFUEND command to complete the firmware update.
 * The device will verify the CRC32, finalize the flash write,
 * and reboot into the new firmware if successful.
 *
 * @param dev Pointer to the Argos SMD device
 * @return 0 on success, negative errno code on failure
 */
int argos_dfu_finish(const struct device *dev);

/**
 * @brief Abort an active DFU session
 *
 * Sends AT+DFUABORT command to cancel the current DFU session
 * and return to idle state.
 *
 * @param dev Pointer to the Argos SMD device
 * @return 0 on success, negative errno code on failure
 */
int argos_dfu_abort(const struct device *dev);

/**
 * @brief Get current DFU status
 *
 * Sends AT+DFUSTATUS=? command to query the current DFU progress and state.
 *
 * @param dev Pointer to the Argos SMD device
 * @param progress Pointer to store current progress (bytes transferred)
 * @param state Pointer to store current DFU state
 * @return 0 on success, negative errno code on failure
 */
int argos_dfu_get_status(const struct device *dev,
			 uint32_t *progress, enum argos_dfu_state *state);

/**
 * @brief Perform a complete OTA firmware update
 *
 * High-level function that performs the entire OTA update sequence:
 * 1. Calculate CRC32 of firmware image
 * 2. Enter bootloader mode
 * 3. Wait for bootloader ready
 * 4. Start DFU session
 * 5. Send all firmware chunks
 * 6. Finalize DFU
 * 7. Verify device rebooted successfully
 *
 * @param dev Pointer to the Argos SMD device
 * @param fw_image Pointer to complete firmware image in memory
 * @param fw_size Size of firmware image in bytes
 * @param progress_cb Optional callback for progress updates (can be NULL)
 * @return 0 on success, negative errno code on failure
 */
int argos_ota_update(const struct device *dev,
		     const uint8_t *fw_image, size_t fw_size,
		     argos_dfu_progress_cb_t progress_cb);

/**
 * @brief Calculate CRC32 checksum
 *
 * Utility function to calculate CRC32 checksum of firmware image.
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

#endif /* ARGOS_DFU_H */
