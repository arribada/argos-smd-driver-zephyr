/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARGOS_DFU_H
#define ARGOS_DFU_H

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <argos-smd/argos_crc.h>  /* For argos_dfu_crc32() */

/**
 * @file argos_dfu.h
 * @brief Argos SMD Device Firmware Update (DFU) API - UART Protocol
 *
 * This API provides functions to perform over-the-air (OTA) firmware updates
 * on Argos SMD modules via UART using AT+DFU commands.
 *
 * Protocol commands (compatible with STM32WL bootloader):
 *   AT+DFU=PING              - Check bootloader is ready
 *   AT+DFU=ERASE             - Erase application flash
 *   AT+DFU=WRITE,<addr>,<hex> - Write data at address
 *   AT+DFU=VERIFY,<crc>      - Verify firmware CRC32
 *   AT+DFU=JUMP              - Jump to application
 *   AT+DFU=ABORT             - Abort DFU session
 *
 * The DFU process involves:
 * 1. Entering bootloader mode from application mode (AT+BOOT)
 * 2. Waiting for bootloader ready (AT+DFU=PING)
 * 3. Erasing flash (AT+DFU=ERASE)
 * 4. Sending firmware data in chunks with addresses (AT+DFU=WRITE)
 * 5. Verifying CRC (AT+DFU=VERIFY)
 * 6. Jumping to new firmware (AT+DFU=JUMP)
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum size of a single DFU data chunk in bytes
 *
 * Each chunk is sent as hex-encoded ASCII (2 chars per byte)
 * over AT+DFU=WRITE command. Chunk size of 64 bytes = 128 hex chars.
 * Must be aligned to 8 bytes for STM32WL flash programming.
 */
#define ARGOS_DFU_CHUNK_SIZE   64

/**
 * @brief Application flash base address (STM32WL)
 */
#define ARGOS_DFU_APP_BASE     0x08000000UL

/**
 * @brief Default timeout for DFU operations in milliseconds
 */
#define ARGOS_DFU_TIMEOUT_MS   5000

/**
 * @brief Timeout for response after sending command
 */
#define ARGOS_DFU_RESPONSE_TIMEOUT_MS  2000

/**
 * @brief Delay after entering bootloader mode
 * Device needs time to reboot into bootloader
 */
#define ARGOS_DFU_BOOT_DELAY_MS  1000

/**
 * @brief Delay after erase command (flash erase takes time)
 * STM32WL erases ~102 pages, requires up to 5 seconds
 */
#define ARGOS_DFU_ERASE_DELAY_MS  5000

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
 * Polls the device with AT+DFU=PING commands until it responds,
 * indicating the bootloader is ready for DFU operations.
 *
 * @param dev Pointer to the Argos SMD device
 * @param timeout Maximum time to wait for bootloader
 * @return 0 on success, -ETIMEDOUT if bootloader doesn't respond
 */
int argos_wait_bootloader_ready(const struct device *dev, k_timeout_t timeout);

/**
 * @brief Ping the bootloader
 *
 * Sends AT+DFU=PING command to check if bootloader is ready.
 *
 * @param dev Pointer to the Argos SMD device
 * @return 0 on success, negative errno code on failure
 */
int argos_dfu_ping(const struct device *dev);

/**
 * @brief Erase application flash
 *
 * Sends AT+DFU=ERASE command to erase the application flash region.
 * Must be called before writing any data.
 *
 * @param dev Pointer to the Argos SMD device
 * @return 0 on success, negative errno code on failure
 */
int argos_dfu_erase(const struct device *dev);

/**
 * @brief Write a chunk of firmware data at specified address
 *
 * Sends AT+DFU=WRITE,<addr>,<hex_data> command to write firmware data.
 * Data is encoded as hex ASCII. Length must not exceed ARGOS_DFU_CHUNK_SIZE bytes.
 *
 * @param dev Pointer to the Argos SMD device
 * @param addr Flash address to write to (must be 8-byte aligned)
 * @param data Pointer to binary firmware data chunk
 * @param len Length of data chunk in bytes (max ARGOS_DFU_CHUNK_SIZE)
 * @return 0 on success, negative errno code on failure
 */
int argos_dfu_write(const struct device *dev, uint32_t addr,
		    const uint8_t *data, size_t len);

/**
 * @brief Verify firmware CRC32
 *
 * Sends AT+DFU=VERIFY,<crc32> command to verify the written firmware.
 *
 * @param dev Pointer to the Argos SMD device
 * @param crc32 Expected CRC32 of the firmware
 * @return 0 on success, negative errno code on failure
 */
int argos_dfu_verify(const struct device *dev, uint32_t crc32);

/**
 * @brief Jump to application
 *
 * Sends AT+DFU=JUMP command to start the new application.
 * Only succeeds if firmware was verified successfully.
 *
 * @param dev Pointer to the Argos SMD device
 * @return 0 on success, negative errno code on failure
 */
int argos_dfu_jump(const struct device *dev);

/**
 * @brief Abort an active DFU session
 *
 * Sends AT+DFU=ABORT command to cancel the current DFU session
 * and return to idle state.
 *
 * @param dev Pointer to the Argos SMD device
 * @return 0 on success, negative errno code on failure
 */
int argos_dfu_abort(const struct device *dev);

/**
 * @brief Get current DFU status
 *
 * Sends AT+DFU=STATUS command to query the current DFU progress and state.
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
 * 2. Enter bootloader mode (AT+BOOT)
 * 3. Wait for bootloader ready (AT+DFU=PING)
 * 4. Erase application flash (AT+DFU=ERASE)
 * 5. Send all firmware chunks with addresses (AT+DFU=WRITE)
 * 6. Verify CRC (AT+DFU=VERIFY)
 * 7. Jump to new application (AT+DFU=JUMP)
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

/* ============== Legacy API compatibility (deprecated) ============== */

/**
 * @brief Start a DFU session (DEPRECATED - use argos_dfu_erase instead)
 *
 * For backward compatibility only. Calls argos_dfu_erase internally.
 * The fw_size and fw_crc are stored for use in argos_dfu_finish.
 */
int argos_dfu_start(const struct device *dev, uint32_t fw_size, uint32_t fw_crc);

/**
 * @brief Send a chunk of firmware data (DEPRECATED - use argos_dfu_write instead)
 *
 * For backward compatibility. Tracks address internally.
 */
int argos_dfu_send_chunk(const struct device *dev, const uint8_t *data, size_t len);

/**
 * @brief Finalize the DFU session (DEPRECATED - use argos_dfu_verify + argos_dfu_jump)
 *
 * For backward compatibility. Calls argos_dfu_verify and argos_dfu_jump.
 */
int argos_dfu_finish(const struct device *dev);

/* CRC32 function is provided by argos_crc.h */

#ifdef __cplusplus
}
#endif

#endif /* ARGOS_DFU_H */
