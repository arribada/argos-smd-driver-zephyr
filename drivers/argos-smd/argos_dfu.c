/*
 * Copyright (c) 2025 Arribada Initiative
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Argos SMD DFU over UART - Compatible with STM32WL Bootloader
 *
 * Protocol Commands:
 *   AT+BOOT              - Enter bootloader mode (handled by app)
 *   AT+DFU=PING          - Check bootloader ready
 *   AT+DFU=ERASE         - Erase application flash
 *   AT+DFU=WRITE,<addr>,<hex> - Write data at address
 *   AT+DFU=VERIFY,<crc>  - Verify firmware CRC32
 *   AT+DFU=JUMP          - Jump to application
 *   AT+DFU=ABORT         - Abort DFU session
 *
 * Response format:
 *   +DFU=OK[,<data>]     - Success
 *   +DFU=ERR,<error>     - Error (CRC_ERROR, ADDR_ERROR, etc.)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <argos-smd/argos_dfu.h>
#include <argos-smd/argos_smd.h>

LOG_MODULE_REGISTER(argos_dfu, CONFIG_ARGOS_SMD_DFU_LOG_LEVEL);

/* Response parsing */
#define DFU_RESPONSE_OK      "+DFU=OK"
#define DFU_RESPONSE_ERR     "+DFU=ERR"

/* Internal state for legacy API compatibility */
static struct {
	uint32_t write_addr;      /* Current write address */
	uint32_t fw_size;         /* Total firmware size */
	uint32_t fw_crc;          /* Expected CRC32 */
	uint32_t bytes_written;   /* Bytes written so far */
	bool session_active;      /* DFU session active */
} dfu_state;

/* Response synchronization */
static struct k_sem response_sem;
static char response_buf[256];
static volatile bool response_received;
static volatile bool response_ok;

/* Response callback */
static void dfu_response_callback(const char *buf, void *user_data)
{
	ARG_UNUSED(user_data);

	if (buf == NULL) {
		return;
	}

	/* Debug: Log all received data */
	LOG_INF("DFU RX: '%s'", buf);

	/* Store response */
	strncpy(response_buf, buf, sizeof(response_buf) - 1);
	response_buf[sizeof(response_buf) - 1] = '\0';

	/* Check if OK or ERR */
	if (strstr(buf, DFU_RESPONSE_OK) != NULL) {
		response_ok = true;
		response_received = true;
	} else if (strstr(buf, DFU_RESPONSE_ERR) != NULL) {
		response_ok = false;
		response_received = true;
	}

	k_sem_give(&response_sem);
}

/* Wait for response with timeout */
static int wait_for_response(const struct device *dev, uint32_t timeout_ms)
{
	response_received = false;
	response_ok = false;

	/* Set our callback */
	argos_smd_set_callback(dev, dfu_response_callback, NULL);

	/* Wait for response */
	int ret = k_sem_take(&response_sem, K_MSEC(timeout_ms));

	/* Clear callback */
	argos_smd_set_callback(dev, NULL, NULL);

	if (ret != 0) {
		LOG_ERR("Response timeout");
		return -ETIMEDOUT;
	}

	if (!response_ok) {
		LOG_ERR("DFU error response: %s", response_buf);
		return -EIO;
	}

	return 0;
}

/* Convert binary data to hexadecimal ASCII string */
static void bin_to_hex(const uint8_t *bin, size_t bin_len, char *hex)
{
	static const char hex_chars[] = "0123456789ABCDEF";

	for (size_t i = 0; i < bin_len; i++) {
		hex[i * 2]     = hex_chars[(bin[i] >> 4) & 0x0F];
		hex[i * 2 + 1] = hex_chars[bin[i] & 0x0F];
	}
	hex[bin_len * 2] = '\0';
}

/* Initialize module */
static int dfu_init(void)
{
	static bool initialized = false;

	if (!initialized) {
		k_sem_init(&response_sem, 0, 1);
		memset(&dfu_state, 0, sizeof(dfu_state));
		initialized = true;
	}

	return 0;
}

int argos_enter_bootloader(const struct device *dev)
{
	dfu_init();

	LOG_INF("TX: 'AT+BOOT' - entering bootloader mode...");

	/* Send AT+BOOT to application - it will jump to bootloader */
	int ret = argos_send_raw(dev, "AT+BOOT");
	if (ret < 0) {
		LOG_ERR("Failed to send BOOT command: %d", ret);
		return ret;
	}

	/* Device will reboot - give it more time (1 second) */
	LOG_INF("Waiting %d ms for device to reboot...", ARGOS_DFU_BOOT_DELAY_MS);
	k_msleep(ARGOS_DFU_BOOT_DELAY_MS);

	/* Reset state */
	memset(&dfu_state, 0, sizeof(dfu_state));
	dfu_state.write_addr = ARGOS_DFU_APP_BASE;

	return 0;
}

int argos_dfu_ping(const struct device *dev)
{
	dfu_init();

	LOG_INF("TX: 'AT+DFU=PING'");

	int ret = argos_send_raw(dev, "AT+DFU=PING");
	if (ret < 0) {
		LOG_ERR("Failed to send PING: %d", ret);
		return ret;
	}

	ret = wait_for_response(dev, ARGOS_DFU_RESPONSE_TIMEOUT_MS);
	if (ret == 0) {
		LOG_INF("Bootloader responded OK");
	}

	return ret;
}

int argos_wait_bootloader_ready(const struct device *dev, k_timeout_t timeout)
{
	LOG_INF("Waiting for bootloader to be ready...");

	int64_t start = k_uptime_get();
	int64_t timeout_ms = k_ticks_to_ms_floor64(timeout.ticks);

	while ((k_uptime_get() - start) < timeout_ms) {
		int ret = argos_dfu_ping(dev);
		if (ret == 0) {
			LOG_INF("Bootloader is ready!");
			return 0;
		}

		/* Wait a bit before retrying */
		k_msleep(200);
	}

	LOG_ERR("Bootloader not responding after timeout");
	return -ETIMEDOUT;
}

int argos_dfu_erase(const struct device *dev)
{
	dfu_init();

	LOG_INF("Erasing application flash...");

	int ret = argos_send_raw(dev, "AT+DFU=ERASE");
	if (ret < 0) {
		LOG_ERR("Failed to send ERASE command: %d", ret);
		return ret;
	}

	/* Erase takes longer - wait with extended timeout */
	ret = wait_for_response(dev, ARGOS_DFU_ERASE_DELAY_MS);
	if (ret < 0) {
		LOG_ERR("Erase failed: %d", ret);
		return ret;
	}

	/* Reset write address */
	dfu_state.write_addr = ARGOS_DFU_APP_BASE;
	dfu_state.bytes_written = 0;
	dfu_state.session_active = true;

	LOG_INF("Flash erased successfully");
	return 0;
}

int argos_dfu_write(const struct device *dev, uint32_t addr,
		    const uint8_t *data, size_t len)
{
	/* Command buffer: "AT+DFU=WRITE," + 8 hex addr + "," + 128 hex data + null */
	char cmd[16 + 8 + 1 + ARGOS_DFU_CHUNK_SIZE * 2 + 1];
	char hex_data[ARGOS_DFU_CHUNK_SIZE * 2 + 1];

	if (len > ARGOS_DFU_CHUNK_SIZE) {
		LOG_ERR("Chunk size %zu exceeds maximum %d", len, ARGOS_DFU_CHUNK_SIZE);
		return -EINVAL;
	}

	if (len == 0) {
		LOG_ERR("Cannot write empty chunk");
		return -EINVAL;
	}

	/* Convert binary data to hex string */
	bin_to_hex(data, len, hex_data);

	/* Build command: AT+DFU=WRITE,<addr_hex>,<data_hex> */
	snprintf(cmd, sizeof(cmd), "AT+DFU=WRITE,%08X,%s", addr, hex_data);

	LOG_DBG("Writing %zu bytes at 0x%08X", len, addr);

	int ret = argos_send_raw(dev, cmd);
	if (ret < 0) {
		LOG_ERR("Failed to send WRITE command: %d", ret);
		return ret;
	}

	ret = wait_for_response(dev, ARGOS_DFU_RESPONSE_TIMEOUT_MS);
	if (ret < 0) {
		LOG_ERR("Write failed at 0x%08X: %d", addr, ret);
		return ret;
	}

	return 0;
}

int argos_dfu_verify(const struct device *dev, uint32_t crc32)
{
	char cmd[32];

	LOG_INF("Verifying firmware CRC32: 0x%08X", crc32);

	snprintf(cmd, sizeof(cmd), "AT+DFU=VERIFY,%08X", crc32);

	int ret = argos_send_raw(dev, cmd);
	if (ret < 0) {
		LOG_ERR("Failed to send VERIFY command: %d", ret);
		return ret;
	}

	ret = wait_for_response(dev, ARGOS_DFU_RESPONSE_TIMEOUT_MS);
	if (ret < 0) {
		LOG_ERR("Verify failed: %d", ret);
		return ret;
	}

	LOG_INF("CRC verification passed!");
	return 0;
}

int argos_dfu_jump(const struct device *dev)
{
	LOG_INF("Jumping to application...");

	int ret = argos_send_raw(dev, "AT+DFU=JUMP");
	if (ret < 0) {
		LOG_ERR("Failed to send JUMP command: %d", ret);
		return ret;
	}

	ret = wait_for_response(dev, ARGOS_DFU_RESPONSE_TIMEOUT_MS);
	if (ret < 0) {
		LOG_WRN("Jump response: %d (device may have jumped)", ret);
		/* Device might have already jumped - not necessarily an error */
	}

	/* Wait for device to reboot */
	k_msleep(ARGOS_DFU_BOOT_DELAY_MS);

	return 0;
}

int argos_dfu_abort(const struct device *dev)
{
	LOG_WRN("Aborting DFU session...");

	int ret = argos_send_raw(dev, "AT+DFU=ABORT");
	if (ret < 0) {
		LOG_ERR("Failed to send ABORT command: %d", ret);
		return ret;
	}

	ret = wait_for_response(dev, ARGOS_DFU_RESPONSE_TIMEOUT_MS);

	/* Reset state */
	dfu_state.session_active = false;

	LOG_INF("DFU session aborted");
	return ret;
}

int argos_dfu_get_status(const struct device *dev,
			 uint32_t *progress, enum argos_dfu_state *state)
{
	LOG_DBG("Getting DFU status...");

	int ret = argos_send_raw(dev, "AT+DFU=STATUS");
	if (ret < 0) {
		return ret;
	}

	ret = wait_for_response(dev, ARGOS_DFU_RESPONSE_TIMEOUT_MS);
	if (ret < 0) {
		return ret;
	}

	/* Parse response - for now just return internal state */
	if (progress) {
		*progress = dfu_state.bytes_written;
	}
	if (state) {
		*state = dfu_state.session_active ? ARGOS_DFU_TRANSFERRING : ARGOS_DFU_IDLE;
	}

	return 0;
}

int argos_ota_update(const struct device *dev,
		     const uint8_t *fw_image, size_t fw_size,
		     argos_dfu_progress_cb_t progress_cb)
{
	int ret;

	LOG_INF("========================================");
	LOG_INF("     UART DFU Update Started");
	LOG_INF("========================================");
	LOG_INF("Firmware size: %zu bytes", fw_size);

	if (!fw_image || fw_size == 0) {
		LOG_ERR("Invalid firmware image");
		return -EINVAL;
	}

	/* Step 1: Calculate CRC32 of the firmware image */
	uint32_t fw_crc = argos_dfu_crc32(fw_image, fw_size);
	LOG_INF("Firmware CRC32: 0x%08X", fw_crc);

	/* Step 2: Enter bootloader mode */
	LOG_INF("Step 1/6: Entering bootloader mode...");
	ret = argos_enter_bootloader(dev);
	if (ret < 0) {
		LOG_ERR("Failed to enter bootloader mode: %d", ret);
		return ret;
	}

	/* Step 3: Wait for bootloader to be ready */
	LOG_INF("Step 2/6: Waiting for bootloader...");
	ret = argos_wait_bootloader_ready(dev, K_SECONDS(10));
	if (ret < 0) {
		LOG_ERR("Bootloader not ready: %d", ret);
		return ret;
	}

	/* Step 4: Erase flash */
	LOG_INF("Step 3/6: Erasing flash...");
	ret = argos_dfu_erase(dev);
	if (ret < 0) {
		LOG_ERR("Flash erase failed: %d", ret);
		return ret;
	}

	/* Step 5: Write all firmware chunks */
	LOG_INF("Step 4/6: Transferring firmware data...");
	uint32_t addr = ARGOS_DFU_APP_BASE;
	size_t offset = 0;
	uint32_t last_progress_percent = 0;

	while (offset < fw_size) {
		size_t chunk_len = MIN(ARGOS_DFU_CHUNK_SIZE, fw_size - offset);

		ret = argos_dfu_write(dev, addr, &fw_image[offset], chunk_len);
		if (ret < 0) {
			LOG_ERR("Failed to write chunk at offset %zu: %d", offset, ret);
			LOG_WRN("Attempting to abort DFU session...");
			argos_dfu_abort(dev);
			return ret;
		}

		addr += chunk_len;
		offset += chunk_len;

		/* Call progress callback if provided */
		if (progress_cb) {
			progress_cb(offset, fw_size);
		}

		/* Log progress every 10% */
		uint32_t progress_percent = (offset * 100) / fw_size;
		if (progress_percent != last_progress_percent &&
		    (progress_percent % 10 == 0)) {
			LOG_INF("Progress: %u%% (%zu/%zu bytes)",
				progress_percent, offset, fw_size);
			last_progress_percent = progress_percent;
		}
	}

	LOG_INF("Firmware written: %zu bytes", offset);

	/* Step 6: Verify CRC */
	LOG_INF("Step 5/6: Verifying CRC...");
	ret = argos_dfu_verify(dev, fw_crc);
	if (ret < 0) {
		LOG_ERR("CRC verification failed: %d", ret);
		argos_dfu_abort(dev);
		return ret;
	}

	/* Step 7: Jump to application */
	LOG_INF("Step 6/6: Jumping to application...");
	ret = argos_dfu_jump(dev);
	if (ret < 0) {
		LOG_ERR("Jump to application failed: %d", ret);
		return ret;
	}

	/* Verify device is responding in app mode */
	k_msleep(2000);  /* Wait for device to fully reboot */

	ret = argos_read_ping(dev);
	if (ret < 0) {
		LOG_WRN("Device not responding after update (may need manual verification)");
	}

	LOG_INF("========================================");
	LOG_INF("     UART DFU Update Successful!");
	LOG_INF("========================================");

	return 0;
}

/* ============== Legacy API compatibility ============== */

int argos_dfu_start(const struct device *dev, uint32_t fw_size, uint32_t fw_crc)
{
	int ret;

	dfu_init();

	/* Store for later use in argos_dfu_finish */
	dfu_state.fw_size = fw_size;
	dfu_state.fw_crc = fw_crc;
	dfu_state.write_addr = ARGOS_DFU_APP_BASE;
	dfu_state.bytes_written = 0;

	/* The new protocol uses ERASE instead of DFUSTART */
	ret = argos_dfu_erase(dev);

	return ret;
}

int argos_dfu_send_chunk(const struct device *dev, const uint8_t *data, size_t len)
{
	if (!dfu_state.session_active) {
		LOG_ERR("DFU session not active - call argos_dfu_start first");
		return -EINVAL;
	}

	int ret = argos_dfu_write(dev, dfu_state.write_addr, data, len);
	if (ret == 0) {
		dfu_state.write_addr += len;
		dfu_state.bytes_written += len;
	}

	return ret;
}

int argos_dfu_finish(const struct device *dev)
{
	int ret;

	LOG_INF("Finalizing DFU session...");

	/* Verify CRC */
	ret = argos_dfu_verify(dev, dfu_state.fw_crc);
	if (ret < 0) {
		LOG_ERR("Verification failed: %d", ret);
		return ret;
	}

	/* Jump to application */
	ret = argos_dfu_jump(dev);

	/* Reset state */
	dfu_state.session_active = false;

	return ret;
}
