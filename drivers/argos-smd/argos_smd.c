/*
 * Copyright (c) 2023 Arribada Initiative CIC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT          arribada_argossmd
#define ARGOS_SMD_INIT_PRIORITY 60

#include <errno.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include "argos_smd.h"

LOG_MODULE_REGISTER(argos_smd, CONFIG_ARGOS_SMD_LOG_LEVEL);

/**
 * @brief Set callback function to be called when a string is received.
 *
 * @param dev UART peripheral device.
 * @param callback New callback function.
 * @param user_data Data to be passed to the callback function.
 */
static void user_set_command_callback(const struct device *dev, argos_smd_callback_t callback,
				      void *user_data)
{
	struct argos_smd_data *data = (struct argos_smd_data *)dev->data;
	data->callback = callback;
	data->user_data = user_data;
}

/**
 * @brief Empty the RX buffer of the UART peripheral.
 *
 * @param dev UART peripheral device.
 */
static void argos_smd_uart_flush(const struct device *dev)
{
	struct argos_smd_data *drv_data = dev->data;
	uint8_t buf;

	while (uart_fifo_read(dev, &buf, 1) > 0) {
		;
	}
	memset(&drv_data->response.data, 0, ARGOS_SMD_BUF_SIZE);

	LOG_DBG("UART RX buffer flushed.");
}

/**
 * @brief Handler for when the UART peripheral receives data.
 *
 * @param dev UART peripheral device.
 * @param dev_m6e Driver device passed to provide access to buffers.
 */
static void uart_rx_handler(const struct device *dev, void *dev_m6e)
{
	const struct device *argos_smd_dev = dev_m6e;
	struct argos_smd_data *drv_data = argos_smd_dev->data;

	int len = 0;
	int offset = 0;

	if (drv_data->status == RESPONSE_CLEAR) {
		drv_data->response.len = 0;
	}

	offset = drv_data->response.len;
	argos_smd_callback_t callback = drv_data->callback;

	if ((uart_irq_update(dev) > 0) && (uart_irq_is_pending(dev) > 0)) {
		while (uart_irq_rx_ready(dev)) {

			len = uart_fifo_read(dev, &drv_data->response.data[offset], 255 - offset);
			LOG_DBG("Received %d bytes", len);

			while (len > 0) {
				LOG_DBG("Data: %X | Offset: %d", drv_data->response.data[offset],
					offset);
				switch (offset) {
				case 0:
					if (drv_data->response.data[offset] == TMR_START_HEADER) {
						LOG_DBG("Msg Header: %X",
							drv_data->response.data[offset]);
						drv_data->status = RESPONSE_PENDING;
					} else if (drv_data->response.data[offset] ==
						   ERROR_COMMAND_RESPONSE_TIMEOUT) {
						drv_data->status = ERROR_COMMAND_RESPONSE_TIMEOUT;
					}
					break;
				case 1:
					drv_data->response.msg_len =
						drv_data->response.data[offset] + 7;
					LOG_DBG("Msg Total Len: %d", drv_data->response.msg_len);
					break;
				case 2:
					LOG_DBG("Msg Opcode: %x", drv_data->response.data[offset]);
					if (drv_data->response.data[offset] ==
					    TMR_SR_OPCODE_VERSION_STARTUP) {
						drv_data->status = RESPONSE_CLEAR;
					};
					break;
				default:
					break;
				}

				if (drv_data->status == ERROR_COMMAND_RESPONSE_TIMEOUT) {
					len = 0;
					offset = 0;
					break;
				}

				offset++;
				len--;
				drv_data->response.len = offset;
			}
		}
	}

	if (offset > drv_data->response.msg_len - 1) {
		drv_data->response.len = 0;
		drv_data->status = RESPONSE_SUCCESS;
		LOG_DBG("Response success.");
	} else if (offset > ARGOS_SMD_BUF_SIZE) {
		drv_data->response.len = 0;
		drv_data->status = RESPONSE_FAIL;
		argos_smd_uart_flush(dev);
		LOG_WRN("Response exceeds buffer, %d.", offset);
	} else if (drv_data->status == ERROR_COMMAND_RESPONSE_TIMEOUT) {
		drv_data->response.len = 0;
		drv_data->status = RESPONSE_FAIL;
		argos_smd_uart_flush(dev);
		LOG_WRN("Command response timeout.");
	}

	if (callback != NULL) {
		callback(dev, dev_m6e);
	}
}

/**
 * @brief Set the command to be transmitted by the UART peripheral.
 *
 * @param dev UART peripheral device.
 * @param command Command to be transmitted.
 * @param length Length of the command.
 * @return int32_t Status of the response.
 */
int send_command(const struct device *dev, uint8_t *command, const uint8_t length,
		      const bool timeout)
{
	int32_t timeout_in_ms = CFG_ARGOS_SMD_SERIAL_TIMEOUT;
	struct argos_smd_data *data = (struct argos_smd_data *)dev->data;
	const struct argos_smd_config *cfg = dev->config;
	struct argos_smd_buf *tx = &data->command;

	memset(tx->data, 0, ARGOS_SMD_BUF_SIZE);

	memcpy(tx->data, command, sizeof(uint8_t) * length);
	tx->len = length;
	LOG_DBG("Length of command: %d", tx->len);

	__ASSERT(tx->len <= 255, "Command length too long.");

	while (data->status == RESPONSE_STARTUP) {
		if (timeout_in_ms < 0) {
			LOG_DBG("Startup event missed...");
			data->status = RESPONSE_CLEAR;
			break;
		}
		timeout_in_ms -= 10;
		k_msleep(10);
	}


	if (CONFIG_ARGOS_SMD_LOG_LEVEL >= LOG_LEVEL_DBG) 
		LOG_DBG("Command to be sent: ");

	for (size_t i = 0; i < tx->len; i++) {
		data->status = RESPONSE_CLEAR;
		uart_poll_out(cfg->uart_dev, tx->data[i]);
		if (CONFIG_ARGOS_SMD_LOG_LEVEL >= LOG_LEVEL_DBG) 
			LOG_DBG("%c", tx->data[i]);
	}

	if (timeout) {
		while (data->status != RESPONSE_SUCCESS) {
			if (timeout_in_ms < 0) {
				LOG_WRN("Command timeout.");
				data->status = RESPONSE_CLEAR;
				return -ETIMEDOUT;
			}
			timeout_in_ms -= 10;
			k_msleep(10);
		}
		return 0;
	}
	return 0;
}
void build_read_command(const char *cmd_define, char *full_command) {
    strcpy(full_command, cmd_define);
    strcat(full_command, "=?");
}

/**
 * @brief Builds a full AT command by appending "=?" to the given command define.
 *
 * This function constructs a full AT command by concatenating the provided
 * command define with the string "=?". It also ensures that the provided buffer
 * size is sufficient to hold the resulting string and the null terminator.
 *
 * @param cmd_define The base command string defined by a #define.
 * @param full_command The buffer where the full command will be stored.
 * @param buffer_size The size of the full_command buffer.
 *
 * @return 0 if the command was successfully built, -1 if the buffer size is insufficient.
 * @return ERROR_CMD_LENGTH if the command size is invalid
 */
int build_command(const char *cmd_define, char *full_command, size_t buffer_size) {
    size_t required_size = strlen(cmd_define) + 2; // Length of cmd_define + length of "=?" (2 characters)

    if (buffer_size < required_size + 1) { // +1 for the null terminator
        fprintf(stderr, "Error: Buffer size (%zu) is too small. Required size is %zu.\n", buffer_size, required_size + 1);
        return ERROR_CMD_LENGTH; // Return an error code
    }

    strncpy(full_command, cmd_define, buffer_size - 1); // Use strncpy to prevent buffer overflow
    strncat(full_command, "=?", buffer_size - strlen(full_command) - 1); // Use strncat to prevent buffer overflow
    return 0; // Return success code
}
/**
 * @brief Initialize the argos smd.
 *
 * @param dev UART peripheral device.
 */
static int argos_smd_init(const struct device *dev)
{
	const struct argos_smd_config *cfg = dev->config;
	struct argos_smd_data *drv_data = dev->data;

	if (!device_is_ready(cfg->uart_dev)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	while (uart_irq_rx_ready(cfg->uart_dev)) {
		argos_smd_uart_flush(cfg->uart_dev);
	}

	struct argos_smd_buf *rx = &drv_data->response;
	rx->len = 0;
	drv_data->status = RESPONSE_STARTUP;

	uart_irq_callback_user_data_set(cfg->uart_dev, uart_rx_handler, (void *)dev);
	uart_irq_rx_enable(cfg->uart_dev);

	return 0;
}


/**
 * @brief Reads the firmware version of the Argos SMD.
 * This function sends the command "AT+FW=?" to the Argos SMD to request its firmware version.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_firmware_version(const struct device *dev) {
    char cmd[sizeof(AT_FW) + 2] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    // Build the command using the AT_FW define
    if (build_command(AT_FW, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read command.\n");
        result = -1;
    }

    return result;
}

/**
 * @brief Reads the address of the Argos SMD.
 * This function sends the command "AT+ADDR=?" to the Argos SMD to request its address.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_address(const struct device *dev) {
    char cmd[sizeof(AT_ADDR) + 2] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    printk("Request Argos address\n");
    if (build_command(AT_ADDR, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the address command.\n");
        result = -1;
    }

    return result;
}

/**
 * @brief Reads the serial number of the Argos SMD.
 * This function sends the command "AT+SN=?" to the Argos SMD to request its serial number.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_serial_number(const struct device *dev) {
    char cmd[sizeof(AT_SN) + 2] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    printk("Request Argos serial number\n");
    if (build_command(AT_SN, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the serial number command.\n");
        result = -1;
    }

    return result;
}

/**
 * @brief Reads the ID of the Argos SMD.
 * This function sends the command "AT+ID=?" to the Argos SMD to request its ID.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_id(const struct device *dev) {
    char cmd[sizeof(AT_ID) + 2] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    printk("Request Argos ID\n");
    if (build_command(AT_ID, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the ID command.\n");
        result = -1;
    }

    return result;
}

/**
 * @brief Reads the configuration of the Argos SMD.
 * This function sends the command "AT+RCONF=?" to the Argos SMD to request its configuration.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_configuration(const struct device *dev) {
    char cmd[sizeof(AT_RCONF) + 2] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    printk("Request Argos configuration\n");
    if (build_command(AT_RCONF, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the configuration command.\n");
        result = -1;
    }

    return result;
}

/**
 * @brief Send an argos message from SMD module.
 * This function sends the command "AT+TX=MSG" to the Argos SMD to send a message.
 *
 * @param dev UART peripheral device.
 * @param TXmessage Message to be sent with the Argos SMD.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_send_message(const struct device *dev, const char *TXmessage) {
    char cmd[256];
    int result = 0;

    snprintf(cmd, sizeof(cmd), "AT+TX=%s", TXmessage);
    printk("Sending Test command to Argos: %s\n", cmd);

    if (send_command(dev, cmd, sizeof(cmd), true) != 0) {
        LOG_ERR("Failed to send the message command.\n");
        result = -1;
    }

    return result;
}
const static struct argos_smd_api api = {
	// .send_command = send_command,
	.set_callback = user_set_command_callback,
};

#define argos_smd_DEFINE(inst)                                                                      \
	static struct argos_smd_data argos_smd_data_##inst = {                                       \
		.response.msg_len = 255,                                                           \
	};                                                                                         \
	static const struct argos_smd_config argos_smd_config_##inst = {                             \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &argos_smd_init, NULL, &argos_smd_data_##inst,                   \
			      &argos_smd_config_##inst, POST_KERNEL, ARGOS_SMD_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(argos_smd_DEFINE)