/*
 * Copyright (c) 2023 Arribada Initiative CIC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT          arribada_argossmd

#include <stdio.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <argos-smd/argos_smd.h>
#include "argos_smd_priv.h"

#define ARGOS_SMD_INIT_PRIORITY 60

LOG_MODULE_REGISTER(ARGOS_SMD, CONFIG_ARGOS_SMD_LOG_LEVEL);

static void argos_smd_uart_flush(const struct device *dev)
{
	const struct argos_smd_config *cfg = dev->config;
	struct argos_smd_data *drv_data = dev->data;

	while (uart_fifo_read(cfg->uart_dev, NULL, 1) > 0) {
	}
	memset(&drv_data->response.data, 0, ARGOS_SMD_BUF_SIZE);

	LOG_DBG("UART RX buffer flushed.");
}

static void uart_rx_handler(const struct device *dev, void *dev_smd)
{	
	const struct device *argos_smd_dev = dev_smd;
	struct argos_smd_data *drv_data = argos_smd_dev->data;

	argos_smd_callback_t callback = drv_data->callback;

	while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
		uint8_t byte;
		int len = uart_fifo_read(dev, &byte, sizeof(byte));
		if (len <= 0) {
			continue;
		}

		if (byte == '+' && drv_data->status == RESPONSE_CLEAR) {
			drv_data->status = RESPONSE_PENDING;
			memset(drv_data->response.data, 0, sizeof(drv_data->response.data));
			drv_data->response.len = 0;
		}

		if (drv_data->status == RESPONSE_PENDING) {
			drv_data->response.len++;
			size_t index = drv_data->response.len - 1;
			drv_data->response.data[index] = byte;

			if (byte == '\n') {
				drv_data->status = RESPONSE_SUCCESS;
				LOG_DBG("Response success.");
				if (callback != NULL) {
					callback(drv_data->response.data, drv_data->response.len, drv_data->user_data);
				}
			}
		}
		if (drv_data->response.len >= sizeof(drv_data->response.data)) {
			drv_data->status = RESPONSE_FAIL;
			continue;
		}
	}
}

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
	__ASSERT(tx->len <= 255, "Command length too long.");

	if (CONFIG_ARGOS_SMD_LOG_LEVEL >= LOG_LEVEL_DBG) {
		LOG_DBG("Command: %s", tx->data);
	}

	for (size_t i = 0; i < tx->len; i++) {
		data->status = RESPONSE_CLEAR;
		uart_poll_out(cfg->uart_dev, (char)tx->data[i]);
	}

	uart_poll_out(cfg->uart_dev, '\r');
	uart_poll_out(cfg->uart_dev, '\n');

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
	}
	return 0;
}

int build_read_cmd(const char *cmd_define, char *full_command, size_t buffer_size) {
    size_t required_size = strlen(cmd_define) + READ_CMD_SIZE_TO_ADD; // Length of cmd_define + length of "=?"


    if (buffer_size < required_size) {
		LOG_ERR("Error: Buffer size (%zu) is too small. Required size is %zu.", buffer_size, required_size);
        return ERROR_CMD_LENGTH; // Return an error code
    }

    strncpy(full_command, cmd_define, buffer_size); // Use strncpy to prevent buffer overflow
    strncat(full_command, "=?", buffer_size - strlen(full_command)); // Use strncat to prevent buffer overflow

    return 0;
}

void argos_smd_set_callback(const struct device *dev, argos_smd_callback_t callback,
				      void *user_data)
{
	struct argos_smd_data *data = (struct argos_smd_data *)dev->data;
	data->callback = callback;
	data->user_data = user_data;
}

int argos_read_ping(const struct device *dev) {
    char cmd[sizeof(AT_PING) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    // Build the command using the AT_FW define
    LOG_INF("Ping Argos device\n");
    if (build_read_cmd(AT_PING, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read command.\n");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_firmware_version(const struct device *dev) {
    char cmd[sizeof(AT_FW) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    // Build the command using the AT_FW define
    LOG_INF("Request Argos firmware version\n");
    if (build_read_cmd(AT_FW, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read command.\n");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_address(const struct device *dev) {
    char cmd[sizeof(AT_ADDR) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos address\n");
    if (build_read_cmd(AT_ADDR, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the address command.\n");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_serial_number(const struct device *dev) {
    char cmd[sizeof(AT_SN) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos serial number\n");
    if (build_read_cmd(AT_SN, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the serial number command.\n");
        result = ERROR_CMD_BUILD;
    }

    return result;
}


int argos_read_id(const struct device *dev) {
    char cmd[sizeof(AT_ID) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; 
    int result = 0;

    LOG_INF("Request Argos ID\n");
    if (build_read_cmd(AT_ID, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the ID command.\n");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_configuration(const struct device *dev) {
    char cmd[sizeof(AT_RCONF) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos configuration\n");
    if (build_read_cmd(AT_RCONF, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the configuration command.\n");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_prepass_enable(const struct device *dev) {
    char cmd[sizeof(AT_PREPASS_EN) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos prepass enable\n");
    if (build_read_cmd(AT_PREPASS_EN, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the argos prepass en command.\n");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_udate(const struct device *dev) {
    char cmd[sizeof(AT_UDATE) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos UTC time configured\n");
    if (build_read_cmd(AT_UDATE, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the update command.\n");
        result = ERROR_CMD_BUILD;
	}
	return result;
}

int argos_read_repetition_configured(const struct device *dev) {
    char cmd[sizeof(AT_ATXRP) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos TX repetition configured\n");
    if (build_read_cmd(AT_ATXRP, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read repetition configured command.\n");
        result = ERROR_CMD_BUILD;
	}
	return result;
}

int argos_send_message(const struct device *dev, const char *TXmessage) {
    size_t message_length = strlen(TXmessage);
    const size_t max_length = TX_MAX_LDA2_PAYLOAD_SIZE / 8;
    
    if (message_length > max_length) {
        LOG_ERR("TXmessage size exceeds the maximum allowed payload size. Message length: %zu, Allowed length: %zu.\n", message_length, max_length);
        return ERROR_CMD_LENGTH;
    }

    size_t cmd_size = sizeof(AT_TX) + message_length; 

    char cmd[max_length];

    snprintf(cmd, cmd_size, "%s%s", AT_TX, TXmessage);

    LOG_INF("Send Argos message: %s\n", TXmessage);
    
    if (send_command(dev, cmd, cmd_size, true) != 0) {
        LOG_ERR("Failed to send the message command.\n");
        return -1;
    }

    return 0;
}

int argos_send_cmd(const struct device *dev, const char *command) {
    size_t message_length = strlen(command);
    const size_t max_length = ARGOS_SMD_BUF_SIZE;
    
    if (message_length > max_length) {
        LOG_ERR("TXmessage size exceeds the maximum allowed payload size. Message length: %zu, Allowed length: %zu.\n", message_length, max_length);
        return ERROR_CMD_LENGTH;
    }
    
    if (send_command(dev, (uint8_t*)command, message_length, true) != 0) {
        LOG_ERR("Failed to send the message command.\n");
        return -1;
    }

    return 0;
}

int build_write_cmd(const char *cmd, const char *data,  char *cmd_buffer, size_t buffer_size) {
		size_t message_length = strlen(cmd) + strlen(data) + WRITE_CMD_SIZE_TO_ADD;


    if (message_length > buffer_size) {
        LOG_ERR("Command size exceeds the provided buffers size. Message length: %zu, Provided length: %zu.\n", message_length, buffer_size);
        return ERROR_CMD_LENGTH;
    }

    
    if (buffer_size > ARGOS_SMD_BUF_SIZE) {
        LOG_ERR("Command size exceeds the maximum allowed payload size. Message length: %zu, Provided length: %zu.\n",ARGOS_SMD_BUF_SIZE , buffer_size);
        return ERROR_CMD_LENGTH;
    }
		
    snprintf(cmd_buffer, buffer_size, "%s=%s\n\r", cmd, data);
    return 0;
}

int argos_set_address(const struct device *dev, const char* address) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos address to %s", address);
    if (build_write_cmd(AT_ADDR, address, cmd,  sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the command.\n");
        result = ERROR_CMD_BUILD;
    }

    return result;
}


int argos_set_serial_number(const struct device *dev, const char* serial_number) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos serial number to %s", serial_number);
    if (build_write_cmd(AT_SN, serial_number, cmd,  sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the command.\n");
        result = ERROR_CMD_BUILD;
    }

    return result;
}


int argos_set_id(const struct device *dev, const char* id) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos ID to %s", id);
    if (build_write_cmd(AT_SN, id, cmd,  sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the command.\n");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_set_radio_config(const struct device *dev, const char* rconf) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos radio config to %s", rconf);
    if (build_write_cmd(AT_SN, rconf, cmd,  sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the command.\n");
        result = ERROR_CMD_BUILD;
    }

    return result;
}


int argos_set_datetime(const struct device *dev, const char* datetime) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos Datetime to %s", datetime);
    if (build_write_cmd(AT_SN, datetime, cmd,  sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the command.\n");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

static int argos_smd_init(const struct device *dev)
{
	const struct argos_smd_config *cfg = dev->config;
	struct argos_smd_data *drv_data = dev->data;

	if (!device_is_ready(cfg->uart_dev)) {
		LOG_ERR("UART device is not ready");
		return -ENODEV;
	}

	argos_smd_uart_flush(dev);

	drv_data->response.len = 0;
	drv_data->status = RESPONSE_CLEAR;

	uart_irq_callback_user_data_set(cfg->uart_dev, uart_rx_handler, (void*) dev);
	uart_irq_rx_enable(cfg->uart_dev);

	return 0;
}

#define ARGOS_SMD_DEFINE(inst)                                                                      \
	static struct argos_smd_data argos_smd_data_##inst = {                                       \
	};                                                                                         \
	static const struct argos_smd_config argos_smd_config_##inst = {                             \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &argos_smd_init, NULL, &argos_smd_data_##inst,                   \
			      &argos_smd_config_##inst, POST_KERNEL, ARGOS_SMD_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ARGOS_SMD_DEFINE)
