/*
 * Copyright (c) 2023 Arribada Initiative CIC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT arribada_argossmd
#define MODULE argos_smd

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

LOG_MODULE_REGISTER(MODULE, CONFIG_ARGOS_SMD_LOG_LEVEL);

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

		if (byte == '+' && atomic_get(&drv_data->status) == RESPONSE_CLEAR) {
      atomic_set(&drv_data->status, RESPONSE_PENDING);
			memset(drv_data->response.data, 0, sizeof(drv_data->response.data));
			drv_data->response.len = 0;
		}

		if ( atomic_get(&drv_data->status) == RESPONSE_PENDING) {
			drv_data->response.len++;
			size_t index = drv_data->response.len - 1;
			drv_data->response.data[index] = byte;

			if (byte == '\n') {
        atomic_set(&drv_data->status, RESPONSE_SUCCESS);
				LOG_DBG("Response success.");
				if (callback != NULL) {
					callback(drv_data->response.data, drv_data->response.len, drv_data->user_data);
				}
			}
		}
		if (drv_data->response.len >= sizeof(drv_data->response.data)) {
      atomic_set(&drv_data->status, RESPONSE_FAIL);
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

  atomic_set(&data->status, RESPONSE_CLEAR);

	for (size_t i = 0; i < tx->len; i++) {
		uart_poll_out(cfg->uart_dev, (char)tx->data[i]);
	}

	uart_poll_out(cfg->uart_dev, '\r');

	if (timeout) {
		while (atomic_get(&data->status) != RESPONSE_SUCCESS) {
			if (timeout_in_ms < 0) {
				LOG_WRN("Command timeout.");
        atomic_set(&data->status, RESPONSE_CLEAR);
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

int argos_read_version(const struct device *dev) {
    char cmd[sizeof(AT_VERSION) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    // Build the command using the AT_FW define
    LOG_INF("Request Argos version");
    if (build_read_cmd(AT_VERSION, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_ping(const struct device *dev) {
    char cmd[sizeof(AT_PING) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    // Build the command using the AT_FW define
    LOG_INF("Ping Argos device");
    if (build_read_cmd(AT_PING, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_firmware_version(const struct device *dev) {
    char cmd[sizeof(AT_FW) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    // Build the command using the AT_FW define
    LOG_INF("Request Argos firmware version");
    if (build_read_cmd(AT_FW, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_address(const struct device *dev) {
    char cmd[sizeof(AT_ADDR) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos address");
    if (build_read_cmd(AT_ADDR, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the address command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_id(const struct device *dev) {
    char cmd[sizeof(AT_ID) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; 
    int result = 0;

    LOG_INF("Request Argos ID");
    if (build_read_cmd(AT_ID, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the ID command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_seckey(const struct device *dev) {
    char cmd[sizeof(AT_SECKEY) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; 
    int result = 0;

    LOG_INF("Request Argos security key");
    if (build_read_cmd(AT_SECKEY, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read seckey command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_serial_number(const struct device *dev) {
    char cmd[sizeof(AT_SN) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos serial number");
    if (build_read_cmd(AT_SN, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the serial number command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}


int argos_read_radioconf(const struct device *dev) {
    char cmd[sizeof(AT_RCONF) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos configuration");
    if (build_read_cmd(AT_RCONF, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the configuration command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_prepass_enable(const struct device *dev) {
    char cmd[sizeof(AT_PREPASS_EN) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos prepass enable");
    if (build_read_cmd(AT_PREPASS_EN, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the argos prepass en command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_read_udate(const struct device *dev) {
    char cmd[sizeof(AT_UDATE) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos UTC time configured");
    if (build_read_cmd(AT_UDATE, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read udate command.");
        result = ERROR_CMD_BUILD;
	}
	return result;
}

int argos_read_lpm(const struct device *dev) {
    char cmd[sizeof(AT_LPM) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos low power mode configured");
    if (build_read_cmd(AT_LPM, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read low power mode command.");
        result = ERROR_CMD_BUILD;
	}
	return result;
}

int argos_read_mc(const struct device *dev) {
    char cmd[sizeof(AT_MC) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos read Mac counter");
    if (build_read_cmd(AT_MC, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read mac counter command.");
        result = ERROR_CMD_BUILD;
	}
	return result;
}

int argos_read_tcxo_wu(const struct device *dev) {
    char cmd[sizeof(AT_TCXO_WU) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos read TCXO warmup timer");
    if (build_read_cmd(AT_TCXO_WU, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read TCXO warmup timer command.");
        result = ERROR_CMD_BUILD;
	}
	return result;
}

int argos_read_kmac(const struct device *dev) {
    char cmd[sizeof(AT_KMAC) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos read KMAC profile ");
    if (build_read_cmd(AT_KMAC, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read KMAC profile command.");
        result = ERROR_CMD_BUILD;
	}
	return result;
}

int argos_read_cw(const struct device *dev) {
    char cmd[sizeof(AT_CW) - 1 + READ_CMD_SIZE_TO_ADD] = {0}; // +2 for "=?" and null terminator
    int result = 0;

    LOG_INF("Request Argos read CW configuration ");
    if (build_read_cmd(AT_CW, cmd, sizeof(cmd)) == 0) {
        send_command(dev, cmd, sizeof(cmd), true);
    } else {
        LOG_ERR("Failed to build the read CW configuration command.");
        result = ERROR_CMD_BUILD;
	}
	return result;
}



int argos_send_cmd(const struct device *dev, const char *command) {
    size_t message_length = strlen(command);
    const size_t max_length = ARGOS_SMD_BUF_SIZE;
    
    if (message_length > max_length) {
        LOG_ERR("TXmessage size exceeds the maximum allowed payload size. Message length: %zu, Allowed length: %zu.", message_length, max_length);
        return ERROR_CMD_LENGTH;
    }
    
    if (send_command(dev, (uint8_t*)command, message_length, true) != 0) {
        LOG_ERR("Failed to send the message command.");
        return -1;
    }

    return 0;
}

int build_set_cmd(const char *cmd, const char *data,  char *cmd_buffer, size_t buffer_size) {
		size_t message_length = strlen(cmd) + strlen(data) + SET_CMD_SIZE_TO_ADD;


    if (message_length > buffer_size) {
        LOG_ERR("Command size exceeds the provided buffers size. Message length: %zu, Provided length: %zu.", message_length, buffer_size);
        return ERROR_CMD_LENGTH;
    }

    
    if (buffer_size > ARGOS_SMD_BUF_SIZE) {
        LOG_ERR("Command size exceeds the maximum allowed payload size. Message length: %zu, Provided length: %zu.",ARGOS_SMD_BUF_SIZE , buffer_size);
        return ERROR_CMD_LENGTH;
    }
		
    snprintf(cmd_buffer, buffer_size, "%s=%s", cmd, data);
    return message_length;
}

int argos_set_address(const struct device *dev, const char* address) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos address to %s", address);
    int cmd_size = build_set_cmd(AT_ADDR, address, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_set_serial_number(const struct device *dev, const char* serial_number) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos serial number to %s", serial_number);
    int cmd_size = build_set_cmd(AT_SN, serial_number, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_set_id(const struct device *dev, const char* id) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos ID to %s", id);
    int cmd_size = build_set_cmd(AT_ID, id, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_set_radioconf(const struct device *dev, const char* rconf) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos radio config to %s", rconf);
    int cmd_size = build_set_cmd(AT_RCONF, rconf, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_set_saveradioconf(const struct device *dev, const char* saveconf) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Save radio config to %s", saveconf);
    int cmd_size = build_set_cmd(AT_SAVE_RCONF, saveconf, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_set_prepass_enable(const struct device *dev, const char* prepass) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos prepass enable to %s", prepass);
    int cmd_size = build_set_cmd(AT_PREPASS_EN, prepass, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_set_udate(const struct device *dev, const char* datetime) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos Datetime to %s", datetime);
    int cmd_size = build_set_cmd(AT_UDATE, datetime, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_set_lpm(const struct device *dev, const char* lpm) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos Low power profile to %s", lpm);
    int cmd_size = build_set_cmd(AT_LPM, lpm, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_set_mc(const struct device *dev, const char* mc) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos Mac Counter to %s", mc);
    int cmd_size = build_set_cmd(AT_MC, mc, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_set_tcxo_wu(const struct device *dev, const char* tcxo_wu) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos TCXO warmup to %s", tcxo_wu);
    int cmd_size = build_set_cmd(AT_TCXO_WU, tcxo_wu, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_set_kmac(const struct device *dev, const char* kmac) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos KMAC to %s", kmac);
    int cmd_size = build_set_cmd(AT_KMAC, kmac, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_set_cw(const struct device *dev, const char* cw) {
    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Setting Argos set Continuous wave RF test to %s", cw);
    int cmd_size = build_set_cmd(AT_CW, cw, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
        result = ERROR_CMD_BUILD;
    }

    return result;
}

int argos_send_payload(const struct device *dev, const char *paylod) {
    size_t message_length = strlen(paylod);
    const size_t max_length = TX_MAX_LDA2_PAYLOAD_SIZE;

    if (message_length > max_length) {
        LOG_ERR("TXmessage size exceeds the maximum allowed payload size. Message length: %zu, Allowed length: %zu.", message_length, max_length);
        return ERROR_CMD_LENGTH;
    }

    char cmd[ARGOS_SMD_BUF_SIZE];
    int result = 0;

    LOG_INF("Transmitting Following Message: %s", paylod);
    int cmd_size = build_set_cmd(AT_TX, paylod, cmd,  sizeof(cmd));
    if ( cmd_size >= 0) {
        send_command(dev, cmd, cmd_size, true);
    } else {
        LOG_ERR("Failed to build the command.");
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
	atomic_set(&drv_data->status, RESPONSE_CLEAR);

	uart_irq_callback_user_data_set(cfg->uart_dev, uart_rx_handler, (void*) dev);
	uart_irq_rx_enable(cfg->uart_dev);

	return 0;
}

#define ARGOS_SMD_DEFINE(inst)                                                                   \
    static struct argos_smd_data argos_smd_data_##inst = {};                                     \
    static const struct argos_smd_config argos_smd_config_##inst = {                             \
      .uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                              \
    };                                                                                           \
                                                                                                 \
    DEVICE_DT_INST_DEFINE(inst, &argos_smd_init, NULL, &argos_smd_data_##inst,                   \
    	      &argos_smd_config_##inst, POST_KERNEL, ARGOS_SMD_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ARGOS_SMD_DEFINE)
