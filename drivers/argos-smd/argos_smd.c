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
#include <zephyr/drivers/gpio.h>

#include <argos_smd.h>

LOG_MODULE_REGISTER(ARGOS_SMD, CONFIG_ARGOS_SMD_LOG_LEVEL);


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
 * @param dev_smd Driver device passed to provide access to buffers.
 */
static void uart_rx_handler(const struct device *dev, void *dev_smd)
{
	const struct device *argos_smd_dev = dev_smd;
	struct argos_smd_data *drv_data = argos_smd_dev->data;

	int len = 0;
	int offset = 0;

	if (drv_data->status == RESPONSE_CLEAR) {
		drv_data->response.len = 0;
	}

	offset = drv_data->response.len;
	argos_smd_callback_t callback = drv_data->callback;

	if ((uart_irq_update(dev) > 0) && (uart_irq_is_pending(dev) > 0)) {
		if (uart_irq_rx_ready(dev)) {

			len = uart_fifo_read(dev, &drv_data->response.data[offset], 255 - offset);
			//LOG_DBG("Received %d bytes - offset %d", len, offset);

			while (len > 0) {
				if (drv_data->response.data[offset] == '+') 
				{
					drv_data->status = RESPONSE_PENDING;
				}
				if ((drv_data->response.data[offset] == '\n') && (drv_data->status == RESPONSE_PENDING))
                {
					drv_data->status = RESPONSE_SUCCESS;
                    LOG_DBG("Response success.");
                    if (callback != NULL) {
                        callback(dev, dev_smd);
                    }
                }

				offset++;
				len--;
				drv_data->response.len = offset;
			}
		}
	}

	if (offset > ARGOS_SMD_BUF_SIZE) {
		drv_data->response.len = 0;
		drv_data->status = RESPONSE_FAIL;
		argos_smd_uart_flush(dev);
		LOG_WRN("Response exceeds buffer, %d.", offset);
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
	__ASSERT(tx->len <= 255, "Command length too long.");

	if (CONFIG_ARGOS_SMD_LOG_LEVEL >= LOG_LEVEL_DBG) {
		LOG_DBG("Command: %s", tx->data);
	}

	for (size_t i = 0; i < tx->len; i++) {
		data->status = RESPONSE_CLEAR;
		uart_poll_out(cfg->uart_dev, (char)tx->data[i]);
	}
	//uart_poll_out(cfg->uart_dev, '\0');
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
int build_read_cmd(const char *cmd_define, char *full_command, size_t buffer_size) {
    size_t required_size = strlen(cmd_define) + READ_CMD_SIZE_TO_ADD; // Length of cmd_define + length of "=?"


    if (buffer_size < required_size) {
		LOG_ERR("Error: Buffer size (%zu) is too small. Required size is %zu.", buffer_size, required_size);
        return ERROR_CMD_LENGTH; // Return an error code
    }

    strncpy(full_command, cmd_define, buffer_size); // Use strncpy to prevent buffer overflow
    strncat(full_command, "=?", buffer_size - strlen(full_command)); // Use strncat to prevent buffer overflow

    return 0; // Return success code
}


/**
 * @brief wake up smd device
 * 
 * @param dev UART peripheral device.
 * @return 0 on success, negative errno value on failure.
 */
int argos_wake_smd(const struct device *dev)
{
    struct argos_smd_config *cfg = (struct argos_smd_config *)dev->config;
    int ret;
    ret = gpio_pin_set_dt(&cfg->gpio_spec, 1);
    if (ret)
        return ret;
    return ret;
}
/**
 * @brief Shutdown or sleep smd device
 * 
 * @param dev UART peripheral device.
 * @return 0 on success, negative errno value on failure.
 */
int argos_sleep_smd(const struct device *dev)
{
    struct argos_smd_config *cfg = (struct argos_smd_config *)dev->config;
    int ret;
    ret = gpio_pin_set_dt(&cfg->gpio_spec, 0);
    if (ret)
        return ret;
    return ret;
}
/**
 * 
/**
 * @brief Initializes GPIO for the Wakeup device.
 * 
 * @param dev UART peripheral device.
 * @return 0 on success, negative errno value on failure.
 */
static int init_gpio(const struct device *dev)
{
    struct argos_smd_config *cfg = (struct argos_smd_config *)dev->config;
    int ret;
    ret = gpio_pin_configure_dt(&cfg->gpio_spec, GPIO_OUTPUT);
    if (ret)
        return ret;
    return ret;
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
        __ASSERT(false, "UART device is not ready");
		return -ENODEV;
	}

    if (!device_is_ready(cfg->gpio_spec.port))
    {
        __ASSERT(false, "GPIO device not ready");
        return -ENODEV;
    }
    if (init_gpio(dev))
    {
        __ASSERT(false, "Failed to initialize GPIO device.");
        return -ENODEV;
    }

	argos_smd_uart_flush(cfg->uart_dev);

	drv_data->response.len = 0;
	drv_data->status = RESPONSE_CLEAR;

	uart_irq_callback_user_data_set(cfg->uart_dev, uart_rx_handler, (void*) dev);
	uart_irq_rx_enable(cfg->uart_dev);

	return 0;
}


/**
 * @brief Ping the Argos SMD.
 * This function sends the command "AT+PING=?" to check if Argos device is ready
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
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

/**
 * @brief Reads the firmware version of the Argos SMD.
 * This function sends the command "AT+FW=?" to the Argos SMD to request its firmware version.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
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

/**
 * @brief Reads the address of the Argos SMD.
 * This function sends the command "AT+ADDR=?" to the Argos SMD to request its address.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
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

/**
 * @brief Reads the serial number of the Argos SMD.
 * This function sends the command "AT+SN=?" to the Argos SMD to request its serial number.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
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

/**
 * @brief Reads the ID of the Argos SMD.
 * This function sends the command "AT+ID=?" to the Argos SMD to request its ID.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
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

/**
 * @brief Reads the configuration of the Argos SMD.
 * This function sends the command "AT+RCONF=?" to the Argos SMD to request its configuration.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
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

/**
 * @brief Reads the prepass enable variable of the Argos SMD.
 * This function sends the command "AT+PREPASS_EN=?" to the Argos SMD to request its configuration.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
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

/**
 * @brief Reads the UTC time configured
 * This function sends the command "AT+UDATE=?" to the Argos SMD to request its configuration.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
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

/**
 * @brief Reads the TX confiuration repetition configured
 * This function sends the command "AT+ATXRP=?" to the Argos SMD to request its configuration.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
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

/**
 * @brief Send an argos message from SMD module.
 * This function sends the command "AT+TX=MSG" to the Argos SMD to send a message.
 *
 * @param dev UART peripheral device.
 * @param TXmessage Message to be sent with the Argos SMD.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_send_message(const struct device *dev, const char *TXmessage) {
    // Check if the message size exceeds the maximum allowed size
    size_t message_length = strlen(TXmessage);
    size_t max_length = TX_MAX_LDA2_PAYLOAD_SIZE / 8;
    
    if (message_length > max_length) {
        LOG_ERR("TXmessage size exceeds the maximum allowed payload size. Message length: %zu, Allowed length: %zu.\n", message_length, max_length);
        return ERROR_CMD_LENGTH;
    }

    // Calculate the required size for cmd array
    size_t cmd_size = sizeof(AT_TX) + message_length; 

    // Allocate memory for the cmd array dynamically
    char *cmd = malloc(cmd_size);
    if (cmd == NULL) {
        LOG_ERR("Failed to allocate memory for command.\n");
        return -1;
    }

    // Format the command to send
    snprintf(cmd, cmd_size, "%s%s", AT_TX, TXmessage);

    LOG_INF("Send Argos message: %s\n", TXmessage);
    
    // Send the command
    if (send_command(dev, cmd, cmd_size, true) != 0) {
        LOG_ERR("Failed to send the message command.\n");
        free(cmd); // Free dynamically allocated memory
        return -1;
    }

    // Free dynamically allocated memory
    free(cmd);

    return 0;
}
const static struct argos_smd_api api = {
	// .send_command = send_command,
	.set_callback = user_set_command_callback,
};

#define ARGOS_SMD_DEFINE(inst)                                                             \
	static struct argos_smd_data argos_smd_data_##inst = {                                 \
		.response.msg_len = 255,                                                           \
	};                                                                                     \
	static const struct argos_smd_config argos_smd_config_##inst = {                       \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                      \
        .gpio_spec = GPIO_DT_SPEC_INST_GET(inst, wakeup_gpios),                            \
	};                                                                                     \
                                                                                           \
	DEVICE_DT_INST_DEFINE(inst, &argos_smd_init, NULL, &argos_smd_data_##inst,             \
			      &argos_smd_config_##inst, POST_KERNEL, ARGOS_SMD_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(ARGOS_SMD_DEFINE)