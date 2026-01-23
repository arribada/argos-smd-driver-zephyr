/*
 * Copyright (c) 2023 Arribada Initiative CIC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT arribada_argossmd
#define MODULE        argos_smd

#include <stdio.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <argos-smd/argos_smd.h>
#include "argos_smd_priv.h"

#define ARGOS_SMD_INIT_PRIORITY 60

LOG_MODULE_REGISTER(MODULE, CONFIG_ARGOS_SMD_LOG_LEVEL);

static void argos_smd_uart_flush(const struct device *dev)
{
	const struct argos_smd_config *cfg = dev->config;
	struct argos_smd_data *drv_data = dev->data;

	uint8_t tmp;
	while (uart_fifo_read(cfg->uart_dev, &tmp, 1) > 0) {
	}
	memset(&drv_data->response.data, 0, ARGOS_SMD_BUF_SIZE);

	LOG_DBG("UART RX buffer flushed.");
}

int argos_smd_wakeup_enable(const struct device *dev)
{
	const struct argos_smd_config *cfg = dev->config;

	if (cfg->wakeup_gpio.port == NULL) {
		LOG_WRN("No wakeup GPIO configured");
		return -ENOTSUP;
	}

	int ret = gpio_pin_set_dt(&cfg->wakeup_gpio, 1);
	if (ret != 0) {
		LOG_ERR("Failed to enable wakeup pin (error: %d)", ret);
		return ret;
	}

	LOG_DBG("Wakeup pin enabled (set HIGH)");
	return 0;
}

int argos_smd_wakeup_disable(const struct device *dev)
{
	const struct argos_smd_config *cfg = dev->config;

	if (cfg->wakeup_gpio.port == NULL) {
		LOG_WRN("No wakeup GPIO configured");
		return -ENOTSUP;
	}

	int ret = gpio_pin_set_dt(&cfg->wakeup_gpio, 0);
	if (ret != 0) {
		LOG_ERR("Failed to disable wakeup pin (error: %d)", ret);
		return ret;
	}

	LOG_DBG("Wakeup pin disabled (set LOW)");
	return 0;
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

		if (atomic_get(&drv_data->status) == RESPONSE_PENDING) {
			drv_data->response.len++;
			size_t index = drv_data->response.len - 1;
			drv_data->response.data[index] = byte;

			if (byte == '\n' || byte == '\r') {
				LOG_DBG("Response successfully received");
				drv_data->response.data[index] = '\0';
				atomic_set(&drv_data->status, RESPONSE_CLEAR);
				if (callback != NULL) {
					callback(drv_data->response.data, drv_data->user_data);
				}
			}
		}

		if (drv_data->response.len >= sizeof(drv_data->response.data)) {
			atomic_set(&drv_data->status, RESPONSE_FAIL);
			continue;
		}
	}
}

int send_command(const struct device *dev, uint8_t *command, const uint8_t length)
{
	struct argos_smd_data *data = (struct argos_smd_data *)dev->data;
	const struct argos_smd_config *cfg = dev->config;

	atomic_set(&data->status, RESPONSE_CLEAR);

	for (size_t i = 0; i < length; i++) {
		uart_poll_out(cfg->uart_dev, (char)command[i]);
	}

	uart_poll_out(cfg->uart_dev, '\r');

	return 0;
}

int send_read_cmd(const struct device *dev, const char *cmd)
{
	char buffer[ARGOS_SMD_BUF_SIZE];
	const int buffer_size = sizeof(buffer);

	size_t message_length = strlen(cmd) + READ_CMD_SIZE_TO_ADD;
	snprintf(buffer, buffer_size, "%s=?", cmd);

	return send_command(dev, buffer, message_length);
}

void argos_smd_set_callback(const struct device *dev, argos_smd_callback_t callback,
			    void *user_data)
{
	struct argos_smd_data *data = (struct argos_smd_data *)dev->data;
	data->callback = callback;
	data->user_data = user_data;
}

int argos_read_version(const struct device *dev)
{
	LOG_INF("Requesting Argos version");
	return send_read_cmd(dev, AT_VERSION);
}

int argos_read_ping(const struct device *dev)
{
	LOG_INF("Pinging Argos device");
	return send_read_cmd(dev, AT_PING);
}

int argos_read_firmware_version(const struct device *dev)
{
	LOG_INF("Requesting Argos firmware version");
	return send_read_cmd(dev, AT_FW);
}

int argos_read_address(const struct device *dev)
{
	LOG_INF("Requesting Argos address");
	return send_read_cmd(dev, AT_ADDR);
}

int argos_read_id(const struct device *dev)
{
	LOG_INF("Requesting Argos ID");
	return send_read_cmd(dev, AT_ID);
}

int argos_read_seckey(const struct device *dev)
{
	LOG_INF("Requesting Argos security key");
	return send_read_cmd(dev, AT_SECKEY);
}

int argos_read_serial_number(const struct device *dev)
{
	LOG_INF("Request Argos serial number");
	return send_read_cmd(dev, AT_SN);
}

int argos_read_radioconf(const struct device *dev)
{
	LOG_INF("Request Argos configuration");
	return send_read_cmd(dev, AT_RCONF);
}

int argos_read_prepass_enable(const struct device *dev)
{
	LOG_INF("Request Argos prepass enable");
	return send_read_cmd(dev, AT_PREPASS_EN);
}

int argos_read_udate(const struct device *dev)
{
	LOG_INF("Request Argos UTC time configured");
	return send_read_cmd(dev, AT_UDATE);
}

int argos_read_lpm(const struct device *dev)
{
	LOG_INF("Request Argos low power mode configured");
	return send_read_cmd(dev, AT_LPM);
}

int argos_read_mc(const struct device *dev)
{
	LOG_INF("Request Argos read Mac counter");
	return send_read_cmd(dev, AT_MC);
}

int argos_read_tcxo_wu(const struct device *dev)
{
	LOG_INF("Request Argos read TCXO warmup timer");
	return send_read_cmd(dev, AT_TCXO_WU);
}

int argos_read_kmac(const struct device *dev)
{
	LOG_INF("Request Argos read KMAC profile ");
	return send_read_cmd(dev, AT_KMAC);
}

int argos_read_cw(const struct device *dev)
{
	LOG_INF("Request Argos read CW configuration ");
	return send_read_cmd(dev, AT_CW);
}

int argos_send_raw(const struct device *dev, const char *command)
{
	size_t message_length = strlen(command);
	const size_t max_length = ARGOS_SMD_BUF_SIZE;

	if (message_length > max_length) {
		LOG_ERR("TXmessage size exceeds the maximum allowed payload size. Message length: "
			"%zu, Allowed length: %zu.",
			message_length, max_length);
		return -EINVAL;
	}

	int ret = send_command(dev, (uint8_t *)command, message_length);
	if (ret != 0) {
		LOG_ERR("Failed to send the message command.");
	}
	return ret;
}

static int send_set_cmd(const struct device *dev, const char *cmd, const char *data)
{
	char buffer[ARGOS_SMD_BUF_SIZE];
	const int buffer_size = sizeof(buffer);

	size_t message_length = strlen(cmd) + strlen(data) + SET_CMD_SIZE_TO_ADD;

	if (message_length > buffer_size) {
		LOG_ERR("Command size exceeds the provided buffers size. Message length: %zu, "
			"Provided length: %zu.",
			message_length, buffer_size);
		return -EINVAL;
	}

	if (buffer_size > ARGOS_SMD_BUF_SIZE) {
		LOG_ERR("Command size exceeds the maximum allowed payload size. Message length: "
			"%zu, Provided length: %zu.",
			ARGOS_SMD_BUF_SIZE, buffer_size);
		return -EINVAL;
	}

	snprintf(buffer, buffer_size, "%s=%s", cmd, data);

	return send_command(dev, buffer, message_length);
}

int argos_set_address(const struct device *dev, const char *address)
{
	LOG_INF("Setting Argos address to %s", address);
	return send_set_cmd(dev, AT_ADDR, address);
}

int argos_set_serial_number(const struct device *dev, const char *serial_number)
{
	LOG_INF("Setting Argos serial number to %s", serial_number);
	return send_set_cmd(dev, AT_SN, serial_number);
}

int argos_set_id(const struct device *dev, const char *id)
{
	LOG_INF("Setting Argos ID to %s", id);
	return send_set_cmd(dev, AT_ID, id);
}

int argos_set_radioconf(const struct device *dev, const char *rconf)
{
	LOG_INF("Setting Argos radio config to %s", rconf);
	return send_set_cmd(dev, AT_RCONF, rconf);
}

int argos_set_saveradioconf(const struct device *dev, const char *saveconf)
{
	LOG_INF("Save radio config to %s", saveconf);
	return send_set_cmd(dev, AT_SAVE_RCONF, saveconf);
}

int argos_set_prepass_enable(const struct device *dev, const char *prepass)
{
	LOG_INF("Setting Argos prepass enable to %s", prepass);
	return send_set_cmd(dev, AT_PREPASS_EN, prepass);
}

int argos_set_udate(const struct device *dev, const char *datetime)
{
	LOG_INF("Setting Argos Datetime to %s", datetime);
	return send_set_cmd(dev, AT_UDATE, datetime);
}

int argos_set_lpm(const struct device *dev, const char *lpm)
{
	LOG_INF("Setting Argos Low power profile to %s", lpm);
	return send_set_cmd(dev, AT_LPM, lpm);
}

int argos_set_mc(const struct device *dev, const char *mc)
{
	LOG_INF("Setting Argos Mac Counter to %s", mc);
	return send_set_cmd(dev, AT_MC, mc);
}

int argos_set_tcxo_wu(const struct device *dev, const char *tcxo_wu)
{
	LOG_INF("Setting Argos TCXO warmup to %s", tcxo_wu);
	return send_set_cmd(dev, AT_TCXO_WU, tcxo_wu);
}

int argos_set_kmac(const struct device *dev, const char *kmac)
{
	LOG_INF("Setting Argos KMAC to %s", kmac);
	return send_set_cmd(dev, AT_KMAC, kmac);
}

int argos_set_cw(const struct device *dev, const char *cw)
{
	LOG_INF("Setting Argos set Continuous wave RF test to %s", cw);
	return send_set_cmd(dev, AT_CW, cw);
}

int argos_send_payload(const struct device *dev, const char *paylod)
{
	size_t message_length = strlen(paylod);
	const size_t max_length = TX_MAX_LDA2_PAYLOAD_SIZE;

	if (message_length > max_length) {
		LOG_ERR("TXmessage size exceeds the maximum allowed payload size. Message length: "
			"%zu, Allowed length: %zu.",
			message_length, max_length);
		return -EINVAL;
	}

	LOG_INF("Transmitting Following Message: %s", paylod);
	return send_set_cmd(dev, AT_TX, paylod);
}

static int argos_smd_init(const struct device *dev)
{
	const struct argos_smd_config *cfg = dev->config;
	struct argos_smd_data *drv_data = dev->data;

	if (!device_is_ready(cfg->uart_dev)) {
		LOG_ERR("UART device is not ready");
		return -ENODEV;
	}

	/* Configure optional wakeup GPIO if present */
	if (cfg->wakeup_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->wakeup_gpio)) {
			LOG_ERR("Wakeup GPIO is not ready");
			return -ENODEV;
		}

		int ret = gpio_pin_configure_dt(&cfg->wakeup_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("Failed to configure wakeup GPIO (error: %d)", ret);
			return ret;
		}

		LOG_INF("Wakeup GPIO configured (initially LOW)");
	} else {
		LOG_INF("No wakeup GPIO configured");
	}

	argos_smd_uart_flush(dev);

	drv_data->response.len = 0;
	atomic_set(&drv_data->status, RESPONSE_CLEAR);

	int ret = uart_irq_callback_user_data_set(cfg->uart_dev, uart_rx_handler, (void *)dev);
	if (ret != 0) {
		return ret;
	}

	uart_irq_rx_enable(cfg->uart_dev);

	return 0;
}

#define ARGOS_SMD_DEFINE(inst)                                                                     \
	static struct argos_smd_data argos_smd_data_##inst = {};                                   \
	static const struct argos_smd_config argos_smd_config_##inst = {                           \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                      \
		.wakeup_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, wakeup_gpios, {0}),                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &argos_smd_init, NULL, &argos_smd_data_##inst,                 \
			      &argos_smd_config_##inst, POST_KERNEL, ARGOS_SMD_INIT_PRIORITY,      \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(ARGOS_SMD_DEFINE)
