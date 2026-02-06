#ifndef ARGOS_SMD_PRV_H
#define ARGOS_SMD_PRV_H

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <argos-smd/argos_smd.h>

#ifdef __cplusplus
extern "C" {
#endif

// List of commands supported by the module
#define READ_CMD_SIZE_TO_ADD      2       // add "=?"
#define SET_CMD_SIZE_TO_ADD       1       // add "="
#define TX_MAX_LDA2_PAYLOAD_SIZE  192 / 8 // Bytes
#define TX_MAX_LDA2L_PAYLOAD_SIZE 196 / 8 // Bytes
#define TX_MAX_VLDA4_PAYLOAD_SIZE 24 / 8  // Bytes
#define TX_MAX_LDK_PAYLOAD_SIZE   152 / 8 // Bytes

// Define all the ways functions can return
#define RESPONSE_PENDING 0
#define ERROR_CMD_LENGTH 1
#define ERROR_CMD_BUILD  2
#define RESPONSE_FAIL    3
#define RESPONSE_CLEAR   4

struct argos_smd_buf {
	char data[ARGOS_SMD_BUF_SIZE];
	size_t len;
};

struct argos_smd_data {
	atomic_t status;
	struct argos_smd_buf response;
	bool has_response;

	argos_smd_callback_t callback;
	void *user_data;
};

struct argos_smd_config {
	struct argos_smd_data *data;
	const struct device *uart_dev;
	const struct gpio_dt_spec wakeup_gpio;
};

// Set command to be transmitted
typedef int (*argos_smd_send_command_t)(const struct device *dev, uint8_t *command,
					const uint8_t length, bool timeout);

// Set the data callback function for the device
typedef void (*argos_smd_set_callback_t)(const struct device *dev, argos_smd_callback_t callback,
					 void *user_data);

#ifdef __cplusplus
}
#endif

#endif
