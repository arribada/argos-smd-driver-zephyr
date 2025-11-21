#ifndef ARGOS_SMD_PRV_H
#define ARGOS_SMD_PRV_H

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>

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

// cmd availables
#define AT_VERSION    "AT+VERSION" // Get version of the module
#define AT_PING       "AT+PING"    // Ping the module
#define AT_FW         "AT+FW"      // Get firmware version
#define AT_ADDR       "AT+ADDR"    // Get MAC address
#define AT_ID         "AT+ID"      // Get device ID
#define AT_SECKEY     "AT+SECKEY"  // Get security key
#define AT_SN         "AT+SN"      // Get serial number
#define AT_RCONF      "AT+RCONF"   // Get radio configuration
// TODO: to be removed saved by default in flash
#define AT_SAVE_RCONF "AT+SAVE_RCONF" // Save radio configuration.
#define AT_LPM        "AT+LPM"        // Get/Set low power mode
#define AT_MC         "AT+MC"         // Get/Set MAC counter
#define AT_TCXO_WU    "AT+TCXO_WU"    // Get/Set TCXO wakeup time
#define AT_TX         "AT+TX"         // Send raw data
#define AT_PREPASS_EN "AT+PREPASS_EN" // Get/Set prepass. Not implemented
#define AT_UDATE      "AT+UDATE"      // UTC datetime update
#define AT_KMAC       "AT+KMAC"       // Get/Set KMAC profile
#define AT_CW         "AT+CW"         // Get/Set Continuous wave test

// Define all the ways functions can return
#define RESPONSE_PENDING 0
#define ERROR_CMD_LENGTH 1
#define ERROR_CMD_BUILD  2
#define RESPONSE_SUCCESS 3
#define RESPONSE_FAIL    4
#define RESPONSE_CLEAR   5

/* wait serial output with 1000ms timeout */
#define CFG_ARGOS_SMD_SERIAL_TIMEOUT 3000

struct argos_smd_buf {
	uint8_t data[ARGOS_SMD_BUF_SIZE];
	size_t len;
};

struct argos_smd_data {
	uint8_t status;
	struct argos_smd_buf command;
	struct argos_smd_buf response;
	bool has_response;

	argos_smd_callback_t callback;
	void *user_data;
};

struct argos_smd_config {
	struct argos_smd_data *data;
	const struct device *uart_dev;
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
