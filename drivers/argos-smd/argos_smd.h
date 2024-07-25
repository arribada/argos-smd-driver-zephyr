#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#ifndef ARGOS_SMD_H
#define ARGOS_SMD_H

#define ARGOS_SMD_BUF_SIZE 255
// List of commands supported by the module
#define READ_CMD_SIZE_TO_ADD 2 // add "=?\r\n" and null terminator
#define TX_MAX_LDA2_PAYLOAD_SIZE 192 // Bits
#define TX_MAX_LDA2L_PAYLOAD_SIZE 196 // Bits
#define TX_MAX_VLDA4_PAYLOAD_SIZE 24 // Bits
#define TX_MAX_LDK_PAYLOAD_SIZE 152 // Bits

// cmd availables
#define AT_VERSION "AT+VERSION" // Get AT command version
#define AT_PING "AT+PING"    // Ping the module
#define AT_FW "AT+FW"        // Get firmware version
#define AT_ADDR "AT+ADDR"    // Get MAC address
#define AT_SN "AT+SN"        // Get serial number
#define AT_ID "AT+ID"        // Get device ID
#define AT_RCONF "AT+RCONF"  // Get/SET radio configuration
#define AT_SAVE_RCONF "AT+SAVE_RCONF"  // SAVE RCONF to non-volatile memory
#define AT_LPM "AT+LPM"  // GET/SET low power mode command
#define AT_TX "AT+TX="       // Send raw data
#define AT_CW "AT+CX="       // Set Continuous wave transmission command
#define AT_PREPASS_EN "AT+PREPASS_EN" // Get/Set prepass 
#define AT_UDATE "AT+UDATE"  // UTC datetime update
#define AT_ATXRP "AT+ATXRP" // get/set repetition commands

// Number of ms before stop waiting for

// Define all the ways functions can return
#define RESPONSE_PENDING               0
#define RESPONSE_SUCCESS               1
#define RESPONSE_FAIL                  2
#define RESPONSE_CLEAR	               3

// Define ERROR_TYPE
#define NO_ERROR		               0
#define ERROR_CMD_LENGTH		       1
#define ERROR_CMD_BUILD 		       2


/* wait serial output with 1000ms timeout */
#define CFG_ARGOS_SMD_SERIAL_TIMEOUT 3000

// Set command to be transmitted
typedef int (*argos_smd_send_command_t)(const struct device *dev, uint8_t *command,
					const uint8_t length, bool timeout);

// Callback
typedef void (*argos_smd_callback_t)(const struct device *dev, void *user_data);

// Set the data callback function for the device
typedef void (*argos_smd_set_callback_t)(const struct device *dev, argos_smd_callback_t callback,
					void *user_data);

struct argos_smd_api {
	// argos_smd_send_command_t send_command;
	argos_smd_set_callback_t set_callback;
};

/**
 * @brief Set command to be transmitted.
 *
 * @param dev Pointer to the device structure.
 * @param command Command to be transmitted.
 * @param length Length of the command (excluding null byte).
 */
// static inline void argos_smd_send_command(const struct device *dev, uint8_t *command,
// 					 const uint8_t length, bool timeout)
// {
// 	struct argos_smd_api *api = (struct argos_smd_api *)dev->api;
// 	return api->send_command(dev, command, length, timeout);
// }

/**
 * @brief Set the data callback function for the device
 *
 * @param dev Pointer to the device structure.
 * @param callback Callback function pointer.
 * @param user_data Pointer to data accessible from the callback function.
 */
static inline void argos_smd_set_callback(const struct device *dev, argos_smd_callback_t callback,
					 void *user_data)
{
	struct argos_smd_api *api = (struct argos_smd_api *)dev->api;
	return api->set_callback(dev, callback, user_data);
}

struct argos_smd_buf {
	uint8_t data[ARGOS_SMD_BUF_SIZE];
	size_t len;
	size_t msg_len;
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
	const struct gpio_dt_spec gpio_spec; 
};

/**
 * @brief Set the command to be transmitted by the UART peripheral.
 *
 * @param dev UART peripheral device.
 * @param command Command to be transmitted.
 * @param length Length of the command.
 * @param timeout Whether to wait for a response from the module.
 * @return int Status of the command.
 */
int send_command(const struct device *dev, uint8_t *command, const uint8_t length, const bool timeout);

/* Library Functions */
/**
 * @brief Ping the Argos SMD.
 * This function sends the command "AT+PING=?" to check if Argos device is ready
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_ping(const struct device *dev);

/**
 * @brief Reads the firmware version of the Argos SMD.
 * This function sends the command "AT+FW=?" to the Argos SMD to request its firmware version.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_firmware_version(const struct device *dev);


/**
 * @brief Reads the address of the Argos SMD.
 * This function sends the command "AT+ADDR=?" to the Argos SMD to request its address.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_address(const struct device *dev);

/**
 * @brief Reads the serial number of the Argos SMD.
 * This function sends the command "AT+SN=?" to the Argos SMD to request its serial number.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_serial_number(const struct device *dev);


/**
 * @brief Reads the ID of the Argos SMD.
 * This function sends the command "AT+ID=?" to the Argos SMD to request its ID.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_id(const struct device *dev);


/**
 * @brief Reads the radio configuration of the Argos SMD.
 * This function sends the command "AT+RCONF=?" to the Argos SMD to request its configuration.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_radio_configuration(const struct device *dev);


/**
 * @brief Reads the prepass enable variable of the Argos SMD.
 * This function sends the command "AT+PREPASS_EN=?" to the Argos SMD to request its configuration.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_prepass_enable(const struct device *dev);


/**
 * @brief Reads the UTC time configured
 * This function sends the command "AT+UPDATE=?" to the Argos SMD to request its configuration.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_udate(const struct device *dev);

/**
 * @brief Reads the TX confiuration repetition configured
 * This function sends the command "AT+ATXRP=?" to the Argos SMD to request its configuration.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_repetition_configured(const struct device *dev);

/**
 * @brief Send an argos message from SMD module.
 * This function sends the command "AT+TX=MSG" to the Argos SMD to send a message.
 *
 * @param dev UART peripheral device.
 * @param TXmessage Message to be sent with the Argos SMD.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_send_message(const struct device *dev, const char *TXmessage);


/**
 * @brief wake up smd device
 * 
 * @param dev UART peripheral device.
 * @return 0 on success, negative errno value on failure.
 */
int argos_wake_smd(const struct device *dev);

/**
 * @brief Shutdown or sleep smd device
 * 
 * @param dev UART peripheral device.
 * @return 0 on success, negative errno value on failure.
 */
int argos_sleep_smd(const struct device *dev);
#endif // argos_smd_PERIPHERAL_H