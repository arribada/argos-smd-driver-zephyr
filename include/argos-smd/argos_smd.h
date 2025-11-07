#ifndef ARGOS_SMD_H
#define ARGOS_SMD_H

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARGOS_SMD_BUF_SIZE 255

typedef void (*argos_smd_callback_t)(uint8_t *buf, size_t size, void *user_data);

/* Library Functions */
int argos_send_cmd(const struct device *dev, const char *command);

/**
 * @brief Set the data callback function for the device
 *
 * @param dev Pointer to the device structure.
 * @param callback Callback function pointer.
 * @param user_data Pointer to data accessible from the callback function.
 */
void argos_smd_set_callback(const struct device *dev, argos_smd_callback_t callback,
			    void *user_data);

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
 * @brief Reads the configuration of the Argos SMD.
 * This function sends the command "AT+RCONF=?" to the Argos SMD to request its configuration.
 *
 * @param dev UART peripheral device.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_configuration(const struct device *dev);

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
 * @brief Reads the TX configuration repetition configured
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

#ifdef __cplusplus
}
#endif

#endif
