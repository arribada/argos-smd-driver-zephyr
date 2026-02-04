#ifndef ARGOS_SMD_H
#define ARGOS_SMD_H

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARGOS_SMD_BUF_SIZE 255

/**
 * @brief Callback invoked by when a response has been received.
 *
 * @param buf        Null-terminated character buffer containing the received data.
 * @param user_data  User-supplied context pointer forwarded from the registration call.
 */
typedef void (*argos_smd_callback_t)(const char *buf, void *user_data);

/* Library Functions */
/**
 * @brief Sets the address of the Argos SMD.
 * This function sends the command "AT+ADDR=<address>" to configure the device's address.
 *
 * @param dev Pointer to the device structure.
 * @param address The address string to be set.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_address(const struct device *dev, const char *address);

/**
 * @brief Sets the serial number of the Argos SMD.
 * This function sends the command "AT+SN=<serial_number>" to configure the device's serial number.
 *
 * @param dev Pointer to the device structure.
 * @param serial_number The serial number string to be set.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_serial_number(const struct device *dev, const char *serial_number);

/**
 * @brief Sets the ID of the Argos SMD.
 * This function sends the command "AT+ID=<id>" to configure the device's identifier.
 *
 * @param dev Pointer to the device structure.
 * @param id The ID string to be set.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_id(const struct device *dev, const char *id);

/**
 * @brief Sets the SECKEY of the Argos SMD.
 * This function sends the command "AT+SECKEY=<seckey>" to configure the device's security key.
 *
 * @param dev Pointer to the device structure.
 * @param seckey The key string to be set.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_seckey(const struct device *dev, const char *seckey);

/**
 * @brief Sets the radio configuration of the Argos SMD.
 * This function sends the command "AT+RCONF=<rconf>" to configure the device's radio settings.
 *
 * @param dev Pointer to the device structure.
 * @param rconf The radio configuration string to be set.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_radioconf(const struct device *dev, const char *rconf);

/**
 * @brief Sets the save radio configuration of the Argos SMD.
 * This function sends the command "AT+SAVE_RCONF=<saveconf>" to configure the device's radio
 * settings. This function is not required anymore should be deleted
 *
 * @param dev Pointer to the device structure.
 * @param saveconf Boolean 1 to save
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_saveradioconf(const struct device *dev, const char *saveconf);

/**
 * @brief Sets the prepass enable value  of the Argos SMD.
 * This function sends the command "AT+PREPASS_EN=<prepass>" to configure the device.
 *
 * @param dev Pointer to the device structure.
 * @param prepass The prepass string to be set.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_prepass_enable(const struct device *dev, const char *prepass);

/**
 * @brief Sets the LPM mode value  of the Argos SMD.
 * This function sends the command "AT+LPM=<lpm>" to configure the device.
 *
 * @param dev Pointer to the device structure.
 * @param lpm The LPM mode string to be set. 0 NONE, 1 SLEEP, 2 STOP, 3 STANDBY, 4 SHUTDOWN
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_lpm(const struct device *dev, const char *lpm);

/**
 * @brief Sets the Mac counter value of the Argos SMD.
 * This function sends the command "AT+MC=<mc>" to configure the device.
 *
 * @param dev Pointer to the device structure.
 * @param mc The MC mode string to be set.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_mc(const struct device *dev, const char *mc);

/**
 * @brief Sets the TCXO Warmup timer value of the Argos SMD.
 * This function sends the command "AT+TCXO_WU=<tcxo_wu>" to configure the device.
 *
 * @param dev Pointer to the device structure.
 * @param tcxo_wu The TCXO Warmup timer string to be set.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_tcxo_wu(const struct device *dev, const char *tcxo_wu);

/**
 * @brief Sets the KMAC profile value of the Argos SMD.
 * This function sends the command "AT+KMAC=<kmac>" to configure the device.
 *
 * @param dev Pointer to the device structure.
 * @param kmac The KMAC profile string to be set.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_kmac(const struct device *dev, const char *kmac);

/**
 * @brief Sets the datetime of the Argos SMD.
 * This function sends the command "AT+UPDATE=<datetime>" to configure the device's UTC time.
 *
 * @param dev Pointer to the device structure.
 * @param datetime The datetime string in UTC format to be set.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_udate(const struct device *dev, const char *datetime);

/**
 * @brief Sets the Continuous wave RF test of the Argos SMD.
 * This function sends the command "AT+CW=<cw>" to configure the device.
 * <cw> should be in the format: "<modulation>,<frequency>,<power>,<duration>"
 * <modulation>: 1 = CW, 2 = LDA2, 3 = LDA2L, 4 = VLDA4, 5 = LDK, 6 = HDA4, 0 = NONE
 * <frequency>: Frequency in Hz (e.g., 434000000 for 434 MHz)
 * <power>: Power level in dBm (e.g., 14)
 * <duration>: Duration in milliseconds
 *
 * @param dev Pointer to the device structure.
 * @param cw The continuous wave RF test string to be set.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_set_cw(const struct device *dev, const char *cw);

/**
 * @brief Send an payload from SMD module.
 * This function sends the command "AT+TX=PAYLOAD" to the Argos SMD to send a message.
 *
 * @param dev Argos SMD Device pointer
 * @param payload Payload to be sent with the Argos SMD.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_send_payload(const struct device *dev, const char *payload);

/**
 * @brief Sends a raw AT command to the Argos SMD.
 * This function transmits the specified command string directly to the device.
 *
 * @param dev Pointer to the device structure.
 * @param command The AT command string to send (e.g., "AT+PING=?"). Must be null terminated.
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_send_raw(const struct device *dev, const char *command);

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
 * @brief Enable the wakeup pin (set it HIGH)
 *
 * This function sets the wakeup GPIO pin to high state if it has been configured
 * in the devicetree. Call this before communicating with the module when it is
 * in low power mode (AT+LPM). The pin will remain high until argos_smd_wakeup_disable
 * is called.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, -ENOTSUP if no wakeup GPIO is configured, negative errno on error.
 */
int argos_smd_wakeup_enable(const struct device *dev);

/**
 * @brief Disable the wakeup pin (set it LOW)
 *
 * This function sets the wakeup GPIO pin to low state if it has been configured
 * in the devicetree. Call this when you're done communicating with the module
 * to allow it to enter low power mode.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, -ENOTSUP if no wakeup GPIO is configured, negative errno on error.
 */
int argos_smd_wakeup_disable(const struct device *dev);

/**
 * @brief Read version of Argos SMD.
 * This function sends the command "AT+VERSION=?" to check if Argos device is ready
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_version(const struct device *dev);

/**
 * @brief Ping the Argos SMD.
 * This function sends the command "AT+PING=?" to check if Argos device is ready
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_ping(const struct device *dev);

/**
 * @brief Reads the firmware version of the Argos SMD.
 * This function sends the command "AT+FW=?" to the Argos SMD to request its firmware version.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_firmware_version(const struct device *dev);

/**
 * @brief Reads the address of the Argos SMD.
 * This function sends the command "AT+ADDR=?" to the Argos SMD to request its address.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_address(const struct device *dev);

/**
 * @brief Reads the serial number of the Argos SMD.
 * This function sends the command "AT+SN=?" to the Argos SMD to request its serial number.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_serial_number(const struct device *dev);

/**
 * @brief Reads the ID of the Argos SMD.
 * This function sends the command "AT+ID=?" to the Argos SMD to request its ID.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_id(const struct device *dev);

/**
 * @brief Reads the Secrete Key of the Argos SMD.
 * This function sends the command "AT+SECKEY=?" to the Argos SMD to request its SECKEY.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_seckey(const struct device *dev);

/**
 * @brief Reads the configuration of the Argos SMD.
 * This function sends the command "AT+RCONF=?" to the Argos SMD to request its configuration.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_radioconf(const struct device *dev);

/**
 * @brief Reads the prepass enable variable of the Argos SMD.
 * This function sends the command "AT+PREPASS_EN=?" to the Argos SMD to request its configuration.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_prepass_enable(const struct device *dev);

/**
 * @brief Reads the UTC time configured
 * This function sends the command "AT+UPDATE=?" to the Argos SMD to request its configuration.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_udate(const struct device *dev);

/**
 * @brief Reads the Low Power Profile configured
 * This function sends the command "AT+LPM=?" to the Argos SMD to request its LPM mode.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_lpm(const struct device *dev);

/**
 * @brief Reads the Mac Counter configured
 * This function sends the command "AT+MC=?" to the Argos SMD to request its MC.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_mc(const struct device *dev);

/**
 * @brief Reads the TCXO Warmup timer configured
 * This function sends the command "AT+TCXO_WU=?" to the Argos SMD to request its TCXO Warmup.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_tcxo_wu(const struct device *dev);

/**
 * @brief Reads the KMAC profile configured
 * This function sends the command "AT+KMAC=?" to the Argos SMD to request its KMAC profile.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_kmac(const struct device *dev);

/**
 * @brief Reads the CW used
 * This function sends the command "AT+CW=?" to the Argos SMD to request its CW configuration.
 *
 * @param dev Argos SMD Device pointer
 * @return 0 if the command was successfully sent, -1 if there was an error in building the command.
 */
int argos_read_cw(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif
