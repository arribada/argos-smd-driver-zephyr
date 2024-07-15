# Argos SMD Driver for Zephyr

This is a Zephyr driver for the Argos SMD module based on STM32WL from Arribada. [Hardware repository](https://github.com/arribada/argos-smd-hw).

## Architecture

The Argos SMD module is a Serial Peripheral and is connected to the Zephyr host via UART. The driver uses the UART Polling API for sending data to the argos smd and the Interrupt API for receiving data.


### Commands

Commands are sent to the ARGOS SMD module following the Kineis AT command:
    - Commands start by AT+
    - Answer start by '+' and finish by end of line
    - Get command is terminated by "=?"   
    - Set command is terminated by "=VALUE"
```bash
# command availables:
AT+PING=?
AT+FW=?
AT+ADDR=?
AT+SN=?
AT+ID=?
AT+RCONF=?
AT+TX='MGS'
AT+PREPASS_EN=?
AT+UDATE=?
AT+ATXRP=?
```

## Setup

1. `west init -m https://github.com/arribada/argos-smd-driver-zephyr --mr development argos-env`
2. `cd argos-env`
3. `west update`
4. `cd argos-smd-driver-zephyr/examples/simple`
5. To build for adafruit feather nrf52840 - `west build -b adafruit_feather_nrf52840 --build-dir ./argos-env/argos-smd-driver-zephyr/examples/simple/build ./argos-env/argos-smd-driver-zephyr/examples/simple`
6. To Flash with Segger Link - `west flash -d .\argos-env\argos-smd-driver-zephyr\examples\simple\build --skip-rebuild -i SEGGER_SN`
