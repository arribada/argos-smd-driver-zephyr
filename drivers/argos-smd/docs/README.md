# Documentation

## Overview

This driver provides a set of APIs to control the [Argos SMD module](https://github.com/arribada/argos-smd-hw). The driver is implemented as a Zephyr device driver and provides a set of APIs to control the communication chipet.

### Command

The command following the same pattern as the AT command used by CLS and their GUI. It's also possible to use the argos smd with the CLS gui.

### Response

The response 

```text
   Request Argos firmware version
   MGR_AT+FW=07c790a__0x1,Feb  5 2024_17:48:12

   Request Argos serial number
   +SN=SMD_01__TEST00

   Request Argos address
   +ADDR=22675ad0

   Request Argos ID
   +ID=XXXXXX

   Request Argos configuration
   +RCONF=401625000,401635000,27,LDA2

   Sending Test command to Argos: AT+TX=FFFFFFFF
   [MCU_MISC] RFIC PA select (0:LPA, 1:HPA) = 0
   [MCU_MISC] RFIC PA level=-9 dBm, external PA gain = 36 dB
   MGR_AT_CMD MAC reported OK to previous command.
   +OK
```

## Usage

The driver provides a set of APIs to control the SMD module as a coprocessor. The following sections describe the APIs provided by the driver.

### Initialization

The driver is initialized by creating a device struct pointing at the `arribada_argossmd` driver. The device struct is then used to control the module.

```c
const struct device *dev = DEVICE_DT_GET_ONE(arribada_argossmd);
```
