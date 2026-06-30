# Argos SMD Driver for Zephyr

## Overview

This driver provides a Zephyr RTOS device driver for the [Argos SMD module](https://github.com/arribada/argos-smd-hw), a satellite communication module based on the STM32WL microcontroller implementing the Kineis/Argos protocol.

The driver supports two communication interfaces:
- **UART** - AT command-based interface at 9600 baud
- **SPI** - Binary Protocol A+ framing at 125 kHz

Both interfaces provide access to module configuration, satellite uplink transmission, and over-the-air (OTA) firmware updates.

See the [GitHub repository](https://github.com/arribada/argos-smd-driver-zephyr) for integration instructions.

## API Modules

The driver API is organized into the following modules:

| Module | Header | Description |
|--------|--------|-------------|
| @ref uart_api | argos_smd.h | UART AT command interface for module configuration and data transmission |
| @ref uart_dfu_api | argos_dfu.h | UART-based device firmware update (DFU) using AT+DFU commands |
| @ref spi_api | argos_smd_spi.h | SPI Protocol A+ interface for module configuration and data transmission |
| @ref spi_dfu_api | argos_smd_dfu_spi.h | SPI-based device firmware update (DFU) using bootloader commands |
| @ref crc_api | argos_crc.h | Shared CRC utilities used by both DFU implementations |

## Architecture

```
+------------------+       +------------------+
|  Application     |       |  Application     |
+------------------+       +------------------+
| UART API         |       | SPI API          |
| (argos_smd.h)    |       | (argos_smd_spi.h)|
+------------------+       +------------------+
| UART DFU         |       | SPI DFU          |
| (argos_dfu.h)    |       | (argos_smd_      |
|                  |       |  dfu_spi.h)      |
+------------------+       +------------------+
| Zephyr UART      |       | Zephyr SPI       |
| Driver           |       | Driver           |
+------------------+       +------------------+
        |                          |
        +--- Argos SMD Module -----+
             (STM32WL)
```

## UART Interface

The UART interface uses AT commands at 9600 baud (8N1). The host sends commands like `AT+PING=?` and receives responses prefixed with `+` (e.g., `+PING=OK`).

### Quick Start (UART)

```c
#include <argos-smd/argos_smd.h>

const struct device *dev = DEVICE_DT_GET_ONE(arribada_argos_smd_uart);

/* Set callback for responses */
argos_smd_set_callback(dev, my_callback, NULL);

/* Read module version */
argos_read_version(dev);

/* Send satellite payload */
argos_send_payload(dev, "DEADBEEF");
```

### DeviceTree Configuration (UART)

```dts
&uart1 {
    status = "okay";
    current-speed = <9600>;

    argos_smd: argos-smd {
        compatible = "arribada,argos-smd-uart";
        wakeup-gpios = <&gpio0 3 GPIO_ACTIVE_HIGH>;  /* optional */
    };
};
```

## SPI Interface

The SPI interface uses Protocol A+ binary framing with a fixed 64-byte transaction size. Each frame contains: magic byte, sequence number, command/status, length, payload, and CRC-8.

### Quick Start (SPI)

```c
#include <argos-smd/argos_smd_spi.h>

const struct device *dev = DEVICE_DT_GET_ONE(arribada_argos_smd_spi);

/* Ping the module */
argos_spi_ping(dev);

/* Get firmware version */
char version[32];
size_t len = sizeof(version);
argos_spi_get_version(dev, version, &len);

/* Send satellite payload */
uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
argos_spi_write_tx(dev, payload, sizeof(payload));
argos_spi_wait_tx_complete(dev, K_SECONDS(30));
```

### DeviceTree Configuration (SPI)

```dts
&spi1 {
    status = "okay";
    cs-gpios = <&gpio0 31 GPIO_ACTIVE_LOW>;

    argos_smd: argos-smd@0 {
        compatible = "arribada,argos-smd-spi";
        reg = <0>;
        spi-max-frequency = <125000>;
        duplex = <SPI_HALF_DUPLEX>;
        irq-gpios = <&gpio0 30 GPIO_ACTIVE_HIGH>;    /* optional */
        reset-gpios = <&gpio0 29 GPIO_ACTIVE_LOW>;    /* optional */
        wakeup-gpios = <&gpio0 26 GPIO_ACTIVE_HIGH>; /* optional: STM32 PB3/WKUP3 */
    };
};
```

### Transaction Spacing

The driver enforces a minimum gap (`ARGOS_SPI_MIN_TX_SPACING_MS`, 8 ms) between
every SPI transaction. The module SPI slave needs a few ms of RX silence to detect
end-of-transaction and re-arm its RX DMA; a transaction clocked into that window
causes an OVR/desync that wedges the slave (silent, MISO=0xFF, only a power-cycle
recovers). Spacing keeps the host out of that window, making sustained back-to-back
traffic safe.

### Low Power Mode (SPI)

The low-power mode value is a **bitmap**, not a 0..4 index:
NONE=0x00, SLEEP=0x01, STOP=0x02, STANDBY=0x04, SHUTDOWN=0x08.

- argos_spi_set_lpm() / argos_spi_get_lpm() set or read the *allowed* mode bitmap.
  Setting the bitmap alone clears the forced mode, so the module never actually
  enters the deep mode.
- argos_spi_set_lpm_forced() sends `[bitmap, forced]` to both allow and force a
  mode (e.g. force STANDBY: bitmap 0x04, forced 0x04).
- argos_spi_wakeup_enable() / argos_spi_wakeup_disable() drive the WKUP3 wake line
  (the configured `wakeup-gpios`, STM32 PB3) HIGH/LOW. A rising edge cold-boots the
  module out of STANDBY; SHUTDOWN over SPI requires NRST (the optional reset GPIO).

## DFU (Firmware Update)

Both interfaces support over-the-air firmware updates. The DFU process follows the same steps regardless of the transport:

1. Enter bootloader mode
2. Wait for bootloader ready (ping)
3. Erase application flash
4. Write firmware in chunks
5. Verify CRC32
6. Jump to new application

### UART DFU Example

```c
#include <argos-smd/argos_dfu.h>

/* firmware_data and firmware_size come from your firmware binary */
int ret = argos_ota_update(dev, firmware_data, firmware_size, progress_callback);
```

### SPI DFU Example

```c
#include <argos-smd/argos_smd_dfu_spi.h>

/* firmware_data and firmware_size come from your firmware binary */
int ret = argos_spi_firmware_update(dev, firmware_data, firmware_size,
                                    progress_callback, NULL);
```

## Samples

| Sample | Description |
|--------|-------------|
| `samples/uart_cmd` | UART AT command interface test |
| `samples/spi_cmd` | SPI Protocol A+ interface test |
| `samples/uart_dfu_test` | UART DFU firmware update test |
| `samples/spi_dfu_test` | SPI DFU firmware update test |
| `samples/lpm_test` | UART low-power-mode wake test across all deep modes |
| `samples/lpm_conso` | UART per-mode current-consumption helper (hold a mode, read an ammeter) |
| `samples/lpm_tx_test` | UART AT+TX after a STANDBY/SHUTDOWN cold boot, config persisting in NVM |
| `samples/lpm_test_spi` | SPI low-power-mode wake test |
| `samples/uart_probe` | UART diagnostic utility (multi-baud / poll-vs-irq / wake), not pass/fail |
