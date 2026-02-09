![N|Solid](https://arribada.org/wp-content/uploads/2022/01/arribada_web_logo_g.svg)
# Argos SMD Driver for Zephyr

This is a Zephyr driver for the Argos SMD module based on STM32WL from Arribada. [Hardware repository](https://github.com/arribada/argos-smd-hw). The driver is compatible with both the Argos SMD Module and the Argos SMD Wing.

The driver supports two communication interfaces:
- **UART** — AT command interface at 9600 baud
- **SPI** — High-speed SPI interface

## Samples

Four sample applications are provided:

| Sample | Interface | Description |
|--------|-----------|-------------|
| `samples/uart_cmd` | UART | Test applicative AT commands over UART |
| `samples/spi_cmd` | SPI | Test applicative AT commands over SPI |
| `samples/uart_dfu_test` | UART | Test firmware DFU upload over UART |
| `samples/spi_dfu_test` | SPI | Test firmware DFU upload over SPI |

## Note Before Usage

**WARNING**: The Argos SMD STM32 firmware must be compiled and flashed with `VERBOSE=0` and `DEBUG=0`. If logs are enabled on the STM32 side, debug output on the serial interface can interfere with AT command responses and cause communication errors.

Before you can transmit any data from your Argos SMD Module/Wing you must setup a [Argos CLS Account](https://www.argos-system.org/get-started/) to get your ID, Address and Key.

## Integrating into your Application

To use this Argos SMD Driver in your Zephyr project you need to add this repo to your `west.yaml`

```
manifest:
  group-filter: [+optional]
  remotes:
    - name: zephyrproject-rtos
      url-base: https://github.com/zephyrproject-rtos
    - name: arribada
      url-base: https://github.com/arribada
  projects:
    - name: zephyr
      remote: zephyrproject-rtos
      revision: v4.2.0
      import:
        name-allowlist:
          - cmsis_6    # required by the ARM port
          - hal_nordic # required for Nordic
          - segger     # Required for RTT
    - name: argos-smd-driver-zephyr
      remote: arribada
      revision: v1.0.0
  self:
    path: my-project
```

### UART Configuration

In your `prj.conf`:

```
CONFIG_ARGOS_SMD=y
CONFIG_UART_INTERRUPT_DRIVEN=y
```

Devicetree overlay:

```dts
&uart0 {
    status = "okay";
    current-speed = <9600>;

    argossmd {
        compatible = "arribada,argos-smd-uart";
        /* Optional: GPIO to wake up module from low power mode */
        /* wakeup-gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>; */
    };
};
```

#### Optional Wakeup GPIO (UART only)

If your Argos SMD module is configured to use low power mode (`AT+LPM`), you can optionally configure a GPIO pin to wake it up before communication.

```dts
argossmd {
    compatible = "arribada,argos-smd-uart";
    wakeup-gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>;
};
```

Then in your code, manually control the wakeup pin:

```c
const struct device *dev_smd = DEVICE_DT_GET_ONE(arribada_argos_smd_uart);

/* Enable wakeup pin before communicating with the module */
argos_smd_wakeup_enable(dev_smd);

/* Perform your AT commands here */
argos_read_ping(dev_smd);
argos_set_address(dev_smd, "ABCDEF01");

/* Disable wakeup pin when done to allow low power mode */
argos_smd_wakeup_disable(dev_smd);
```

If no wakeup GPIO is configured, the functions will return `-ENOTSUP` and you can continue normal operation.

### SPI Configuration

In your `prj.conf`:

```
CONFIG_ARGOS_SMD=y
CONFIG_ARGOS_SMD_SPI=y
CONFIG_SPI=y
CONFIG_GPIO=y
```

Devicetree overlay:

```dts
&spi1 {
    compatible = "nordic,nrf-spim";
    status = "okay";
    cs-gpios = <&gpio0 10 GPIO_ACTIVE_LOW>;

    argos_smd: argos-smd@0 {
        compatible = "arribada,argos-smd-spi";
        reg = <0>;
        spi-max-frequency = <125000>;
        /* Optional: hardware reset GPIO */
        /* reset-gpios = <&gpio0 7 GPIO_ACTIVE_LOW>; */
        /* Optional: interrupt/ready GPIO */
        /* irq-gpios = <&gpio0 9 GPIO_ACTIVE_HIGH>; */
    };
};
```

You can now add `#include <argos-smd/argos_smd.h>` to your code and use the API outlined in the [docs](https://arribada.github.io/argos-smd-driver-zephyr). See `samples/uart_cmd` or `samples/spi_cmd` for basic examples.

## Architecture

The Argos SMD module is a Serial Peripheral and can be connected to the Zephyr host via UART or SPI:
- **UART**: baudrate 9600, uses the UART Polling API for TX and the Interrupt API for RX.
- **SPI**: uses the Zephyr SPI API for full-duplex communication.

## Contributing/Developement  

```
# All run in the root directory of the project

# Builds all test and run local tests
make 

# Builds documentation
make docs 

# Same as `make`
make test

# Builds and runs target based tests
make target_test
```
