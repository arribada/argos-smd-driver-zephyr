# Argos SMD Driver for Zephyr

This is a Zephyr driver for the Argos SMD module based on STM32WL from Arribada. [Hardware repository](https://github.com/arribada/argos-smd-hw). The driver is compatible with both the Arogs SMD Module and the Argos SMD Wing.

## Note Before Usage

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

Then in your `prj.conf` enable the driver:

```
CONFIG_ARGOS_SMD=y
CONFIG_UART_INTERRUPT_DRIVEN=y
```

Now add the driver to your DTS, or a create an overlay:

```
&uart0 {
    status = "okay";
    current-speed = <9600>;

    argossmd {
        compatible = "arribada,argossmd";
        /* Optional: GPIO to wake up module from low power mode */
        /* wakeup-gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>; */
    };
};
```

### Optional Wakeup GPIO

If your Argos SMD module is configured to use low power mode (`AT+LPM`), you can optionally configure a GPIO pin to wake it up before communication.

To enable the wakeup GPIO, add the `wakeup-gpios` property to your devicetree:

```
argossmd {
    compatible = "arribada,argossmd";
    wakeup-gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>;
};
```

Then in your code, you need to manually control the wakeup pin:

```c
const struct device *dev_smd = DEVICE_DT_GET_ONE(arribada_argossmd);

/* Enable wakeup pin before communicating with the module */
int ret = argos_smd_wakeup_enable(dev_smd);
if (ret == 0) {
    /* Pin successfully enabled, module is now awake */
}

/* Perform your AT commands here */
argos_read_ping(dev_smd);
argos_set_address(dev_smd, "ABCDEF01");
/* ... */

/* Disable wakeup pin when done to allow low power mode */
argos_smd_wakeup_disable(dev_smd);
```

This approach gives you full control over when the module is awake, allowing you to:
- Keep the module awake for multiple commands (more efficient)
- Decide when to let it enter low power mode
- Optimize power consumption based on your application needs

If no wakeup GPIO is configured, the functions will return `-ENOTSUP` and you can continue normal operation.

You can now add `#include <argos-smd/argos_smd.h>` to you code and use the API outlined in the [docs](https://arribada.github.io/argos-smd-driver-zephyr). Also see `samples/read_and_write` for an basic example.

## Architecture

The Argos SMD module is a Serial Peripheral and is connected to the Zephyr host via UART at a baudrate of 9600. The driver uses the UART Polling API for sending data to the argos smd and the Interrupt API for receiving data.

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
