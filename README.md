# Argos SMD Driver for Zephyr

This is a Zephyr driver for the Argos SMD module based on STM32WL from Arribada. [Hardware repository](https://github.com/arribada/argos-smd-hw). The driver is compatible with both the Arogs SMD Module and the Argos SMD Wing.


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
          - segger
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
    };
};
```

You can now add `#include <argos-smd/argos_smd.h>` to your file and use the API outlined in the [docs]().

// TODO: HOST AND LINK DOCS

See `samples/read_and_write` for an example. 

## Architecture

The Argos SMD module is a Serial Peripheral and is connected to the Zephyr host via UART. The driver uses the UART Polling API for sending data to the argos smd and the Interrupt API for receiving data. The SMD is connected by UART at a baud of 9600.


## Contributing/Developement Setup  

### Commands
```
# All run in the root directory of the project

# Builds and run local tests
make 

# Builds documentation
make docs 

# Builds and runs targets tests
make target_test

```
