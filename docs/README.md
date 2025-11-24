# Introduction

## Overview

This driver provides a set of APIs to control the [Argos SMD module](https://github.com/arribada/argos-smd-hw). The driver is implemented as a Zephyr device driver and provides a set of APIs to control the communication chipset.

See the [Argos SMD Driver for Zephyr Repo](https://github.com/arribada/argos-smd-driver-zephyr) for information on integrating the driver into your Zephyr project.

## Usage

The driver provides a set of APIs to control the SMD module as a coprocessor. The following sections describe the APIs provided by the driver. See argos_smd.h for the full API.

### Access

The driver is accessed by creating a device struct pointing at the `arribada_argossmd` driver. The device struct is then used to control the module.

```c
const struct device *dev = DEVICE_DT_GET_ONE(arribada_argossmd);
```
