# Arribada Argos SMD Zephyr Driver Integration Test

This test is designed to run on a Adafruit Feather nRF52840 with a [Arribada Argos SMD Wing](https://github.com/arribada/featherwings-argos-smd-hw/wiki).

This test can be run with twister by running the following in the projects root directory

```
make test_target
```

The Feather should be connected with a JLink for flashing and over USB for ACM CDC UART which is used for Twister's communication. 
