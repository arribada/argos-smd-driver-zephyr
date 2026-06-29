# Argos SMD — DFU / firmware-flash over SPI

Pushes a firmware image to the STM32WL55 bootloader over SPI: enter DFU, write
the binary in chunks, verify the CRC, and jump to the application.

## Status

- ✅ **Transport works**: enter-DFU → write (~74 KB) → per-chunk + global CRC →
  JUMP all succeed over SPI.
- ⚠️ **App-boot after DFU is UNPROVEN**: after the JUMP (+ power-cycle) the DFU'd
  image did not come up as a responsive SPI app (module answered `-61`, same as a
  known-good image that needs reflashing). The SPI **flash path** is validated;
  that the flashed app then **boots and responds** has not been confirmed on this
  setup. The UART DFU path (`uart_dfu_test`) is validated end-to-end (`result=0`).

## Wiring (Adafruit Feather nRF52840)

| Feather | nRF pin | Argos SMD / STM32 | Role            |
|---------|---------|-------------------|-----------------|
| SCK     | P0.14   | SPI_CLK           | SPI clock       |
| MOSI    | P0.13   | SPI_MOSI          |                 |
| MISO    | P0.15   | SPI_MISO          |                 |
| CS      | P0.10   | SPI_NSS           | chip select     |
| RST     | P0.07   | NRST              | reset to enter bootloader |

SPI runs at 125 kHz (nRF52840 SPIM minimum). The image to flash is the
`argos-smd-at-kineis-firmware_dfu_spi.bin` blob in the repo root.

## Notes / fixes baked in

- SPI clocked at **125 kHz** (100 kHz is rejected by the nRF SPIM as `-EINVAL`).
- DFU response bounds-checked against the transaction size (was an OOB → HardFault).
- Large DFU tx/rx buffers made **static** (were a ~560 B stack overflow).

## Build & run

```
west build -b adafruit_feather_nrf52840 -- \
  -DZEPHYR_EXTRA_MODULES=<path>/argos-smd-driver-zephyr \
  -DDTS_ROOT=<path>/argos-smd-driver-zephyr
west flash
```
