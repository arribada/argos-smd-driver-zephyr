# Argos SMD SPI Cmd Sample

This sample exercises the high-level SPI API of the Argos SMD driver (Protocol A+) on an
Argos SMD module attached to the Adafruit Feather nRF52840: sync, reset, ping, read
version/serial/ID/address/config, write ID/address/secret-key/config, and a TX message.

## Build and Run

```
# Build
west build -b adafruit_feather_nrf52840

# Flash
west flash

# RTT
west rtt
```

## Wiring

See [`boards/adafruit_feather_nrf52840.overlay`](boards/adafruit_feather_nrf52840.overlay).

| Feather pin | STM32 pin | Purpose |
|-------------|-----------|---------|
| SCK  (P0.14) | PA1  | SPI clock |
| MOSI (P0.13) | PB5  | SPI MOSI |
| MISO (P0.15) | PB4  | SPI MISO |
| CS   (P0.10) | PA15 | SPI chip-select |
| RST  (P0.07) | NRST | reset / wake from SHUTDOWN (cold boot) |
| D9   (P0.26) | PB3 / WKUP3 | *optional* stateful wake (no cold boot) |

## Waking the module from SHUTDOWN (important)

The Argos SMD is an STM32WL55. In its **SHUTDOWN** low-power mode the STM32WL powers
down its whole VCORE domain: **the SPI bus is dead**. The module can only be woken by:

- the **NRST** pin (a reset = cold boot),
- a **rising edge on WKUP3 = STM32 PB3**, or
- its internal **RTC** timer.

Two ways to wake it, depending on your wiring:

1. **NRST (recommended, no extra wire).** The `RST (P0.07) -> NRST` wire already present in
   the overlay is enough. The sample's `RESET` step calls `argos_spi_reset()`, which pulses
   NRST and cold-boots the module out of SHUTDOWN. This loses the module's RAM state.

2. **WKUP3/PB3 (optional, stateful).** Wire `D9 (P0.26) -> STM32 PB3/WKUP3`, uncomment the
   `wakeup-gpios` property in the overlay, and the driver's `argos_spi_wakeup_enable()` /
   `argos_spi_wakeup_disable()` will wake the module **without** a cold boot, preserving its
   state. The sample's `WAKEUP` step calls `argos_spi_wakeup_enable()`; it is **SKIPped**
   when no `wakeup-gpios` pin is configured.

> **Without either wake path a module in SHUTDOWN is unreachable over SPI** — it will not
> respond to any command.

## STM32 firmware note

The Argos SMD STM32 firmware must be built with `VERBOSE=0` and `DEBUG=0`.
