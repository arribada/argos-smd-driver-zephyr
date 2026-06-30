# Argos SMD UART Cmd Sample

This sample is designed for the Argos SMD Wing attached to the Adafruit Feather nRF52840.
It runs a sequence of AT commands over UART (ping, read radio config, set/read address,
send a payload) to verify communication with the module.

## Build and Run

```
# Build
west build -b adafruit_feather_nrf52840

# Flash
west flash

# RTT
west rtt
```

You can also see the log output over USB ACM CDC UART.

## Waking the module from SHUTDOWN (important)

The Argos SMD is an STM32WL55. In its **SHUTDOWN** low-power mode the STM32WL powers
down its whole VCORE domain: **the UART (and SPI) are dead**. The module can only be
woken by one of:

- a **rising edge on WKUP3 = STM32 PB3** (the Kineis firmware arms this),
- the **NRST** pin (a reset = cold boot), or
- its internal **RTC** timer.

So if the module may be in SHUTDOWN (e.g. after `AT+LPM=...`), the host **must** drive a
wake line before talking to it. This sample uses a GPIO wired to PB3/WKUP3:

| Feather pin | STM32 pin | Purpose |
|-------------|-----------|---------|
| **D9 (P0.26)** | **PB3 / WKUP3** | wake from SHUTDOWN (rising edge) |

This wire is declared as `wakeup-gpios` in
[`boards/adafruit_feather_nrf52840.overlay`](boards/adafruit_feather_nrf52840.overlay).
The sample calls `argos_smd_wakeup_enable()` (drives D9 HIGH) before communicating and
`argos_smd_wakeup_disable()` (LOW) when done. **Without this wire/property a module in
SHUTDOWN is unreachable over UART** — it will simply never answer.

> If your board does not have the D9->PB3 wire, remove the `wakeup-gpios` property from
> the overlay; the sample then continues without wake control, which only works if the
> module is kept out of SHUTDOWN by other means.

## STM32 firmware note

The Argos SMD STM32 firmware must be built with `VERBOSE=0` and `DEBUG=0`. Debug logs on
the module's serial line interfere with AT-command responses.
