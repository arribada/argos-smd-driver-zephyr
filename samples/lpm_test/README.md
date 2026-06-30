# Argos SMD — LPM (low-power mode) wake test over UART

Exercises the STM32WL55 low-power profiles through the UART transport and
verifies the host can wake the module from each one. Power consumption itself is
measured separately with a current meter — this test proves the **wake path**.

## Low-power profiles (`AT+LPM=<n>`)

| value | mode     | wake source while asleep            | wake = cold boot? |
|-------|----------|-------------------------------------|-------------------|
| 0     | NONE     | always on                           | —                 |
| 1     | SLEEP    | UART activity                       | no                |
| 2     | STOP     | UART activity / EXTI                | no                |
| 3     | STANDBY  | rising edge on WKUP3/PB3, NRST, RTC | yes               |
| 4     | SHUTDOWN | rising edge on WKUP3/PB3, NRST, RTC | yes               |

In STANDBY/SHUTDOWN the STM32WL powers down VCORE: the UART is dead until a
rising edge on **WKUP3 = STM32 PB3** wakes (and cold-boots) the module, so the
host must drive that line HIGH before talking to a possibly-asleep module.

## Wiring (Adafruit Feather nRF52840)

| Feather | nRF pin | Argos SMD / STM32 | Role                          |
|---------|---------|-------------------|-------------------------------|
| TX/RX   | uart0   | module UART       | AT command link @ 9600 8N1    |
| **D9**  | P0.26   | **PB3 / WKUP3**   | **wake line (REQUIRED here)** |

The console/logs are on USB-CDC + RTT so `uart0` is free for the module link.

## What the test does

It first pulses the module NRST (Feather P0.07 → STM32 NRST) so it starts from a
known-good, responsive state. Then for each mode 1..4 it:
1. drives the wake line HIGH (+boot delay) and pings (pre-check),
2. sets `AT+LPM=0x<mode>` (**HEX** — see notes),
3. drops the wake line and dwells 3 s so the module enters the mode,
4. drives the wake line HIGH again (+cold-boot delay) and pings → **WAKE OK / FAIL**.

It finishes by restoring `AT+LPM=0x0 (NONE)` and printing a per-mode summary. Every
response wait is timeout-bounded, so a mode that fails to wake reports `WAKE FAIL`
instead of hanging.

Expected result (validated 2026-06 on `_Tx_gui_basic_Pa_Uart` v0.8.1 @ 9600):

```
SLEEP    (LPM=0x1): set=ack    wake=OK
STOP     (LPM=0x2): set=ack    wake=OK
STANDBY  (LPM=0x3): set=slept  wake=OK
SHUTDOWN (LPM=0x4): set=slept  wake=OK
```

## Notes (hard-won)

- **`AT+LPM` takes HEX.** The firmware wants `AT+LPM=0x1`; a decimal `AT+LPM=1`
  is rejected with `+ERROR=1200`. `AT+LPM=?` reads back hex (`+LPM=0x1`).
- **No ack on STANDBY/SHUTDOWN is normal.** Those modes power down the UART on
  entry, so the module sleeps before it can send `+OK`. "set=slept" (no ack)
  therefore *confirms* the deep mode engaged; SLEEP/STOP keep the UART up and ack.
- **If the module looks silent (0 bytes), it's almost always module/port STATE,
  not the read method.** After heavy DFU/SPI cycling the module sits silent on
  UART until it gets a fresh NRST — that is why the sample pulses NRST at startup.
  (An A/B test confirmed `uart_poll_in` and interrupt RX both receive the full
  response fine once the module is reset and responsive — polling is not the
  problem.)
- The Kineis firmware can emit debug/trace lines on the UART when built with
  DEBUG/VERBOSE enabled. Those are not AT responses — display them but don't treat
  them as command replies. The argos driver already filters to `+`-prefixed lines.

## Build & run

```sh
west build -b adafruit_feather_nrf52840 samples/lpm_test -- \
  -DZEPHYR_EXTRA_MODULES=<path>/argos-smd-driver-zephyr \
  -DDTS_ROOT=<path>/argos-smd-driver-zephyr
west flash
```

Logs go to RTT (`JLinkRTTClient`) and USB-CDC. The module must be running the
**UART** firmware (separate from the SPI firmware used by `lpm_test_spi`).

## Measuring consumption

Put the module supply on a current meter and run the test (or hold a single
mode): the meter reading during the 3 s dwell of each mode is the sleep current;
the wake line going HIGH should bring it back to the active current.
