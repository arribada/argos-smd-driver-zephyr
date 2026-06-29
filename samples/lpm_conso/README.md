# Argos SMD — LPM current-consumption helper (UART)

Drives the STM32WL55 through each low-power profile and **holds** it there for a
fixed window so you can read the current on an external ammeter. This sample is
about **power**, not the wake path — the wake path is proven by `lpm_test`.

## Low-power profiles — `AT+LPM` is a BITMAP (not a 0..4 index)

| bitmap | mode     |
|--------|----------|
| `0x00` | NONE     |
| `0x01` | SLEEP    |
| `0x02` | STOP     |
| `0x04` | STANDBY  |
| `0x08` | SHUTDOWN |

To actually *enter* a mode you must **force** it: `AT+LPM=0x<bitmap>,0x<forced>`
(sending only the bitmap sets the allowed mask and clears the forced mode, so the
module never sleeps). All values are **hex** (`AT+LPM=1` decimal → `+ERROR=1200`).

## Wiring (Adafruit Feather nRF52840)

| Feather | nRF pin | Argos SMD / STM32 | Role                         |
|---------|---------|-------------------|------------------------------|
| TX/RX   | uart0   | module UART       | AT link @ 9600 8N1           |
| **D9**  | P0.26   | **PB3 / WKUP3**   | wake line (drop LOW to sleep)|

**WKUP3 must be LOW before forcing STANDBY/SHUTDOWN** — if it is HIGH at entry
the active level wakes the module immediately and it draws full active current.
The sample calls `argos_smd_wakeup_disable()` before each forced mode.

## What it does

For each mode it drops WKUP3 LOW, forces the mode, prints a marker, and dwells
`HOLD_MS` (default 60 s) so you can record the steady current, then restores NONE.
Build with `-DHOLD_MODE=<n>` to hold a single mode indefinitely for one reading.

## Measured (this board/firmware, clean 60 s windows)

| mode     | current  |
|----------|----------|
| ACTIVE/NONE | 5.5 mA |
| SLEEP    | 2.50 mA  |
| STOP     | 1.38 mA  |
| STANDBY  | 0.93 µA  |
| SHUTDOWN | 0.93 µA  |

STANDBY ≈ SHUTDOWN (~0.93 µA, likely a board leakage floor). Big drop is
STOP → STANDBY (−99.9 %).

## Build & run

```
west build -b adafruit_feather_nrf52840 -- \
  -DZEPHYR_EXTRA_MODULES=<path>/argos-smd-driver-zephyr \
  -DDTS_ROOT=<path>/argos-smd-driver-zephyr
west flash
```
