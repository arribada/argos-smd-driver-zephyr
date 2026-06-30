# Argos SMD — TX after deep-sleep without re-provisioning (UART)

Verifies that the radio configuration (KMAC) **persists across a deep-sleep cold
boot**, i.e. that you do **not** have to re-send `AT+KMAC` after the module wakes
from STANDBY/SHUTDOWN before transmitting.

## Why this matters

STANDBY and SHUTDOWN power down VCORE and **cold-boot** the module on wake. If
the radio config lived only in RAM it would be lost and every wake would need a
fresh `AT+KMAC` before `AT+TX`. This test proves the config is in NVM and
survives.

## What it does

1. Baseline `AT+TX` → expect `+TX=0,...` (config present).
2. Force **STANDBY** (`AT+LPM=0x04,0x04`, WKUP3 LOW first), wake via WKUP3.
3. `AT+TX` **without** re-sending `AT+KMAC` → expect `+TX=0,...`.
4. Repeat for **SHUTDOWN** (`AT+LPM=0x08,0x08`).

## Result (this firmware)

All three `AT+TX` return `+TX=0,FFFFFFFF` — the KMAC/config **persists** across
both STANDBY and SHUTDOWN cold boots. No re-provisioning needed after a wake.

## Wiring & build

Same as `lpm_test` (UART link + **D9/P0.26 → PB3/WKUP3** wake line). See
`lpm_test/README.md`. LPM values are a **bitmap** in hex (see `lpm_conso`).

```
west build -b adafruit_feather_nrf52840 -- \
  -DZEPHYR_EXTRA_MODULES=<path>/argos-smd-driver-zephyr \
  -DDTS_ROOT=<path>/argos-smd-driver-zephyr
west flash
```
