# Argos SMD — LPM (low-power mode) wake test over SPI

Exercises the STM32WL55 low-power profiles through the SPI transport and verifies
the host can put the module to sleep and **wake it back**. Power consumption is
measured separately (`lpm_conso`, UART) — this test proves the **SPI wake path**.

## Low-power profiles — LPM is a BITMAP (not a 0..4 index)

`argos_spi_set_lpm(bitmap)` / `argos_spi_set_lpm_forced(bitmap, forced)`
(SPI cmds `WRITE_LPM_REQ 0x12` + `WRITE_LPM 0x13`).

| bitmap | mode     | how it wakes over SPI                         | recoverable by host? |
|--------|----------|-----------------------------------------------|----------------------|
| `0x00` | NONE     | always on                                     | —                    |
| `0x01` | SLEEP    | SPI activity                                  | yes                  |
| `0x02` | STOP     | SPI **NSS edge** (re-sleeps → grace-window)   | yes (grace-window)   |
| `0x04` | STANDBY  | **WKUP3 rising edge** (cold-boot)             | yes (WKUP3)          |
| `0x08` | SHUTDOWN | **NRST only** (no WKUP3, no SPI)              | NO over SPI → NRST   |

Key SPI-specific behaviour (validated on this board/firmware):

- **STOP** wakes on an NSS edge but immediately **re-sleeps** (bitmap retained).
  The firmware exposes a ~500 ms **grace window** after an NSS wake: send a wake
  transaction, then within the window `set_lpm(0x00)` to clear the bitmap and
  keep it awake (retry — each wake re-arms a fresh window).
- **STANDBY** cold-boots on a **WKUP3 rising edge** (P0.26 LOW→HIGH); the cold
  boot clears the bitmap so it stays awake.
- **SHUTDOWN** does **not** wake on WKUP3 over SPI — it needs **NRST**. With no
  host NRST wired, a SHUTDOWN module is stuck until NRST/power-cycle. Don't write
  `0x08` over SPI unless you have NRST control (see the optional `reset-gpios`).

## Wiring (Adafruit Feather nRF52840)

| Feather | nRF pin | Argos SMD / STM32 | Role                          |
|---------|---------|-------------------|-------------------------------|
| SCK     | P0.14   | SPI_CLK           | SPI clock                     |
| MOSI    | P0.13   | SPI_MOSI          | SPI MOSI                      |
| MISO    | P0.15   | SPI_MISO          | SPI MISO                      |
| D10     | P0.10   | SPI_NSS           | chip select                   |
| **D9**  | P0.26   | **PB3 / WKUP3**   | **wake line (REQUIRED here)** |
| D6      | P0.07   | NRST              | OPTIONAL host reset (see overlay) |

`wakeup-gpios` (P0.26) is enabled and there is **no** `irq-gpios` (the driver is
pipelined/polled). `reset-gpios` (P0.07→NRST) is left available but **optional** —
wire it only if you want host NRST (e.g. to recover SHUTDOWN).

## What the test does

After a warmup-tolerant alive check it validates, in order (STANDBY/STOP first,
SHUTDOWN last because it bricks the module):
- WRITE-PATH — `set_lpm(0x00)` returns a clean ACK,
- STANDBY — forced entry sleeps (SPI goes silent), a WKUP3 rising edge wakes it,
- STOP — entry, then grace-window recovery (wake tx + `set_lpm(0x00)`),
- SHUTDOWN — forced entry sleeps; WKUP3 does **not** wake it (NRST required).

## Important: SPI transaction spacing

The module SPI slave needs ~3 ms of RX silence to close a transaction and re-arm
its RX DMA; a transaction clocked into that window causes an OVR/desync that
**wedges the slave** (muet, MISO=0xFF, only a power-cycle recovers). The driver
now enforces `ARGOS_SPI_MIN_TX_SPACING_MS` (8 ms) between **every** transaction —
this is what makes sustained SPI load safe (200/200 back-to-back pings + 100/100
LPM writes with zero degradation). Don't remove it.

## Build & run

```sh
west build -b adafruit_feather_nrf52840 -- \
  -DZEPHYR_EXTRA_MODULES=<path>/argos-smd-driver-zephyr \
  -DDTS_ROOT=<path>/argos-smd-driver-zephyr
west flash
```

Logs go to RTT (`JLinkRTTClient`). The module must run the **SPI** firmware. A
module left in SHUTDOWN (or wedged before the spacing fix) needs a **power-cycle**
or NRST — a plain reset of the host nRF is not enough.
