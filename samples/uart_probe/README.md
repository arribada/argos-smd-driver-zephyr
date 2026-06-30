# Argos SMD — UART probe (diagnostic utility)

A low-level **diagnostic tool**, not a pass/fail validation test. It was written
to debug "module is silent on UART" situations and answer questions that the
higher-level samples assume away:

- is the module driving the UART at all, and at which **baud rate**?
- does `uart_poll_in` vs interrupt-driven RX change what we capture? (it does not)
- can a plain UART frame wake the module from a light sleep mode?

## What it does

Configurable at the top of `main.c`:
- multi-baud sweep — send `AT+FW` / `AT+PING` at several baud rates and dump the
  raw bytes received, to find the link speed and confirm the module answers;
- poll vs IRQ A/B read of the same response;
- a UART-wake probe (drive a frame, see if a sleeping module answers).

It just prints what it sees — there is no automated verdict.

## Findings it produced (kept for reference)

- The module answers `+FW=..._Tx_gui_basic_Pa_Uart` at **9600 8N1**.
- `uart_poll_in` is **fine** — earlier 0-byte reads were a module/port-state issue
  (module not woken / TX-RX swapped), not the read method.

## Wiring & build

UART link (uart0) + the **D9/P0.26 → PB3/WKUP3** wake line, same as `lpm_test`.

```
west build -b adafruit_feather_nrf52840 -- \
  -DZEPHYR_EXTRA_MODULES=<path>/argos-smd-driver-zephyr \
  -DDTS_ROOT=<path>/argos-smd-driver-zephyr
west flash
```
