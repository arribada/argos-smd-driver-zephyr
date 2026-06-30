# Command Reference {#command_reference}

Complete reference of all commands supported by the Argos SMD module.

## UART AT Commands

The UART interface uses text-based AT commands at 9600 baud (8N1).
Each command is terminated with `\r`. Responses are prefixed with `+`.

### Read Commands

Read commands use the format `AT+<CMD>=?` and return `+<CMD>=<value>`.

| AT Command | API Function | Description |
|------------|-------------|-------------|
| `AT+PING=?` | argos_read_ping() | Check module is alive |
| `AT+VERSION=?` | argos_read_version() | Module hardware version |
| `AT+FW=?` | argos_read_firmware_version() | Firmware version string |
| `AT+ADDR=?` | argos_read_address() | Argos device address (4 bytes hex) |
| `AT+ID=?` | argos_read_id() | Argos device ID (4 bytes hex) |
| `AT+SN=?` | argos_read_serial_number() | Device serial number |
| `AT+SECKEY=?` | argos_read_seckey() | Secret encryption key |
| `AT+RCONF=?` | argos_read_radioconf() | Radio configuration (decoded) |
| `AT+RCONFRAW=?` | argos_read_radioconf_raw() | Raw radio configuration (16 bytes hex) |
| `AT+PREPASS_EN=?` | argos_read_prepass_enable() | Prepass enable status |
| `AT+LPM=?` | argos_read_lpm() | Low power mode setting |
| `AT+MC=?` | argos_read_mc() | MAC counter value |
| `AT+TCXO_WU=?` | argos_read_tcxo_wu() | TCXO warmup timer value |
| `AT+KMAC=?` | argos_read_kmac() | KMAC profile |
| `AT+UDATE=?` | argos_read_udate() | UTC date/time configured |
| `AT+CW=?` | argos_read_cw() | Continuous wave test configuration |

### Write Commands

Write commands use the format `AT+<CMD>=<value>` and return `+<CMD>=OK` or `+<CMD>=ERROR`.

| AT Command | API Function | Description |
|------------|-------------|-------------|
| `AT+ADDR=<hex>` | argos_set_address() | Set device address |
| `AT+ID=<hex>` | argos_set_id() | Set device ID |
| `AT+SN=<string>` | argos_set_serial_number() | Set serial number |
| `AT+RCONF=<hex>` | argos_set_radioconf() | Set radio configuration |
| `AT+SAVE_RCONF=<0\|1>` | argos_set_saveradioconf() | Save radio config (deprecated) |
| `AT+PREPASS_EN=<0\|1>` | argos_set_prepass_enable() | Enable/disable prepass |
| `AT+LPM=<bitmap>[,<forced>]` | argos_set_lpm() | Set low power mode (hex bitmap, see below) |
| `AT+MC=<value>` | argos_set_mc() | Set MAC counter |
| `AT+TCXO_WU=<value>` | argos_set_tcxo_wu() | Set TCXO warmup timer |
| `AT+KMAC=<profile>` | argos_set_kmac() | Set KMAC profile |
| `AT+UDATE=<datetime>` | argos_set_udate() | Set UTC date/time |
| `AT+CW=<m>,<f>,<p>,<d>` | argos_set_cw() | Start CW RF test |
| `AT+TX=<payload>` | argos_send_payload() | Send satellite uplink payload |

### LPM Mode Values

The LPM value is a **bitmap**, not a 0..4 index. `AT+LPM` expects the value in
**hex** (a decimal value such as `AT+LPM=1` is rejected with `+ERROR=1200`).

| Value | Mode | Description |
|-------|------|-------------|
| 0x00 | NONE | No low power |
| 0x01 | SLEEP | Sleep mode |
| 0x02 | STOP | Stop mode |
| 0x04 | STANDBY | Standby mode |
| 0x08 | SHUTDOWN | Shutdown mode |

The bitmap alone only sets the **allowed** modes and clears any forced mode, so
the module never actually enters the deep mode. To **enter** a mode you must
force it with a second argument:

```
AT+LPM=0x<bitmap>,0x<forced>
```

For example `AT+LPM=0x04,0x04` forces STANDBY, while `AT+LPM=0x04` only allows
STANDBY (forced cleared). Use `0` as the forced value to clear the force.

### UART DFU Commands

DFU commands use the format `AT+DFU=<subcmd>`. The module must be in bootloader mode first.

| AT Command | API Function | Description |
|------------|-------------|-------------|
| `AT+BOOT` | argos_enter_bootloader() | Reboot into bootloader mode |
| `AT+DFU=PING` | argos_dfu_ping() | Check bootloader is ready |
| `AT+DFU=ERASE` | argos_dfu_erase() | Erase application flash (~5s) |
| `AT+DFU=WRITE,<addr>,<hex>` | argos_dfu_write() | Write firmware chunk at address |
| `AT+DFU=VERIFY,<crc32>` | argos_dfu_verify() | Verify firmware CRC32 |
| `AT+DFU=JUMP` | argos_dfu_jump() | Jump to new application |
| `AT+DFU=ABORT` | argos_dfu_abort() | Abort DFU session |
| `AT+DFU=STATUS` | argos_dfu_get_status() | Query DFU progress |

---

## SPI Protocol A+ Commands

The SPI interface uses binary Protocol A+ framing at 125 kHz.
Each transaction is a fixed 64-byte full-duplex SPI exchange.

The protocol is **pipelined**: the response to a command is received in
the **next** SPI transaction. Send a NOP (0x00) to retrieve the response.

### Application Commands (0x00 - 0x2F)

| Code | Define | API Function | Direction | Description |
|------|--------|-------------|-----------|-------------|
| 0x00 | `ARGOS_SPI_CMD_NOP` | (internal) | - | No operation / get previous response |
| 0x01 | `ARGOS_SPI_CMD_READ` | (internal) | Read | Generic read |
| 0x02 | `ARGOS_SPI_CMD_PING` | argos_spi_ping() | Read | Ping module |
| 0x03 | `ARGOS_SPI_CMD_MAC_STATUS` | argos_spi_get_mac_status() | Read | MAC/TX status |
| 0x04 | `ARGOS_SPI_CMD_SPI_STATUS` | (internal) | Read | SPI status |
| 0x05 | `ARGOS_SPI_CMD_READ_VERSION` | argos_spi_get_version() | Read | Firmware version |
| 0x06 | `ARGOS_SPI_CMD_READ_FIRMWARE` | (internal) | Read | Firmware detailed info |
| 0x07 | `ARGOS_SPI_CMD_READ_ADDR` | argos_spi_get_addr() | Read | Device address |
| 0x08 | `ARGOS_SPI_CMD_READ_ID` | argos_spi_get_id() | Read | Device ID |
| 0x09 | `ARGOS_SPI_CMD_READ_SN` | argos_spi_get_sn() | Read | Serial number |
| 0x0A | `ARGOS_SPI_CMD_READ_RCONF` | argos_spi_get_rconf() | Read | Radio configuration |
| 0x0B | `ARGOS_SPI_CMD_WRITE_RCONF_REQ` | argos_spi_set_rconf() | Write | Write radio config (request) |
| 0x0C | `ARGOS_SPI_CMD_WRITE_RCONF` | argos_spi_set_rconf() | Write | Write radio config (data) |
| 0x0D | `ARGOS_SPI_CMD_SAVE_RCONF` | argos_spi_save_rconf() | Write | Save radio config to NVM |
| 0x0E | `ARGOS_SPI_CMD_READ_KMAC` | argos_spi_get_kmac() | Read | KMAC profile |
| 0x0F | `ARGOS_SPI_CMD_WRITE_KMAC_REQ` | argos_spi_set_kmac() | Write | Write KMAC (request) |
| 0x10 | `ARGOS_SPI_CMD_WRITE_KMAC` | argos_spi_set_kmac() | Write | Write KMAC (data) |
| 0x11 | `ARGOS_SPI_CMD_READ_LPM` | argos_spi_get_lpm() | Read | Low power mode |
| 0x12 | `ARGOS_SPI_CMD_WRITE_LPM_REQ` | argos_spi_set_lpm() | Write | Write LPM (request) |
| 0x13 | `ARGOS_SPI_CMD_WRITE_LPM` | argos_spi_set_lpm() | Write | Write LPM (data) |
| 0x14 | `ARGOS_SPI_CMD_WRITE_TX_REQ` | argos_spi_write_tx() | Write | TX uplink (request) |
| 0x15 | `ARGOS_SPI_CMD_WRITE_TX_SIZE` | argos_spi_write_tx() | Write | TX payload size |
| 0x16 | `ARGOS_SPI_CMD_WRITE_TX` | argos_spi_write_tx() | Write | TX payload data |
| 0x17 | `ARGOS_SPI_CMD_READ_CW` | (internal) | Read | CW test parameters |
| 0x18 | `ARGOS_SPI_CMD_WRITE_CW_REQ` | (internal) | Write | Write CW (request) |
| 0x19 | `ARGOS_SPI_CMD_WRITE_CW` | (internal) | Write | Write CW (data) |
| 0x1A | `ARGOS_SPI_CMD_READ_PREPASSEN` | (internal) | Read | Prepass enable status |
| 0x1B | `ARGOS_SPI_CMD_WRITE_PREPASSEN_REQ` | (internal) | Write | Write prepass (request) |
| 0x1C | `ARGOS_SPI_CMD_WRITE_PREPASSEN` | (internal) | Write | Write prepass (data) |
| 0x1D | `ARGOS_SPI_CMD_READ_UDATE` | (internal) | Read | UTC date/time |
| 0x1E | `ARGOS_SPI_CMD_WRITE_UDATE_REQ` | (internal) | Write | Write date (request) |
| 0x1F | `ARGOS_SPI_CMD_WRITE_UDATE` | (internal) | Write | Write date (data) |
| 0x20 | `ARGOS_SPI_CMD_WRITE_ID_REQ` | argos_spi_set_id() | Write | Write ID (request) |
| 0x21 | `ARGOS_SPI_CMD_WRITE_ID` | argos_spi_set_id() | Write | Write ID (data) |
| 0x22 | `ARGOS_SPI_CMD_WRITE_ADDR_REQ` | argos_spi_set_addr() | Write | Write address (request) |
| 0x23 | `ARGOS_SPI_CMD_WRITE_ADDR` | argos_spi_set_addr() | Write | Write address (data) |
| 0x24 | `ARGOS_SPI_CMD_READ_SECKEY` | argos_spi_get_secret_key() | Read | Secret key |
| 0x25 | `ARGOS_SPI_CMD_WRITE_SECKEY_REQ` | argos_spi_set_secret_key() | Write | Write secret key (request) |
| 0x26 | `ARGOS_SPI_CMD_WRITE_SECKEY` | argos_spi_set_secret_key() | Write | Write secret key (data) |
| 0x27 | `ARGOS_SPI_CMD_READ_SPIMAC_STATE` | argos_spi_get_spimac_state() | Read | SPI MAC state (debug) |
| 0x28 | `ARGOS_SPI_CMD_READ_TCXO_WU` | argos_spi_get_tcxo_wu() | Read | TCXO warmup timer |
| 0x29 | `ARGOS_SPI_CMD_WRITE_TCXOWU_REQ` | argos_spi_set_tcxo_wu() | Write | Write TCXO (request) |
| 0x2A | `ARGOS_SPI_CMD_WRITE_TCXOWU` | argos_spi_set_tcxo_wu() | Write | Write TCXO (data) |
| 0x2B | `ARGOS_SPI_CMD_READ_RCONF_RAW` | argos_spi_get_rconf_raw() | Read | Raw radio config (16 bytes) |
| 0x2C | `ARGOS_SPI_CMD_READ_MC` | argos_spi_get_mc() | Read | Message counter (uint16 LE) |
| 0x2D | `ARGOS_SPI_CMD_WRITE_MC_REQ` | argos_spi_set_mc() | Write | Write message counter (request) |
| 0x2E | `ARGOS_SPI_CMD_WRITE_MC` | argos_spi_set_mc() | Write | Write message counter (data, mod 512) |
| 0x2F | `ARGOS_SPI_CMD_READ_KCFG` | argos_spi_get_kcfg() | Read | Stack config bitmap (uint32 LE, read-only) |

### SPI Low Power Mode

LPM over SPI uses the same **bitmap** as the UART `AT+LPM` command
(NONE=0x00, SLEEP=0x01, STOP=0x02, STANDBY=0x04, SHUTDOWN=0x08).

| API Function | Description |
|-------------|-------------|
| argos_spi_get_lpm() | Read the current LPM setting (CMD 0x11) |
| argos_spi_set_lpm() | Set the allowed-modes bitmap; clears the forced mode (CMD 0x12/0x13) |
| argos_spi_set_lpm_forced() | Set the bitmap **and** force a mode: `(dev, bitmap, forced)`; `forced=0` clears the force |
| argos_spi_wakeup_enable() | Drive the WKUP3 wake line (Feather P0.26 -> STM32 PB3) HIGH |
| argos_spi_wakeup_disable() | Drive the WKUP3 wake line LOW |

As on the UART side, `argos_spi_set_lpm(bitmap)` only sets the allowed mask and
clears the forced mode, so the module never enters the deep mode. To enter a
mode, force it with `argos_spi_set_lpm_forced(dev, bitmap, forced)`. STANDBY
(0x04) and SHUTDOWN (0x08) wake on a **rising edge** of WKUP3 (a held HIGH does
not wake); SHUTDOWN over SPI additionally requires NRST.

### SPI Write Sequence

SPI write commands use a 2 or 3-step request/data pattern:

```
Step 1: WRITE_xxx_REQ  -> Module prepares for write
Step 2: WRITE_xxx      -> Send actual data
        (NOP to read response)
```

For TX uplink, there is a 3-step sequence:

```
Step 1: WRITE_TX_REQ   -> Module prepares for TX
Step 2: WRITE_TX_SIZE  -> Send payload size (uint16)
Step 3: WRITE_TX       -> Send payload data
        (Poll MAC_STATUS until TX_DONE or TX_TIMEOUT)
```

### DFU Bootloader Commands (0x30 - 0x3F)

These commands are only available when the module is in bootloader mode.
Use `CMD_DFU_ENTER` (0x3F) from application mode to reboot into bootloader.

| Code | Define | API Function | Description |
|------|--------|-------------|-------------|
| 0x30 | `ARGOS_SPI_DFU_CMD_PING` | argos_dfu_ping() | Ping bootloader |
| 0x31 | `ARGOS_SPI_DFU_CMD_GET_INFO` | argos_dfu_get_info() | Bootloader version and flash layout |
| 0x32 | `ARGOS_SPI_DFU_CMD_ERASE` | argos_dfu_erase() | Erase application flash (2-3s) |
| 0x33 | `ARGOS_SPI_DFU_CMD_WRITE_REQ` | argos_dfu_write_chunk() | Write request (address + size) |
| 0x34 | `ARGOS_SPI_DFU_CMD_WRITE_DATA` | argos_dfu_write_chunk() | Write flash data chunk |
| 0x35 | `ARGOS_SPI_DFU_CMD_READ_REQ` | argos_dfu_read() | Read request (address + size) |
| 0x36 | `ARGOS_SPI_DFU_CMD_READ_DATA` | argos_dfu_read() | Read flash data |
| 0x37 | `ARGOS_SPI_DFU_CMD_VERIFY` | argos_dfu_verify() | Verify firmware CRC32 |
| 0x38 | `ARGOS_SPI_DFU_CMD_RESET` | argos_dfu_reset() | Reset device |
| 0x39 | `ARGOS_SPI_DFU_CMD_JUMP` | argos_dfu_jump() | Jump to application |
| 0x3A | `ARGOS_SPI_DFU_CMD_GET_STATUS` | argos_dfu_get_status_spi() | DFU session status |
| 0x3B | `ARGOS_SPI_DFU_CMD_ABORT` | argos_dfu_abort() | Abort DFU session |
| 0x3C | `ARGOS_SPI_DFU_CMD_SET_HEADER` | argos_dfu_set_header() | Set application header (256 bytes) |
| 0x3F | `ARGOS_SPI_CMD_DFU_ENTER` | argos_dfu_enter() | Enter DFU mode (from app) |

### Protocol Status Codes

Status codes returned in the response frame (enum argos_protocol_status):

| Code | Define | Description |
|------|--------|-------------|
| 0x00 | `PROT_OK` | Success |
| 0x01 | `PROT_ERROR` | Generic error |
| 0x02 | `PROT_CRC_ERROR` | Data CRC mismatch |
| 0x03 | `PROT_ADDR_ERROR` | Invalid address |
| 0x04 | `PROT_SIZE_ERROR` | Invalid size |
| 0x05 | `PROT_FLASH_ERROR` | Flash operation failed |
| 0x06 | `PROT_BUSY` | Busy, retry later |
| 0x07 | `PROT_INVALID_CMD` | Unknown command |
| 0x08 | `PROT_TIMEOUT` | Operation timeout |
| 0x09 | `PROT_NOT_READY` | Prerequisite missing |
| 0x0A | `PROT_INVALID_HEADER` | Bad application header |
| 0x0B | `PROT_VERIFY_ERROR` | Verification failed |
| 0x10 | `PROT_FRAME_CRC_ERROR` | Frame CRC error (resend) |
| 0x11 | `PROT_SEQ_ERROR` | Sequence number mismatch |
| 0x12 | `PROT_FRAME_ERROR` | Malformed frame |

### MAC Status Codes

TX result codes from `CMD_MAC_STATUS` (0x03) (enum argos_mac_status):

| Code | Define | Description |
|------|--------|-------------|
| 0x00 | `MAC_UNKNOWN` | Unknown state |
| 0x01 | `MAC_OK` | Ready |
| 0x02 | `MAC_TX_DONE` | TX success |
| 0x03 | `MAC_TX_SIZE_ERROR` | Invalid TX payload size |
| 0x04 | `MAC_TXACK_DONE` | TX with ACK success |
| 0x05 | `MAC_TX_TIMEOUT` | TX failed (timeout) |
| 0x06 | `MAC_TXACK_TIMEOUT` | TX ACK failed (timeout) |
| 0x07 | `MAC_RX_ERROR` | RX failed |
| 0x08 | `MAC_RX_TIMEOUT` | RX failed (timeout) |
| 0x09 | `MAC_ERROR` | Generic error |
| 0x0A | `MAC_TX_IN_PROGRESS` | TX queued, poll until done |
| 0x0B | `MAC_RX_RECEIVED` | RX data available |
| 0x0C | `MAC_SAT_DETECTED` | Satellite detected |
| 0x0D | `MAC_SAT_LOST` | Satellite lost |
| 0x0E | `MAC_RF_ABORTED` | RF operation aborted |
