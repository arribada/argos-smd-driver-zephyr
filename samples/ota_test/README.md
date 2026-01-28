# Argos SMD OTA Test Application

This sample application demonstrates and tests the Device Firmware Update (DFU) functionality for Argos SMD modules via UART.

## Overview

The Argos SMD module supports firmware updates via UART using AT commands. The module can operate in two modes:

- **Application Mode**: Normal AT commands + command to enter DFU mode
- **Bootloader Mode**: DFU-specific commands to receive and flash firmware

This test application validates the complete OTA update flow including:
- Entering bootloader mode
- Starting DFU sessions
- Transferring firmware data in chunks
- Verifying CRC32 checksums
- Handling errors gracefully
- Recovering from failures

## Hardware Requirements

- Target board with UART support (e.g., nRF52840 DK)
- Argos SMD module connected via UART
- Proper devicetree configuration

## Building the Sample

### Basic Build

```bash
west build -b <your_board> samples/ota_test
```

### Build with Full OTA Test Enabled

```bash
west build -b <your_board> samples/ota_test -- -DCONFIG_ARGOS_OTA_TEST_FULL_UPDATE=y
```

**⚠️ WARNING**: Enabling full OTA test will attempt a real firmware update on the device!

### Build with Stress Tests

```bash
west build -b <your_board> samples/ota_test -- -DCONFIG_ARGOS_OTA_TEST_STRESS=y
```

## Devicetree Configuration

You need to configure the UART and Argos SMD device in your devicetree overlay:

```dts
/* boards/<your_board>.overlay */

&uart0 {
    status = "okay";
    current-speed = <9600>;

    argossmd {
        compatible = "arribada,argossmd";
        /* Optional: wakeup GPIO for low power mode */
        /* wakeup-gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>; */
    };
};
```

## AT Commands Reference

### Application Mode Commands

| Command | Description | Response |
|---------|-------------|----------|
| `AT+BOOT\r\n` | Reboot into bootloader mode | `+OK\r\n` then reboot |
| `AT+VER=?\r\n` | Read firmware version | `+VER=<version>\r\n+OK\r\n` |
| `AT+FW=?\r\n` | Read firmware info | `+FW=<info>\r\n+OK\r\n` |

### Bootloader Mode Commands

| Command | Description | Response |
|---------|-------------|----------|
| `AT+DFUSTART=<size>,<crc32>\r\n` | Start DFU session | `+OK\r\n` or `+ERROR=<code>\r\n` |
| `AT+DFUDATA=<hex_data>\r\n` | Send data chunk (max 128 hex chars = 64 bytes) | `+OK\r\n` or `+ERROR=<code>\r\n` |
| `AT+DFUEND\r\n` | Finalize and verify CRC | `+OK\r\n` then reboot to app |
| `AT+DFUABORT\r\n` | Cancel active DFU session | `+OK\r\n` |
| `AT+DFUSTATUS=?\r\n` | Read DFU status | `+DFUSTATUS=<progress>,<state>\r\n+OK\r\n` |
| `AT+PING\r\n` | Test bootloader connection | `+OK\r\n` |

### DFU Error Codes

```c
#define DFU_ERR_NONE           0    /* No error */
#define DFU_ERR_INVALID_SIZE   1    /* Invalid firmware size */
#define DFU_ERR_INVALID_CRC    2    /* CRC32 mismatch */
#define DFU_ERR_FLASH_WRITE    3    /* Flash write error */
#define DFU_ERR_FLASH_ERASE    4    /* Flash erase error */
#define DFU_ERR_NOT_STARTED    5    /* DFU session not started */
#define DFU_ERR_OVERFLOW       6    /* Data exceeds declared size */
#define DFU_ERR_TIMEOUT        7    /* Timeout between chunks */
```

## OTA Update Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                        APPLICATION MODE                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Host                              Argos SMD                     │
│    |                                 |                           │
│    |-- AT+VER=?\r\n ---------------->|                           │
│    |<-- +VER=v1.0.0\r\n+OK\r\n ------|  Check current version   │
│    |                                 |                           │
│    |-- AT+BOOT\r\n ----------------->|                           │
│    |<-- +OK\r\n ---------------------|  Enter bootloader        │
│    |                                 |                           │
│    |        ~~~ REBOOT ~~~           |                           │
│    |                                 |                           │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                        BOOTLOADER MODE                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│    |-- AT+PING\r\n ----------------->|                           │
│    |<-- +OK\r\n ---------------------|  Bootloader ready         │
│    |                                 |                           │
│    |-- AT+DFUSTART=32768,A1B2C3D4--->|  Size=32KB, CRC          │
│    |<-- +OK\r\n ---------------------|  Session started          │
│    |                                 |                           │
│    |-- AT+DFUDATA=4B4E5346010000..-->|  Chunk 0 (64 bytes)      │
│    |<-- +OK\r\n ---------------------|                           │
│    |                                 |                           │
│    |        ... repeat chunks ...    |                           │
│    |                                 |                           │
│    |-- AT+DFUDATA=<last_chunk> ----->|  Last chunk              │
│    |<-- +OK\r\n ---------------------|                           │
│    |                                 |                           │
│    |-- AT+DFUEND\r\n --------------->|  Finalize                │
│    |<-- +OK\r\n ---------------------|  CRC OK, flash OK        │
│    |                                 |                           │
│    |        ~~~ REBOOT TO APP ~~~    |                           │
│    |                                 |                           │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    VERIFICATION POST-UPDATE                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│    |-- AT+PING\r\n ----------------->|                           │
│    |<-- +OK\r\n ---------------------|  App started              │
│    |                                 |                           │
│    |-- AT+VER=?\r\n ---------------->|                           │
│    |<-- +VER=v1.1.0\r\n+OK\r\n ------|  New version confirmed!  │
│    |                                 |                           │
└─────────────────────────────────────────────────────────────────┘
```

## Test Phases

The test application runs through multiple phases:

### Phase 1: Basic Communication Tests
- **PING**: Verify device responds in application mode
- **READ VERSION**: Get current firmware version
- **READ FIRMWARE VERSION**: Get detailed firmware info

### Phase 2: Bootloader Mode Tests
- **ENTER BOOTLOADER**: Reboot into bootloader mode
- **BOOTLOADER PING**: Verify bootloader is responsive

### Phase 3: DFU Session Tests
- **DFU START**: Initialize a DFU session
- **SEND SINGLE CHUNK**: Transfer one data chunk
- **DFU ABORT**: Cancel an active session

### Phase 4: Error Handling Tests
- **INVALID CRC**: Verify bad CRC is rejected
- **CHUNK TOO LARGE**: Verify oversized chunks are rejected
- **DFU WITHOUT START**: Verify data rejected without session

### Phase 5: Full OTA Update (Optional)
- **FULL OTA**: Complete firmware update with all steps
  - Only runs if `CONFIG_ARGOS_OTA_TEST_FULL_UPDATE=y`
  - **⚠️ WARNING**: This performs a real firmware update!

### Phase 6: Stress Tests (Optional)
- **REPEATED OTA**: Multiple consecutive updates
  - Only runs if `CONFIG_ARGOS_OTA_TEST_STRESS=y`
  - Tests reliability and edge cases

## Expected Output

```
========================================
   Argos SMD OTA Test Application
========================================
Test firmware size: 512 bytes
DFU chunk size: 64 bytes

Argos SMD device ready

======================================
  PHASE 1: BASIC COMMUNICATION TESTS
======================================

--- Test: PING ---
RESULT: PASS - Device responding to PING

--- Test: READ VERSION ---
RESULT: PASS - Version read successfully

--- Test: READ FIRMWARE VERSION ---
RESULT: PASS - Firmware version read successfully

======================================
  PHASE 2: BOOTLOADER MODE TESTS
======================================

--- Test: ENTER BOOTLOADER MODE ---
Entering bootloader mode...
Waiting for bootloader to be ready...
Bootloader is ready!
RESULT: PASS - Bootloader mode entered successfully

--- Test: PING IN BOOTLOADER MODE ---
RESULT: PASS - Bootloader responding to PING

======================================
  PHASE 3: DFU SESSION TESTS
======================================

--- Test: DFU START ---
Starting DFU session: size=512 bytes, crc32=0xABCD1234
RESULT: PASS - DFU session started

--- Test: SEND SINGLE CHUNK ---
RESULT: PASS - Single chunk sent successfully

--- Test: DFU ABORT ---
RESULT: PASS - DFU session aborted successfully

======================================
  PHASE 4: ERROR HANDLING TESTS
======================================

--- Test: INVALID CRC (should fail gracefully) ---
RESULT: PASS - DFU with bad CRC rejected

--- Test: CHUNK TOO LARGE (should be rejected) ---
RESULT: PASS - Oversized chunk rejected

--- Test: DFU DATA WITHOUT START (should fail) ---
RESULT: PASS - Chunk without session rejected

======================================
         TEST RESULTS SUMMARY
======================================
   ✓ ALL TESTS PASSED!
========================================
```

## API Usage Example

### Simple OTA Update

```c
#include <argos-smd/argos_dfu.h>

/* Progress callback (optional) */
void my_progress_cb(uint32_t current, uint32_t total)
{
    printk("Progress: %u%%\n", (current * 100) / total);
}

/* Perform OTA update */
int perform_ota(const struct device *dev,
                const uint8_t *fw_image, size_t fw_size)
{
    return argos_ota_update(dev, fw_image, fw_size, my_progress_cb);
}
```

### Manual DFU Process

```c
#include <argos-smd/argos_dfu.h>

int manual_dfu(const struct device *dev,
               const uint8_t *fw_image, size_t fw_size)
{
    int ret;

    /* Calculate CRC32 */
    uint32_t crc = argos_dfu_crc32(fw_image, fw_size);

    /* Enter bootloader */
    ret = argos_enter_bootloader(dev);
    if (ret < 0) return ret;

    /* Wait for bootloader ready */
    ret = argos_wait_bootloader_ready(dev, K_SECONDS(10));
    if (ret < 0) return ret;

    /* Start DFU session */
    ret = argos_dfu_start(dev, fw_size, crc);
    if (ret < 0) return ret;

    /* Send chunks */
    size_t offset = 0;
    while (offset < fw_size) {
        size_t len = MIN(ARGOS_DFU_CHUNK_SIZE, fw_size - offset);
        ret = argos_dfu_send_chunk(dev, &fw_image[offset], len);
        if (ret < 0) {
            argos_dfu_abort(dev);
            return ret;
        }
        offset += len;
    }

    /* Finalize */
    ret = argos_dfu_finish(dev);
    if (ret < 0) return ret;

    /* Wait for reboot */
    k_msleep(2000);

    /* Verify device is back */
    return argos_read_ping(dev);
}
```

## Troubleshooting

### Device Not Ready
- Check UART connections and baud rate (9600)
- Verify devicetree configuration
- Check power supply to Argos SMD module

### Bootloader Not Responding
- Increase timeout in `argos_wait_bootloader_ready()`
- Check if module supports bootloader mode
- Verify AT+BOOT command is supported

### DFU Transfer Fails
- Check CRC32 calculation is correct
- Verify firmware image format is valid
- Ensure chunk sizes don't exceed limits (64 bytes)
- Check for UART buffer overruns

### CRC Mismatch
- Ensure complete firmware image is sent
- Verify no data corruption during transfer
- Check CRC32 polynomial (0xEDB88320 reflected)

## References

- [Argos SMD Driver Documentation](../../README.md)
- [DFU API Header](../../include/argos-smd/argos_dfu.h)
- [Zephyr UART Documentation](https://docs.zephyrproject.org/latest/hardware/peripherals/uart.html)
