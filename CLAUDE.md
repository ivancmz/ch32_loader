# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Implementation Status

**✅ COMPLETE** - All 8 phases of implementation are finished:
- ✅ Component structure and protocol foundation
- ✅ Basic ISP commands (identify, read/write config, ISP key, etc.)
- ✅ High-level API and bootloader control
- ✅ Flash operations (erase, write, verify)
- ✅ Complete flash programming API
- ✅ Option bytes support
- ✅ Console command integration
- ✅ Firmware embedding support

**Status:** Implementation complete and successfully compiled. Ready for hardware testing.

## Project Overview

This is an ESP-IDF project that implements a WCH ISP (In-System Programming) component for programming CH32V30x microcontrollers via UART from an ESP32. The project enables firmware updates of CH32V30x devices after an ESP32 OTA update in IoT devices.

**Target:** ESP32 (ESP-IDF 5.5.1)
**ISP Target MCUs:** CH32V303, CH32V305, CH32V307 (all variants)
**Communication:** UART (115200 8N1) + GPIO control (RST, BOOT0)

## Key Features

- ✅ **Full CH32V30x family support** - Works with all V303/V305/V307 variants
- ✅ **Code-based pin configuration** - No menuconfig needed, configure in code
- ✅ **Serial passthrough** - Automatically reads and displays CH32 serial output
- ✅ **Embedded firmware** - CH32 firmware embedded in ESP32 binary
- ✅ **Interactive console** - Easy-to-use commands for flashing and verification
- ✅ **Read protection support** - Lock/unlock device flash
- ✅ **Automatic device detection** - Identifies chip variant and flash size

## Build Commands

### Native Build (requires ESP-IDF installed)

```bash
# Set IDF target (first time only)
idf.py set-target esp32

# Build the project
idf.py build

# Flash to ESP32
idf.py flash

# Monitor serial output
idf.py monitor

# Combined flash and monitor
idf.py flash monitor

# Clean build
idf.py fullclean
```

### Docker Build (recommended for Windows)

```bash
# Build with Docker using ESP-IDF v5.5 image
# IMPORTANT: On Windows/Git Bash, use MSYS_NO_PATHCONV=1 to prevent path conversion
MSYS_NO_PATHCONV=1 docker run --rm -v "$(pwd):/project" -w /project espressif/idf:v5.5 idf.py build

# Flash to ESP32 (requires esptool in host system or use native build for flashing)
idf.py flash

# Monitor serial output
idf.py monitor
```

## Project Structure

```
ch32_loader/
├── main/                       # ESP32 application
│   ├── main.c                  # Main entry point with console commands
│   ├── CMakeLists.txt
│   └── idf_component.yml
├── ch32_code/                  # CH32V303 firmware source (reference)
│   └── User/
│       └── main.c              # Example CH32 firmware (GPIO toggle)
├── reference/
│   └── wch-isp-master/         # C reference implementation
│       ├── main.c              # Reference ISP tool
│       ├── wch_if_uart.c       # UART protocol implementation
│       ├── wch_if.h            # Interface abstraction
│       └── devices/            # Device configuration YAML files
├── build/                      # Build output (gitignored)
├── CMakeLists.txt              # Top-level CMake file
└── sdkconfig                   # ESP-IDF configuration
```

## Architecture

### ESP32 Application (main/)

The ESP32 runs a console application with commands to test the WCH ISP component:
- `flash` - Flash firmware to CH32V303
- Console commands defined in `main.c:18-21` (not yet implemented)
- Uses `console_simple_init.h` component for CLI

### WCH ISP Component (to be implemented)

The component structure should be:
```
components/wch_isp/
├── CMakeLists.txt
├── Kconfig
├── include/
│   └── wch_isp.h          # Public API
└── src/
    ├── wch_isp.c          # Main implementation
    ├── wch_isp_protocol.c # ISP protocol (packets, commands)
    ├── wch_isp_protocol.h
    ├── wch_isp_flash.c    # Flash operations (erase, write, verify)
    └── wch_isp_flash.h
```

### ISP Protocol Architecture

The WCH ISP protocol uses a packet-based UART communication:

**Command packet (ESP32 → CH32):**
```
| 0x57 0xAB | Cmd | Len (2B) | Data | Checksum |
```

**Response packet (CH32 → ESP32):**
```
| 0x55 0xAA | Cmd | 0x00 | Len (2B) | Data | Checksum |
```

**Key Commands:**
- `0xA1` IDENTIFY - Authenticate and get chip info
- `0xA2` END - End session and reset
- `0xA3` XOR_KEY - Calculate XOR encryption key
- `0xA4` ERASE - Erase flash sectors
- `0xA5` WRITE - Write to flash (XOR encrypted)
- `0xA6` VERIFY - Verify flash (XOR encrypted)
- `0xA7` READ_CONFIG - Read option bytes, bootloader version, UID
- `0xA8` WRITE_CONFIG - Write option bytes

### Programming Sequence

1. Enter bootloader mode (BOOT0=1, pulse RST)
2. IDENTIFY (0xA1) - authenticate, get chip type
3. READ_CONFIG (0xA7) - read UID for XOR key calculation
4. XOR_KEY (0xA3) - calculate encryption key from UID
5. ERASE (0xA4) - erase flash
6. WRITE (0xA5) - write firmware in 64-byte chunks (XOR encrypted)
7. XOR_KEY (0xA3) - recalculate key for verify
8. VERIFY (0xA6) - verify written data (XOR encrypted)
9. [Optional] WRITE_CONFIG (0xA8) - update option bytes
10. END (0xA2) - reset to application

### Hardware Control

**Bootloader Entry:**
```c
1. Assert BOOT0 (high)
2. Assert RST (low)
3. Wait 50ms
4. Release RST (high)
5. Wait 100ms
6. Flush UART RX buffer
```

**Normal Boot:**
```c
1. Deassert BOOT0 (low)
2. Pulse RST
```

## Reference Implementation

The `reference/wch-isp-master/` directory contains a C implementation by COKPOWEHEU. Key files:

- **wch_if_uart.c** - UART protocol implementation (packet framing, send/recv)
- **wch_if.h** - Interface abstraction for USB/UART
- **main.c** - Command-line tool implementation showing full programming flow
- **devices/*.yaml** - Device configurations (CH32V303 is device ID 0x17)

**Important:** The reference code includes features NOT needed for this ESP-IDF component:
- YAML parsing (use static CH32V303 config instead)
- File I/O (firmware will be embedded in ESP32 binary)
- USB handling (UART only)

Study the reference for:
- Packet structure and checksum calculation
- Command/response handling
- XOR key calculation algorithm
- Flash write buffering (64-byte chunks)

## CH32V303 Specific Values

```c
#define CH32V303_DEVICE_TYPE     0x30
#define CH32V303_FLASH_SIZE      (256 * 1024)  // 256KB
#define CH32V303_PAGE_SIZE       256
#define CH32V303_SECTOR_SIZE     4096
```

## Embedding CH32 Firmware

Add to component CMakeLists.txt:
```cmake
idf_component_register(
    SRCS "..."
    INCLUDE_DIRS "include"
    REQUIRES driver
    EMBED_FILES ch32_firmware.bin
)
```

Access from code:
```c
extern const uint8_t ch32_firmware_start[] asm("_binary_ch32_firmware_bin_start");
extern const uint8_t ch32_firmware_end[] asm("_binary_ch32_firmware_bin_end");
size_t size = ch32_firmware_end - ch32_firmware_start;
```

## XOR Encryption

Flash write/verify data is XOR-encrypted using device UID:

```c
// Calculate UID checksum (after READ_CONFIG)
uint8_t uid_checksum = 0;
for (int i = 0; i < 8; i++) {
    uid_checksum += unique_id[i];
}

// Calculate XOR key
uint8_t xor_key[8];
for (int i = 0; i < 7; i++) {
    xor_key[i] = uid_checksum;
}
xor_key[7] = (uid_checksum + device_variant) & 0xFF;

// Encrypt/decrypt data
for (int i = 0; i < data_len; i++) {
    data[i] ^= xor_key[i % 8];
}
```

## Development Notes

- **UART Configuration:** 115200 8N1, no flow control
- **Response Timeout:** 1000ms
- **Erase Timeout:** 5000ms (full chip erase is slow)
- **Max Data Chunk:** 64 bytes for write operations
- **Checksum:** Sum of payload bytes mod 256 (excludes packet header)

## Pin Configuration

Pin configuration is done **in code**, not through menuconfig. Modify the defines in `main/main.c` (lines 25-30):

```c
#define CH32_UART_PORT      UART_NUM_2
#define CH32_UART_TX_PIN    GPIO_NUM_17
#define CH32_UART_RX_PIN    GPIO_NUM_16
#define CH32_UART_BAUDRATE  115200
#define CH32_RST_PIN        GPIO_NUM_4
#define CH32_BOOT0_PIN      GPIO_NUM_5
```

These values are used by `create_isp_handle()` to configure the ISP interface. Change them according to your hardware connections.

## Serial Passthrough

The application automatically reads serial data from the CH32 when not performing ISP operations:

- **Initialization**: UART passthrough is initialized on startup
- **Continuous monitoring**: Main loop reads data every 10ms
- **Output format**: Received data is printed with "CH32: " prefix
- **Automatic management**: ISP commands automatically stop/restart passthrough
  - Commands call `stop_ch32_uart_passthrough()` before ISP operations
  - Commands call `init_ch32_uart_passthrough()` after completion

This allows you to see debug output or status messages from your CH32 firmware while the ESP32 is running.

## Supported Chips

The component supports the entire **CH32V30x family** without modification:

| Chip Variant | Chip ID | Device Type | Flash Size | Status |
|--------------|---------|-------------|------------|--------|
| CH32V303VCT6 | 0x30 | 0x17 | 256KB | ✅ Supported |
| CH32V303RCT6 | 0x31 | 0x17 | 256KB | ✅ Supported |
| CH32V303RBT6 | 0x32 | 0x17 | 128KB | ✅ Supported |
| CH32V303CBT6 | 0x33 | 0x17 | 128KB | ✅ Supported |
| CH32V305RBT6 | 0x50 | 0x17 | 128KB | ✅ Supported |
| CH32V307VCT6 | 0x70 | 0x17 | 256KB | ✅ Supported |
| CH32V307RCT6 | 0x71 | 0x17 | 256KB | ✅ Supported |
| CH32V307WCU6 | 0x73 | 0x17 | 256KB | ✅ Supported |

All variants share:
- Same Device Type (0x17)
- Same ISP protocol
- Same bootloader interface
- Same option bytes structure

The component automatically detects the specific chip variant and flash size when you run the `info` command.

## Console Commands Usage

All commands are fully implemented and registered. After flashing the ESP32, use the serial monitor to access the console:

### Available Commands

**`info`** - Get CH32 device information
```
> info
Getting CH32 device info...
=== CH32 Device Information ===
Device:      CH32V30x
Type:        0x30
ID:          0x17
Flash size:  262144 bytes (256 KB)
Bootloader:  v0209
UID:         CD-AB-1D-36-51-BC-3B-9E
===============================
```

**`flash`** - Flash embedded CH32 firmware
```
> flash
Flashing CH32 firmware...
Embedded firmware size: 1024 bytes
Flashing 1024 bytes...
Progress: 1024 / 1024 bytes (100%)
Flash complete!
Verifying 1024 bytes...
Progress: 1024 / 1024 bytes (100%)
Verify complete!
Programming successful!
```

**`verify`** - Verify CH32 flash against embedded firmware
```
> verify
Verifying CH32 flash...
Embedded firmware size: 1024 bytes
Verifying 1024 bytes...
Progress: 1024 / 1024 bytes (100%)
Verify successful!
```

**`rdp`** - Manage read protection
```
> rdp lock     # Enable read protection (WARNING: can only unlock by erasing!)
> rdp unlock   # Disable read protection (WARNING: erases flash!)
```

### Replacing the Embedded Firmware

The current `main/ch32_firmware.bin` is a 1KB placeholder. To use your actual CH32 firmware:

1. Build your CH32V303 firmware and obtain the `.bin` file
2. Replace `main/ch32_firmware.bin` with your firmware binary
3. Rebuild the ESP32 project: `idf.py build`
4. Flash to ESP32: `idf.py flash`
5. Use the `flash` command to program the CH32

## Important Warnings

- **Read Protection:** Writing `RDPR != 0xA5` in option bytes enables read protection and may erase flash
- **Option Bytes:** Handle with care - incorrect values can brick the device
- **GPIO Polarity:** Verify RST and BOOT0 active levels match hardware (typically RST active-low, BOOT0 active-high)

## References

- Full API specification in `CLAUDE_CODE_PROMPT.md`
- Protocol docs: https://github.com/basilhussain/ch32v003-bootloader-docs
- Rust implementation: https://github.com/ch32-rs/wchisp
- CH32V303 datasheets from WCH website
