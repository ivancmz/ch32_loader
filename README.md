# CH32 ISP Loader for ESP32

ESP-IDF component for programming CH32V30x RISC-V microcontrollers via UART from ESP32.

## Features

- ✅ **Full CH32V30x Family Support** - Compatible with all CH32V303, CH32V305, and CH32V307 variants
- ✅ **Interactive Console Example Application** - Easy-to-use commands for device info, flashing, and verification
- ✅ **Embedded Firmware** - CH32 firmware embedded directly in ESP32 binary
- ✅ **Serial Passthrough** - Monitor CH32 serial output in real-time
- ✅ **Automatic Detection** - Identifies chip variant, UID, and flash size
- ✅ **Read Protection** - Lock/unlock device flash memory
- ✅ **Verification** - Automatic verification after programming

## Supported Hardware

### ESP32
- **Platform**: ESP-IDF 5.5.1
- **Target**: ESP32 (Xtensa)
- **Requirements**: UART port + 2 GPIO pins (RST, BOOT0)

### CH32V30x Microcontrollers

All CH32V30x family variants are supported:

| Chip Variant | Chip ID | Flash Size | Package | Status |
|--------------|---------|------------|---------|--------|
| CH32V303VCT6 | 0x30 | 256KB | LQFP100 | ✅ Supported |
| CH32V303RCT6 | 0x31 | 256KB | LQFP64 | ✅ Supported |
| CH32V303RBT6 | 0x32 | 128KB | LQFP64 | ✅ Supported |
| CH32V303CBT6 | 0x33 | 128KB | LQFP48 | ✅ Supported |
| CH32V305RBT6 | 0x50 | 128KB | LQFP64 | ✅ Supported |
| CH32V307VCT6 | 0x70 | 256KB | LQFP100 | ✅ Supported |
| CH32V307RCT6 | 0x71 | 256KB | LQFP64 | ✅ Supported |
| CH32V307WCU6 | 0x73 | 256KB | QFN68 | ✅ Supported |

## Hardware Connections

### Default Pin Configuration

```
ESP32          CH32V30x
─────────────────────────
GPIO19   ───►  UART RX
GPIO18   ◄───  UART TX
GPIO5    ───►  RST
GPIO4    ───►  BOOT0
GND      ───   GND
```

### Pin Configuration in Code

Modify pins in `main/main.c` (lines 25-30):

```c
#define CH32_UART_PORT      UART_NUM_2
#define CH32_UART_TX_PIN    GPIO_NUM_19
#define CH32_UART_RX_PIN    GPIO_NUM_18
#define CH32_UART_BAUDRATE  115200
#define CH32_RST_PIN        GPIO_NUM_5
#define CH32_BOOT0_PIN      GPIO_NUM_4
```

## Quick Start

### 1. Clone Repository

```bash
git clone <repository-url>
cd ch32_loader
```

### 2. Build Project

```bash
idf.py build
```

### 3. Flash to ESP32

```bash
idf.py -p PORT flash monitor
```

Replace `PORT` with your serial port (e.g., `COM3` on Windows, `/dev/ttyUSB0` on Linux).

### 4. Connect Hardware

Connect ESP32 and CH32V30x according to the pin configuration table above.

### 5. Use Console Commands

After flashing, the console will be available. Try these commands:

```
> info          # Get CH32 device information
> flash         # Flash embedded firmware to CH32
> verify        # Verify CH32 flash contents
> rdp lock      # Enable read protection
> rdp unlock    # Disable read protection (erases flash!)
> reset         # Reset CH32 device
> passthrough   # Enter serial passthrough mode (press Enter to exit)
```

## Console Commands

### `info` - Get Device Information

Reads and displays CH32 device information:

```
> info
Getting CH32 device info...
=== CH32 Device Information ===
Chip:        CH32V303CBT6
Chip ID:     0x33 (Family: 0x17)
Flash:       128 KB
Bootloader:  v0209
UID:         CD-AB-DA-9D-4A-BC-F1-05
===============================
```

### `flash` - Flash Firmware

Programs the embedded firmware to CH32 and verifies:

```
> flash
Flashing CH32 firmware...
Embedded firmware size: 4096 bytes
Flashing 4096 bytes...
Progress: 1024 / 4096 bytes (25%)
Progress: 2048 / 4096 bytes (50%)
Progress: 3072 / 4096 bytes (75%)
Progress: 4096 / 4096 bytes (100%)
Flash complete!
Verifying 4096 bytes...
Progress: 4096 / 4096 bytes (100%)
Verify complete!
Programming successful!
```

### `verify` - Verify Flash

Verifies CH32 flash contents against embedded firmware:

```
> verify
Verifying CH32 flash...
Embedded firmware size: 4096 bytes
Verifying 4096 bytes...
Progress: 4096 / 4096 bytes (100%)
Verify successful!
```

### `rdp` - Read Protection

Manage flash read protection:

```
> rdp lock      # Enable protection (requires unlock to read/write)
> rdp unlock    # Disable protection (WARNING: erases all flash!)
```

### `reset` - Reset CH32 Device

Pulse the RST pin to reset the CH32 microcontroller:

```
> reset
Resetting CH32 device...
CH32 reset complete
```

This is useful to restart the CH32 after programming or to exit a crashed state.

### `passthrough` - Serial Passthrough Mode

Enter serial passthrough mode to monitor CH32 UART output in real-time:

```
> passthrough
Entering serial passthrough mode...
Press ENTER to exit passthrough mode

Hello from CH32V303!
Firmware version: 1.0.0
System initialized
```

Features:
- Monitor CH32 serial output in real-time
- Debug CH32 firmware during development
- Verify firmware is running correctly
- Press ENTER key to exit and return to ESP32 console

**Note**: ISP commands cannot be used while in passthrough mode. Exit passthrough first before running other commands.

## Embedding Your CH32 Firmware

The current `main/ch32_firmware.bin` is a 1KB placeholder. To program your own firmware:

### 1. Build Your CH32 Firmware

Build your CH32V30x project and locate the `.bin` file.

### 2. Replace Embedded Firmware

```bash
cp /path/to/your/firmware.bin main/ch32_firmware.bin
```

### 3. Rebuild ESP32 Project

```bash
idf.py build
idf.py flash
```

### 4. Program CH32

Use the `flash` command in the console to program your firmware to the CH32.

## Build Options

Requires ESP-IDF 5.5.1 installed:

```bash
# First time setup
idf.py set-target esp32

# Build
idf.py build

# Flash and monitor
idf.py flash monitor

# Clean build
idf.py fullclean
```

## Project Structure

```
ch32_loader/
├── main/                       # Development workspace (fast iteration)
│   ├── main.c                  # Main entry with console commands
│   ├── ch32_firmware.bin       # Embedded CH32 firmware
│   ├── CMakeLists.txt
│   └── idf_component.yml
├── components/
│   └── wch_isp/               # WCH ISP Component (publishable)
│       ├── examples/          # Component examples (ESP-IDF registry)
│       │   └── ch32_loader/   # Standalone example project
│       │       ├── main/      # Example application code
│       │       ├── CMakeLists.txt
│       │       ├── README.md
│       │       └── sdkconfig.defaults
│       ├── include/
│       │   └── wch_isp.h      # Public API
│       ├── src/
│       │   ├── wch_isp.c      # Main implementation
│       │   ├── wch_isp_protocol.c  # ISP protocol
│       │   └── wch_isp_flash.c     # Flash operations
│       ├── idf_component.yml  # Component manifest
│       ├── README.md          # Component documentation
│       └── LICENSE
├── ch32_code/                 # Example CH32 firmware source
├── build/                     # Build output (gitignored)
├── CMakeLists.txt
├── sdkconfig
└── README.md
```

### Dual Structure Explained

This project uses a **dual structure** to optimize both development and distribution:

1. **Root `main/` directory**: Development workspace for fast iteration
   - Quick builds and testing
   - Used during component development
   - Not published to component registry

2. **`components/wch_isp/examples/` directory**: Distribution examples
   - Complies with ESP-IDF Component Manager guidelines
   - Each example is a standalone project
   - Published with the component to the registry
   - Users can download and run independently

## Technical Details

### ISP Protocol

- **Communication**: UART 115200 8N1
- **Packet Format**: `[0x57 0xAB | Cmd | Len(2B) | Data | Checksum]`
- **Response Format**: `[0x55 0xAA | Cmd | 0x00 | Len(2B) | Data | Checksum]`
- **Encryption**: XOR encryption using device UID
- **Max Chunk Size**: 56 bytes per write/verify operation

### Commands Supported

- `0xA1` IDENTIFY - Authenticate and get chip info
- `0xA2` END - Exit ISP mode and reset
- `0xA3` ISP_KEY - Calculate XOR encryption key
- `0xA4` ERASE - Erase flash sectors
- `0xA5` PROGRAM - Write to flash (XOR encrypted)
- `0xA6` VERIFY - Verify flash (XOR encrypted)
- `0xA7` READ_CONFIG - Read option bytes, bootloader version, UID
- `0xA8` WRITE_CONFIG - Write option bytes

## Troubleshooting

### Build Issues

**Problem**: `Failed to initialize WCH ISP`
**Solution**: Check UART port configuration and GPIO pins

### Hardware Issues

**Problem**: `Failed to enter bootloader`
**Solution**:
- Verify RST and BOOT0 connections
- Check power supply to CH32
- Ensure correct GPIO pin assignments

**Problem**: `Verify failed`
**Solution**:
- Check UART RX/TX connections (ensure they're crossed)
- Reduce UART baud rate if experiencing communication errors
- Verify CH32 is not running firmware that interferes with UART

### Communication Issues

**Problem**: No output from CH32 in passthrough mode
**Solution**:
- Run the `passthrough` command to enter passthrough mode
- Verify CH32 firmware is sending data via UART
- Check UART baud rate matches (115200)
- Ensure CH32 UART TX is connected to ESP32 RX pin
- If using `reset` command, run `passthrough` immediately after to capture boot messages

## Development Workflow

### Quick Development Cycle

For rapid development and testing, work in the root `main/` directory:

```bash
# Make changes to main/main.c or component code
vim main/main.c

# Build and test immediately
idf.py build flash monitor

# Iterate quickly
```

### Publishing Updates

When you're ready to publish component updates with example changes:

1. **Test your changes** in the root `main/` workspace
2. **Copy updated files** to the example directory:
   ```bash
   cp main/main.c components/wch_isp/examples/ch32_loader/main/
   cp main/ch32_firmware.bin components/wch_isp/examples/ch32_loader/main/
   ```
3. **Verify the example builds standalone**:
   ```bash
   cd components/wch_isp/examples/ch32_loader
   idf.py build
   cd ../../../..
   ```
4. **Update version** in `components/wch_isp/idf_component.yml`
5. **Commit and publish** to the ESP-IDF Component Registry

### Testing the Example

To test the example as users would download it:

```bash
cd components/wch_isp/examples/ch32_loader
idf.py build
idf.py -p PORT flash monitor
```

The example uses `override_path` in its manifest to reference the local component during development.

## Contributing

Contributions are welcome! This project aims to provide a reliable and easy-to-use WCH ISP implementation for ESP-IDF.

### How to Contribute

1. **Fork the repository** on GitHub
2. **Create a feature branch**: `git checkout -b feature/my-new-feature`
3. **Make your changes** following the guidelines below
4. **Test thoroughly**:
   - Build and test in root `main/` workspace
   - Copy changes to example and verify it builds standalone
   - Test on actual hardware if possible
5. **Commit your changes**: `git commit -am 'Add some feature'`
6. **Push to the branch**: `git push origin feature/my-new-feature`
7. **Submit a Pull Request** with a clear description of your changes

### Contribution Guidelines

- **Code Style**: Follow ESP-IDF coding conventions
- **Documentation**: Update READMEs and comments for new features
- **Testing**: Ensure changes work on real CH32V30x hardware
- **Compatibility**: Maintain backward compatibility when possible
- **Examples**: Update examples if API changes affect them

### Areas for Contribution

We welcome contributions in these areas:

- **Additional CH32 families**: Support for CH32V00x, CH32V20x, CH32X035
- **Performance improvements**: Faster flashing, better error handling
- **Documentation**: Tutorials, diagrams, troubleshooting guides
- **Examples**: Additional use cases (OTA updates, factory programming, etc.)
- **Testing**: Unit tests, CI/CD integration, hardware test reports
- **Bug fixes**: Report and fix issues

### Reporting Issues

Found a bug or have a feature request?

1. **Search existing issues** to avoid duplicates
2. **Create a new issue** with:
   - Clear description of the problem or feature
   - Steps to reproduce (for bugs)
   - Expected vs actual behavior
   - Hardware setup details (ESP32 variant, CH32 variant)
   - Relevant logs or error messages

### Code of Conduct

- Be respectful and constructive
- Focus on technical merit
- Help newcomers learn and contribute
- Provide constructive feedback in reviews

## References

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [CH32V30x Datasheet](https://www.wch-ic.com/products/CH32V307.html)
- [WCH ISP Protocol Reference](https://github.com/COKPOWEHEU/wch-isp)
- [CH32 Bootloader Documentation](https://github.com/basilhussain/ch32v003-bootloader-docs)

## License

This project is licensed under the **GNU General Public License v2.0** - see the [LICENSE](LICENSE) file for details.

This is the same license as the original [wch-isp](https://github.com/COKPOWEHEU/wch-isp) project upon which this implementation is based.

## Credits

- WCH ISP protocol implementation based on [COKPOWEHEU/wch-isp](https://github.com/COKPOWEHEU/wch-isp)
- ESP-IDF framework by Espressif Systems
