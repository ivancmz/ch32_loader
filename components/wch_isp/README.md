# WCH ISP Component

ESP-IDF component for programming CH32V30x RISC-V microcontrollers via UART. Implements the WCH ISP (In-System Programming) protocol for firmware updates over serial connection.

## Features

- ✅ Full CH32V30x family support (CH32V303/V305/V307)
- ✅ UART-based ISP protocol implementation
- ✅ Firmware flashing with XOR encryption
- ✅ Automatic device detection and identification
- ✅ Flash verification
- ✅ Read protection management
- ✅ Bootloader entry/exit control

## Installation

### Using ESP-IDF Component Manager

Add to your project's `idf_component.yml`:

```yaml
dependencies:
  ivancmz/wch_isp: "^1.0.0"
```

### Manual Installation

Copy the `wch_isp` folder to your project's `components/` directory.

## Examples

See the [examples](examples/) directory for complete usage examples:

- **[ch32_loader](examples/ch32_loader/)** - Full-featured CH32V30x programmer with interactive console interface. Demonstrates device identification, firmware flashing, verification, read protection, and serial passthrough.

Each example can be built as a standalone ESP-IDF project.

## Quick Start

### 1. Include Header

```c
#include "wch_isp.h"
```

### 2. Configure Hardware

```c
wch_isp_config_t config = {
    .uart_port = UART_NUM_2,
    .uart_tx_pin = GPIO_NUM_17,
    .uart_rx_pin = GPIO_NUM_16,
    .uart_baudrate = 115200,
    .rst_pin = GPIO_NUM_4,
    .boot0_pin = GPIO_NUM_5
};
```

### 3. Initialize and Program

```c
// Initialize ISP
wch_isp_handle_t handle;
wch_isp_err_t err = wch_isp_init(&config, &handle);

// Get device information
wch_isp_device_info_t info;
err = wch_isp_get_device_info(handle, &info);
printf("Chip: %s\n", info.device_name);
printf("Chip ID: 0x%02X (Family: 0x%02X)\n", info.chip_id, info.device_type);
printf("Flash: %lu KB\n", info.flash_size / 1024);
printf("UID: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n",
       info.uid[0], info.uid[1], info.uid[2], info.uid[3],
       info.uid[4], info.uid[5], info.uid[6], info.uid[7]);

// Flash firmware
err = wch_isp_flash_firmware(handle, firmware_data, firmware_size, 0x08000000);

// Verify firmware
err = wch_isp_verify_firmware(handle, firmware_data, firmware_size, 0x08000000);

// Clean up
wch_isp_deinit(handle);
```

## API Reference

### Initialization

- `wch_isp_init()` - Initialize ISP with configuration
- `wch_isp_deinit()` - Clean up and release resources

### Device Information

- `wch_isp_get_device_info()` - Read device type, UID, bootloader version

### Flash Operations

- `wch_isp_flash_firmware()` - Erase and program firmware
- `wch_isp_verify_firmware()` - Verify flash contents
- `wch_isp_erase_flash()` - Erase flash sectors

### Protection

- `wch_isp_read_option_bytes()` - Read option bytes
- `wch_isp_write_option_bytes()` - Write option bytes
- `wch_isp_unlock_device()` - Disable read protection (erases flash!)

## Hardware Connections

```
ESP32          CH32V30x
─────────────────────────
TX Pin   ───►  UART RX
RX Pin   ◄───  UART TX
RST Pin  ───►  RST
BOOT0    ───►  BOOT0
GND      ───   GND
```

## Supported Devices

All CH32V30x family variants:

| Chip Variant | Chip ID | Flash Size |
|--------------|---------|------------|
| CH32V303VCT6 | 0x30    | 256KB      |
| CH32V303RCT6 | 0x31    | 256KB      |
| CH32V303RBT6 | 0x32    | 128KB      |
| CH32V303CBT6 | 0x33    | 128KB      |
| CH32V305RBT6 | 0x50    | 128KB      |
| CH32V307VCT6 | 0x70    | 256KB      |
| CH32V307RCT6 | 0x71    | 256KB      |
| CH32V307WCU6 | 0x73    | 256KB      |

## Technical Details

- **Protocol**: WCH ISP over UART (115200 8N1)
- **Encryption**: XOR encryption using device UID
- **Max Write Chunk**: 56 bytes
- **Flash Address**: 0x08000000 (default)
- **Bootloader Entry**: BOOT0=1, pulse RST

## Error Handling

All functions return `wch_isp_err_t`:

- `WCH_ISP_OK` - Success
- `WCH_ISP_ERR_INVALID_ARG` - Invalid argument
- `WCH_ISP_ERR_NO_MEM` - Memory allocation failed
- `WCH_ISP_ERR_TIMEOUT` - Communication timeout
- `WCH_ISP_ERR_PROTOCOL` - Protocol error
- `WCH_ISP_ERR_VERIFY` - Verification failed

## License

GNU General Public License v2.0 - see [LICENSE](LICENSE) file.

Based on [wch-isp](https://github.com/COKPOWEHEU/wch-isp) by COKPOWEHEU.

## References

- [Project Repository](https://github.com/ivancmz/ch32_loader)
- [WCH ISP Protocol](https://github.com/COKPOWEHEU/wch-isp)
- [CH32V30x Datasheet](https://www.wch-ic.com/products/CH32V307.html)
