# CH32 Loader Example

This example demonstrates how to use the `wch_isp` component to program CH32V30x RISC-V microcontrollers from an ESP32 via UART.

## Features

- Device identification and information display
- Firmware flashing with embedded binary
- Flash verification
- Read protection lock/unlock
- Serial passthrough for CH32 debugging
- Interactive console interface

## Hardware Connections

```
ESP32          CH32V30x
─────────────────────────
GPIO19   ───►  UART RX
GPIO18   ◄───  UART TX
GPIO5    ───►  RST
GPIO4    ───►  BOOT0
GND      ───   GND
```

## Pin Configuration

Default pins are defined in `main/main.c`:

```c
#define CH32_UART_PORT      UART_NUM_2
#define CH32_UART_TX_PIN    GPIO_NUM_19
#define CH32_UART_RX_PIN    GPIO_NUM_18
#define CH32_RST_PIN        GPIO_NUM_5
#define CH32_BOOT0_PIN      GPIO_NUM_4
```

Modify these to match your hardware setup.

## How to Use

### Build and Flash

```bash
idf.py build
idf.py -p PORT flash monitor
```

Replace `PORT` with your serial port (e.g., `COM3` on Windows, `/dev/ttyUSB0` on Linux).

### Console Commands

Once running, try these commands:

```
> info          # Get CH32 device information
> flash         # Flash embedded firmware to CH32
> verify        # Verify CH32 flash contents
> rdp lock      # Enable read protection
> rdp unlock    # Disable read protection (erases flash!)
> reset         # Reset CH32 device
> passthrough   # Enter serial passthrough mode (press Enter to exit)
```

## Embedding Your Firmware

Replace `main/ch32_firmware.bin` with your CH32 binary, then rebuild:

```bash
cp /path/to/your/firmware.bin main/ch32_firmware.bin
idf.py build flash
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

## Troubleshooting

### Failed to Enter Bootloader

**Symptoms**: `Failed to enter bootloader` error

**Solutions**:
- Verify RST and BOOT0 connections
- Check power supply to CH32 (3.3V)
- Ensure correct GPIO pin assignments
- Try adding pull-down resistor on BOOT0 (10kΩ to GND)

### Verify Failed

**Symptoms**: `Verify failed` error

**Solutions**:
- Check UART RX/TX connections (ensure they're crossed: ESP32 TX → CH32 RX)
- Verify CH32 is not running firmware that interferes with UART
- Reduce UART baud rate if experiencing communication errors
- Check for loose connections or noise on UART lines

### No Output in Passthrough Mode

**Symptoms**: No data visible when using `passthrough` command

**Solutions**:
- Verify CH32 firmware is sending data via UART
- Check UART baud rate matches (115200)
- Ensure CH32 UART TX is connected to ESP32 RX pin (GPIO18)
- Use `reset` command followed by `passthrough` to capture boot messages

## Example Output

```
=== CH32 Device Information ===
Chip:        CH32V303CBT6
Chip ID:     0x33 (Family: 0x17)
Flash:       128 KB
Bootloader:  v0209
UID:         CD-AB-DA-9D-4A-BC-F1-05
===============================
```

## See Also

- [wch_isp Component Documentation](../../README.md)
- [WCH ISP Protocol Reference](https://github.com/COKPOWEHEU/wch-isp)
- [CH32V30x Datasheet](https://www.wch-ic.com/products/CH32V307.html)
