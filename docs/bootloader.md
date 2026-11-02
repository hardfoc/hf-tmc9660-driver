---
layout: default
title: "🚀 Bootloader Initialization"
description: "Critical bootloader setup guide for TMC9660"
nav_order: 8
parent: "📚 Documentation"
permalink: /docs/special_feature_bootloader/
---

# Bootloader Initialization

The TMC9660 bootloader initialization is **CRITICAL** and must be completed successfully before any motor control operations. This guide covers the essential bootloader setup.

## Why Bootloader Initialization is Required

The TMC9660 operates in two modes:
- **Bootloader Mode**: Configuration and firmware update
- **Parameter Mode**: Motor control operation

**Motor control will NOT work** until the bootloader is properly initialized and the device transitions to Parameter Mode.

## Critical Configuration

```cpp
tmc9660::BootloaderConfig cfg{};

// ⚠️ CRITICAL: Must set boot mode to Parameter
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;

// Start motor control after configuration
cfg.boot.start_motor_control = true;

// SPI configuration
cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;

// UART configuration (if using UART)
cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
cfg.uart.tx_pin = tmc9660::bootcfg::UartTxPin::GPIO6;
cfg.uart.rx_pin = tmc9660::bootcfg::UartRxPin::GPIO7;

// Clock configuration
cfg.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;

// Initialize bootloader
auto result = driver.bootloaderInit(&cfg);
if (result != tmc9660::TMC9660::BootloaderInitResult::Success) {
    // Handle failure - motor control will not work!
    return -1;
}
```

## Command Ordering (CRITICAL)

**When `START_MOTOR_CONTROL = 1` is written, the bootloader EXITS IMMEDIATELY!**

### ❌ WRONG: Writing START_MOTOR_CONTROL early

```cpp
// WRONG! Bootloader exits after this write
cfg.boot.start_motor_control = true;  // Written first
// ... other configs ...
// ❌ These configs fail because bootloader already exited!
```

### ✅ CORRECT: Writing START_MOTOR_CONTROL last

The `bootloaderInit()` method automatically writes `START_MOTOR_CONTROL = 1` **LAST**, after all other configurations. This is the correct order.

## Initialization Sequence

1. **Hardware Reset** (if `performReset=true`)
   - Assert RST pin for 100ms
   - Release RST pin
   - Wait 100ms for stabilization

2. **Mode Detection**
   - Check if device is in bootloader or parameter mode
   - If in parameter mode and config provided, reinitialize

3. **Configuration**
   - Write all bootloader settings
   - **LAST**: Write `START_MOTOR_CONTROL = 1` (if enabled)

4. **Motor Control Startup** (if `start_motor_control=true`)
   - Consume SESSION_START
   - Verify TMCL communication

## OTP vs Runtime Configuration

### OTP Configuration (Production)

When `START_MOTOR_CONTROL = 1` is burned into OTP:
- ✅ Autonomous operation (no host needed)
- ✅ Fastest startup
- ❌ Cannot reconfigure without hardware reset
- ❌ Bootloader commands not accepted after power-up

### Runtime Configuration (Development)

When `START_MOTOR_CONTROL = 1` is written at runtime:
- ✅ Flexible configuration during development
- ✅ Can test different settings
- ⚠️ Requires host to send configuration
- ⚠️ Longer startup time

## Common Issues

### Bootloader Initialization Fails

1. **Check Hardware Reset**
   - Verify RST pin toggling
   - Wait at least 100ms after reset

2. **Verify Communication**
   - Check SPI/UART wiring
   - Verify CS pin (SPI) or TX/RX pins (UART)
   - Confirm clock speed ≤ 4 MHz (SPI)

3. **Check Configuration**
   - Must set `boot_mode = Parameter`
   - Verify all required settings are provided

### No Reply from Bootloader

1. **Check Pin Connections**
   - Verify TX/RX pins not swapped (UART)
   - Verify MOSI/MISO not swapped (SPI)

2. **Verify Protocol**
   - Check CRC/checksum calculation
   - Confirm frame format

3. **Check Timing**
   - Add delays between commands
   - Respect SPI delayed reply timing

## Best Practices

1. **Always Initialize Bootloader First**
   ```cpp
   // This MUST be done before any motor control
   auto result = driver.bootloaderInit(&cfg);
   ```

2. **Use Complete Configuration**
   ```cpp
   // Set all required fields
   cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
   cfg.boot.start_motor_control = true;
   ```

3. **Handle Errors**
   ```cpp
   if (result != tmc9660::TMC9660::BootloaderInitResult::Success) {
       // Motor control will not work without successful initialization
       return false;
   }
   ```

4. **Verify Communication After Init**
   ```cpp
   // Test that TMCL communication works
   auto version = driver.bootloaderGetVersion();
   if (!version.has_value()) {
       // Communication failed
   }
   ```

## Bootloader Commands Quick Reference

### Command Codes

| Code | Command | Description |
|------|---------|-------------|
| 0x00 | GET_INFO | Get bootloader info |
| 0x08 | GET_BANK | Get current bank |
| 0x09 | SET_BANK | Select memory bank |
| 0x0A | GET_ADDRESS | Get current address |
| 0x0B | SET_ADDRESS | Set memory address |
| 0x0C | READ_32 | Read 32-bit data |
| 0x0D | READ_32_INC | Read 32-bit + increment |
| 0x0E | READ_16 | Read 16-bit data |
| 0x0F | READ_16_INC | Read 16-bit + increment |
| 0x10 | READ_8 | Read 8-bit data |
| 0x11 | READ_8_INC | Read 8-bit + increment |
| 0x12 | WRITE_32 | Write 32-bit value |
| 0x13 | WRITE_32_INC | Write 32-bit + increment |
| 0x14 | WRITE_16 | Write 16-bit value |
| 0x15 | WRITE_16_INC | Write 16-bit + increment |
| 0x16 | WRITE_8 | Write 8-bit value |
| 0x17 | WRITE_8_INC | Write 8-bit + increment |
| 0x1D | NO_OP | No operation (get reply) |
| 0x1E | OTP_LOAD | Load OTP page |
| 0x1F | OTP_BURN | Burn OTP page |

### Status Codes

| Code | Name | Description |
|------|------|-------------|
| 0 | OK | Command executed successfully |
| 1 | CMD_NOT_FOUND | Invalid command number |
| 3 | INVALID_ADDR | Invalid memory address |
| 4 | INVALID_VALUE | Invalid request value |
| 14 | INVALID_BANK | Invalid memory bank |
| 15 | BUSY | Bootloader busy (SPI only) |
| 17 | MEM_UNCONFIGURED | External memory not configured |
| 18 | OTP_ERROR | OTP command failed |
| 19 | SESSION_START | First SPI datagram after power-on (SPI only) |
| 20 | CMD_NOT_AVAILABLE | Command not available |
| 21 | BOOTLOADER_RESUMED | Returned to bootloader from motor control (SPI only) |

### Memory Banks

| Bank | Name | Description |
|------|------|-------------|
| 0 | RAM | Internal SRAM |
| 1 | OTP | One-time programmable memory |
| 2 | SPI Flash | External SPI flash memory |
| 3 | I2C EEPROM | External I2C EEPROM |
| 4 | Reserved | Reserved for future use |
| 5 | CONFIG | Configuration memory bank |

## Next Steps

- **[Troubleshooting](troubleshooting.md)** - Common issues and solutions
- **[Platform Integration](platform_integration.md)** - Communication interface setup
- **[Quick Start](quickstart.md)** - Minimal working example

