# ESP32-C6 TMC9660 Comprehensive Test Suite

This directory contains comprehensive test suites for the TMC9660 motor driver using the ESP32-C6 DevKit-M-1. The
tests support both the **TMC9660-3PH-EVKIT** (BLDC motor board) and **TMC9660-STP-EVKIT** (Stepper motor board).

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [Pin Connections](#pin-connections)
- [Hardware Setup](#hardware-setup)
  - [BLDC Motor Board Setup](#bldc-motor-board-setup-tmc9660-3ph-evkit)
  - [Stepper Motor Board Setup](#stepper-motor-board-setup-tmc9660-stp-evkit)
- [Building the Tests](#building-the-tests)
- [Running the Tests](#running-the-tests)
- [Test Suites](#test-suites)
- [Troubleshooting](#troubleshooting)

---

## Hardware Overview

### ESP32-C6 DevKit-M-1

The ESP32-C6 DevKit-M-1 serves as the host controller for communicating with the TMC9660 motor driver via SPI or UART.

```
┌─────────────────────────────────────────────────┐
│        ESP32-C6 DevKit-M-1                      │
│                                                 │
│  ┌──────────────────────────────────────────┐   │
│  │        ESP32-C6 Microcontroller          │   │
│  │                                          │   │
│  │  GPIO Pins:                              │   │
│  │  • SPI: MOSI, MISO, SCLK, CS             │   │
│  │  • UART: TX, RX                          │   │
│  │  • Control: RST, DRV_EN, FAULTN, WAKE    │   │
│  └──────────────────────────────────────────┘   │
│                                                 │
│  USB-C Connector                                │
│  (Power + Serial Communication)                 │
└─────────────────────────────────────────────────┘
```

### TMC9660-3PH-EVKIT (BLDC Motor Board)

Evaluation kit for 3-phase BLDC motor control with Hall sensors and ABN encoders.

```
┌─────────────────────────────────────────────────┐
│      TMC9660-3PH-EVKIT                          │
│                                                 │
│  ┌──────────────────────────────────────────┐   │
│  │        TMC9660 IC                        │   │
│  │                                          │   │
│  │  Features:                               │   │
│  │  • 3-phase BLDC motor driver             │   │
│  │  • Hall sensor inputs (U, V, W)          │   │
│  │  • Dual ABN encoder inputs               │   │
│  │  • SPI Flash memory                      │   │
│  │  • Brake chopper & mechanical brake      │   │
│  └──────────────────────────────────────────┘   │
│                                                 │
│  Motor Connectors:                              │
│  • U, V, W (3-phase motor)                      │
│  • Hall sensors (GPIO2, 3, 4)                   │
│  • ABN encoder 1 (GPIO8, 13, 14)                │
│  • ABN encoder 2 (GPIO15, 16)                   │
└─────────────────────────────────────────────────┘
```

### TMC9660-STP-EVKIT (Stepper Motor Board)

Evaluation kit for stepper motor control with reference switches and ABN encoders.

```
┌─────────────────────────────────────────────────┐
│      TMC9660-STP-EVKIT                          │
│                                                 │
│  ┌──────────────────────────────────────────┐   │
│  │        TMC9660 IC                        │   │
│  │                                          │   │
│  │  Features:                               │   │
│  │  • Stepper motor driver                  │   │
│  │  • Reference switches (Left, Right, Home)│   │
│  │  • Dual ABN encoder inputs               │   │
│  │  • SPI Flash memory                      │   │
│  └──────────────────────────────────────────┘   │
│                                                 │
│  Motor Connectors:                              │
│  • Step/Direction interface                     │
│  • Reference switches (GPIO2, 3, 4)             │
│  • ABN encoder 1 (GPIO8, 13, 14)                │
│  • ABN encoder 2 (GPIO15, 16)                   │
└─────────────────────────────────────────────────┘
```

---

## Pin Connections

### SPI Communication Interface

```
ESP32-C6 DevKit-M-1          TMC9660-3PH-EVKIT / STP-EVKIT
──────────────────          ───────────────────────────────
GPIO7  (MOSI)      ───────> SPI MOSI
GPIO2  (MISO)      <─────── SPI MISO
GPIO6  (SCLK)      ───────> SPI SCLK
GPIO18 (CS)        ───────> SPI CS

Control Pins:
──────────────────          ───────────────────────────────
GPIO22 (RST)       ───────> RST     (Active HIGH)
GPIO20 (DRV_EN)    ───────> DRV_EN  (Active HIGH)
GPIO19 (FAULTN)    <─────── FAULTN  (Active LOW, open-drain)
GPIO21 (WAKE)      ───────> WAKE    (Active LOW)
```

### UART Communication Interface (Alternative)

```
ESP32-C6 DevKit-M-1          TMC9660-3PH-EVKIT / STP-EVKIT
──────────────────          ───────────────────────────────
GPIO5  (TX)        ───────> UART RX (GPIO6 on TMC9660)
GPIO4  (RX)        <─────── UART TX (GPIO7 on TMC9660)
```

### Power and Common

```
ESP32-C6 DevKit-M-1          TMC9660-3PH-EVKIT / STP-EVKIT
──────────────────          ───────────────────────────────
GND                ───────> GND (Common ground)
3.3V / 5V          ───────> VCC (if needed for level shifting)
```

---

## Hardware Setup

### BLDC Motor Board Setup (TMC9660-3PH-EVKIT)

#### Connection Diagram

```
┌─────────────────────┐         ┌─────────────────────┐
│  ESP32-C6 DevKit    │         │ TMC9660-3PH-EVKIT   │
│                     │         │                     │
│  GPIO7 (MOSI)  ─────┼─────────┼─> SPI MOSI          │
│  GPIO2 (MISO)  <────┼─────────┼─< SPI MISO          │
│  GPIO6 (SCLK)  ─────┼─────────┼─> SPI SCLK          │
│  GPIO18(CS)    ─────┼─────────┼─> SPI CS            │
│                     │         │                     │
│  GPIO22(RST)   ─────┼─────────┼─> RST               │
│  GPIO20(DRV_EN) ────┼─────────┼─> DRV_EN            │
│  GPIO19(FAULTN)<────┼─────────┼─< FAULTN            │
│  GPIO21(WAKE)  ─────┼─────────┼─> WAKE              │
│                     │         │                     │
│  GND           ─────┼─────────┼─> GND               │
│                     │         │                     │
│                     │         │  Motor Connections: │
│                     │         │  • U, V, W (3-phase)│
│                     │         │  • Hall (2,3,4)     │
│                     │         │  • Encoder1(8,13,14)│
│                     │         │  • Encoder2 (15,16) │
└─────────────────────┘         └─────────────────────┘
```

#### Setup Steps

1. **Power Off**: Ensure both boards are powered off before making connections.

2. **Connect SPI Interface**:
   - Connect ESP32 GPIO7 → TMC9660 SPI MOSI
   - Connect ESP32 GPIO2 → TMC9660 SPI MISO
   - Connect ESP32 GPIO6 → TMC9660 SPI SCLK
   - Connect ESP32 GPIO18 → TMC9660 SPI CS

3. **Connect Control Pins**:
   - Connect ESP32 GPIO22 → TMC9660 RST
   - Connect ESP32 GPIO20 → TMC9660 DRV_EN
   - Connect ESP32 GPIO19 → TMC9660 FAULTN
   - Connect ESP32 GPIO21 → TMC9660 WAKE

4. **Connect Ground**: Connect GND between ESP32 and TMC9660 boards.

5. **Connect Motor** (if testing with actual motor):
   - Connect 3-phase BLDC motor to U, V, W terminals
   - Connect Hall sensors to Hall connector (if using Hall feedback)
   - Connect ABN encoder to Encoder connector (if using encoder feedback)

6. **Power On**:
   - Power ESP32 via USB-C
   - Power TMC9660 board via its power supply (typically 12V-48V)

### Stepper Motor Board Setup (TMC9660-STP-EVKIT)

#### Connection Diagram

```
┌─────────────────────┐         ┌─────────────────────┐
│  ESP32-C6 DevKit    │         │ TMC9660-STP-EVKIT   │
│                     │         │                     │
│  GPIO7 (MOSI)  ─────┼─────────┼─> SPI MOSI          │
│  GPIO2 (MISO)  <────┼─────────┼─< SPI MISO          │
│  GPIO6 (SCLK)  ─────┼─────────┼─> SPI SCLK          │
│  GPIO18(CS)    ─────┼─────────┼─> SPI CS            │
│                     │         │                     │
│  GPIO22(RST)   ─────┼─────────┼─> RST               │
│  GPIO20(DRV_EN) ────┼─────────┼─> DRV_EN            │
│  GPIO19(FAULTN)<────┼─────────┼─< FAULTN            │
│  GPIO21(WAKE)  ─────┼─────────┼─> WAKE              │
│                     │         │                     │
│  GND           ─────┼─────────┼─> GND               │
│                     │         │                     │
│                     │         │  Motor Connections: │
│                     │         │  • Step/Dir         │
│                     │         │  • Ref Switches     │
│                     │         │    (Left,Right,Home)│
│                     │         │  • Encoder1 (8,13,14)│
│                     │         │  • Encoder2 (15,16) │
└─────────────────────┘         └─────────────────────┘
```

#### Setup Steps

1. **Power Off**: Ensure both boards are powered off before making connections.

2. **Connect SPI Interface**: (Same as BLDC board)
   - Connect ESP32 GPIO7 → TMC9660 SPI MOSI
   - Connect ESP32 GPIO2 → TMC9660 SPI MISO
   - Connect ESP32 GPIO6 → TMC9660 SPI SCLK
   - Connect ESP32 GPIO18 → TMC9660 SPI CS

3. **Connect Control Pins**: (Same as BLDC board)
   - Connect ESP32 GPIO22 → TMC9660 RST
   - Connect ESP32 GPIO20 → TMC9660 DRV_EN
   - Connect ESP32 GPIO19 → TMC9660 FAULTN
   - Connect ESP32 GPIO21 → TMC9660 WAKE

4. **Connect Ground**: Connect GND between ESP32 and TMC9660 boards.

5. **Connect Motor** (if testing with actual motor):
   - Connect stepper motor to motor connector
   - Connect reference switches to REF SWITCHES connector (Left=GPIO2, Right=GPIO3, Home=GPIO4)
   - Connect ABN encoder to Encoder connector (if using encoder feedback)

6. **Power On**:
   - Power ESP32 via USB-C
   - Power TMC9660 board via its power supply

---

## Building the Tests

### Prerequisites

- **ESP-IDF**: Version 5.5 (release/v5.5) or later
- **Python**: Version 3.8 or later (for ESP-IDF tools)
- **CMake**: Version 3.16 or later
- **Git**: For cloning the repository
- **Bash**: For running the build scripts (Linux/macOS/Git Bash on Windows)

### Installation Steps

1. **Install ESP-IDF** (if not already installed):

   ```bash
   # Clone ESP-IDF
   git clone --recursive https://github.com/espressif/esp-idf.git
   cd esp-idf
   
   # Checkout release version 5.5
   git checkout release/v5.5
   git submodule update --init --recursive
   
   # Install ESP-IDF (Linux/macOS)
   ./install.sh esp32c6
   
   # Set up environment (add to ~/.bashrc or ~/.zshrc for persistence)
   . ./export.sh
   ```

2. **Clone the Repository**:

   ```bash
   git clone <repository-url>
   cd hf-tmc9660-driver
   ```

3. **Navigate to ESP32 Examples**:

   ```bash
   cd examples/esp32
   ```

4. **Setup Repository** (First time only):

   ```bash
   # Make scripts executable and setup the build environment
   chmod +x scripts/*.sh
   ./scripts/setup_repo.sh
   ```

### Available Test Applications

The test suites use a centralized build system with scripts. Available applications:

| **Application Name** | **Description** | **Board** |
|----------------------|----------------|-----------|
| `bldc_comprehensive_test` | Comprehensive BLDC motor testing with Hall sensors, ABN encoders, and velocity control | TMC9660-3PH-EVKIT |
| `stepper_comprehensive_test` | Comprehensive Stepper motor testing with FOC and step/dir control | TMC9660-STP-EVKIT |
| `dc_comprehensive_test` | Comprehensive DC motor testing with current and velocity control | TMC9660-3PH-EVKIT |
| `telemetry_comprehensive_test` | Comprehensive telemetry monitoring for temperature, current, voltage, and status | TMC9660-3PH-EVKIT |
| `bootloader_comprehensive_test` | Comprehensive bootloader testing for initialization and configuration | TMC9660-3PH-EVKIT |

### List Available Applications

```bash
# List all available applications
./scripts/build_app.sh list
```

Output:
```
Available Applications:
  • bldc_comprehensive_test         - Comprehensive BLDC motor testing suite
  • stepper_comprehensive_test      - Comprehensive Stepper motor testing suite
  • dc_comprehensive_test           - Comprehensive DC motor testing suite
  • telemetry_comprehensive_test    - Comprehensive telemetry monitoring test suite
  • bootloader_comprehensive_test   - Comprehensive bootloader testing suite
```

### Building Applications

#### Build Syntax

```bash
./scripts/build_app.sh <app_type> <build_type>
```

**Parameters:**
- `<app_type>`: Application name (e.g., `bldc_comprehensive_test`)
- `<build_type>`: Build configuration - `Debug` or `Release`

#### Build Examples

```bash
# Build BLDC comprehensive test (Release build)
./scripts/build_app.sh bldc_comprehensive_test Release

# Build Stepper comprehensive test (Debug build)
./scripts/build_app.sh stepper_comprehensive_test Debug

# Build DC motor comprehensive test (Release build)
./scripts/build_app.sh dc_comprehensive_test Release

# Build Telemetry comprehensive test (Debug build)
./scripts/build_app.sh telemetry_comprehensive_test Debug

# Build Bootloader comprehensive test (Release build)
./scripts/build_app.sh bootloader_comprehensive_test Release
```

#### Build Output

Build artifacts are located in:
```text
build-app-<app_type>-type-<build_type>-target-esp32c6-idf-<idf_version>/
```

Example:
```text
build-app-bldc_comprehensive_test-type-Release-target-esp32c6-idf-release_v5.5/
```

### Build Types

| **Build Type** | **Optimization** | **Debug Info** | **Use Case** |
|----------------|------------------|----------------|--------------|
| `Debug` | `-O0` (no optimization) | Full (`-g3`) | Development, debugging |
| `Release` | `-O2` (optimized) | Minimal (`-g0`) | Production, performance testing |

---

## Running the Tests

### Flashing and Monitoring

The build scripts provide convenient commands for flashing and monitoring your ESP32-C6.

#### Flash and Monitor (Auto-detect Port)

```bash
./scripts/flash_app.sh <app_type> <build_type>
```

This command will:
1. Auto-detect the ESP32 serial port
2. Flash the firmware
3. Start monitoring serial output

#### Flash and Monitor (Specify Port)

```bash
./scripts/flash_app.sh <app_type> <build_type> /dev/ttyUSB0    # Linux
./scripts/flash_app.sh <app_type> <build_type> /dev/cu.usbserial-*  # macOS
./scripts/flash_app.sh <app_type> <build_type> COM3            # Windows
```

#### Flash Only (No Monitor)

```bash
./scripts/flash_app.sh <app_type> <build_type> --no-monitor
```

#### Monitor Only (Reconnect)

```bash
./scripts/monitor_app.sh
./scripts/monitor_app.sh /dev/ttyUSB0    # Linux (specify port)
```

### Complete Workflow Example

```bash
# 1. Build the application
./scripts/build_app.sh bldc_comprehensive_test Release

# 2. Flash and monitor
./scripts/flash_app.sh bldc_comprehensive_test Release

# Output will show:
# ✅ Building app: bldc_comprehensive_test
# ✅ Build type: Release
# ✅ Flashing firmware...
# ✅ Starting monitor...
```

### Expected Output

After flashing and running, you should see output like:

```
╔══════════════════════════════════════════════════════════════════════════════╗
║                    ESP32-C6 BLDC COMPREHENSIVE TEST SUITE                    ║
║                         HardFOC TMC9660 Driver Tests                         ║
╚══════════════════════════════════════════════════════════════════════════════╝
Debug logging enabled for TMCL communication traces

╔══════════════════════════════════════════════════════════════════════════════╗
║                 BLDC TEST SECTION CONFIGURATION                                ║
╚══════════════════════════════════════════════════════════════════════════════╝

I (1234) BLDC_Test: Running core BLDC functionality tests...
I (5678) BLDC_Test: ✅ SPI bootloader initialization successful with EVKIT config
...
```

### Test Progression Indicator

The test suite uses GPIO14 as a visual test progression indicator:
- **LOW → HIGH → LOW**: Each transition indicates a completed test
- Monitor GPIO14 with an oscilloscope or logic analyzer for visual feedback

### Available Scripts

| **Script** | **Purpose** | **Usage** |
|------------|-------------|-----------|
| `build_app.sh` | Build specific application | `./scripts/build_app.sh <app> <build_type>` |
| `flash_app.sh` | Flash and monitor application | `./scripts/flash_app.sh <app> <build_type> [port]` |
| `monitor_app.sh` | Monitor serial output | `./scripts/monitor_app.sh [port]` |
| `setup_repo.sh` | Initial repository setup | `./scripts/setup_repo.sh` |
| `clean_app.sh` | Clean build artifacts | `./scripts/clean_app.sh <app> <build_type>` |

### Advanced Build Options

For advanced configuration, you can use ESP-IDF's menuconfig:

```bash
# Configure build options
./scripts/config_app.sh <app_type> <build_type>
```

This opens the ESP-IDF menuconfig where you can customize:
- **Component config → TMC9660 Driver**: Driver configuration options
- **Serial flasher config**: Serial port and flash settings
- **Compiler options**: Optimization and debug settings

---

## Test Suites

### BLDC Comprehensive Test (`BLDCComprehensiveTest.cpp`)

Tests for 3-phase BLDC motor control:

- ✅ Bootloader initialization and configuration
- ✅ Motor type configuration (BLDC with various pole pairs)
- ✅ Hall sensor configuration and testing
- ✅ ABN encoder configuration and testing
- ✅ FOC control loop configuration
- ✅ Velocity and current control
- ✅ Commutation modes (FOC_HALL, FOC_ABN, FOC_OPENLOOP)
- ✅ Telemetry monitoring
- ✅ Performance benchmarking
- ✅ Error handling and edge cases

**Board**: TMC9660-3PH-EVKIT

### Stepper Comprehensive Test (`StepperComprehensiveTest.cpp`)

Tests for stepper motor control:

- ✅ Bootloader initialization and configuration
- ✅ Motor type configuration (Stepper)
- ✅ Reference switch configuration (Left, Right, Home)
- ✅ ABN encoder configuration
- ✅ FOC position control
- ✅ Step/Direction control
- ✅ Microstepping configuration
- ✅ Position and velocity control
- ✅ Stall detection
- ✅ Telemetry monitoring

**Board**: TMC9660-STP-EVKIT

### DC Motor Comprehensive Test (`DCComprehensiveTest.cpp`)

Tests for DC motor control:

- ✅ Bootloader initialization and configuration
- ✅ Motor type configuration (DC motor)
- ✅ Current control and torque limiting
- ✅ Velocity control with encoder feedback
- ✅ Open-loop current mode
- ✅ H-bridge control
- ✅ Protection features
- ✅ Telemetry monitoring

**Board**: TMC9660-3PH-EVKIT (same board as BLDC)

### Telemetry Comprehensive Test (`TelemetryComprehensiveTest.cpp`)

Tests for telemetry and monitoring:

- ✅ Temperature monitoring
- ✅ Current monitoring
- ✅ Voltage monitoring
- ✅ Position and velocity feedback
- ✅ System status reporting
- ✅ Performance benchmarking
- ✅ Multi-device monitoring

**Board**: TMC9660-3PH-EVKIT

### Bootloader Comprehensive Test (`BootloaderComprehensiveTest.cpp`)

Tests for bootloader functionality:

- ✅ Bootloader initialization
- ✅ Mode detection (bootloader vs parameter)
- ✅ Configuration reading and writing
- ✅ Flash memory operations
- ✅ Reset and recovery procedures

**Board**: TMC9660-3PH-EVKIT or TMC9660-STP-EVKIT

---

## Test Configuration

Each test suite supports enabling/disabling specific test categories via compile-time flags at the top of each test file:

```cpp
// Enable/disable specific test categories
static constexpr bool ENABLE_CORE_TESTS = true;
static constexpr bool ENABLE_HALL_SENSOR_TESTS = true;
static constexpr bool ENABLE_ABN_ENCODER_TESTS = true;
// ... etc
```

Edit these flags to customize which tests run.

---

## Troubleshooting

### Common Issues

#### 1. **Communication Timeout**

**Symptoms**: Tests fail with communication timeout errors

**Solutions**:
- Verify all SPI/UART connections are secure
- Check that GPIO pins match the configuration
- Ensure proper ground connection between boards
- Check power supply voltage (TMC9660 may need 12V-48V)

#### 2. **Bootloader Not Detected**

**Symptoms**: Tests fail at bootloader initialization

**Solutions**:
- Verify RST pin is properly connected (GPIO22)
- Check that FAULTN pin is connected (GPIO19)
- Try power cycling both boards
- Verify SPI communication with a logic analyzer

#### 3. **Motor Not Starting**

**Symptoms**: Motor configuration succeeds but motor doesn't run

**Solutions**:
- Verify DRV_EN pin is set (GPIO20)
- Check motor connections (U, V, W for BLDC)
- Verify current limits are set appropriately
- Check that PWM frequency is configured before setting current limits
- Ensure motor feedback (Hall/encoder) is properly connected if using closed-loop control

#### 4. **FAULTN Asserted**

**Symptoms**: FAULTN pin is low (fault condition)

**Solutions**:
- Check TMC9660 datasheet for fault conditions
- Verify power supply voltage is within range
- Check for overcurrent conditions
- Verify motor connections are correct
- Check temperature (overtemperature protection)

#### 5. **Build Errors**

**Symptoms**: `./scripts/build_app.sh` fails

**Solutions**:
- Verify ESP-IDF is properly installed and sourced: `. $IDF_PATH/export.sh`
- Check that all submodules are initialized: `git submodule update --init --recursive`
- Verify script is executable: `chmod +x scripts/build_app.sh`
- Clean build: `./scripts/clean_app.sh <app_type> <build_type>` then rebuild
- Verify CMake and Python versions meet requirements
- Check that app name is valid: `./scripts/build_app.sh list`

#### 6. **Script Not Found**

**Symptoms**: `./scripts/build_app.sh: No such file or directory`

**Solutions**:
- Ensure you're in the `examples/esp32` directory
- Verify scripts exist: `ls scripts/`
- Make scripts executable: `chmod +x scripts/*.sh`
- Run setup script: `./scripts/setup_repo.sh`

### Debugging Tips

1. **Enable Debug Logging**: The test suites enable DEBUG logging by default. Monitor serial output for detailed
   communication traces.

2. **Use GPIO14 Indicator**: Monitor GPIO14 with an oscilloscope to see test progression visually.

3. **Logic Analyzer**: Use a logic analyzer to monitor SPI/UART communication for detailed protocol analysis.

4. **Oscilloscope**: Monitor control pins (RST, DRV_EN, FAULTN, WAKE) to verify timing and states.

---

## Hardware Setup Photos

*Add photos of your actual hardware setup here:*
- ESP32-C6 DevKit connected to TMC9660-3PH-EVKIT
- ESP32-C6 DevKit connected to TMC9660-STP-EVKIT
- Motor connections
- Overall test setup

---

## Additional Resources

- **TMC9660 Datasheet**: [TMC9660.pdf](../Datasheet/tmc9660.pdf)
- **TMC9660-3PH-EVKIT User Guide**: [tmc9660-3ph-eval-kit-ug.pdf](../Datasheet/tmc9660-3ph-eval-kit-ug.pdf)
- **TMC9660 Configuration Guide**: [tmc9660-configuration-and-bootstrapping.pdf](../Datasheet/tmc9660-configuration-and-bootstrapping.pdf)
- **TMCL Protocol Reference**: [TMCL_reference_2015.pdf](../Datasheet/TMCL_reference_2015.pdf)

---

## License

Copyright (c) 2025 HardFOC. All rights reserved.

---

## Contributing

Contributions are welcome! Please see the main repository's CONTRIBUTING.md for guidelines.

---

**Happy Testing! 🚀**