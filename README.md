---
layout: default
title: "HardFOC TMC9660 Driver"
description: "C++20 hardware-agnostic driver for Trinamic TMC9660 motor controller with FOC control, telemetry, and TMCL scripting"
nav_order: 1
permalink: /
---

# HF-TMC9660 Driver
**C++20 hardware-agnostic driver for Trinamic TMC9660 motor controller with FOC control, telemetry, and TMCL scripting**

[![C++](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![License](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![CI](https://github.com/hardfoc/hf-tmc9660-driver/actions/workflows/esp32-examples-build-ci.yml/badge.svg?branch=main)](https://github.com/hardfoc/hf-tmc9660-driver/actions/workflows/esp32-examples-build-ci.yml)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://hardfoc.github.io/hf-tmc9660-driver/)

## 📚 Table of Contents
1. [Overview](#-overview)
2. [Features](#-features)
3. [Quick Start](#-quick-start)
4. [Installation](#-installation)
5. [API Reference](#-api-reference)
6. [Examples](#-examples)
7. [Documentation](#-documentation)
8. [References](#-references)
9. [Contributing](#-contributing)
10. [License](#-license)

## 📦 Overview

> **📖 [📚🌐 Live Complete Documentation](https://hardfoc.github.io/hf-tmc9660-driver/)** - 
> Interactive guides, examples, and step-by-step tutorials

**HF-TMC9660** is a portable C++20 driver for the **Trinamic TMC9660** motor controller IC. The TMC9660 is a sophisticated motor driver supporting BLDC, stepper, and DC motors with advanced Field-Oriented Control (FOC), comprehensive telemetry, and TMCL scripting capabilities. The driver provides hardware-agnostic communication interfaces, allowing it to run on any platform (ESP32, STM32, etc.) with SPI or UART.

The driver exposes the full parameter mode interface, providing access to all 300+ TMC9660 parameters through an intuitive C++ API. It supports bootloader initialization, motor configuration, sensor integration (Hall sensors, encoders), FOC control loops, real-time telemetry, and protection systems.

![TMC9660 Field-Oriented Control (FOC) signal flow](docs/assets/tmc9660-foc.svg)

## ✨ Features

- ✅ **Multiple Motor Types**: BLDC/PMSM, Stepper, and DC motor support
- ✅ **FOC Control**: Advanced Field-Oriented Control with torque, velocity, and position loops
- ✅ **Sensor Integration**: Hall sensors, incremental encoders, SPI encoders
- ✅ **Comprehensive Telemetry**: Real-time temperature, current, voltage, and position monitoring
- ✅ **Protection Systems**: Overcurrent, overtemperature, overvoltage protection
- ✅ **TMCL Scripting**: Execute custom scripts on device microcontroller
- ✅ **Hardware Agnostic**: SPI or UART interface for platform independence
- ✅ **Modern C++20**: Type-safe API with RAII principles
- ✅ **Parameter Mode**: Full access to 300+ TMC9660 parameters
- ✅ **Bring-up diagnostics**: Optional compile-time raw SPI TMCL frame logging (`TMC9660_LOG_TMCL_RAW_FRAMES`, off by default)

## 🚀 Quick Start

```cpp
#include "inc/tmc9660.hpp"

// 1. Implement the communication interface (see platform_integration.md)
class MySPI : public tmc9660::SpiCommInterface {
    // ... implement required methods
};

// 2. Create driver instance
MySPI spi;
tmc9660::TMC9660 driver(spi);

// 3. CRITICAL: Initialize bootloader for Parameter Mode
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
cfg.boot.start_motor_control = true;

if (driver.bootloaderInit(&cfg) != tmc9660::TMC9660::BootloaderInitResult::Success) {
    // Handle error
    return;
}

// 4. Configure and start motor
driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7); // 7 pole pairs
driver.focControl.setTargetVelocity(1000);
```

For detailed setup, see [Installation](docs/installation.md) and [Quick Start Guide](docs/quickstart.md).

## 🔧 Installation

1. **Clone or copy** the driver files into your project
2. **Implement the communication interface** for your platform (see [Platform Integration](docs/platform_integration.md))
3. **Include the header** in your code:
   ```cpp
   #include "inc/tmc9660.hpp"
   ```
4. Compile with a **C++20** or newer compiler

For detailed installation instructions, see [docs/installation.md](docs/installation.md).

## 📖 API Reference

| Method | Description |
|--------|-------------|
| `bootloaderInit()` | Initialize bootloader for Parameter Mode (CRITICAL) |
| `motorConfig.setType()` | Set motor type and pole pairs |
| `getMotorContext()` | Read live `MotorContext` (motor type, PP, sensor, CPR) |
| `velocityControl.setTargetVelocityRpm(rpm, ctx)` | Unit-aware velocity command |
| `positionControl.setTargetPositionDegMech(deg, ctx)` | Unit-aware position command |
| `torqueFluxControl.setOpenloopVoltage()` | Open-loop voltage magnitude (preferred for unloaded startup) |
| `torqueFluxControl.setOpenloopCurrent()` | Open-loop d-axis current magnitude |
| `ramp.configureAuto(Ramp::buildRampConfig(...))` | Trapezoidal profile from engineering units |
| `diagnostics.summary()` / `diagnostics.snapshot()` | POD health/state dumps |
| `telemetry.getTemperature()` | Read motor temperature |
| `telemetry.getCurrent()` | Read motor current |

For complete API documentation, see [docs/api_reference.md](docs/api_reference.md)
and [docs/units_and_diagnostics.md](docs/units_and_diagnostics.md).

## 📊 Examples

For ESP32 examples, see the [examples/esp32](examples/esp32/) directory.

Detailed example walkthroughs are available in [docs/examples.md](docs/examples.md).

## 📚 Documentation

For complete documentation, see the [docs directory](docs/index.md).

### Special Features

- **[Bootloader Initialization](docs/special_feature_bootloader.md)** - Critical bootloader setup guide (must be called before motor control)

## 🔗 References

| Resource | Link |
|----------|------|
| Trinamic TMC9660 product page | <https://www.analog.com/en/products/tmc9660.html> |
| TMC9660 datasheet (ADI/Trinamic) | <https://www.analog.com/media/en/technical-documentation/data-sheets/tmc9660_datasheet.pdf> |
| TMC9660 parameter mode reference | <https://www.analog.com/media/en/technical-documentation/user-guides/tmc9660_parameter_mode.pdf> |
| TMCL-IDE & docs | <https://www.analog.com/en/resources/technical-articles/trinamic-motion-control-language-tmcl.html> |
| ESP-IDF SPI / UART | <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/index.html> |
| C++20 language reference | <https://en.cppreference.com/w/cpp/20> |

## 🤝 Contributing

Pull requests and suggestions are welcome! Please follow the existing code style and include tests for new features.

## 📄 License

This project is licensed under the **GNU General Public License v3.0**.
See the [LICENSE](LICENSE) file for details.
