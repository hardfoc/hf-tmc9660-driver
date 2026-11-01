---
layout: default
title: "📚 Documentation"
description: "Complete documentation for the HardFOC TMC9660 Driver"
nav_order: 2
parent: "HardFOC TMC9660 Driver"
permalink: /docs/
has_children: true
---

# HF-TMC9660 Documentation

Welcome! This directory contains step-by-step guides for installing, building, and using the **HF-TMC9660** library.

## 📚 Documentation Structure

### **Getting Started**

1. **[🛠️ Installation](installation.md)** – Prerequisites and how to obtain the source
2. **[⚡ Quick Start](quickstart.md)** – Minimal working example to get you running
3. **[🔌 Hardware Setup](hardware_setup.md)** – Wiring diagrams and pin connections

### **Integration**

4. **[🔧 Platform Integration](platform_integration.md)** – Implement the SPI/UART interface for your platform
5. **[⚙️ Configuration](configuration.md)** – Configuration options and settings

### **Reference**

6. **[📖 API Reference](api_reference.md)** – Complete API documentation
7. **[💡 Examples](examples.md)** – Detailed example walkthroughs
8. **[📐 Units & Diagnostics](units_and_diagnostics.md)** – Engineering-unit conversions, `MotorContext`, `MotorSummary` / `MotorSnapshot`

### **Advanced Features**

8. **[🚀 Bootloader Initialization](special_feature_bootloader.md)** – Critical bootloader setup guide
9. **[📡 Communication Protocols](special_feature_protocols.md)** – Detailed protocol specifications
9. **[⏱️ Timing and Throughput](timing_and_throughput.md)** – Cost of one TMCL access, timing configuration, pipelined batches

### **Troubleshooting**

10. **[🐛 Troubleshooting](troubleshooting.md)** – Common issues and solutions

---

## 🚀 Quick Start Path

**New to TMC9660?** Follow this recommended path:

1. Start with **[Installation](installation.md)** to prepare your environment
2. Follow **[Hardware Setup](hardware_setup.md)** to wire your hardware
3. Read **[Quick Start](quickstart.md)** for a minimal working example
4. **CRITICAL**: Review **[Bootloader Initialization](special_feature_bootloader.md)** - this is required!
5. Check **[Platform Integration](platform_integration.md)** to implement the communication interface
6. Explore **[Examples](examples.md)** for more advanced usage

---

## ⚠️ Critical Requirements

> **BOOTLOADER INITIALIZATION IS MANDATORY**
>
> The TMC9660 must be properly configured for **Parameter Mode** operation via bootloader initialization before any motor control functions will work. This is the #1 source of issues for new users.

**Essential Setup Sequence:**
```cpp
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;  // ESSENTIAL!
cfg.boot.start_motor_control = true;
auto result = driver.bootloaderInit(&cfg);
```

See [Bootloader Initialization](special_feature_bootloader.md) for complete details.

---

## 💡 Need Help?

- **🐛 Found a bug?** Check the [Troubleshooting](troubleshooting.md) guide
- **❓ Have questions?** Review the [API Reference](api_reference.md)
- **📝 Want to contribute?** See the contributing guidelines in the main README

---

**Navigation**
➡️ [Installation](installation.md)
