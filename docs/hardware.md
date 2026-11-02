---
layout: default
title: "🔌 Hardware Setup"
description: "Hardware wiring and connection guide for TMC9660 driver"
nav_order: 3
parent: "📚 Documentation"
permalink: /docs/hardware_setup/
---

# Hardware Setup

![TMC9660 Field-Oriented Control (FOC) signal flow](assets/tmc9660-foc.svg)

This guide covers the hardware connections and requirements for the TMC9660 motor controller.

## Pin Connections

### SPI Communication Interface

```
MCU                    TMC9660
──────────────────     ───────────────────────────────
MOSI (GPIO7)      ───────> SPI MOSI
MISO (GPIO2)      <─────── SPI MISO
SCLK (GPIO6)      ───────> SPI SCLK
CS   (GPIO18)     ───────> SPI CS
```

### Control Pins

```
MCU                    TMC9660
──────────────────     ───────────────────────────────
RST  (GPIO22)     ───────> RST     (Reset, active HIGH)
DRV_EN (GPIO20)   ───────> DRV_EN  (Driver Enable, active HIGH)
FAULTN (GPIO19)   <─────── FAULTN  (Fault Output, active LOW, open-drain)
WAKE (GPIO21)     ───────> WAKE    (Wake Input, active LOW)
```

### UART Communication Interface (Alternative)

```
MCU                    TMC9660
──────────────────     ───────────────────────────────
TX   (GPIO5)      ───────> UART RX (GPIO6 on TMC9660)
RX   (GPIO4)      <─────── UART TX (GPIO7 on TMC9660)
```

### Power and Ground

```
MCU                    TMC9660
──────────────────     ───────────────────────────────
GND              ───────> GND (Common ground)
3.3V / 5V        ───────> VCC (if needed for level shifting)
```

## Pin Functions

### Control Pins (Output from Host)

- **RST (Pin 22)**: External System Reset Input
  - Device remains in reset while this pin is in its active state
  - Has internal pull-down resistor
  - Active level depends on board implementation

- **DRV_EN (Pin 21)**: Driver Enable Input
  - Enables the motor driver outputs
  - Has internal pull-down resistor
  - Active level depends on board implementation

- **WAKE (Pin 19)**: Wake Input
  - Drive to active state to enable power-up and exit from hibernation mode
  - When not shorted to VSA, external pull-down resistor recommended
  - Active level depends on board implementation

### Status Pins (Input to Host)

- **FAULTN (Pin 20)**: FAULT Output Signal (open drain)
  - Indicates busy state during bootstrapping or severe error
  - Active level depends on board implementation
  - Use `gpioRead(TMC9660CtrlPin::FAULTN, signal)` to check fault status

## Electrical Specifications

### SPI Interface

- **Speed**: Up to 4 MHz
- **Data Size**: 8 bytes per transaction
- **Mode**: SPI Mode 0 or 3 (CPOL=0, CPHA=0 or CPOL=1, CPHA=1)
- **CS Polarity**: Active LOW

### UART Interface

- **Baud Rate**: 9600 to 1000000 baud, or autobaud
- **Data Format**: 8 data bits, 1 stop bit, no parity
- **Frame Size**: 9 bytes per transaction

## Power Requirements

- **Motor Supply (VM)**: 12V to 48V (depends on motor)
- **Logic Supply (VCC)**: 3.3V or 5V
- **Common Ground**: Must be connected between MCU and TMC9660

## Motor Connections

### BLDC Motor (3-Phase)

- **U, V, W**: Three-phase motor connections
- **Hall Sensors**: Connect to Hall connector (GPIO2, 3, 4)
- **ABN Encoder**: Connect to Encoder connector (GPIO8, 13, 14)

### Stepper Motor

- **Step/Direction**: External step/direction interface
- **Reference Switches**: Left (GPIO2), Right (GPIO3), Home (GPIO4)
- **ABN Encoder**: Connect to Encoder connector (GPIO8, 13, 14)

## Typical Application Circuit

![TMC9660 wiring](assets/tmc9660-wiring.svg)

```
┌─────────────────────┐         ┌─────────────────────┐
│  MCU (ESP32/STM32) │         │   TMC9660 Board    │
│                     │         │                     │
│  SPI Interface:     │         │                     │
│  • MOSI      ──────┼─────────┼─> SPI MOSI          │
│  • MISO      <──────┼─────────┼─< SPI MISO          │
│  • SCLK      ───────┼─────────┼─> SPI SCLK          │
│  • CS        ───────┼─────────┼─> SPI CS            │
│                     │         │                     │
│  Control Pins:      │         │                     │
│  • RST       ───────┼─────────┼─> RST               │
│  • DRV_EN    ───────┼─────────┼─> DRV_EN            │
│  • FAULTN    <───────┼─────────┼─< FAULTN            │
│  • WAKE      ───────┼─────────┼─> WAKE              │
│                     │         │                     │
│  GND         ───────┼─────────┼─> GND               │
│                     │         │                     │
│                     │         │  Motor:             │
│                     │         │  • U, V, W (BLDC)   │
│                     │         │  • Step/Dir (Stepper)│
└─────────────────────┘         └─────────────────────┘
```

## Setup Steps

1. **Power Off**: Ensure both MCU and TMC9660 are powered off
2. **Connect SPI Interface**: Wire MOSI, MISO, SCLK, CS
3. **Connect Control Pins**: Wire RST, DRV_EN, FAULTN, WAKE
4. **Connect Ground**: Common ground connection
5. **Connect Motor**: Wire motor and sensors (if applicable)
6. **Power On**: Power MCU first, then TMC9660

## Next Steps

- **[Quick Start](quickstart.md)** - Get a minimal example running
- **[Platform Integration](platform_integration.md)** - Implement the communication interface

