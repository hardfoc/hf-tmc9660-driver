---
layout: default
title: "📖 API Reference"
description: "Complete API reference for the TMC9660 driver"
nav_order: 6
parent: "📚 Documentation"
permalink: /docs/api_reference/
---

# API Reference

Complete reference documentation for all public methods and types in the TMC9660 driver.

## Source Code

- **Main Header**: [`inc/tmc9660.hpp`](../inc/tmc9660.hpp)
- **Communication Interface**: [`inc/tmc9660_comm_interface.hpp`](../inc/tmc9660_comm_interface.hpp)
- **Bootloader**: [`inc/bootloader/tmc9660_bootloader.hpp`](../inc/bootloader/tmc9660_bootloader.hpp)
- **Parameter Mode**: [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp)

## TMC9660 Class

Main driver class for TMC9660 motor controller.

**Location**: [`inc/tmc9660.hpp#L117`](../inc/tmc9660.hpp#L117)

### Constructor

| Method | Signature | Location |
|--------|-----------|----------|
| `TMC9660()` | `TMC9660(CommInterface& comm, uint8_t address = 0, const BootloaderConfig* bootCfg = nullptr)` | [`inc/tmc9660.hpp#L130`](../inc/tmc9660.hpp#L130) |

### Core Methods

| Method | Signature | Location |
|--------|-----------|----------|
| `comm()` | `CommInterface& comm()` | [`inc/tmc9660.hpp#L139`](../inc/tmc9660.hpp#L139) |
| `bootloaderInit()` | `BootloaderInitResult bootloaderInit(const BootloaderConfig* cfg, bool performReset = true, bool retrieveBootloaderInfo = false, bool failOnVerifyError = true)` | [`inc/tmc9660.hpp#L212`](../inc/tmc9660.hpp#L212) |
| `getBootloader()` | `TMC9660Bootloader* getBootloader()` | [`inc/tmc9660.hpp#L248`](../inc/tmc9660.hpp#L248) |
| `enterBootloaderMode()` | `bool enterBootloaderMode()` | [`inc/tmc9660.hpp#L275`](../inc/tmc9660.hpp#L275) |
| `writeParameter()` | `bool writeParameter(tmcl::Parameters id, uint32_t value)` | [`inc/tmc9660.hpp#L296`](../inc/tmc9660.hpp#L296) |
| `readParameter()` | `bool readParameter(tmcl::Parameters id, uint32_t& value)` | [`inc/tmc9660.hpp#L306`](../inc/tmc9660.hpp#L306) |
| `writeGlobalParameter()` | `bool writeGlobalParameter(GlobalParamBankVariant id, uint8_t bank, uint32_t value)` | [`inc/tmc9660.hpp#L320`](../inc/tmc9660.hpp#L320) |
| `readGlobalParameter()` | `bool readGlobalParameter(GlobalParamBankVariant id, uint8_t bank, uint32_t& value)` | [`inc/tmc9660.hpp#L330`](../inc/tmc9660.hpp#L330) |
| `sendCommand()` | `bool sendCommand(tmcl::Op opcode, uint16_t type = 0, uint8_t motor = 0, uint32_t value = 0, uint32_t* reply = nullptr)` | [`inc/tmc9660.hpp#L334`](../inc/tmc9660.hpp#L334) |
| `readGlobal()` / `writeGlobal()` | `bool readGlobal(BankEnum id, uint32_t& value)` — bank deduced from `GlobalParamBank0/2/3`; the overloads taking `tmcl::Parameters` are deleted | See source |
| `readParameters()` / `writeParameters()` | `bool readParameters(std::span<ParameterAccess> items)` — pipelined GAP/SAP batch | See source |
| `readGlobalParameters()` / `writeGlobalParameters()` | `bool readGlobalParameters(std::span<GlobalParameterAccess> items)` — pipelined GGP/SGP batch, bank from the enum type | See source |
| `sendCommandBatch()` | `bool sendCommandBatch(std::span<const TMCLFrame> rq, std::span<TMCLReply> rp, size_t* completed = nullptr)` | See source |

See [Timing and Throughput](timing_and_throughput.md) for the cost of each access and the batch rules.

### Enums

| Type | Values | Location |
|------|--------|----------|
| `BootloaderInitResult` | `Success`, `NoConfig`, `Failure` | [`inc/tmc9660.hpp#L153`](../inc/tmc9660.hpp#L153) |

## MotorConfig Subsystem

Motor configuration and control subsystem.

**Location**: [`inc/tmc9660.hpp#L361`](../inc/tmc9660.hpp#L361)

### Methods

| Method | Signature | Location |
|--------|-----------|----------|
| `setType()` | `bool setType(tmcl::MotorType type, uint8_t polePairs = 1)` | [`inc/tmc9660.hpp#L375`](../inc/tmc9660.hpp#L375) |
| `setDirection()` | `bool setDirection(tmcl::MotorDirection direction)` | [`inc/tmc9660.hpp#L384`](../inc/tmc9660.hpp#L384) |
| `setPWMFrequency()` | `bool setPWMFrequency(uint32_t frequencyHz)` | [`inc/tmc9660.hpp#L391`](../inc/tmc9660.hpp#L391) |
| `setCommutationMode()` | `bool setCommutationMode(tmcl::CommutationMode mode)` | [`inc/tmc9660.hpp#L423`](../inc/tmc9660.hpp#L423) |
| `setOutputVoltageLimit()` | `bool setOutputVoltageLimit(uint16_t limit)` | [`inc/tmc9660.hpp#L433`](../inc/tmc9660.hpp#L433) |
| `getOutputVoltageLimit()` | `bool getOutputVoltageLimit(uint16_t& limit)` | [`inc/tmc9660.hpp#L440`](../inc/tmc9660.hpp#L440) |
| `setMaxTorqueCurrent()` | `bool setMaxTorqueCurrent(uint16_t milliamps)` | [`inc/tmc9660.hpp#L452`](../inc/tmc9660.hpp#L452) |
| `setMaxFluxCurrent()` | `bool setMaxFluxCurrent(uint16_t milliamps)` | [`inc/tmc9660.hpp#L465`](../inc/tmc9660.hpp#L465) |
| `setPWMSwitchingScheme()` | `bool setPWMSwitchingScheme(tmcl::PwmSwitchingScheme scheme)` | [`inc/tmc9660.hpp#L484`](../inc/tmc9660.hpp#L484) |
| `setIdleMotorPWMBehavior()` | `bool setIdleMotorPWMBehavior(tmcl::IdleMotorPwmBehavior pwmOffWhenIdle)` | [`inc/tmc9660.hpp#L575`](../inc/tmc9660.hpp#L575) |
| `configureAuto()` | `bool configureAuto(const MotorProfile& profile)` | [`inc/tmc9660.hpp#L639`](../inc/tmc9660.hpp#L639) |

## FOC Control Subsystems

The TMC9660's FOC stack is split into three cooperating subsystems on
`TMC9660`. There is **no** monolithic `focControl` member.

| Member                   | Class             | Responsibility                                               |
|--------------------------|-------------------|--------------------------------------------------------------|
| `driver.torqueFluxControl` | `TorqueFluxControl` | Torque/flux PI gains, target torque/flux, open-loop magnitude (`setOpenloopCurrent`, `setOpenloopVoltage`) |
| `driver.velocityControl` | `VelocityControl` | Velocity PI gains, target velocity, velocity offset/feedforward, **unit-aware** setters |
| `driver.positionControl` | `PositionControl` | Position P gain, target position, **unit-aware** setters    |
| `driver.ramp`            | `Ramp`            | Trapezoidal ramper (VMAX/AMAX/DMAX/V1/V2/VSTART/VSTOP/...)   |

Each subsystem exposes:

- low-level raw integer setters/getters that map 1:1 to TMCL parameters,
- unit-aware setters that take a `MotorContext` (see [Units &
  Diagnostics](units_and_diagnostics.md)),
- `configureAuto(...)` for batch configuration from a profile struct.

### `VelocityControl` — selected methods

| Method | Notes |
|--------|-------|
| `setTargetVelocity(double, VelocityUnit, MotorContext const&)` | Generic unit-aware setter |
| `setTargetVelocityRpm(double, MotorContext const&)`            | RPM convenience wrapper |
| `setTargetVelocityRadPerSec(double, MotorContext const&)`      | rad/s convenience wrapper |
| `setTargetVelocityRaw(int32_t)`                                | Direct internal-units write |
| `getActualVelocityRpm(double&, MotorContext const&)`           | Read NR `ACTUAL_VELOCITY` and convert |
| `setVelocityP/I/...`                                           | PI gain plumbing |

### `PositionControl` — selected methods

| Method | Notes |
|--------|-------|
| `setTargetPosition(double, PositionUnit, MotorContext const&)` | Generic unit-aware setter |
| `setTargetPositionDegMech(double, MotorContext const&)`        | Mechanical-degree wrapper |
| `setTargetPositionRaw(int32_t)`                                | Direct counts write |

### `TorqueFluxControl` — selected methods

| Method | Notes |
|--------|-------|
| `setOpenloopCurrent(uint16_t milliamps)` | Id-only injection used by `FOC_OPENLOOP_CURRENT_MODE` |
| `setOpenloopVoltage(uint16_t units)`     | `|U|` injection used by `FOC_OPENLOOP_VOLTAGE_MODE` (preferred for unloaded startup) |
| `setTargetTorque(int16_t mA)`            | Closed-loop Iq target |
| `setTargetFlux(int16_t mA)`              | Closed-loop Id target |

### `Ramp::buildRampConfig` (static)

Produces a fully-populated single-segment trapezoidal `RampConfig` from
engineering units; pass to `ramp.configureAuto()`. See [Units &
Diagnostics §3](units_and_diagnostics.md#3-unit-aware-chip-api).

```cpp
auto rc = TMC9660<MyComm>::Ramp::buildRampConfig(
    600.0, units::VelocityUnit::Rpm,
    50.0,  units::AccelerationUnit::RpmPerSec,
    ctx);
```

## Diagnostics Subsystem

**Location**: [`inc/tmc9660.hpp`](../inc/tmc9660.hpp), POD types in
[`inc/tmc9660_diagnostics.hpp`](../inc/tmc9660_diagnostics.hpp).

| Method | Signature |
|--------|-----------|
| `summary()`  | `bool summary (diagnostics::MotorSummary&,  units::MotorContext const&)` |
| `snapshot()` | `bool snapshot(diagnostics::MotorSnapshot&, units::MotorContext const&)` |

`MotorSummary` is ~64 B (high-rate health gauge); `MotorSnapshot` is the full
read-only state dump including raw + engineering-unit velocity/position,
electrical angle, FOC currents/voltages, and the four chip status/error
registers. Both echo the input `MotorContext` so consumers can reproduce
conversions later. See [Units & Diagnostics §4](units_and_diagnostics.md#4-diagnostics).

## MotorContext (live)

```cpp
units::MotorContext ctx;
driver.getMotorContext(ctx);          // out-param overload
const auto ctx2 = driver.getMotorContext();   // value-returning overload
```

Reads `MOTOR_TYPE`, `MOTOR_POLE_PAIRS`, `VELOCITY_SENSOR_SELECTION`, and
`ABN_*_STEPS` (for ABN selections). Refresh after any of those parameters
change.

## Telemetry Subsystem

Telemetry and monitoring subsystem.

**Location**: [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp)

### Methods

| Method | Signature | Location |
|--------|-----------|----------|
| `getTemperature()` | `bool getTemperature(int16_t& temp)` | See source |
| `getCurrent()` | `bool getCurrent(int16_t& current)` | See source |
| `getVoltage()` | `bool getVoltage(uint16_t& voltage)` | See source |
| `getPosition()` | `bool getPosition(int32_t& position)` | See source |

## Communication Interfaces

### SpiCommInterface

SPI communication interface base class.

**Location**: [`inc/tmc9660_comm_interface.hpp`](../inc/tmc9660_comm_interface.hpp)

| Method | Signature | Location |
|--------|-----------|----------|
| `spiTransferTMCL()` | `bool spiTransferTMCL(std::array<uint8_t,8>& tx, std::array<uint8_t,8>& rx)` (implemented by the platform) | See source |
| `transferTMCL()` | One access: request, NOT_READY-polled reply-request | See source |
| `transferTMCLBatch()` | N accesses in N + 1 datagrams (idempotent requests only) | See source |
| `setTmclTiming()` / `tmclTiming()` | `TmclSpiTiming`: `frameGapUs`, `pollIntervalUs`, `pollIntervalMaxUs`, `maxPolls`, `verifyReplyOpcode`; `TmclSpiTiming::legacy()` | See source |
| `tmclStats()` / `resetTmclStats()` | `TmclSpiStats`: frames, NOT_READY re-sends, stale replies, recoveries, failures | See source |

### UartCommInterface

UART communication interface base class.

**Location**: [`inc/tmc9660_comm_interface.hpp`](../inc/tmc9660_comm_interface.hpp)

| Method | Signature | Location |
|--------|-----------|----------|
| `uartTransfer()` | `bool uartTransfer(const TMCLFrame& tx, TMCLReply& reply, uint8_t address)` | See source |

## Types

### Enums

| Type | Location |
|------|----------|
| `tmcl::MotorType` | [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp) |
| `tmcl::CommutationMode` | [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp) |
| `tmcl::MotorDirection` | [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp) |
| `tmcl::PwmSwitchingScheme` | [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp) |
| `tmcl::Parameters` | [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp) |
| `tmcl::Op` | [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp) |
| `bootcfg::BootMode` | [`inc/bootloader/bootloader_config.hpp`](../inc/bootloader/bootloader_config.hpp) |

### Structs

| Type | Location |
|------|----------|
| `BootloaderConfig` | [`inc/bootloader/bootloader_config.hpp`](../inc/bootloader/bootloader_config.hpp) |
| `MotorProfile` | [`inc/tmc9660.hpp`](../inc/tmc9660.hpp) |
| `TMCLFrame` | [`inc/tmc9660_comm_interface.hpp`](../inc/tmc9660_comm_interface.hpp) |
| `TMCLReply` | [`inc/tmc9660_comm_interface.hpp`](../inc/tmc9660_comm_interface.hpp) |

