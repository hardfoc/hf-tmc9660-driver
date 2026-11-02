# API guide

How to do the common jobs with the driver: boot, configure, command, read, batch and handle errors.

Every snippet on this page is compiled by the host CI from
[`tests/docs/api_tasks.cpp`](../tests/docs/api_tasks.cpp), with `Driver` standing for `tmc9660::TMC9660<MySpi>`
and `tmcl` for `tmc9660::tmcl`. The [API reference](api-reference.md) lists every member.

## The shape of the API

One `TMC9660<Comm>` object per chip. Its public members are the chip's subsystems; each one groups the TMCL
parameters of one function and offers three levels of access:

| Level | Example | Use it for |
| --- | --- | --- |
| `configureAuto(Config)` | `velocityControl.configureAuto(vel)` | Bring-up: one struct, sensible defaults, applied in the right order |
| Unit-aware calls | `setTargetVelocityRpm(1500.0, ctx)` | Commands and readings in RPM, degrees, rad/s |
| Raw setters and parameters | `setTargetVelocityRaw(...)`, `writeParameter(...)` | Anything the chip has that a helper does not cover |

| Member | Owns |
| --- | --- |
| `motorConfig` | Motor type, pole pairs, PWM, commutation mode, current limits |
| `currentSensing` | Current-sense amplifiers, offsets and calibration |
| `gateDriver` | Gate-driver timing and power-stage protection |
| `feedbackSense` | Hall, ABN and SPI encoders |
| `torqueFluxControl` | Torque / flux PI loop, target torque, open-loop modes |
| `velocityControl` | Velocity PI loop and targets |
| `positionControl` | Position loop, targets and limits |
| `ramp` | Trapezoidal ramp generator |
| `stepDir` | Step / direction input |
| `referenceSearch`, `brake`, `iit`, `stopEvents`, `protection` | Homing, brake chopper, I²t, stop conditions, thresholds |
| `telemetry`, `diagnostics` | Readings, status and error flags, summaries |
| `script`, `ramDebug`, `nvmStorage`, `heartbeat`, `globals`, `gpio`, `power` | TMCL scripts, sampling, flash storage, watchdog, global parameters, chip GPIOs, hibernation |

Every call returns `bool` (or a result enum) and never throws. Check it: `false` means the chip did not accept
the value or the transport failed.

## Boot the chip

Nothing works until `bootloaderInit()` succeeds: it resets the chip, writes the bootloader configuration
(clock, interfaces, feedback pins) and starts the motor-control firmware in parameter mode.

```cpp
--8<-- "tests/docs/quickstart.cpp:quickstart"
```

[Bootloader](bootloader.md) covers every field of `BootloaderConfig`, including SPI encoders and step/dir.

## Configure the motor

Configure in this order: motor, feedback, control loops, then select the commutation mode last. The chip runs
the mode as soon as you set it.

```cpp
--8<-- "tests/docs/api_tasks.cpp:configure"
```

`MotorProfile` needs four values (type, pole pairs, PWM frequency, maximum phase current); the rest have
defaults. `configureAuto()` returns `false` at the first value the chip rejects. Tune the loops afterwards with
`TorqueFluxConfig`, `VelocityConfig` and `PositionConfig`; [Configuration](configuration.md) lists their fields.

| Commutation mode | When |
| --- | --- |
| `SYSTEM_OFF` | Default after reset; outputs off |
| `FOC_OPENLOOP_VOLTAGE_MODE`, `FOC_OPENLOOP_CURRENT_MODE` | First spin without feedback, encoder alignment |
| `FOC_HALL_SENSOR` | BLDC with digital Hall sensors |
| `FOC_ABN` | Incremental encoder |
| `FOC_SPI_ENC` | Absolute SPI encoder |

## Command motion

```cpp
--8<-- "tests/docs/api_tasks.cpp:command"
```

The `MotorContext` holds what the unit conversions need: motor type, pole pairs, velocity sensor and scaling.
Read it once after configuration and again only when you change one of those. [Units and
diagnostics](units-and-diagnostics.md) explains the conversions and their limits.

## Read state

```cpp
--8<-- "tests/docs/api_tasks.cpp:read"
```

Each read is one TMCL access (two SPI datagrams). To read several values at the same instant, batch them.

## Read or write several parameters at once

```cpp
--8<-- "tests/docs/api_tasks.cpp:batch"
```

A batch pipelines the accesses: N reads cost N + 1 datagrams instead of 2N. Each element carries its own
`status`. Batches accept only idempotent requests (SAP, GAP, SGP, GGP), because a lost reply makes the driver
re-issue one. `writeParameters()`, `readGlobalParameters()` and `writeGlobalParameters()` work the same way.

## Use any parameter directly

```cpp
--8<-- "tests/docs/api_tasks.cpp:raw"
```

Axis parameters are `tmcl::Parameters`; global parameters are `tmcl::GlobalParamBank0`, `GlobalParamBank2` and
`GlobalParamBank3`, and the bank follows from the type, so an axis id cannot be sent to a global bank by mistake.
The [parameter-mode reference](datasheet/tmc9660-parameter-mode-reference-manual.pdf) lists every parameter.

## Handle errors and faults

```cpp
--8<-- "tests/docs/api_tasks.cpp:errors"
```

| Signal | Meaning | Where to look |
| --- | --- | --- |
| A call returns `false` | The chip rejected the value, or the transport failed | The log (`debugLog`), `comm().tmclStats()` |
| `has_general_error` | Over-current, over-temperature, supply or loop errors | `general_error_flags`, decoded in [Units and diagnostics](units-and-diagnostics.md) |
| `has_gate_driver_error` | Short circuit, gate-drive under-voltage, bootstrap faults | `gate_driver_error_flags` |
| FAULTN active | Bootstrapping, or a severe fault the chip latched | Read the flags, fix the cause, clear them |

Error flags are write-1-to-clear: clear the bits you read, after you have handled the cause. A flag that comes
back at once means the cause is still there. [Troubleshooting](troubleshooting.md) maps symptoms to causes.

## Tune the transport

```cpp
--8<-- "tests/docs/api_tasks.cpp:timing"
```

The defaults follow the datasheet: no fixed gap, NOT_READY polling with back-off, at most about 11.5 ms per
access. In a fast control loop lower `maxPolls` so a busy chip costs you a stale sample, not a missed deadline.
[Timing and throughput](timing.md) has the cost model and the measurement app.
