# Configuration

The settings a TMC9660 needs before it drives a motor, in the order to apply them, with the struct for each.

Most subsystems have a `configureAuto(Config)` call: required fields first, optional fields with defaults, values
written in a safe order and verified. Use it for bring-up; the individual setters stay available for tuning.
Every snippet here is compiled from [`tests/docs/configuration.cpp`](../tests/docs/configuration.cpp).

## Order

| Step | Call | Why this position |
| --- | --- | --- |
| 1 | `bootloaderInit(&cfg)` | Clock, interfaces and feedback pins are bootloader settings ([Bootloader](bootloader.md)) |
| 2 | `gateDriver.configurePowerStageProtection(PowerStageProfile)` | Timing and protection of the power stage, before any PWM |
| 3 | `currentSensing.configureAuto(...)`, `calibrateOffsets()` | Offsets are measured with the motor off |
| 4 | `motorConfig.configureAuto(MotorProfile)` | Motor type, pole pairs, PWM, current limits |
| 5 | `feedbackSense.configureAuto(HallConfig / AbnConfig / SpiEncoderConfig)` | The sensor the loops will use |
| 6 | `torqueFluxControl`, `velocityControl`, `positionControl` `.configureAuto(...)` | Loop gains and scaling |
| 7 | `protection` | Voltage, temperature and current thresholds |
| 8 | `motorConfig.setCommutationMode(...)` | Last: the chip starts driving as soon as it is set |

## Current sensing

```cpp
--8<-- "tests/docs/configuration.cpp:sensing"
```

| Field | Required | Meaning |
| --- | :---: | --- |
| `shuntResistance_mOhm` | yes | Nominal shunt value |
| `expectedPeakCurrent_A` | yes | Highest phase current you expect; sets the amplifier gain |
| `usePeakScaling` | | Scale for peak rather than RMS current |
| `actualShuntR_adc0_mOhm` ... `adc3` | | Measured shunt values per channel, if they differ from nominal |

## Motor

The [API guide](api-guide.md#configure-the-motor) shows `MotorProfile` in context.

| Field | Required | Default | Meaning |
| --- | :---: | --- | --- |
| `motorType` | yes | | `BLDC_MOTOR`, `STEPPER_MOTOR` or `DC_MOTOR` |
| `polePairs` | yes | | BLDC: rotor pole pairs. Stepper: full steps per revolution / 4 |
| `pwmFrequency_Hz` | yes | | 20 to 25 kHz is typical |
| `maxPhaseCurrent_A` | yes | | Sets `MAX_TORQUE` and `MAX_FLUX` |
| `direction` | | `FORWARD` | Swap to reverse without rewiring |
| `pwmSwitchingScheme` | | `SVPWM` | Space-vector PWM for BLDC |
| `maxFluxCurrent_A` | | 20 % of the phase current | Flux limit |
| `outputVoltageLimit` | | 8000 | FOC output voltage limit (internal units) |
| `idlePwmBehavior` | | PWM off when idle | |
| `commutationMode` | | unchanged | Applied last, if set |

## Control loops

```cpp
--8<-- "tests/docs/configuration.cpp:loops"
```

Gains are the chip's integer PI parameters. Start with the defaults, raise P until the response is fast without
overshoot, then raise I to remove the steady-state error. Tune torque first, then velocity, then position.

| Struct | Required | Notable optional fields |
| --- | --- | --- |
| `TorqueFluxConfig` | | `torqueP`, `torqueI`, `separateTorqueFluxLoops`, `enableFieldWeakening` |
| `VelocityConfig` | `sensorSelection` | `velocityP`, `velocityI`, `pNormalization`, `iNormalization`, scaling |
| `PositionConfig` | `sensorSelection` | `positionP`, `positionI`, normalisation |

## Protection

```cpp
--8<-- "tests/docs/configuration.cpp:protection"
```

The voltage thresholds are in 0.1 V. Set them around your supply's real range: a 24 V system might warn at 30 V
and 18 V. `protection.configureAuto(ProtectionConfig)` sets all of them at once.

## Where the defaults come from

Each field's default is in its struct in [`inc/tmc9660.hpp`](../inc/tmc9660.hpp) and in the
[API reference](api-reference.md); the chip's own reset values are in the
[parameter-mode reference](datasheet/tmc9660-parameter-mode-reference-manual.pdf).
