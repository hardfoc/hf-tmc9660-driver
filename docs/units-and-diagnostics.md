---
layout: default
title: "📐 Units & Diagnostics"
description: "Strongly-typed engineering units, MotorContext, and diagnostic snapshots"
nav_order: 8
parent: "📚 Documentation"
permalink: /docs/units_and_diagnostics/
---

# Units & Diagnostics

The TMC9660 stores velocity, acceleration, position, and electrical angle in
opaque **internal units** whose meaning depends on the active velocity-feedback
source and the motor's pole-pair count. The driver ships two header-only
modules that turn these raw integers into engineering quantities and structured
diagnostic dumps:

- [`inc/tmc9660_units.hpp`](../inc/tmc9660_units.hpp) — strong-typed unit enums,
  `MotorContext`, and conversion helpers.
- [`inc/tmc9660_diagnostics.hpp`](../inc/tmc9660_diagnostics.hpp) — POD
  `MotorSummary` and `MotorSnapshot` structs.
- [`inc/tmc9660.hpp`](../inc/tmc9660.hpp) — the `Diagnostics` subsystem on
  `TMC9660` populates the snapshots, and unit-aware setters/getters live on
  `VelocityControl`, `PositionControl`, and `Ramp`.

---

## 1. `MotorContext`

`MotorContext` bundles the chip-side facts needed to map raw counts to RPM,
revolutions, electrical degrees, etc.

```cpp
struct tmc9660::units::MotorContext {
    tmcl::MotorType                motor_type;       // BLDC / STEPPER / DC
    uint8_t                        pole_pairs;       // BLDC: rotor PP. Stepper: full_steps/4.
    tmcl::VelocitySensorSelection  velocity_sensor;  // NR 132
    uint32_t                       encoder_cpr;      // ABN/SPI counts/rev (else ignored)

    uint32_t cpr()    const noexcept;   // counts per mech rev (0 = unknown)
    double   k_rpm()  const noexcept;   // internal velocity units per mech RPM
    bool     valid() const noexcept;    // true when conversions are meaningful
};
```

CPR derivation:

| `velocity_sensor`        | `cpr()` |
|--------------------------|---------|
| `SAME_AS_COMMUTATION`    | `65536 * pole_pairs` |
| `DIGITAL_HALL`           | `6 * pole_pairs` |
| `ABN1_ENCODER` / `ABN2_ENCODER` / `SPI_ENCODER` | `encoder_cpr` |

`k_RPM` follows the datasheet: `k_RPM = cpr * 2^24 / (40 MHz · 60)`.

### Acquiring the context

You can either construct a context from constants you already know:

```cpp
constexpr tmc9660::units::MotorContext ctx{
    tmc9660::tmcl::MotorType::BLDC_MOTOR,
    /* pole_pairs */          7,
    tmc9660::tmcl::VelocitySensorSelection::SAME_AS_COMMUTATION,
    /* encoder_cpr */         0u,
};
```

…or read it live from the chip after bootloader/parameter setup:

```cpp
const auto ctx = driver.getMotorContext();   // value-returning convenience
// or
tmc9660::units::MotorContext c{};
if (!driver.getMotorContext(c)) { /* SAP read failed */ }
```

`getMotorContext()` reads:

- `MOTOR_TYPE` (NR 8)
- `MOTOR_POLE_PAIRS` (NR 9)
- `VELOCITY_SENSOR_SELECTION` (NR 132)
- `ABN_1_STEPS` / `ABN_2_STEPS` (NR 175 / 178) for ABN selections

Refresh your cached context whenever any of those parameters change.

---

## 2. Strongly-typed unit enums

Each physical dimension has its own `enum class`; mixing them is a compile
error, so `setTargetVelocity(80, PositionUnit::DegMech, ctx)` will not build.

| Enum                                | Members |
|-------------------------------------|---------|
| `tmc9660::units::VelocityUnit`      | `Rpm`, `Rps`, `RadPerSec`, `DegPerSec`, `Internal` |
| `tmc9660::units::PositionUnit`      | `MechRevs`, `DegMech`, `RadMech`, `Counts` |
| `tmc9660::units::AccelerationUnit`  | `RpmPerSec`, `RpsPerSec`, `RadPerSec2`, `DegPerSec2`, `Internal` (= raw `RAMP_A` register integer) |
| `tmc9660::units::AngleUnit`         | `DegElec`, `RadElec`, `DegMech`, `RadMech`, `PhiERaw` |

### Conversion helpers (free functions)

```cpp
namespace tmc9660::units {
double  convertVelocity     (double, VelocityUnit from, VelocityUnit to, MotorContext const&);
int32_t velocityToInternal  (double, VelocityUnit,                MotorContext const&);
double  convertPosition     (double, PositionUnit from, PositionUnit to, MotorContext const&);
int32_t positionToCounts    (double, PositionUnit,                MotorContext const&);
uint32_t accelerationToInternal(double, AccelerationUnit,         MotorContext const&);
double  convertAngle        (double, AngleUnit from, AngleUnit to, MotorContext const&);
}
```

All conversions saturate to the chip's actual register ranges (no UB on
overflow):

- `velocityToInternal` saturates to **±(2^27−1)** = ±134,217,727 (NR 124 `TARGET_VELOCITY`).
- `accelerationToInternal` saturates to **1 .. 2^23−1** = 1 .. 8,388,607 (NR 54..59 `RAMP_A*` / `RAMP_D*`).
- `positionToCounts` saturates to int32 range.

When `ctx.valid() == false`, conversions that depend on the sensor/CPR
return 0.

### Datasheet scaling formulas (used internally)

| Quantity | Formula | Reference |
|----------|---------|-----------|
| `k_RPM` (vel-internal per mech RPM) | `CPR × 2^24 / (40 MHz · 60)` | Param Mode manual p. 68 |
| `CPR` (SAME_AS_COMMUTATION) | `2^16 × MOTOR_POLE_PAIRS` | Param Mode manual p. 68 |
| `CPR` (DIGITAL_HALL) | `6 × MOTOR_POLE_PAIRS` | Param Mode manual p. 68 |
| `RAMP_A` register | `(dV_internal/dt) × 2^17 / fCLK` | Param Mode manual p. 72 / Register Mode manual p. 44 |
| `PHI_E` LSB | `1 elec_rev / 2^16` (65536 LSB = 1 elec rev) | Param Mode manual |

---

## 3. Unit-aware chip API

### `VelocityControl`

```cpp
// Generic
bool setTargetVelocity(double value, VelocityUnit unit, MotorContext const& ctx);
bool getActualVelocity(double& value, VelocityUnit unit, MotorContext const& ctx);

// Wrappers — type-locked, slightly cheaper
bool setTargetVelocityRpm       (double rpm,  MotorContext const& ctx);
bool setTargetVelocityRadPerSec (double rps,  MotorContext const& ctx);
bool setTargetVelocityRaw       (int32_t internal);

bool getActualVelocityRpm       (double& rpm, MotorContext const& ctx);

// Velocity offset (open-loop bias) — same shape
bool setVelocityOffsetRaw       (int32_t internal);
```

### `PositionControl`

```cpp
bool setTargetPosition          (double value, PositionUnit unit, MotorContext const& ctx);
bool setTargetPositionDegMech   (double deg,                  MotorContext const& ctx);
bool setTargetPositionRaw       (int32_t counts);
```

### `Ramp::buildRampConfig`

`buildRampConfig()` produces a fully-populated single-segment trapezoidal
profile from engineering units; pass the result to `Ramp::configureAuto()`:

```cpp
auto rc = TMC9660<MyComm>::Ramp::buildRampConfig(
    /* max_velocity     */ 600.0, VelocityUnit::Rpm,
    /* max_acceleration */ 50.0,  AccelerationUnit::RpmPerSec,
    ctx);
rc.enableRamp               = true;     // optional override
rc.enableDirectVelocityMode = false;    // OPENLOOP: ramper drives φe (Hall FOC wants true)
driver.ramp.configureAuto(rc);
```

Fields populated by `buildRampConfig()`:

| Register | Field on `RampConfig` | Default value |
|----------|------------------------|---------------|
| `RAMP_VMAX`     | `maxVelocity`               | from `max_velocity` |
| `RAMP_AMAX` / `A1` / `A2` | `maxAcceleration`, `acceleration1`, `acceleration2` | from `max_acceleration` |
| `RAMP_DMAX` / `D1` / `D2` | `maxDeceleration`, `deceleration1`, `deceleration2` | mirrored from acceleration |
| `RAMP_V1` / `V2`          | `velocityThreshold1`, `velocityThreshold2` | 0 (single-segment) |
| `RAMP_VSTART` / `VSTOP`   | `startVelocity`, `stopVelocity` | 0 / 1 |
| `RAMP_TVMAX` / `TZEROWAIT` | `timeAtVmax`, `timeZeroWait` | 0 / 0 |

Feedforward, ramp enable, and direct-velocity-mode flags are intentionally
left as `std::optional` so callers can override them after the build call.

---

## 4. Diagnostics

Two PODs in `tmc9660::diagnostics`:

| Struct          | Size  | Use                                                |
|-----------------|-------|----------------------------------------------------|
| `MotorSummary`  | ~64 B | High-rate polling, constrained transports, simple health gauges |
| `MotorSnapshot` | ~256 B | Full debug dump: every read-only state + decoded flags |

`MotorSnapshot` echoes the `MotorContext` it was populated with (so consumers
can reproduce conversions later) and carries both raw and engineering-unit
fields for velocity, position, and electrical angle, plus full FOC currents/
voltages and the four chip status/error registers.

### Populating

```cpp
tmc9660::diagnostics::MotorSummary  s;
tmc9660::diagnostics::MotorSnapshot snap;

driver.diagnostics.summary (s,    ctx);
driver.diagnostics.snapshot(snap, ctx);

if (snap.regulating_velocity && snap.gate_driver_error_flags == 0) {
    printf("vel=%.2f RPM, Iq=%d mA, T=%.1f °C\n",
           snap.actual_velocity_rpm, snap.iq_ma, snap.chip_temp_c);
}
```

Both methods are best-effort: a failed individual SAP leaves the corresponding
field at its default; `valid_*` bits flip true only on success.

---

## 5. Worked example — open-loop spin in voltage mode

```cpp
namespace tmcl  = tmc9660::tmcl;
namespace units = tmc9660::units;

// 1. Acquire context (after bootloader + motor parameters are stable).
const units::MotorContext ctx = driver.getMotorContext();

// 2. Build a coherent ramp profile.
auto rc = tmc9660::TMC9660<MyComm>::Ramp::buildRampConfig(
    /* VMAX */ 600.0, units::VelocityUnit::Rpm,
    /* AMAX */ 50.0,  units::AccelerationUnit::RpmPerSec,
    ctx);
rc.enableRamp               = true;
rc.enableDirectVelocityMode = false;   // required so the ramper drives phi_e
driver.ramp.configureAuto(rc);

// 3. Set the open-loop magnitude *before* arming the mode.
driver.torqueFluxControl.setOpenloopVoltage(800);   // ~5 % duty (range 0..16383)
driver.motorConfig.setCommutationMode(
    tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE);   // = 3 in NR 4 COMMUTATION_MODE

// 4. Command the velocity in engineering units.
driver.velocityControl.setTargetVelocityRpm(80.0, ctx);

// 5. Periodic snapshot.
tmc9660::diagnostics::MotorSnapshot snap;
driver.diagnostics.snapshot(snap, ctx);
```

> **Why voltage mode for unloaded startup?** In `FOC_OPENLOOP_CURRENT_MODE`
> the chip injects pure d-axis flux current; pull-out torque is bounded by
> `K_t · Id` at 90° elec lag. Cogging + bearing friction + acceleration·J
> can exceed that limit on small (≤ 30 W) BLDCs, causing the rotor to slip
> while the chip's `actual_velocity` (which echoes the integrated phi_e
> ramp under `SAME_AS_COMMUTATION`) reports the commanded RPM. Voltage mode
> applies a fixed `|U|` at the rotating phi_e angle; `Iq` emerges naturally
> as the rotor lags, giving substantially more low-speed torque and the
> documented TMC9660 unloaded test behavior.

---

**Navigation**
⬅ [API Reference](api_reference.md) | [Examples](examples.md) ➡
