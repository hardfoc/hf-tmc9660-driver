# Troubleshooting

Symptoms you may see while bringing up a TMC9660, their causes and the fix for each.

Start with `comm().tmclStats()` and the log of a failing call: they tell transport problems (no reply, stale
replies) apart from configuration problems (the chip answers but rejects the value).

## Datasheet-vs-driver math errata

The driver's unit conversions were audited against the TMC9660 Parameter
Mode and Register Mode reference manuals (May 2026). One bug was found
and fixed:

- **Ramp acceleration scaling** — earlier driver versions wrote
  `(rpm/s × k_RPM)` directly to the `RAMP_AMAX/A1/A2/DMAX/D1/D2`
  registers (NR 54..59). The chip's actual scaling is
  `RAMP_A_register = (dV_internal/dt) × 2^17 / fCLK` (Parameter Mode
  manual p. 72; Register Mode manual p. 44). With the default
  `fCLK = 40 MHz` the missing factor is `2^17 / 40e6 ≈ 0.0032768`, so a
  request of *N* RPM/s was previously executing at roughly *N × 305*
  RPM/s. `accelerationToInternal()` now applies the correct factor and
  `AccelerationUnit::Internal` is the raw `RAMP_A` register integer.

- **`COMMUTATION_MODE` enum values** — the actual chip values are
  `FOC_OPENLOOP_VOLTAGE_MODE = 3` and `FOC_OPENLOOP_CURRENT_MODE = 4`
  (NR 4 in the Parameter Mode manual). Older internal notes had these
  swapped/offset; the C++ `enum class CommutationMode` is and has been
  correct.

- **`OPENLOOP_VOLTAGE` range** — NR 47 is **0..16383** (14-bit duty
  cycle, 16383 = 100 %). Older driver doc-comments listed 0..32767.
  The chip does **not** current-limit voltage mode, so use a low value
  (≈ 5 % = 800) for unloaded startup.

## Open-loop spin: rotor moves slowly while `ACTUAL_VELOCITY` reads correct

### Symptom
The chip is in `FOC_OPENLOOP_CURRENT_MODE`, the ramp is configured and
running, `ACTUAL_VELOCITY` reads back the commanded RPM, but the rotor
visibly turns at a small fraction of that — sometimes 5–20% — and the
shaft feels "notchy" or "stutter-y".

### Cause
In `FOC_OPENLOOP_CURRENT_MODE` the chip injects a pure d-axis current at
the commanded electrical angle. Pull-out torque on a PM rotor is bounded
by `T_max = K_t · Id` at 90° elec lag. When inertia + cogging + bearing
friction exceed that ceiling, the rotor falls behind, slips a pole pair,
and recovers — average mechanical speed collapses.

`ACTUAL_VELOCITY` does **not** measure the rotor in this mode. With
`VELOCITY_SENSOR_SELECTION = SAME_AS_COMMUTATION` it is derived from the
integrated `phi_e` ramp, so it always echoes whatever the chip is
*commanding* — not what the rotor is doing. Confirming rotor speed
requires Hall, ABN, or SPI-encoder feedback (or an external tachometer).

### Fix
Use **`FOC_OPENLOOP_VOLTAGE_MODE`** for unloaded / lightly-loaded startup.
Voltage mode applies a fixed `|U|` at the rotating phi_e angle; `Iq`
emerges naturally from `V − back_EMF` as the rotor lags, so both d- and
q-axis current flow and pull-out torque is much higher at low speed.

```cpp
namespace tmcl = tmc9660::tmcl;

driver.torqueFluxControl.setOpenloopVoltage(800);   // ≈5 % modulation
driver.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE);
```

Other mitigations:

- Lower the ramp slope (`AMAX`) so torque demand stays inside the
  pull-out limit.
- Raise `OPENLOOP_CURRENT` (current mode) within `MAX_FLUX` headroom.
- Add real feedback (Hall / ABN / SPI encoder) and switch to closed-loop
  FOC, where stator current is locked at 90° to rotor flux for maximum
  torque per amp.

## Bootloader Initialization Fails

### Symptom
`bootloaderInit()` returns `BootloaderInitResult::Failure`

### Solutions

1. **Check Hardware Reset**
   - Ensure RST pin is properly toggled
   - Verify FAULTN pin monitoring during reset
   - Wait at least 100ms after reset

2. **Verify Communication Interface**
   - Check SPI/UART wiring
   - Verify CS pin (SPI) or TX/RX pins (UART)
   - Confirm clock speed ≤ 4 MHz (SPI)

3. **Check Bootloader Configuration**
   ```cpp
   // CRITICAL: Must set boot mode to Parameter
   cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
   ```

4. **Verify Power Supply**
   - Ensure stable power to TMC9660
   - Check the motor supply is within 7.7 V to 70 V

## No Reply from Device

### Symptom
Communication timeouts, no response to commands

### Solutions

1. **SPI Issues**
   - Verify MOSI/MISO not swapped
   - Check CS pin polarity (active LOW)
   - Confirm SPI mode 3 (CPOL = 1, CPHA = 1)
   - Reduce the clock to 1 MHz while bringing up
   - Do not add delays in `spiTransferTMCL()`: the driver polls NOT_READY itself. `REPLY_INVALID_CMD` on back-to-back
     transfers was a bug of releases up to 1.0.0 ([Communication
     protocols](protocols.md#spi-tmcl-reply_invalid_cmd-on-back-to-back-transfers))
   - On a shared SPI bus, the TMC9660 keeps driving MISO after CS goes high ([Hardware](hardware.md#errata-that-affect-the-host))

2. **UART Issues**
   - Verify TX/RX pins not swapped
   - Check baud rate matches configuration
   - Ensure GPIO pins match bootloader config
   - Try autobaud mode
   - For TMCL, confirm the UART adapter reads a **full 9-byte reply** per transaction; partial reads produce checksum
     and status errors (see [Platform Integration](porting.md))

3. **Check Connections**
   - Verify all connections are secure
   - Check for loose wires
   - Verify common ground

## Motor Not Starting

### Symptom
Motor configured but not rotating

### Solutions

1. **Verify Bootloader Initialization**
   ```cpp
   // Must initialize bootloader first
   cfg.boot.start_motor_control = true;
   auto result = driver.bootloaderInit(&cfg);
   ```

2. **Check Motor Configuration**
   ```cpp
   // Verify motor type is set
   driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
   
   // Verify commutation mode
   driver.motorConfig.setCommutationMode(
       tmc9660::tmcl::CommutationMode::FOC_HALL_SENSOR);
   ```

3. **Check Current Limits**
   ```cpp
   // Ensure current limits are set
   driver.motorConfig.setMaxTorqueCurrent(2000);
   ```

4. **Verify Target Values**
   ```cpp
   // Set target velocity or torque
   driver.velocityControl.setTargetVelocityRpm(1000.0, driver.getMotorContext());
   ```

## Communication Errors

### Symptom
CRC errors, checksum failures, invalid replies

### Solutions

1. **Check Timing**
   - Add delays between commands
   - Respect SPI delayed reply timing
   - Wait for UART reply before next command

2. **Verify Protocol**
   - Check CRC/checksum calculation
   - Verify frame format (8 bytes SPI, 9 bytes UART)
   - Confirm address encoding

3. **Reduce Noise**
   - Use shorter cables
   - Add pull-up/pull-down resistors
   - Check for ground loops

## Sensor Issues

### Symptom
Hall sensors or encoder not working

### Solutions

1. **Check Sensor Wiring**
   - Verify Hall sensor connections
   - Check encoder A/B signals
   - Confirm sensor power supply

2. **Verify Sensor Configuration**
   ```cpp
   // For Hall sensors
   driver.feedbackSense.configureHall();
   
   // For encoder
   driver.feedbackSense.configureABNEncoder(2048);
   ```

3. **Check Commutation Mode**
   ```cpp
   // Must match sensor type
   driver.motorConfig.setCommutationMode(
       tmc9660::tmcl::CommutationMode::FOC_HALL_SENSOR); // or FOC_ENCODER
   ```

## Protection Faults

### Symptom
Overcurrent, undervoltage, or overtemperature faults

### Solutions

1. **Check Current Limits**
   ```cpp
   // Reduce current limits if overcurrent
   driver.motorConfig.setMaxTorqueCurrent(1500);
   ```

2. **Verify Power Supply**
   - Check voltage levels
   - Ensure adequate current capacity
   - Check for voltage drops

3. **Check Temperature**
   - Ensure adequate cooling
   - Monitor temperature via telemetry
   - Reduce load if overheating

## Debugging Tips

1. **Enable Debug Logging**
   - Remove `TMC9660_DISABLE_DEBUG_LOGGING` define
   - Check serial output for detailed logs

2. **Verify Communication**
   ```cpp
   // Test basic communication
   auto version = driver.bootloaderGetVersion();
   if (version.has_value()) {
       printf("Communication OK\n");
   }
   ```

3. **Check Telemetry**
   ```cpp
   // Monitor device status
   int16_t temp;
   driver.telemetry.getTemperature(temp);
   printf("Temperature: %d\n", temp);
   ```

## Getting Help

- Check the [Bootloader Initialization](bootloader.md) guide
- Review [Platform Integration](porting.md) for interface issues
- See [Examples](examples.md) for working code
- Consult the [API Reference](api-guide.md) for method details

