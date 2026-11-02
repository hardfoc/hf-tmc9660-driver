/**
 * @file configuration.cpp
 * @brief The Configuration page's snippets, compiled by the host build (never run).
 */
#include "my_spi.hpp"

using Driver = tmc9660::TMC9660<MySpi>;
namespace tmcl = tmc9660::tmcl;

bool bring_up(Driver& driver) {
  // --8<-- [start:sensing]
  Driver::CurrentSensing::AutoConfig cs{};
  cs.shuntResistance_mOhm = 3.0f;     // from the schematic
  cs.expectedPeakCurrent_A = 6.0f;    // picks the amplifier gain
  if (!driver.currentSensing.configureAuto(cs) ||
      !driver.currentSensing.calibrateOffsets(true, 2000)) {  // motor off, wait up to 2 s
    return false;
  }
  // --8<-- [end:sensing]

  // --8<-- [start:loops]
  Driver::TorqueFluxControl::TorqueFluxConfig tf{};
  tf.torqueP = 50;                    // unset gains keep the driver's defaults (50 / 100)
  tf.torqueI = 100;
  if (!driver.torqueFluxControl.configureAuto(tf)) {
    return false;
  }

  Driver::VelocityControl::VelocityConfig vel{};
  vel.sensorSelection = tmcl::VelocitySensorSelection::DIGITAL_HALL;
  vel.velocityP = 800;
  vel.velocityI = 1;
  if (!driver.velocityControl.configureAuto(vel)) {
    return false;
  }
  // --8<-- [end:loops]

  // --8<-- [start:protection]
  driver.protection.configureVoltage(/*over*/ 300, /*under*/ 100);  // 0.1 V units: 30.0 V, 10.0 V
  driver.protection.configureTemperature(100.0f, 120.0f);          // warning, shutdown in degrees C
  driver.protection.setOvercurrentEnabled(true);
  // --8<-- [end:protection]
  return true;
}
