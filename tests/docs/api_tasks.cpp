/**
 * @file api_tasks.cpp
 * @brief The API guide's task snippets, compiled by the host build (never run).
 */
#include "my_spi.hpp"

#include <cstdio>

using Driver = tmc9660::TMC9660<MySpi>;
namespace tmcl = tmc9660::tmcl;

bool configure(Driver& driver) {
  // --8<-- [start:configure]
  Driver::MotorConfig::MotorProfile motor{};
  motor.motorType = tmcl::MotorType::BLDC_MOTOR;
  motor.polePairs = 7;
  motor.pwmFrequency_Hz = 25000;
  motor.maxPhaseCurrent_A = 3.0f;                 // sets MAX_TORQUE and MAX_FLUX
  if (!driver.motorConfig.configureAuto(motor)) {
    return false;
  }

  Driver::FeedbackSense::HallConfig hall{};       // defaults: no offset, not inverted
  if (!driver.feedbackSense.configureAuto(hall)) {
    return false;
  }

  Driver::VelocityControl::VelocityConfig vel{};
  vel.sensorSelection = tmcl::VelocitySensorSelection::SAME_AS_COMMUTATION;
  if (!driver.velocityControl.configureAuto(vel)) {
    return false;
  }
  return driver.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_HALL_SENSOR);
  // --8<-- [end:configure]
}

void command(Driver& driver) {
  // --8<-- [start:command]
  const auto ctx = driver.getMotorContext();      // read once, reuse for every conversion

  driver.velocityControl.setTargetVelocityRpm(1500.0, ctx);
  driver.positionControl.setTargetPositionDegMech(90.0, ctx);
  driver.torqueFluxControl.setTargetTorque(800);  // mA

  driver.velocityControl.stop();                  // ramp to zero, stay in the current mode
  driver.motorConfig.setCommutationMode(tmcl::CommutationMode::SYSTEM_OFF);
  // --8<-- [end:command]
}

void read(Driver& driver) {
  // --8<-- [start:read]
  const auto ctx = driver.getMotorContext();

  double rpm = 0.0;
  if (driver.velocityControl.getActualVelocityRpm(rpm, ctx)) {
    std::printf("velocity %.1f rpm\n", rpm);
  }
  const float vbus = driver.telemetry.getSupplyVoltage();     // V
  const float temp = driver.telemetry.getChipTemperature();   // degrees C
  const int16_t current = driver.telemetry.getMotorCurrent(); // mA
  std::printf("%.1f V  %.1f C  %d mA\n", vbus, temp, current);
  // --8<-- [end:read]
}

void batch(Driver& driver) {
  // --8<-- [start:batch]
  Driver::ParameterAccess rd[] = {{tmcl::Parameters::ACTUAL_VELOCITY},
                                  {tmcl::Parameters::ACTUAL_POSITION},
                                  {tmcl::Parameters::GENERAL_STATUS_FLAGS}};
  if (driver.readParameters(rd)) {                // 3 accesses, 4 SPI datagrams
    std::printf("v=%ld p=%ld\n", static_cast<long>(static_cast<int32_t>(rd[0].value)),
                static_cast<long>(static_cast<int32_t>(rd[1].value)));
  }
  // --8<-- [end:batch]
}

void raw(Driver& driver) {
  // --8<-- [start:raw]
  uint32_t maxTorque = 0;
  if (driver.readParameter(tmcl::Parameters::MAX_TORQUE, maxTorque)) {   // any axis parameter
    (void)driver.writeParameter(tmcl::Parameters::MAX_TORQUE, 2500);    // mA
  }
  uint32_t loops = 0;
  (void)driver.readGlobal(tmcl::GlobalParamBank0::MAIN_LOOPS, loops);  // bank from the enum type
  // --8<-- [end:raw]
}

void errors(Driver& driver) {
  // --8<-- [start:errors]
  tmc9660::diagnostics::MotorSummary s{};
  if (driver.diagnostics.summary(s, driver.getMotorContext())) {
    if (s.has_general_error || s.has_gate_driver_error) {
      std::printf("errors: general 0x%08lx gate 0x%08lx\n",
                  static_cast<unsigned long>(s.general_error_flags),
                  static_cast<unsigned long>(s.gate_driver_error_flags));
      driver.telemetry.clearGeneralErrorFlags(s.general_error_flags);       // write-1-to-clear
      driver.telemetry.clearGateDriverErrorFlags(s.gate_driver_error_flags);
    }
  }

  tmc9660::GpioSignal fault{};
  if (driver.comm().gpioRead(tmc9660::TMC9660CtrlPin::FAULTN, fault) &&
      fault == tmc9660::GpioSignal::ACTIVE) {
    // the chip reports a severe fault or is still bootstrapping
  }
  // --8<-- [end:errors]
}

void timing(MySpi& spi) {
  // --8<-- [start:timing]
  tmc9660::TmclSpiTiming t{};   // datasheet defaults: no gap, 20 -> 200 us back-off, 60 polls
  t.maxPolls = 4;               // in a 1 kHz control task: fail fast, mark the sample stale
  spi.setTmclTiming(t);

  const auto& stats = spi.tmclStats();  // frames, notReady, staleReplies, recoveries, failures
  (void)stats;
  // --8<-- [end:timing]
}
