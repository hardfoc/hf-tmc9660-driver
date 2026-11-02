/**
 * @file quickstart.cpp
 * @brief The Quickstart page's example, compiled by the host build (never run).
 */
#include "my_spi.hpp"

// --8<-- [start:quickstart]
#include "tmc9660.hpp"

int spin_bldc_at_1000_rpm(MySpi& spi) {
  tmc9660::TMC9660<MySpi> driver(spi);

  tmc9660::BootloaderConfig cfg{};                          // 1. boot into parameter mode
  cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
  cfg.boot.start_motor_control = true;
  if (driver.bootloaderInit(&cfg) != tmc9660::TMC9660<MySpi>::BootloaderInitResult::Success) {
    return -1;                                              // wiring, power or SPI mode
  }

  driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);  // 2. motor: 7 pole pairs
  driver.motorConfig.setPWMFrequency(25000);

  const auto ctx = driver.getMotorContext();                // 3. unit conversions from the chip
  driver.velocityControl.setTargetVelocityRpm(1000.0, ctx); // 4. spin
  return 0;
}
// --8<-- [end:quickstart]
