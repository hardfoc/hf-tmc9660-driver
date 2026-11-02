/**
 * @file my_spi.hpp
 * @brief The SPI port shown on the documentation's Porting page, compiled by the host build.
 *
 * Every platform call goes through the four `platform_*` functions declared below; a real port
 * replaces them with its SDK (ESP-IDF, STM32 HAL, Zephyr, Linux spidev). The class is what the
 * driver needs, nothing more.
 */
#pragma once

#include "tmc9660.hpp"

#include <array>
#include <cstdarg>
#include <cstddef>
#include <cstdint>

// Platform hooks: your SDK's calls. Declared only, so the docs build checks the port compiles.
bool platform_spi_transfer(const uint8_t* tx, uint8_t* rx, std::size_t len) noexcept;  // CS low .. CS high
bool platform_gpio_write(int gpio, bool level) noexcept;
bool platform_gpio_read(int gpio, bool& level) noexcept;
void platform_delay_us(uint32_t us) noexcept;

// --8<-- [start:port]
class MySpi : public tmc9660::SpiCommInterface<MySpi> {
public:
  // Physical level of each control pin when ACTIVE: RST, DRV_EN, WAKE, FAULTN.
  MySpi() noexcept : SpiCommInterface<MySpi>(true, true, false, false) {}

  // One 64-bit TMCL datagram, full duplex, SPI mode 3, CS held low for all 8 bytes.
  bool spiTransferTMCL(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept {
    return platform_spi_transfer(tx.data(), rx.data(), tx.size());
  }

  // One 40-bit bootloader datagram (used by bootloaderInit() before parameter mode).
  bool spiTransferBootloader(std::array<uint8_t, 5>& tx, std::array<uint8_t, 5>& rx) noexcept {
    return platform_spi_transfer(tx.data(), rx.data(), tx.size());
  }

  // ACTIVE / INACTIVE to a GPIO level, using the active levels given to the constructor.
  bool gpioSet(tmc9660::TMC9660CtrlPin pin, tmc9660::GpioSignal signal) noexcept {
    return platform_gpio_write(gpioOf(pin), signalToGpioLevel(pin, signal));
  }

  bool gpioRead(tmc9660::TMC9660CtrlPin pin, tmc9660::GpioSignal& signal) noexcept {
    bool level = false;
    if (!platform_gpio_read(gpioOf(pin), level)) {
      return false;
    }
    signal = gpioLevelToSignal(pin, level);
    return true;
  }

  void delayMs(uint32_t ms) noexcept { platform_delay_us(ms * 1000U); }
  void delayUs(uint32_t us) noexcept { platform_delay_us(us); }

  // Optional: route driver logs to your logger (level 0 = error ... 4 = verbose).
  void debugLog(int /*level*/, const char* /*tag*/, const char* /*format*/, va_list /*args*/) noexcept {}

private:
  static int gpioOf(tmc9660::TMC9660CtrlPin pin) noexcept {
    switch (pin) {
    case tmc9660::TMC9660CtrlPin::RST: return 22;
    case tmc9660::TMC9660CtrlPin::DRV_EN: return 20;
    case tmc9660::TMC9660CtrlPin::FAULTN: return 19;
    case tmc9660::TMC9660CtrlPin::WAKE: return 21;
    }
    return -1;
  }
};
// --8<-- [end:port]
