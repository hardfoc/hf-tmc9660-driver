/**
 * @file my_uart.hpp
 * @brief The UART port shown on the documentation's Porting page, compiled by the host build.
 */
#pragma once

#include "my_spi.hpp"  // platform_gpio_*, platform_delay_us

bool platform_uart_write(const uint8_t* data, std::size_t len) noexcept;
// Blocks until exactly `len` bytes arrived or the timeout expired; false on timeout.
bool platform_uart_read_exact(uint8_t* data, std::size_t len, uint32_t timeout_ms) noexcept;
void platform_uart_flush_rx() noexcept;

// --8<-- [start:port]
class MyUart : public tmc9660::UartCommInterface<MyUart> {
public:
  MyUart() noexcept : UartCommInterface<MyUart>(true, true, false, false) {}

  bool uartSendTMCL(const std::array<uint8_t, 9>& data) noexcept {
    return platform_uart_write(data.data(), data.size());
  }

  // Return true only once all 9 reply bytes are in `data`.
  bool uartReceiveTMCL(std::array<uint8_t, 9>& data) noexcept {
    return platform_uart_read_exact(data.data(), data.size(), 20);
  }

  bool uartTransferBootloader(const std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept {
    platform_uart_flush_rx();
    return platform_uart_write(tx.data(), tx.size()) && platform_uart_read_exact(rx.data(), rx.size(), 20);
  }

  bool gpioSet(tmc9660::TMC9660CtrlPin pin, tmc9660::GpioSignal signal) noexcept {
    return platform_gpio_write(static_cast<int>(pin), signalToGpioLevel(pin, signal));
  }

  bool gpioRead(tmc9660::TMC9660CtrlPin pin, tmc9660::GpioSignal& signal) noexcept {
    bool level = false;
    if (!platform_gpio_read(static_cast<int>(pin), level)) {
      return false;
    }
    signal = gpioLevelToSignal(pin, level);
    return true;
  }

  void delayMs(uint32_t ms) noexcept { platform_delay_us(ms * 1000U); }
  void delayUs(uint32_t us) noexcept { platform_delay_us(us); }
  void debugLog(int, const char*, const char*, va_list) noexcept {}
};
// --8<-- [end:port]
