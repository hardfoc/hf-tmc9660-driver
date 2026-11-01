/**
 * @file mock_tmc9660.hpp
 * @brief Host-side mock transports that model the TMC9660 parameter-mode TMCL slave.
 *
 * The SPI mock follows the reference-manual rules the driver relies on:
 * - the reply to datagram k is clocked out with datagram k + 1;
 * - while the chip is "processing" (virtual time < busy-until) every datagram is answered
 *   with NOT_READY (0xF0) and its command is ignored;
 * - optional fault injection replaces a reply with all-zero bytes (status 0x00) while the
 *   datagram's own command is still executed, which is the ambiguous case seen on the bench.
 *
 * Time is virtual: each datagram costs @ref MockTmc9660Spi::frameUs and delayUs() advances
 * the clock, so tests can assert on the configured gap and poll back-off exactly.
 */
#pragma once

#include "tmc9660.hpp"

#include <array>
#include <cstdarg>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace tmc9660_test {

/// Decoded request as the mock chip saw it.
struct SeenFrame {
  uint8_t opcode;
  uint16_t type;
  uint8_t motor;
  uint32_t value;
  bool ignored; ///< Answered NOT_READY and therefore not executed.
};

/// Common TMCL command model shared by the SPI and UART mocks.
class Tmc9660Model {
public:
  std::map<uint32_t, uint32_t> axis;    ///< key = motor << 16 | parameter
  std::map<uint32_t, uint32_t> globals; ///< key = bank << 16 | parameter
  std::map<uint32_t, int> executions;   ///< key = opcode << 24 | motor << 16 | type
  std::vector<SeenFrame> seen;          ///< Every datagram received (ignored ones included)
  uint8_t ramDebugState = 0;
  uint32_t ramDebugCaptured = 0;

  /// Execute one request and return {status, value} of its reply.
  std::pair<uint8_t, uint32_t> execute(const tmc9660::TMCLFrame& f) {
    executions[(static_cast<uint32_t>(f.opcode) << 24) | (static_cast<uint32_t>(f.motor) << 16) |
               f.type]++;
    switch (f.opcode) {
    case 5: // SAP
      axis[key(f.motor, f.type)] = f.value;
      return {100, f.value};
    case 6: // GAP
      return {100, axis[key(f.motor, f.type)]};
    case 9: // SGP
      globals[key(f.motor, f.type)] = f.value;
      return {100, f.value};
    case 10: // GGP
      return {100, globals[key(f.motor, f.type)]};
    case 142: // RamDebug
      if (f.type == 8) {
        return {100, ramDebugState};
      }
      if (f.type == 9) {
        return (f.value < ramDebugCaptured) ? std::pair<uint8_t, uint32_t>{100, 0xA000U + f.value}
                                            : std::pair<uint8_t, uint32_t>{4, 0};
      }
      return {100, 0};
    case 0xFF: // reply-request: not a valid TMCL command
      return {2, 0};
    default:
      return {100, 0};
    }
  }

  [[nodiscard]] int executionsOf(uint8_t opcode, uint16_t type, uint8_t motor = 0) const {
    const auto it = executions.find((static_cast<uint32_t>(opcode) << 24) |
                                    (static_cast<uint32_t>(motor) << 16) | type);
    return it == executions.end() ? 0 : it->second;
  }

  [[nodiscard]] std::vector<SeenFrame> accepted() const {
    std::vector<SeenFrame> out;
    for (const auto& s : seen) {
      if (!s.ignored && s.opcode != 0xFF) {
        out.push_back(s);
      }
    }
    return out;
  }

  static uint32_t key(uint8_t hi, uint16_t lo) {
    return (static_cast<uint32_t>(hi) << 16) | lo;
  }
};

/// SPI transport + chip model.
class MockTmc9660Spi : public tmc9660::SpiCommInterface<MockTmc9660Spi>, public Tmc9660Model {
public:
  MockTmc9660Spi() : tmc9660::SpiCommInterface<MockTmc9660Spi>(true, true, false, false) {}

  // ---- model knobs ----
  uint32_t frameUs = 8;        ///< Virtual cost of one 64-bit datagram.
  uint32_t processUs = 0;      ///< Processing time after an accepted datagram.
  int zeroReplies = 0;         ///< Next N accepted datagrams return all-zero bytes.
  int failTransfersAfter = -1; ///< Bus error from the N-th datagram on (-1 = never).

  // ---- observations ----
  uint64_t nowUs = 0;
  std::vector<uint32_t> delays; ///< Every delayUs() argument, in order.
  uint32_t datagrams = 0;

  bool spiTransferTMCL(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept {
    if (failTransfersAfter >= 0 && static_cast<int>(datagrams) >= failTransfersAfter) {
      return false;
    }
    ++datagrams;
    nowUs += frameUs;
    const tmc9660::TMCLFrame f = tmc9660::TMCLFrame::fromSpi(tx);
    if (tmc9660::tmclChecksum(tx.data(), 7) != tx[7]) {
      rx = {0x00, 0, 0, 0, 0, 0, 0, 0};
      return true;
    }
    if (nowUs < busyUntil_) {
      seen.push_back({f.opcode, f.type, f.motor, f.value, true});
      rx = {0xF0, 0, 0, 0, 0, 0, 0, 0};
      rx[7] = tmc9660::tmclChecksum(rx.data(), 7);
      return true;
    }
    seen.push_back({f.opcode, f.type, f.motor, f.value, false});
    if (zeroReplies > 0) {
      --zeroReplies;
      rx = {0, 0, 0, 0, 0, 0, 0, 0};
    } else {
      rx = pending_;
    }
    const auto [status, value] = execute(f);
    pending_ = {0xFF,
                status,
                f.opcode,
                static_cast<uint8_t>(value >> 24),
                static_cast<uint8_t>(value >> 16),
                static_cast<uint8_t>(value >> 8),
                static_cast<uint8_t>(value),
                0};
    pending_[7] = tmc9660::tmclChecksum(pending_.data(), 7);
    busyUntil_ = nowUs + processUs;
    return true;
  }

  bool spiTransferBootloader(std::array<uint8_t, 5>&, std::array<uint8_t, 5>& rx) noexcept {
    rx = {};
    return true;
  }
  bool gpioSet(tmc9660::TMC9660CtrlPin, tmc9660::GpioSignal) noexcept {
    return true;
  }
  bool gpioRead(tmc9660::TMC9660CtrlPin, tmc9660::GpioSignal& s) noexcept {
    s = tmc9660::GpioSignal::INACTIVE;
    return true;
  }
  void debugLog(int, const char*, const char*, va_list) noexcept {}
  void delayMs(uint32_t ms) noexcept {
    delayUs(ms * 1000U);
  }
  void delayUs(uint32_t us) noexcept {
    delays.push_back(us);
    nowUs += us;
  }

  void clearObservations() {
    seen.clear();
    delays.clear();
    executions.clear();
    datagrams = 0;
    resetTmclStats();
  }

private:
  std::array<uint8_t, 8> pending_{0x0C, 0, 0, 0, 0, 0, 0, 0x0C}; // FIRST_CMD after reset
  uint64_t busyUntil_ = 0;
};

/// UART transport + chip model (request/reply, no pipelining).
class MockTmc9660Uart : public tmc9660::UartCommInterface<MockTmc9660Uart>, public Tmc9660Model {
public:
  MockTmc9660Uart() : tmc9660::UartCommInterface<MockTmc9660Uart>(true, true, false, false) {}

  bool uartSendTMCL(const std::array<uint8_t, 9>& d) noexcept {
    tmc9660::TMCLFrame f{};
    f.opcode = d[1];
    f.type = static_cast<uint16_t>(d[2] | ((d[3] & 0xF0U) << 4));
    f.motor = d[3] & 0x0FU;
    f.value = (static_cast<uint32_t>(d[4]) << 24) | (static_cast<uint32_t>(d[5]) << 16) |
              (static_cast<uint32_t>(d[6]) << 8) | d[7];
    seen.push_back({f.opcode, f.type, f.motor, f.value, false});
    const auto [status, value] = execute(f);
    reply_ = {0x02,
              static_cast<uint8_t>(d[0] & 0xFEU),
              status,
              f.opcode,
              static_cast<uint8_t>(value >> 24),
              static_cast<uint8_t>(value >> 16),
              static_cast<uint8_t>(value >> 8),
              static_cast<uint8_t>(value),
              0};
    reply_[8] = tmc9660::tmclChecksum(reply_.data(), 8);
    return true;
  }
  bool uartReceiveTMCL(std::array<uint8_t, 9>& d) noexcept {
    d = reply_;
    return true;
  }
  bool uartTransferBootloader(const std::array<uint8_t, 8>&, std::array<uint8_t, 8>& rx) noexcept {
    rx = {};
    return true;
  }
  bool gpioSet(tmc9660::TMC9660CtrlPin, tmc9660::GpioSignal) noexcept {
    return true;
  }
  bool gpioRead(tmc9660::TMC9660CtrlPin, tmc9660::GpioSignal& s) noexcept {
    s = tmc9660::GpioSignal::INACTIVE;
    return true;
  }
  void debugLog(int, const char*, const char*, va_list) noexcept {}
  void delayMs(uint32_t) noexcept {}
  void delayUs(uint32_t) noexcept {}

private:
  std::array<uint8_t, 9> reply_{};
};

} // namespace tmc9660_test
