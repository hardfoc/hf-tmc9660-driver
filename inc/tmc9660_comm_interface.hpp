/**
 * @file tmc9660_comm_interface.hpp
 * @brief Communication interfaces for TMC9660 Parameter Mode devices using TMCL protocol over SPI
 *
 * @defgroup TMC9660_CommInterface Communication Interfaces
 * @brief Core communication interface classes and protocols
 *
 * @defgroup TMC9660_TMCLProtocol TMCL Protocol Structures
 * @brief TMCL command and reply structures for SPI/UART communication
 *
 * @defgroup TMC9660_GPIOControl GPIO Control Interface
 * @brief GPIO pin control and signal management
 *
 * @defgroup TMC9660_CommTypes Type Definitions
 * @brief Enums and type definitions for communication interfaces
 *
 * For parameter read/write access, either UART or SPI may be used. Both interfaces share the same
 * TMCL command structure and follow a strict command/reply order.
 *
 * ## SPI Command Format
 * Table 3: Command format for parameter read/write access through SPI
 * | BYTE | 0         | 1-2       | 3          | 4-6       | 7        |
 * |------|-----------|-----------|------------|-----------|----------|
 * | Bits | 0-7       | 8-19      | 20-23      | 24-55     | 56-63    |
 * | Desc | Operation | Type      | Motor/Bank | Data      | Checksum |
 *
 * ## SPI Reply Format
 * Table 4: Reply format for parameter read/write access through SPI
 * | BYTE | 0          | 1          | 2          | 3-6       | 7        |
 * |------|------------|------------|------------|-----------|----------|
 * | Bits | 0-7        | 8-15       | 16-23      | 24-55     | 56-63    |
 * | Desc | SPI Status | TMCL Status| Operation  | Data      | Checksum |
 *
 * Datasheet bit rows show bits 8–19 / 20–23 across bytes 1–2 (like Table 3’s 12+4 layout). Observed
 * parameter-mode replies are **octet-aligned**: byte 1 = TMCL status, byte 2 = echoed opcode
 * (e.g. `FF 64 03…` = SPI OK, TMCL 100, echo MST=3). Do **not** fold byte 2’s low nibble into TMCL
 * status — that would turn 100 into `0x364` and break `isOK()`.
 *
 * ## UART Command Format
 * | BYTE | 0               | 1        | 2-3       | 4          | 5-8       | 9        |
 * |------|----------------|----------|-----------|------------|-----------|----------|
 * | Desc | Sync+Address   | Command  | Type      | Motor/Bank | Data      | Checksum |
 *
 * ## UART Reply Format
 * | BYTE | 0            | 1             | 2          | 3          | 4-7       | 8        |
 * |------|--------------|---------------|------------|------------|-----------|----------|
 * | Desc | Host Address | Sync+Address  | TMCL Status| Operation  | Data      | Checksum |
 *
 * ## GPIO Control Interface
 * The TMC9660 provides several GPIO pins for system control and status monitoring.
 * The interface uses board-agnostic naming and signal levels to accommodate different
 * board implementations (direct connection, inverters, etc.).
 *
 * ### Control Pins (Output from Host)
 * - **RST (Pin 22)**: External System Reset Input
 *   - Device remains in reset while this pin is in its active state
 *   - Has internal pull-down resistor
 *   - Active level depends on board implementation (configure via set_pin_active_level)
 *
 * - **DRV_EN (Pin 21)**: Driver Enable Input
 *   - Enables the motor driver outputs
 *   - Has internal pull-down resistor
 *   - Active level depends on board implementation (configure via set_pin_active_level)
 *
 * - **WAKE (Pin 19)**: Wake Input
 *   - Drive to active state to enable power-up and exit from hibernation mode
 *   - When not shorted to VSA, external pull-down resistor recommended
 *   - Active level depends on board implementation (configure via set_pin_active_level)
 *
 * ### Status Pins (Input to Host)
 * - **FAULTN (Pin 20)**: FAULT Output Signal (open drain)
 *   - Indicates busy state during bootstrapping or severe error
 *   - Active level depends on board implementation (configure via set_pin_active_level)
 *   - Use gpioRead(TMC9660CtrlPin::FAULTN, signal) to check fault status
 * 
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 */
#pragma once
#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <span>
#include <string>

namespace tmc9660 {

/**
 * @brief Compile-time debug logging control for TMC9660 library
 *
 * Define TMC9660_DISABLE_DEBUG_LOGGING before including this header to completely
 * disable all debug logging. When disabled, all logDebug() calls are optimized out
 * at compile time, including argument evaluation (via macro replacement).
 *
 * Example usage:
 * @code
 * #define TMC9660_DISABLE_DEBUG_LOGGING
 * #include "TMC9660CommInterface.hpp"
 * @endcode
 *
 * Or via compiler flags:
 * @code{.sh}
 * -DTMC9660_DISABLE_DEBUG_LOGGING
 * @endcode
 */
#ifndef TMC9660_DISABLE_DEBUG_LOGGING
// Debug logging enabled - use actual function call
#define TMC9660_LOG_DEBUG(comm_obj, level, tag, ...) (comm_obj).logDebug(level, tag, __VA_ARGS__)
#else
// Debug logging disabled - optimize out completely (arguments not evaluated)
#define TMC9660_LOG_DEBUG(comm_obj, level, tag, ...) ((void)0)
#endif

/**
 * @brief Log raw SPI TMCL 8-byte TX/RX frames (`[TMCL TX n]`, `[TMCL RX n]`).
 *
 * Default **0** (off): keeps serial usable for bench results. Set to **1** locally or via
 * `-DTMC9660_LOG_TMCL_RAW_FRAMES=1` when you need wire-level bring-up traces again.
 */
#ifndef TMC9660_LOG_TMCL_RAW_FRAMES
#define TMC9660_LOG_TMCL_RAW_FRAMES 0
#endif

#if TMC9660_LOG_TMCL_RAW_FRAMES
#define TMC9660_LOG_TMCL_RAW_FRAME(comm_obj, level, tag, ...) TMC9660_LOG_DEBUG(comm_obj, level, tag, __VA_ARGS__)
#else
#define TMC9660_LOG_TMCL_RAW_FRAME(comm_obj, level, tag, ...) ((void)0)
#endif

/**
 * @brief Supported physical communication modes for TMC9660.
 *
 * Defines the available communication interfaces that can be used
 * to communicate with the TMC9660 motor driver.
 */
enum class CommMode {
  SPI, ///< SPI (Serial Peripheral Interface) mode - 4-wire synchronous communication
  UART ///< UART (Universal Asynchronous Receiver-Transmitter) mode - 2-wire asynchronous
       ///< communication
};

/**
 * @brief TMC9660 control pin identifiers with board-agnostic naming.
 *
 * These pin identifiers abstract the physical pin assignments to provide
 * a consistent interface regardless of board implementation (direct connection,
 * inverters, level shifters, etc.).
 */
enum class TMC9660CtrlPin {
  RST,    ///< External System Reset Input (pin 22) - Active high reset signal
  DRV_EN, ///< Driver enable input (pin 21) - Enables motor driver outputs
  FAULTN, ///< FAULT output signal (pin 20) - Open drain fault indication
  WAKE    ///< Wake input (pin 19) - Wake from hibernation mode
};

/**
 * @brief GPIO signal states with board-agnostic naming.
 *
 * These signal states abstract the physical voltage levels to provide
 * a consistent interface regardless of board implementation (active-high
 * or active-low signals, inverters, etc.).
 */
enum class GpioSignal {
  INACTIVE = 0, ///< Inactive signal state (logical low)
  ACTIVE = 1    ///< Active signal state (logical high)
};

/**
 * @brief SPI status codes as per TMC9660 Parameter Mode specification.
 *
 * These status codes are returned in the first byte of SPI replies
 * to indicate the communication status and any errors that occurred.
 */
enum class SPIStatus : uint8_t {
  OK = 0xFF,             ///< Operation successful
  CHECKSUM_ERROR = 0x00, ///< Checksum verification failed
  FIRST_CMD = 0x0C,      ///< First command after initialization
  NOT_READY = 0xF0,      ///< System busy, resend datagram
};

/**
 * @brief Timing of the parameter-mode SPI request/reply handshake.
 *
 * @details What the vendor documents (TMC9660 Parameter Mode Reference Manual Rev. 0, section
 * "Serial-Peripheral Interface", Tables 3–5; TMC9660 datasheet "SPI Peripheral — Electrical
 * Characteristics"; both PDFs are in `docs/datasheet/`):
 *
 * - The reply to a 64-bit request is clocked out during the **next** SPI datagram.
 * - `SPI_STATUS_NOT_READY` (0xF0) in byte 0 means the chip is still processing. The manual
 *   says the reply in that datagram is not the reply to the processed command, the command
 *   in that datagram is ignored, and the datagram should simply be re-sent until the status
 *   goes away.
 * - The only inter-datagram timing that is specified is CS_ high time tCSH ≥ 4 × T_CLK
 *   (about 100 ns at the 40 MHz system clock) and fSCK ≤ 10 MHz. No minimum processing time or
 *   turnaround time for a TMCL request is given.
 *
 * The protocol therefore needs **no blind delay**: the ready indication is the NOT_READY status,
 * and polling it is the documented way to wait. The defaults below follow that rule: no frame
 * gap, then re-send with a short exponential back-off (@ref pollIntervalUs doubling up to
 * @ref pollIntervalMaxUs) for at most @ref maxPolls re-sends. With the defaults the worst-case
 * wait is 20+40+80+160 + 56×200 µs ≈ 11.5 ms, the same order as the previous fixed 50 × 200 µs
 * budget, which is only acceptable from thread context.
 *
 * @ref frameGapUs is kept for buses or firmware revisions that need a settle time between the
 * request and the next datagram; set it from a measurement (see the `tmcl_timing` example),
 * not from a guess. Releases before 1.1.0 hard-coded 1000 µs here (see @ref legacy()).
 */
struct TmclSpiTiming {
  /// Wait between the request datagram and the datagram that clocks out its reply [µs].
  uint32_t frameGapUs = 0;
  /// First wait before re-sending a datagram that came back NOT_READY / not yet valid [µs].
  uint32_t pollIntervalUs = 20;
  /// Upper bound for the doubling back-off between re-sends [µs].
  uint32_t pollIntervalMaxUs = 200;
  /// Maximum number of re-sends of one datagram before the access fails.
  uint16_t maxPolls = 60;
  /// Treat a reply whose echoed operation (byte 2) differs from the request as not valid yet.
  /// GetVersion (136) and Boot (242) are exempt because they reply in other formats.
  bool verifyReplyOpcode = true;

  /// Wait before re-send number @p attempt (0-based) [µs].
  [[nodiscard]] constexpr uint32_t pollWaitUs(uint16_t attempt) const noexcept {
    uint32_t step = pollIntervalUs;
    for (uint16_t i = 0; i < attempt && step < pollIntervalMaxUs; ++i) {
      step *= 2U;
    }
    return (step > pollIntervalMaxUs) ? pollIntervalMaxUs : step;
  }

  /// Worst-case accumulated poll wait for one datagram with this configuration [µs].
  [[nodiscard]] constexpr uint32_t worstCasePollWaitUs() const noexcept {
    uint32_t total = 0;
    for (uint16_t i = 0; i < maxPolls; ++i) {
      total += pollWaitUs(i);
    }
    return total;
  }

  /// Settings of releases before 1.1.0: 1000 µs gap, fixed 200 µs × 50 polls, no echo check.
  [[nodiscard]] static constexpr TmclSpiTiming legacy() noexcept {
    TmclSpiTiming t{};
    t.frameGapUs = 1000;
    t.pollIntervalUs = 200;
    t.pollIntervalMaxUs = 200;
    t.maxPolls = 50;
    t.verifyReplyOpcode = false;
    return t;
  }
};

/**
 * @brief Counters for the SPI TMCL transport (diagnostics and the `tmcl_timing` example).
 */
struct TmclSpiStats {
  uint32_t accesses = 0;     ///< Logical TMCL accesses (single, or one batch element).
  uint32_t frames = 0;       ///< 64-bit SPI datagrams clocked, re-sends included.
  uint32_t notReady = 0;     ///< Datagrams answered with NOT_READY (0xF0) and re-sent.
  uint32_t staleReplies = 0; ///< Replies with 0x00 status, bad checksum or wrong opcode echo.
  uint32_t recoveries = 0;   ///< Batch elements re-issued as a single access after a stale reply.
  uint32_t failures = 0;     ///< Accesses that ran out of polls or hit a bus error.
};

/// @brief True if the SPI reply to @p opcode echoes the opcode in byte 2.
[[nodiscard]] constexpr bool tmclReplyEchoesOpcode(uint8_t opcode) noexcept {
  return opcode != 136U /* GetVersion */ && opcode != 242U /* Boot */;
}

/**
 * @brief Calculate 8-bit checksum using sum of bytes method.
 *
 * This function implements the TMCL checksum algorithm which is simply
 * the sum of all bytes in the datagram. The checksum is used to verify
 * data integrity during communication.
 *
 * @param bytes Pointer to the data bytes to checksum
 * @param n Number of bytes to include in the checksum calculation
 * @param start Starting byte offset for checksum calculation (default: 0)
 * @return 8-bit checksum value
 */
static constexpr uint8_t tmclChecksum(const uint8_t* bytes, size_t n, uint8_t start = 0) noexcept {
  uint8_t sum = 0;
  for (size_t i = start; i < n; ++i)
    sum += bytes[i];
  return sum;
}

// -----------------------------------------------------------------------
// TMCL scripting support structures and enums
// -----------------------------------------------------------------------

/**
 * @brief Reply structure returned by TMCL command operations.
 *
 * This structure contains the decoded reply from a TMCL command, including
 * SPI status, TMCL status, operation code, and the returned value. The structure
 * also stores raw bytes for advanced parsing and provides helper methods for
 * extracting version strings and handling protocol mismatches.
 */
struct TMCLReply {
  SPIStatus spi_status = SPIStatus::OK; ///< SPI status byte
  uint8_t status = 0;                  ///< TMCL status code (100=OK, 101=LOADED)
  uint8_t opcode = 0;                  ///< Echoed operation code
  uint32_t value = 0;                  ///< Optional returned value

  // ✅ NEW: Raw bytes for advanced parsing (handle protocol mismatches)
  std::array<uint8_t, 8> rawBytes =
      {}; ///< Raw SPI bytes (8-byte) or UART bytes mapped to 8-byte format

  [[nodiscard]] bool isOK() const noexcept {
    return spi_status == SPIStatus::OK && (status == 100 || status == 101);
  }

  /**
   * @brief Extract 32-bit value from raw bytes at specified offset (MSB-first)
   * @param offset Starting byte offset (0-4 for 32-bit value)
   * @return 32-bit value extracted from rawBytes[offset..offset+3]
   *
   * Useful for extracting misaligned data when protocol mismatch occurs
   * (e.g., bootloader 5-byte SESSION_START parsed as 8-byte TMCL).
   */
  [[nodiscard]] uint32_t extractRawValue(size_t offset) const noexcept {
    if (offset + 3 >= rawBytes.size())
      return 0;
    return (static_cast<uint32_t>(rawBytes[offset]) << 24) |
           (static_cast<uint32_t>(rawBytes[offset + 1]) << 16) |
           (static_cast<uint32_t>(rawBytes[offset + 2]) << 8) |
           static_cast<uint32_t>(rawBytes[offset + 3]);
  }

  /**
   * @brief Extract version string from GetVersion reply (Command 136, Type 0)
   * @return Version string (up to 8 characters) or empty string if not a version reply
   *
   * GetVersion with Type=0 returns: [Host Address][Version String 8 chars]
   * This method extracts the version string portion
   */
  [[nodiscard]] std::string getVersionString() const noexcept {
    if (opcode != 136)
      return ""; // Not a GetVersion reply

    std::string version;
    // Extract version string from bytes 1-8 (skip host address at byte 0)
    for (size_t i = 1; i < 8; ++i) {
      if (rawBytes[i] >= 0x20 && rawBytes[i] <= 0x7E) { // Printable ASCII
        version += static_cast<char>(rawBytes[i]);
      } else {
        break; // Stop at first non-printable character
      }
    }
    return version;
  }

  /**
   * @brief Decode and validate reply from SPI TMCL datagram.
   *
   * Parses an 8-byte SPI reply, validates checksum, and handles special reply
   * formats such as SESSION_START and GetVersion string replies. Raw bytes are
   * always stored for advanced parsing even if initial parsing fails.
   *
   * @param in Input buffer containing 8 bytes from SPI
   * @param r Output parameter to receive decoded reply
   * @param sent_opcode Optional opcode of command that was sent (for handling special replies)
   * @param sent_type Optional type field of command that was sent (for handling special replies)
   * @return true if reply was successfully decoded and validated
   *
   * @note Handles special cases: SESSION_START (0x13), GetVersion string format
   * @note Raw bytes are always stored for debugging protocol mismatches
   */
  static bool fromSpi(std::span<const uint8_t, 8> in, TMCLReply& r, uint8_t sent_opcode = 0,
                      uint16_t sent_type = 0) noexcept {
    // Store raw bytes for advanced parsing (ALWAYS, even if parsing fails)
    std::copy(in.begin(), in.end(), r.rawBytes.begin());

    // Check for SESSION_START status codes (bootloader or parameter mode)
    uint8_t raw_status = in[0];
    if (raw_status == 0x13) {
      // SESSION_START from bootloader (0x13)
      // These use different protocols, so TMCL checksum will fail
      // But we still want to return success so caller can use rawBytes
      r.spi_status = static_cast<SPIStatus>(raw_status);
      r.status = in[1];
      r.opcode = in[2];
      r.value = (static_cast<uint32_t>(in[3]) << 24) | (static_cast<uint32_t>(in[4]) << 16) |
                (static_cast<uint32_t>(in[5]) << 8) | static_cast<uint32_t>(in[6]);
      return true; // Allow SESSION_START even if checksum fails
    }

    // Check for GetVersion string format (Command 136, Type 0)
    // GetVersion with Type=0 returns a special string format without checksum
    // Format: [Host Address][Version String 8 chars]
    if (sent_opcode == 136 && sent_type == 0) {
      // We sent GetVersion Type=0, expect string format reply (no checksum validation)
      r.spi_status = SPIStatus::OK; // Assume OK for version string
      r.status = 100;              // REPLY_OK for successful version retrieval
      r.opcode = 136;              // GetVersion command
      r.value = 0;                 // No value field for string format
      return true;                 // Success - skip checksum validation for string format
    }

    // Standard TMCL reply validation
    if (tmclChecksum(in.data(), 7) != in[7])
      return false;
    r.spi_status = static_cast<SPIStatus>(in[0]);
    if (r.spi_status == SPIStatus::NOT_READY || r.spi_status == SPIStatus::CHECKSUM_ERROR)
      return false;
    r.status = in[1];
    r.opcode = in[2];
    r.value = (static_cast<uint32_t>(in[3]) << 24) | (static_cast<uint32_t>(in[4]) << 16) |
              (static_cast<uint32_t>(in[5]) << 8) | static_cast<uint32_t>(in[6]);
    return true;
  }

  /**
   * @brief Decode and validate reply from UART TMCL datagram.
   *
   * Parses a 9-byte UART reply, validates module address and checksum, and handles
   * special reply formats. UART replies include host address and sync bits not present
   * in SPI format.
   *
   * @param in Input buffer containing 9 bytes from UART
   * @param addr Expected 7-bit module address for address validation
   * @param r Output parameter to receive decoded reply
   * @param sent_opcode Optional opcode of command that was sent (for handling special replies)
   * @param sent_type Optional type field of command that was sent (for handling special replies)
   * @return true if reply was successfully decoded and validated
   *
   * @note UART format: [HOST_ADDR] [SYNC+ADDR] [STATUS] [OPCODE] [VALUE(32)] [CRC8]
   * @note Validates module address and checksum before returning success
   */
  static bool fromUart(std::span<const uint8_t, 9> in, uint8_t addr, TMCLReply& r,
                       uint8_t sent_opcode = 0, uint16_t sent_type = 0) noexcept {
    // Store raw bytes (map 9-byte UART to 8-byte format, skip first byte)
    std::copy(in.begin() + 1, in.end(), r.rawBytes.begin());

    // UART Reply Format (per datasheet):
    // byte0: Host Address (8 bits)
    // byte1: Sync bit (bit 0) + Module Address (bits 1-7)
    // byte2: TMCL Status
    // byte3: Operation
    // byte4-7: Data (big-endian)
    // byte8: Checksum

    // Check for GetVersion string format (Command 136, Type 0)
    // GetVersion with Type=0 returns a special string format without valid checksum
    // Format: [Host Address][Sync+Address][Version String 7 chars]
    if (sent_opcode == 136 && sent_type == 0) {
      // We sent GetVersion Type=0, expect string format reply (no checksum validation)
      r.spi_status = SPIStatus::OK; // Assume OK for version string
      r.status = 100;              // REPLY_OK for successful version retrieval
      r.opcode = 136;              // GetVersion command
      r.value = 0;                 // No value field for string format
      return true;                 // Success - skip checksum validation for string format
    }

    // Verify module address in byte 1 (bits 1-7)
    // "The module address reuses the upper 7 bits of the bootloader device address"
    // Compare the upper 7 bits of byte 1 with the upper 7 bits of the address
    if ((in[1] & 0xFE) != (addr & 0xFE))
      return false;

    // Verify checksum over first 8 bytes
    if (tmclChecksum(in.data(), 8) != in[8])
      return false;

    r.spi_status = SPIStatus::OK; // UART has no SPI status field
    r.status = in[2];            // TMCL Status at byte 2
    r.opcode = in[3];            // Operation at byte 3
    r.value = (static_cast<uint32_t>(in[4]) << 24) | (static_cast<uint32_t>(in[5]) << 16) |
              (static_cast<uint32_t>(in[6]) << 8) | static_cast<uint32_t>(in[7]);
    return true;
  }
};

/**
 * @brief Frame structure for TMCL commands.
 *
 * Supports conversion to/from SPI (8 bytes) and UART (9 bytes) formats.
 */
struct TMCLFrame {
  uint8_t opcode = 0; ///< Operation code field (BYTE 0, bits 0-7).
  uint16_t type = 0;  ///< 12-bit parameter / command type (see `toSpi` / `toUart`).
  uint8_t motor = 0;  ///< 4-bit motor or bank index (packed with `type[11:8]` per transport).
  uint32_t value = 0; ///< 32-bit data value (big-endian in bytes 3-6 on SPI, 4-7 on UART).

  /**
   * @brief Serialize frame into 8-byte SPI buffer.
   * @param out Span of 8 bytes: opcode, type[7:0], merged type[11:8]|motor, value[31:0], checksum.
   *
   * @note Bytes 1–2 use the **same 12-bit TYPE + 4-bit MOTOR nibble packing** as UART bytes 2–3
   *       (`TMC9660.c::sendRequestUART` / `toUart`): `type[11:8]` in the **high** nibble of the
   *       third frame byte, `motor[3:0]` in the **low** nibble. A legacy implementation swapped
   *       those nibbles for SPI only; that made every axis GAP/SAP with NR ≥ 256 return
   *       `REPLY_WRONG_TYPE` on SPI while UART succeeded (bench: 72 extra SPI failures in the
   *       Table 57 sweep for `motor == 0`).
   */
  void toSpi(std::span<uint8_t, 8> out) const noexcept {
    out[0] = opcode;
    out[1] = static_cast<uint8_t>(type & 0xFF);
    out[2] = static_cast<uint8_t>(((type & 0xF00u) >> 4) | (motor & 0x0Fu));
    out[3] = static_cast<uint8_t>(value >> 24);
    out[4] = static_cast<uint8_t>(value >> 16);
    out[5] = static_cast<uint8_t>(value >> 8);
    out[6] = static_cast<uint8_t>(value);
    out[7] = tmclChecksum(out.data(), 7);
  }

  /**
   * @brief Serialize frame into 9-byte UART buffer, including sync bit and checksum.
   * @param addr 7-bit module address; MSB (sync bit) set automatically.
   * @param out Span of 9 bytes to fill: sync+addr, fields, checksum.
   */
  void toUart(uint8_t addr, std::span<uint8_t, 9> out) const noexcept {
    // Byte 0: [Sync bit (bit 0)] [Module Address (bits 1-7)]
    // "The module address reuses the upper 7 bits of the bootloader device address"
    // This means we take bits 7-1 of the address and place them in bits 7-1 of byte 0
    // Then set bit 0 (sync bit) to 1
    out[0] = (addr & 0xFE) | 0x01; // Keep upper 7 bits of address, set sync bit
    out[1] = opcode;
    // Bytes 2-3: 12-bit TYPE field + 4-bit MOTOR/BANK field, packed exactly like the
    // Trinamic reference (`examples/TMC9660/TMC9660.c::sendRequestUART`):
    //   data[2] = type & 0xFF;                          // type[7:0]
    //   data[3] = (type >> 8) << 4 | (index & 0x0F);    // type[11:8] hi-nibble | motor lo-nibble
    // The previous packing dropped type[11:8] entirely and put motor in the low
    // nibble, which silently aliased every parameter NR >= 256 down into the
    // 0..255 range. That made the gate-driver overcurrent / VGS / supply / temp
    // SAPs (NR 254..299) write the wrong target on the chip, and made the read
    // scan return totally bogus values for NRs 256..334.
    out[2] = static_cast<uint8_t>(type & 0xFFu);
    out[3] = static_cast<uint8_t>(((type & 0xF00u) >> 4) | (motor & 0x0Fu));
    out[4] = static_cast<uint8_t>(value >> 24);
    out[5] = static_cast<uint8_t>(value >> 16);
    out[6] = static_cast<uint8_t>(value >> 8);
    out[7] = static_cast<uint8_t>(value);
    out[8] = tmclChecksum(out.data(), 8);
  }

  /**
   * @brief Deserialize an SPI buffer into a TMCLFrame without status check.
   * @param in Span of 8 received bytes.
   * @return Populated TMCLFrame.
   */
  static TMCLFrame fromSpi(std::span<const uint8_t, 8> in) noexcept {
    TMCLFrame f;
    f.opcode = in[0];
    f.type = static_cast<uint16_t>(in[1] | ((static_cast<uint16_t>(in[2]) & 0xF0u) << 4));
    f.motor = in[2] & 0x0Fu;
    f.value = (static_cast<uint32_t>(in[3]) << 24) | (static_cast<uint32_t>(in[4]) << 16) |
              (static_cast<uint32_t>(in[5]) << 8) | static_cast<uint32_t>(in[6]);
    return f;
  }

  /**
   * @brief Deserialize an SPI buffer with status and checksum validation.
   * @param in Span of 8 received bytes.
   * @param out_frame Reference to store the valid frame.
   * @return false if NOT_READY or CHECKSUM_ERROR or invalid status; true otherwise.
   */
  static bool fromSpiChecked(std::span<const uint8_t, 8> in, TMCLFrame& out_frame) noexcept {
    TMCLReply reply{};
    if (!TMCLReply::fromSpi(in, reply))
      return false;
    out_frame.opcode = reply.opcode;
    out_frame.value = reply.value;
    return true;
  }

  /**
   * @brief Deserialize a UART buffer with address and checksum validation.
   * @param in Span of 9 received bytes.
   * @param expected_addr 7-bit expected address of host.
   * @param out_frame Reference to store the valid frame.
   * @return true if address and checksum match.
   */
  static bool fromUart(std::span<const uint8_t, 9> in, uint8_t expected_addr,
                       TMCLFrame& out_frame) noexcept {
    TMCLReply rep{};
    if (!TMCLReply::fromUart(in, expected_addr, rep))
      return false;
    out_frame.opcode = rep.opcode;
    out_frame.value = rep.value;
    return true;
  }

  /**
   * @brief Calculate 8-bit checksum (sum of bytes).
   * @param bytes Pointer to data bytes to sum.
   * @param n Number of bytes to include in sum.
   * @return 8-bit checksum value.
   */
  static constexpr uint8_t calculateChecksum(const uint8_t* bytes, size_t n) noexcept {
    uint8_t sum = 0;
    for (size_t i = 0; i < n; ++i)
      sum += bytes[i];
    return sum;
  }
};

/**
 * @brief How a received 8-byte SPI datagram relates to the request it should answer.
 */
enum class TmclSpiRxClass : uint8_t {
  NOT_READY, ///< Status 0xF0: chip busy, the datagram just sent was ignored — re-send it.
  VALID,     ///< Well-formed reply to the expected request.
  STALE,     ///< Not a usable reply (0x00 status, bad checksum, FIRST_CMD, wrong opcode echo).
};

/**
 * @brief Classify a received SPI datagram as the reply to a request with @p expected_opcode.
 *
 * @param rx              Received datagram.
 * @param expected_opcode Opcode of the request whose reply @p rx should carry.
 * @param verify_opcode   Require byte 2 to echo @p expected_opcode (see
 *                        @ref TmclSpiTiming::verifyReplyOpcode).
 *
 * Opcodes that do not echo (see @ref tmclReplyEchoesOpcode) and bootloader SESSION_START (0x13)
 * replies are passed through as VALID unless NOT_READY, because their reply format is not
 * a checksummed TMCL reply; @ref TMCLReply::fromSpi decodes them.
 */
[[nodiscard]] constexpr TmclSpiRxClass classifyTmclSpiReply(std::span<const uint8_t, 8> rx,
                                                            uint8_t expected_opcode,
                                                            bool verify_opcode) noexcept {
  if (rx[0] == static_cast<uint8_t>(SPIStatus::NOT_READY)) {
    return TmclSpiRxClass::NOT_READY;
  }
  if (!tmclReplyEchoesOpcode(expected_opcode) || rx[0] == 0x13U) {
    return TmclSpiRxClass::VALID;
  }
  if (rx[0] != static_cast<uint8_t>(SPIStatus::OK)) {
    return TmclSpiRxClass::STALE;
  }
  if (tmclChecksum(rx.data(), 7) != rx[7]) {
    return TmclSpiRxClass::STALE;
  }
  if (verify_opcode && rx[2] != expected_opcode) {
    return TmclSpiRxClass::STALE;
  }
  return TmclSpiRxClass::VALID;
}

/**
 * @brief SPI status codes as per TMC9660 Parameter Mode.
 *
 * Table 4: SPI status codes:
 * - 0xFF: OK (operation successful)
 * - 0x00: CHECKSUM_ERROR
 * - 0x0C: FIRST_CMD (initial response after initialization)
 * - 0xF0: NOT_READY (system busy, resend datagram)
 */
/**
 * @brief CRTP-based communication interface for sending/receiving TMCL datagrams.
 *
 * Defines the common API for higher-level code to send and receive 64-bit TMCL frames
 * (via SPI) or 72-bit (via UART) without knowledge of the underlying transport.
 * Also provides GPIO control interface for TMC9660 control pins.
 *
 * This template class uses the CRTP (Curiously Recurring Template Pattern) for
 * compile-time polymorphism, providing zero runtime overhead compared to virtual functions.
 *
 * Benefits of CRTP:
 * - Compile-time polymorphism (no virtual function overhead)
 * - Static dispatch instead of dynamic dispatch
 * - Better optimization opportunities for the compiler
 * - No vtable overhead
 *
 * Example usage:
 * @code
 * class MySPI : public tmc9660::SpiCommInterface<MySPI> {
 * public:
 *   CommMode mode() const noexcept { return CommMode::SPI; }
 *   bool spiTransferTMCL(...) { ... }
 *   bool gpioSet(...) { ... }
 *   // ... implement other required methods
 * };
 * @endcode
 *
 * @tparam Derived The derived class type (CRTP pattern)
 */
template <typename Derived>
class CommInterface {
public:
  /**
   * @brief Construct communication interface with pin active level configuration
   * @param rst_active_level Physical GPIO level for RST pin when ACTIVE (true=HIGH, false=LOW)
   * @param drv_en_active_level Physical GPIO level for DRV_EN pin when ACTIVE (true=HIGH, false=LOW)
   * @param wake_active_level Physical GPIO level for WAKE pin when ACTIVE (true=HIGH, false=LOW)
   * @param faultn_active_level Physical GPIO level for FAULTN pin when ACTIVE (true=HIGH, false=LOW)
   */
  CommInterface(bool rst_active_level, bool drv_en_active_level, bool wake_active_level,
                       bool faultn_active_level) noexcept
      : pinActiveLevels_{rst_active_level, drv_en_active_level, wake_active_level, faultn_active_level} {}

  /**
   * @brief Get the underlying communication mode used by this interface.
   *
   * Returns whether this interface uses SPI or UART for communication.
   * This information is useful for protocol-specific optimizations or
   * debugging purposes.
   *
   * @return Communication mode (::CommMode::SPI or ::CommMode::UART)
   */
  CommMode mode() const noexcept {
    return static_cast<const Derived*>(this)->mode();
  }

  /**
   * @brief Perform a full duplex TMCL transfer for parameter mode communication.
   *
   * For SPI: Performs two transactions - sends command, receives first reply,
   * then sends second command (or NO_OP if not provided), receives second reply.
   *
   * For UART: Single transaction - sends command, receives reply.
   *
   * @param tx TMCL command frame to transmit
   * @param reply Reference to store the final reply
   * @param address TMC9660 module address (UART only, ignored for SPI)
   * @param first_reply Optional pointer to capture first reply (SPI only)
   * @param second_command Optional second command to send (SPI only, defaults to NO_OP)
   * @return true if transfer succeeded
   * @note This is for TMCL parameter mode communication (8-byte SPI, 9-byte UART)
   */
  bool transferTMCL(const TMCLFrame& tx, TMCLReply& reply, uint8_t address,
                            TMCLReply* first_reply = nullptr,
                            const TMCLFrame* second_command = nullptr) noexcept {
    return static_cast<Derived*>(this)->transferTMCL(tx, reply, address, first_reply, second_command);
  }

  /**
   * @brief Set GPIO pin signal state (output control).
   *
   * Used for controlling TMC9660 control pins like RST, DRV_EN, and WAKE.
   * The implementation must handle the specific hardware GPIO configuration and
   * map ACTIVE/INACTIVE to the appropriate physical levels based on board design.
   *
   * @param pin The TMC9660 control pin to control
   * @param signal The desired signal state (ACTIVE or INACTIVE)
   * @return true if the GPIO was set successfully, false otherwise
   */
  bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept {
    return static_cast<Derived*>(this)->gpioSet(pin, signal);
  }

  /**
   * @brief Read GPIO pin signal state (input state).
   *
   * Used for reading TMC9660 status pins like FAULTN.
   * The implementation must handle the specific hardware GPIO configuration and
   * map physical levels to ACTIVE/INACTIVE based on board design.
   *
   * @param pin The TMC9660 control pin to read
   * @param signal Reference to store the current signal state
   * @return true if the GPIO was read successfully, false otherwise
   */
  bool gpioRead(TMC9660CtrlPin pin, GpioSignal& signal) noexcept {
    return static_cast<Derived*>(this)->gpioRead(pin, signal);
  }

  /**
   * @brief Convert signal state to physical GPIO level
   * @param pin The TMC9660 control pin
   * @param signal The signal state (ACTIVE or INACTIVE)
   * @return Physical GPIO level (true=HIGH, false=LOW)
   */
  bool signalToGpioLevel(TMC9660CtrlPin pin, GpioSignal signal) const noexcept {
    bool active_level = pinActiveLevels_[static_cast<int>(pin)];
    return (signal == GpioSignal::ACTIVE) ? active_level : !active_level;
  }

  /**
   * @brief Convert physical GPIO level to signal state
   * @param pin The TMC9660 control pin
   * @param gpio_level Physical GPIO level (true=HIGH, false=LOW)
   * @return Signal state (ACTIVE or INACTIVE)
   */
  GpioSignal gpioLevelToSignal(TMC9660CtrlPin pin, bool gpio_level) const noexcept {
    bool active_level = pinActiveLevels_[static_cast<int>(pin)];
    return (gpio_level == active_level) ? GpioSignal::ACTIVE : GpioSignal::INACTIVE;
  }

  /**
   * @brief Set GPIO pin to active state (convenience method).
   *
   * @param pin The TMC9660 control pin to set active
   * @return true if the GPIO was set successfully, false otherwise
   */
  bool gpioSetActive(TMC9660CtrlPin pin) noexcept {
    return gpioSet(pin, GpioSignal::ACTIVE);
  }

  /**
   * @brief Set GPIO pin to inactive state (convenience method).
   *
   * @param pin The TMC9660 control pin to set inactive
   * @return true if the GPIO was set successfully, false otherwise
   */
  bool gpioSetInactive(TMC9660CtrlPin pin) noexcept {
    return gpioSet(pin, GpioSignal::INACTIVE);
  }

  /**
   * @brief Configure the active level for a specific pin.
   *
   * This method allows the user to configure which physical GPIO level (HIGH or LOW)
   * corresponds to the ACTIVE state for each pin. This accommodates different board
   * implementations (direct connection, inverters, etc.).
   *
   * @param pin The TMC9660 control pin to configure
   * @param active_level The physical GPIO level that represents ACTIVE state
   * @return true if the configuration was successful, false otherwise
   */
  bool set_pin_active_level(TMC9660CtrlPin pin, bool active_level) noexcept {
    pinActiveLevels_[static_cast<int>(pin)] = active_level;
    return true;
  }

  /**
   * @brief Set the SPI TMCL handshake timing (gap, poll back-off, poll budget, echo check).
   * @param timing New timing; see @ref TmclSpiTiming for what each field means and where the
   *               defaults come from. Takes effect on the next access.
   * @note Has no effect on UART transports.
   */
  void setTmclTiming(const TmclSpiTiming& timing) noexcept {
    tmclTiming_ = timing;
  }

  /** @brief Current SPI TMCL handshake timing. */
  [[nodiscard]] const TmclSpiTiming& tmclTiming() const noexcept {
    return tmclTiming_;
  }

  /** @brief Transport counters since construction or the last @ref resetTmclStats(). */
  [[nodiscard]] const TmclSpiStats& tmclStats() const noexcept {
    return tmclStats_;
  }

  /** @brief Reset the transport counters. */
  void resetTmclStats() noexcept {
    tmclStats_ = TmclSpiStats{};
  }

  /**
   * @brief Set the maximum number of re-sends while the reply is not ready.
   * @param max_retries Re-sends per datagram (0 = fail on the first NOT_READY).
   * @note Kept for compatibility; equivalent to setting @ref TmclSpiTiming::maxPolls.
   */
  void setSpiRetryMaxCount(uint8_t max_retries) noexcept {
    tmclTiming_.maxPolls = max_retries;
  }

  /** @brief Current re-send budget (@ref TmclSpiTiming::maxPolls, saturated to 255). */
  uint8_t getSpiRetryMaxCount() const noexcept {
    return (tmclTiming_.maxPolls > 255U) ? uint8_t{255}
                                         : static_cast<uint8_t>(tmclTiming_.maxPolls);
  }

  /**
   * @brief Set a fixed wait between re-sends while the reply is not ready.
   * @param interval_us Wait in microseconds.
   * @note Kept for compatibility: sets both @ref TmclSpiTiming::pollIntervalUs and
   *       @ref TmclSpiTiming::pollIntervalMaxUs, which disables the back-off.
   */
  void setSpiRetryInterval(uint32_t interval_us) noexcept {
    tmclTiming_.pollIntervalUs = interval_us;
    tmclTiming_.pollIntervalMaxUs = interval_us;
  }

  /** @brief First re-send wait (@ref TmclSpiTiming::pollIntervalUs). */
  uint32_t getSpiRetryInterval() const noexcept {
    return tmclTiming_.pollIntervalUs;
  }

protected:
  /**
   * @brief Pin active level configuration storage.
   *
   * Stores the physical GPIO level (HIGH or LOW) that corresponds to the
   * ACTIVE state for each TMC9660 control pin. This configuration enables
   * board-agnostic pin control by abstracting physical voltage levels.
   *
   * Array indices: [RST, DRV_EN, WAKE, FAULTN]
   *
   * @see ::set_pin_active_level() for configuration
   * @see ::signalToGpioLevel() for usage
   */
  bool pinActiveLevels_[4];

  /**
   * @brief SPI handshake timing.
   *
   * Parameter-mode SPI is serviced by chip firmware; with the FOC engine running it can
   * withhold a reply for several ms (GIM4305 bench), so the default poll budget is about
   * 11.5 ms. That bound is only acceptable from thread context, never from an ISR or a
   * high-priority control task — lower @ref TmclSpiTiming::maxPolls there.
   */
  TmclSpiTiming tmclTiming_{};

  /** @brief Transport counters (see @ref tmclStats()). */
  TmclSpiStats tmclStats_{};

  /**
   * @brief Debug logging function for detailed debugging information.
   *
   * This function allows the TMC9660 driver to output debug information
   * through the communication interface, which can be routed to platform-specific
   * logging systems (e.g., ESP-IDF logging for ESP32).
   *
   * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
   * @param tag Log tag for categorization
   * @param format printf-style format string
   * @param args Variable arguments list
   */
  void debugLog(int level, const char* tag, const char* format, va_list args) noexcept {
    static_cast<Derived*>(this)->debugLog(level, tag, format, args);
  }

public:
  /**
   * @brief Delay execution for specified milliseconds
   * @param ms Milliseconds to delay
   * @note This function should be implemented by the user to provide platform-specific delay
   * @note Used by bootloader operations that require timing (e.g., VDRV voltage drop wait)
   */
  void delayMs(uint32_t ms) noexcept {
    static_cast<Derived*>(this)->delayMs(ms);
  }

  /**
   * @brief Delay execution for specified microseconds
   * @param us Microseconds to delay
   * @note This function should be implemented by the user to provide platform-specific delay
   * @note Used for fine-grained retry timing in SPI communication
   * @note Default implementation converts to milliseconds (may lose precision for < 1ms)
   * @note Platform implementations should provide accurate microsecond delays when possible
   */
  void delayUs(uint32_t us) noexcept {
    static_cast<Derived*>(this)->delayUs(us);
  }

protected:
  /**
   * @brief Protected destructor
   * @note Derived classes can have public destructors
   */
  ~CommInterface() = default;

  // Prevent copying
  CommInterface(const CommInterface&) = delete;
  CommInterface& operator=(const CommInterface&) = delete;

  // Allow moving
  CommInterface(CommInterface&&) = default;
  CommInterface& operator=(CommInterface&&) = default;

public:
  /**
   * @brief Public debug logging wrapper for external classes.
   *
   * This function allows external classes like TMC9660Bootloader to output debug information
   * through the communication interface. Does not append a trailing newline; the platform
   * debugLog() implementation should format one line per call.
   *
   * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
   * @param tag Log tag for categorization
   * @param format printf-style format string
   * @param ... Variable arguments for format string
   *
   * @note When TMC9660_DISABLE_DEBUG_LOGGING is defined, this function becomes a no-op
   *       and all logging code is optimized out at compile time.
   */
#ifndef TMC9660_DISABLE_DEBUG_LOGGING
  void logDebug(int level, const char* tag, const char* format, ...) noexcept {
    va_list args;
    va_start(args, format);
    // Pass @p format through unchanged. Backends (e.g. HardFOC Logger → ESP_LOG) already emit
    // one line per call; appending '\n' here produced a visible blank line after every message.
    debugLog(level, tag, format, args);
    va_end(args);
  }
#else
  // Debug logging disabled - function optimized out completely
  inline void logDebug(int /*level*/, const char* /*tag*/, const char* /*format*/, ...) noexcept {
    // Empty function body - all logging optimized out
  }
#endif
};

/**
 * @brief CRTP-based SPI implementation of TMC9660CommInterface.
 *
 * Uses a 4-wire SPI bus (mode 3) to exchange 64-bit datagrams (8 bytes).
 * Data is sent MSB-first, big-endian. Replies match the previous command; initial reply uses
 * FIRST_CMD status.
 *
 * This template class uses CRTP for compile-time polymorphism, providing zero runtime overhead.
 *
 * Example usage:
 * @code
 * class MySPI : public tmc9660::SpiCommInterface<MySPI> {
 * public:
 *   CommMode mode() const noexcept { return CommMode::SPI; }
 *   bool spiTransferTMCL(...) { ... }
 *   bool spiTransferBootloader(...) { ... }
 *   bool gpioSet(...) { ... }
 *   bool gpioRead(...) { ... }
 *   void debugLog(...) { ... }
 *   void delayMs(...) { ... }
 *   void delayUs(...) { ... }
 * };
 * @endcode
 *
 * @tparam Derived The derived class type (CRTP pattern)
 */
template <typename Derived>
class SpiCommInterface : public CommInterface<Derived> {
public:
  /**
   * @brief Construct SPI communication interface with pin active level configuration
   * @param rst_active_level Physical GPIO level for RST pin when ACTIVE (true=HIGH, false=LOW)
   * @param drv_en_active_level Physical GPIO level for DRV_EN pin when ACTIVE (true=HIGH, false=LOW)
   * @param wake_active_level Physical GPIO level for WAKE pin when ACTIVE (true=HIGH, false=LOW)
   * @param faultn_active_level Physical GPIO level for FAULTN pin when ACTIVE (true=HIGH, false=LOW)
   */
  SpiCommInterface(bool rst_active_level, bool drv_en_active_level, bool wake_active_level,
                       bool faultn_active_level) noexcept
      : CommInterface<Derived>(rst_active_level, drv_en_active_level, wake_active_level, faultn_active_level) {
  }

  /**
   * @brief Get communication mode (always SPI for this interface)
   * @return CommMode::SPI
   */
  CommMode mode() const noexcept {
    return CommMode::SPI;
  }

  /**
   * @brief Low-level SPI transfer for TMCL parameter mode communication.
   * @param tx Buffer containing 8 bytes to transmit (TMCL format).
   * @param rx Buffer to receive 8 bytes from device (TMCL format).
   * @return true if the SPI transfer completed successfully.
   * @note This is for TMCL parameter mode (8-byte frames)
   */
  bool spiTransferTMCL(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept {
    return static_cast<Derived*>(this)->spiTransferTMCL(tx, rx);
  }

  /**
   * @brief Low-level SPI transfer for bootloader communication.
   *
   * The bootloader uses a 40-bit (5-byte) protocol:
   * - TX: [COMMAND(8)] [VALUE(32)]
   * - RX: [STATUS(8)] [VALUE(32)]
   *
   * @param tx Buffer containing 5 bytes to transmit (bootloader format).
   * @param rx Buffer to receive 5 bytes from device (bootloader format).
   * @return true if the SPI transfer completed successfully.
   * @note This is for bootloader mode (5-byte frames)
   */
  bool spiTransferBootloader(std::array<uint8_t, 5>& tx, std::array<uint8_t, 5>& rx) noexcept {
    return static_cast<Derived*>(this)->spiTransferBootloader(tx, rx);
  }

  /**
   * @brief Set GPIO pin signal state for SPI interface.
   * @param pin The TMC9660 control pin to control
   * @param signal The desired signal state
   * @return true if the GPIO was set successfully
   */
  bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept {
    return static_cast<Derived*>(this)->gpioSet(pin, signal);
  }

  /**
   * @brief Read GPIO pin signal state for SPI interface.
   * @param pin The TMC9660 control pin to read
   * @param signal Reference to store the current signal state
   * @return true if the GPIO was read successfully
   */
  bool gpioRead(TMC9660CtrlPin pin, GpioSignal& signal) noexcept {
    return static_cast<Derived*>(this)->gpioRead(pin, signal);
  }

  /// Opcode of the reply-request datagram (`FF 00 00 00 00 00 00 FF`); see @ref transferTMCL().
  static constexpr uint8_t kReplyRequestOpcode = 0xFFU;

  /**
   * @brief Exchange one TMCL datagram over SPI using the two-transaction reply protocol.
   *
   * @details Parameter-mode SPI is pipelined: the reply to a command is not clocked out
   * during that command's own transaction, but during the *next* one. A single logical
   * request therefore costs two 8-byte transactions:
   *
   * 1. **Transaction 1** clocks out @p tx. The bytes received back belong to whatever
   *    command preceded it, and are surfaced through @p first_reply. If they carry
   *    `NOT_READY`, the chip ignored @p tx (reference manual, SPI section), so @p tx is
   *    re-sent; this cannot duplicate a write because the ignored copy was never executed.
   * 2. **Transaction 2** clocks a *reply-request* frame to shift out the reply to @p tx.
   *    Unless @p second_command is supplied this is opcode `0xFF` with all other fields
   *    zero (wire bytes `FF 00 .. 00 FF`), matching ADI's `tmc9660_param_sendCommand_SPI`.
   *
   * Between the two transactions the driver waits @ref TmclSpiTiming::frameGapUs (default
   * 0 µs: the vendor documents no processing time, only the NOT_READY handshake). Transaction 2
   * is then re-sent with a doubling back-off while the reply is NOT_READY or not yet valid
   * (status 0x00, bad checksum, or — with @ref TmclSpiTiming::verifyReplyOpcode — an opcode
   * echo that does not match @p tx), at most @ref TmclSpiTiming::maxPolls times.
   *
   * When @c Derived exposes @c hostSpiTransferTMCLPair() that primitive clocks both
   * transactions inside a single bus reservation (required on shared buses where a peer may
   * re-program the SPI mode between the frames) and receives the gap as an argument.
   *
   * @param tx           TMCL command frame to transmit.
   * @param reply        Receives the reply to @p tx (parsed with @p tx opcode/type context,
   *                     so multi-frame replies such as GetVersion decode correctly).
   * @param first_reply  Optional out-param capturing the reply carried by transaction 1,
   *                     i.e. the answer to the *previous* command. Pass @c nullptr to drop it.
   * @param second_command Optional frame to use in place of the `0xFF` reply-request, letting
   *                     a caller pipeline the next command into transaction 2. Prefer
   *                     @ref transferTMCLBatch(), which also collects that command's reply.
   *                     A transaction 2 that comes back with a stale (not NOT_READY) reply is
   *                     re-sent, so @p second_command must be idempotent.
   * @return true if a well-formed reply to @p tx was parsed; false on bus error, or when the
   *         reply was still not ready or not valid after the poll budget.
   *
   * @warning Do not substitute an all-zero frame for the `0xFF` reply-request. The chip parses
   *          opcode 0 as a real (invalid) command and queues a checksum-error reply while the
   *          requested reply is being shifted out, which corrupts the value bytes.
   * @note The @c address parameter is UART-only and is ignored here.
   */
  bool transferTMCL(const TMCLFrame& tx, TMCLReply& reply, uint8_t /*address*/,
                    TMCLReply* first_reply, const TMCLFrame* second_command) noexcept {
    ++this->tmclStats_.accesses;
    return exchangeSingle(tx, reply, first_reply, second_command);
  }

  /**
   * @brief Pipelined batch: @p requests.size() TMCL accesses in N + 1 SPI datagrams.
   *
   * @details Datagram *k* carries request *k* and clocks out the reply to request *k* − 1;
   * the last datagram is the reply-request `0xFF` that clocks out the reply to request
   * N − 1. Compared with N calls of @ref transferTMCL() this saves N − 1 datagrams and
   * N − 1 handshakes. Correctness rules:
   *
   * - **First element:** the reply carried by datagram 0 belongs to whatever preceded the
   *   batch and is discarded.
   * - **Last element:** its reply is collected with the trailing reply-request datagram.
   * - **NOT_READY:** the datagram just sent was ignored by the chip and is re-sent after the
   *   back-off; no request is skipped or duplicated.
   * - **Stale reply** (status 0x00, bad checksum, wrong opcode echo): it is not known whether
   *   the request in that datagram was executed. The previous request is re-issued as a
   *   single @ref transferTMCL() and the pipeline restarts at the current request, which may
   *   therefore execute twice. Only pass **idempotent** requests (SAP/GAP/SGP/GGP, RamDebug
   *   reads). @ref TmclSpiStats::recoveries counts these events.
   *
   * @param requests  Frames to send, in order.
   * @param replies   Receives one reply per request (same index). Must be at least as long
   *                  as @p requests.
   * @param address   UART-only module address; ignored here (signature parity).
   * @param completed Optional: number of leading elements whose reply is valid when the
   *                  function returns (equals @p requests.size() on success).
   * @return true if every request got a well-formed reply. Each reply still carries its own
   *         TMCL status; check @ref TMCLReply::isOK() per element.
   */
  bool transferTMCLBatch(std::span<const TMCLFrame> requests, std::span<TMCLReply> replies,
                         uint8_t /*address*/, size_t* completed = nullptr) noexcept {
    const size_t n = requests.size();
    if (completed != nullptr) {
      *completed = 0;
    }
    if (replies.size() < n) {
      return false;
    }
    if (n == 0) {
      return true;
    }
    this->tmclStats_.accesses += static_cast<uint32_t>(n);
    const TmclSpiTiming& timing = this->tmclTiming_;
    TMCLFrame reply_request{};
    reply_request.opcode = kReplyRequestOpcode;

    size_t k = 0;        // datagram index: request k, or the reply-request when k == n
    bool discard = true; // datagram k's reply is not needed (k == 0, or after a recovery)
    while (k <= n) {
      std::array<uint8_t, 8> tx_buf{};
      std::array<uint8_t, 8> rx_buf{};
      (k < n ? requests[k] : reply_request).toSpi(tx_buf);
      if (!discard && timing.frameGapUs > 0U) {
        this->delayUs(timing.frameGapUs);
      }
      if (!sendUntilAccepted(tx_buf, rx_buf)) {
        return false;
      }
      if (!discard) {
        const TMCLFrame& prev = requests[k - 1];
        if (classifyTmclSpiReply(rx_buf, prev.opcode, timing.verifyReplyOpcode) ==
            TmclSpiRxClass::VALID) {
          (void)TMCLReply::fromSpi(rx_buf, replies[k - 1], prev.opcode, prev.type);
        } else {
          ++this->tmclStats_.staleReplies;
          ++this->tmclStats_.recoveries;
          if (!exchangeSingle(prev, replies[k - 1], nullptr, nullptr)) {
            return false;
          }
          if (completed != nullptr) {
            *completed = k;
          }
          if (k == n) {
            break;
          }
          discard = true; // the chip now holds the reply-request's reply; re-send request k
          continue;
        }
        if (completed != nullptr) {
          *completed = k;
        }
      }
      discard = false;
      ++k;
    }
    return true;
  }

private:
  /// One datagram, counted.
  bool sendFrame(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept {
    ++this->tmclStats_.frames;
    return spiTransferTMCL(tx, rx);
  }

  /// Send @p tx and re-send it while the chip answers NOT_READY (which means it ignored @p tx).
  bool sendUntilAccepted(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept {
    const TmclSpiTiming& timing = this->tmclTiming_;
    for (uint16_t polls = 0;; ++polls) {
      if (!sendFrame(tx, rx)) {
        ++this->tmclStats_.failures;
        return false;
      }
      if (rx[0] != static_cast<uint8_t>(SPIStatus::NOT_READY)) {
        return true;
      }
      ++this->tmclStats_.notReady;
      if (polls >= timing.maxPolls) {
        ++this->tmclStats_.failures;
        TMC9660_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI_TMCL",
                          "NOT_READY: re-send budget (%u) exhausted", timing.maxPolls);
        return false;
      }
      this->delayUs(timing.pollWaitUs(polls));
    }
  }

  /// Body of @ref transferTMCL() without the access counter (also used for batch recovery).
  bool exchangeSingle(const TMCLFrame& tx, TMCLReply& reply, TMCLReply* first_reply,
                      const TMCLFrame* second_command) noexcept {
    const TmclSpiTiming& timing = this->tmclTiming_;
    std::array<uint8_t, 8> tx_buf{};
    std::array<uint8_t, 8> rx1_buf{};
    std::array<uint8_t, 8> tx2_buf{};
    std::array<uint8_t, 8> rx_buf{};
    tx.toSpi(tx_buf);

    /* Opcode 0xFF, not 0x00 — see the @warning on transferTMCL(). */
    TMCLFrame cmd2{};
    cmd2.opcode = kReplyRequestOpcode;
    if (second_command != nullptr) {
      cmd2 = *second_command;
    }
    cmd2.toSpi(tx2_buf);

    TMC9660_LOG_TMCL_RAW_FRAME(*static_cast<Derived*>(this), 2, "SPI_TMCL",
                               "[TMCL TX 1 ] %02X %02X %02X %02X %02X %02X %02X %02X", tx_buf[0],
                               tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4], tx_buf[5], tx_buf[6],
                               tx_buf[7]);

    // Transactions 1 and 2. Transaction 1 is re-sent while it comes back NOT_READY.
    if constexpr (requires(Derived& d, std::array<uint8_t, 8>& a, uint32_t g) {
                    d.hostSpiTransferTMCLPair(a, a, a, a, g);
                  }) {
      for (uint16_t polls = 0;; ++polls) {
        this->tmclStats_.frames += 2U;
        if (!static_cast<Derived*>(this)->hostSpiTransferTMCLPair(tx_buf, rx1_buf, tx2_buf, rx_buf,
                                                                  timing.frameGapUs)) {
          ++this->tmclStats_.failures;
          return false;
        }
        if (rx1_buf[0] != static_cast<uint8_t>(SPIStatus::NOT_READY)) {
          break;
        }
        ++this->tmclStats_.notReady;
        if (polls >= timing.maxPolls) {
          ++this->tmclStats_.failures;
          return false;
        }
        this->delayUs(timing.pollWaitUs(polls));
      }
    } else {
      if (!sendUntilAccepted(tx_buf, rx1_buf)) {
        return false;
      }
      if (timing.frameGapUs > 0U) {
        this->delayUs(timing.frameGapUs);
      }
      if (!sendFrame(tx2_buf, rx_buf)) {
        ++this->tmclStats_.failures;
        return false;
      }
    }

    TMC9660_LOG_TMCL_RAW_FRAME(*static_cast<Derived*>(this), 2, "SPI_TMCL",
                               "[TMCL RX 1 ] %02X %02X %02X %02X %02X %02X %02X %02X", rx1_buf[0],
                               rx1_buf[1], rx1_buf[2], rx1_buf[3], rx1_buf[4], rx1_buf[5],
                               rx1_buf[6], rx1_buf[7]);

    // Optionally capture first reply (this is the reply to the PREVIOUS command due to SPI delay)
    if (first_reply != nullptr) {
      (void)TMCLReply::fromSpi(rx1_buf, *first_reply);
    }

    // Poll transaction 2 until it carries a valid reply to tx. Only the reply-request (or the
    // caller's second_command) is re-sent, never tx, so a write cannot be duplicated.
    for (uint16_t polls = 0;; ++polls) {
      TMC9660_LOG_TMCL_RAW_FRAME(*static_cast<Derived*>(this), 2, "SPI_TMCL",
                                 "[TMCL RX 2 ] %02X %02X %02X %02X %02X %02X %02X %02X", rx_buf[0],
                                 rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5], rx_buf[6],
                                 rx_buf[7]);
      const TmclSpiRxClass cls = classifyTmclSpiReply(rx_buf, tx.opcode, timing.verifyReplyOpcode);
      if (cls == TmclSpiRxClass::VALID) {
        break;
      }
      if (cls == TmclSpiRxClass::NOT_READY) {
        ++this->tmclStats_.notReady;
      } else {
        ++this->tmclStats_.staleReplies;
      }
      // A well-formed reply that echoes the reply-request opcode means the chip has already
      // answered transaction 2 itself, so the reply to tx was consumed by an earlier (stale)
      // datagram and will not arrive: fail now instead of polling out the whole budget.
      const bool reply_lost =
          cls == TmclSpiRxClass::STALE && second_command == nullptr &&
          tx.opcode != kReplyRequestOpcode && rx_buf[0] == static_cast<uint8_t>(SPIStatus::OK) &&
          rx_buf[2] == kReplyRequestOpcode && tmclChecksum(rx_buf.data(), 7) == rx_buf[7];
      if (reply_lost || polls >= timing.maxPolls) {
        ++this->tmclStats_.failures;
        TMC9660_LOG_DEBUG(*static_cast<Derived*>(this), 1, "SPI_TMCL",
                          "No valid reply after %u re-sends (last SPI status 0x%02X%s)",
                          static_cast<unsigned>(polls), static_cast<unsigned>(rx_buf[0]),
                          reply_lost ? ", reply lost" : "");
        std::copy(rx_buf.begin(), rx_buf.end(), reply.rawBytes.begin());
        reply.spi_status = static_cast<SPIStatus>(rx_buf[0]);
        reply.status = rx_buf[1];
        reply.opcode = rx_buf[2];
        reply.value = (static_cast<uint32_t>(rx_buf[3]) << 24) |
                      (static_cast<uint32_t>(rx_buf[4]) << 16) |
                      (static_cast<uint32_t>(rx_buf[5]) << 8) | static_cast<uint32_t>(rx_buf[6]);
        return false;
      }
      this->delayUs(timing.pollWaitUs(polls));
      if (!sendFrame(tx2_buf, rx_buf)) {
        ++this->tmclStats_.failures;
        return false;
      }
    }

    // Parse reply with command context (for handling special reply formats like GetVersion string)
    return TMCLReply::fromSpi(rx_buf, reply, tx.opcode, tx.type);
  }

protected:
  /**
   * @brief Protected destructor
   * @note Derived classes can have public destructors
   */
  ~SpiCommInterface() = default;

  // Prevent copying
  SpiCommInterface(const SpiCommInterface&) = delete;
  SpiCommInterface& operator=(const SpiCommInterface&) = delete;

  // Allow moving
  SpiCommInterface(SpiCommInterface&&) = default;
  SpiCommInterface& operator=(SpiCommInterface&&) = default;
};

/**
 * @brief CRTP-based UART implementation of TMC9660CommInterface.
 *
 * Uses UART_TXD and UART_RXD signals; supports external transceivers via UART_TXEN.
 * Frames consist of 9 bytes: sync+address, command, type, motor, 4-byte data, checksum.
 * LSB-first transmission; checksum is 8-bit sum of first 8 bytes.
 *
 * This template class uses CRTP for compile-time polymorphism, providing zero runtime overhead.
 *
 * Example usage:
 * @code
 * class MyUART : public tmc9660::UartCommInterface<MyUART> {
 * public:
 *   CommMode mode() const noexcept { return CommMode::UART; }
 *   bool uartSendTMCL(...) { ... }
 *   bool uartReceiveTMCL(...) { ... }
 *   bool uartTransferBootloader(...) { ... }
 *   bool gpioSet(...) { ... }
 *   bool gpioRead(...) { ... }
 *   void debugLog(...) { ... }
 *   void delayMs(...) { ... }
 *   void delayUs(...) { ... }
 * };
 * @endcode
 *
 * @tparam Derived The derived class type (CRTP pattern)
 */
template <typename Derived>
class UartCommInterface : public CommInterface<Derived> {
public:
  /**
   * @brief Construct UART communication interface with pin active level configuration
   * @param rst_active_level Physical GPIO level for RST pin when ACTIVE (true=HIGH, false=LOW)
   * @param drv_en_active_level Physical GPIO level for DRV_EN pin when ACTIVE (true=HIGH, false=LOW)
   * @param wake_active_level Physical GPIO level for WAKE pin when ACTIVE (true=HIGH, false=LOW)
   * @param faultn_active_level Physical GPIO level for FAULTN pin when ACTIVE (true=HIGH, false=LOW)
   */
  UartCommInterface(bool rst_active_level, bool drv_en_active_level, bool wake_active_level,
                       bool faultn_active_level) noexcept
      : CommInterface<Derived>(rst_active_level, drv_en_active_level, wake_active_level, faultn_active_level) {
  }

  /**
   * @brief Get communication mode (always UART for this interface)
   * @return CommMode::UART
   */
  CommMode mode() const noexcept {
    return CommMode::UART;
  }

  /**
   * @brief Send raw 9-byte UART TMCL datagram for parameter mode communication.
   * @param data Array of 9 bytes including sync, fields, and checksum (TMCL format).
   * @return true if transmission succeeded.
   * @note This is for TMCL parameter mode (9-byte frames)
   */
  bool uartSendTMCL(const std::array<uint8_t, 9>& data) noexcept {
    return static_cast<Derived*>(this)->uartSendTMCL(data);
  }

  /**
   * @brief Receive raw 9-byte UART TMCL datagram for parameter mode communication.
   * @param data Array to store 9 received bytes (TMCL format).
   * @return true if reception succeeded.
   * @note This is for TMCL parameter mode (9-byte frames)
   */
  bool uartReceiveTMCL(std::array<uint8_t, 9>& data) noexcept {
    return static_cast<Derived*>(this)->uartReceiveTMCL(data);
  }

  /**
   * @brief Transfer 8-byte UART bootloader datagram (send and receive).
   *
   * The bootloader uses a different 8-byte protocol:
   * - TX: [SYNC(0x55)] [DEVICE_ADDR] [COMMAND] [VALUE(32)] [CRC8]
   * - RX: [SYNC(0x55)] [HOST_ADDR] [STATUS] [VALUE(32)] [CRC8]
   *
   * @param tx Buffer containing 8 bytes to transmit (bootloader format).
   * @param rx Buffer to receive 8 bytes from device (bootloader format).
   * @return true if transfer succeeded.
   * @note This is for bootloader mode (8-byte frames)
   */
  bool uartTransferBootloader(const std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept {
    return static_cast<Derived*>(this)->uartTransferBootloader(tx, rx);
  }

  /**
   * @brief Set GPIO pin signal state for UART interface.
   * @param pin The TMC9660 control pin to control
   * @param signal The desired signal state
   * @return true if the GPIO was set successfully
   */
  bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept {
    return static_cast<Derived*>(this)->gpioSet(pin, signal);
  }

  /**
   * @brief Read GPIO pin signal state for UART interface.
   * @param pin The TMC9660 control pin to read
   * @param signal Reference to store the current signal state
   * @return true if the GPIO was read successfully
   */
  bool gpioRead(TMC9660CtrlPin pin, GpioSignal& signal) noexcept {
    return static_cast<Derived*>(this)->gpioRead(pin, signal);
  }

  /**
   * @brief Exchange one TMCL datagram over UART as a single request/response pair.
   *
   * @details Unlike the SPI path, UART is not pipelined: the reply to @p tx arrives in its
   * own 9-byte frame, so one logical request costs one transaction and no reply-request
   * frame is needed. UART frames carry the module @p address that SPI omits.
   *
   * @param tx      TMCL command frame to transmit.
   * @param reply   Receives the reply to @p tx (parsed with @p tx opcode/type context, so
   *                multi-frame replies such as GetVersion decode correctly).
   * @param address TMC9660 module address to address the frame to.
   * @param first_reply Ignored — exists only to match the SPI signature.
   * @param second_command Ignored — exists only to match the SPI signature.
   * @return true if a well-formed reply for @p address was received and parsed.
   */
  bool transferTMCL(const TMCLFrame& tx, TMCLReply& reply, uint8_t address, TMCLReply* first_reply,
                    const TMCLFrame* second_command) noexcept {
    (void)first_reply;    // SPI-only: UART is not pipelined.
    (void)second_command; // SPI-only: UART is not pipelined.

    std::array<uint8_t, 9> frame;
    tx.toUart(address, frame);
    if (!uartSendTMCL(frame))
      return false;
    if (!uartReceiveTMCL(frame))
      return false;
    // Parse reply with command context (for handling special reply formats like GetVersion string)
    return TMCLReply::fromUart(frame, address, reply, tx.opcode, tx.type);
  }

  /**
   * @brief Batch counterpart of @ref transferTMCL() for API parity with the SPI transport.
   *
   * UART TMCL is strictly request/reply, so there is nothing to pipeline: the requests are
   * sent one after the other and the batch stops at the first transport failure.
   *
   * @param requests  Frames to send, in order.
   * @param replies   One reply per request (same index); at least as long as @p requests.
   * @param address   Module address.
   * @param completed Optional: number of leading elements with a valid reply.
   * @return true if every request got a well-formed reply.
   */
  bool transferTMCLBatch(std::span<const TMCLFrame> requests, std::span<TMCLReply> replies,
                         uint8_t address, size_t* completed = nullptr) noexcept {
    if (completed != nullptr) {
      *completed = 0;
    }
    if (replies.size() < requests.size()) {
      return false;
    }
    for (size_t i = 0; i < requests.size(); ++i) {
      if (!transferTMCL(requests[i], replies[i], address, nullptr, nullptr)) {
        return false;
      }
      if (completed != nullptr) {
        *completed = i + 1;
      }
    }
    return true;
  }

protected:
  /**
   * @brief Protected destructor
   * @note Derived classes can have public destructors
   */
  ~UartCommInterface() = default;

  // Prevent copying
  UartCommInterface(const UartCommInterface&) = delete;
  UartCommInterface& operator=(const UartCommInterface&) = delete;

  // Allow moving
  UartCommInterface(UartCommInterface&&) = default;
  UartCommInterface& operator=(UartCommInterface&&) = default;
};

} // namespace tmc9660
