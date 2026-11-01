/**
 * @file tmc9660.hpp
 * @brief Main TMC9660 motor driver interface and subsystem classes
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 *
 * @defgroup TMC9660_Core Core TMC9660 Driver
 * @brief Main TMC9660 driver class and core functionality
 *
 * @defgroup TMC9660_Subsystems Subsystem Interfaces
 * @brief Specialized subsystem classes for different aspects of motor control
 *
 * @defgroup TMC9660_Types Type Definitions
 * @brief Enums, structs, and type definitions used throughout the driver
 *
 * @defgroup TMC9660_Utilities Utility Functions
 * @brief Helper functions and utilities for TMC9660 operations
 */
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <variant>
#include <vector>

#include "tmc9660_comm_interface.hpp"
#include "tmc9660_version.h"
#include "bootloader/tmc9660_bootloader.hpp"
#include "parameter_mode/tmc9660_param_mode_tmcl.hpp"
#include "tmc9660_units.hpp"
#include "tmc9660_diagnostics.hpp"

namespace tmc9660 {

/**
 * @brief Main class representing a TMC9660 motor driver in Parameter Mode.
 * @ingroup TMC9660_Core
 *
 * The TMC9660 class provides a comprehensive high-level interface for configuring
 * and controlling the TMC9660 motor driver chip. This class abstracts the complex
 * low-level register operations into intuitive, easy-to-use methods for motor
 * control applications.
 *
 * ## Key Features
 *
 * The TMC9660 class supports a wide range of motor control features:
 *
 * - **Motor Type Support**: DC, BLDC/PMSM, and Stepper motors
 * - **Control Algorithms**: Field-Oriented Control (FOC), open-loop, and closed-loop control
 * - **Sensor Integration**: Hall sensors, incremental encoders, SPI encoders
 * - **Protection Systems**: Overcurrent, overtemperature, overvoltage protection
 * - **Communication**: TMCL command-based parameter access
 * - **Scripting**: Execute custom scripts on the device's microcontroller
 * - **Telemetry**: Real-time monitoring of temperature, current, voltage, and position
 *
 * ## Communication Interface
 *
 * The class uses a CRTP-based communication interface for communication, making it completely
 * agnostic to the physical communication layer. This allows the same code to work
 * with SPI, UART, or other communication methods by simply providing the appropriate
 * communication interface implementation.
 *
 * The driver is a template class that takes the communication interface type as a template
 * parameter, providing compile-time polymorphism with zero runtime overhead.
 *
 * ## Parameter Mode Operation
 *
 * All configuration and control is performed by sending TMCL (Trinamic Motion Control
 * Language) commands to read and write parameter values. This provides a standardized
 * interface that is consistent across different Trinamic motor drivers.
 *
 * ## Initialization Requirements
 *
 * @warning **CRITICAL**: Before using any motor control functions, you MUST initialize
 * the bootloader for parameter mode operation:
 *
 * @code
 * tmc9660::BootloaderConfig cfg{};
 * cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;  // Essential!
 * cfg.boot.startMotorControl = true;
 * auto result = driver.bootloaderInit(&cfg);
 * if (result != TMC9660::BootloaderInitResult::Success) {
 *     // Handle initialization failure
 * }
 * @endcode
 *
 * Without proper bootloader initialization, the TMC9660 will not respond to
 * TMCL commands and motor control will not function.
 *
 * ## Usage Example
 *
 * @code
 * // Create communication interface (SPI example)
 * class MySPI : public tmc9660::SpiCommInterface<MySPI> {
 *   // ... implement required methods
 * };
 *
 * MySPI spiComm;
 *
 * // Create TMC9660 driver with template parameter
 * tmc9660::TMC9660<MySPI> driver(spi_comm);
 *
 * // Initialize bootloader
 * tmc9660::BootloaderConfig cfg{};
 * cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
 * cfg.boot.startMotorControl = true;
 * driver.bootloaderInit(&cfg);
 *
 * // Configure motor
 * driver.motor.setType(tmc9660::tmcl::MotorType::BLDC, 7);  // 7 pole pairs
 * driver.motor.setPWMFrequency(20000);  // 20 kHz PWM
 *
 * // Start motor control
 * driver.motor.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_HALL);
 * driver.motor.enable();
 * @endcode
 *
 * @tparam CommType The communication interface type (must inherit from
 *                  SpiCommInterface<CommType> or UartCommInterface<CommType>)
 */
template <typename CommType>
class TMC9660 {
public:
  //================================================================================
  // @name Core Initialization and Management
  // @{
  //================================================================================

  /** @brief Construct a TMC9660 driver instance.
   * @param comm Reference to a user-implemented communication interface (SPI,
   * UART, etc).
   * @param address (Optional) Module address if multiple TMC9660 devices are on
   * one bus. For SPI, this is typically 0.
   * @param bootCfg Optional bootloader configuration; nullptr skips bootloaderInit
   * until the caller invokes it.
   */
  TMC9660(CommType& comm, uint8_t address = 0,
          const tmc9660::BootloaderConfig* bootCfg = nullptr) noexcept;

  /** @brief Destructor for TMC9660, cleans up resources */
  ~TMC9660() noexcept;

  /** @brief Get the communication interface used by this TMC9660 instance.
   * @return Reference to the communication interface (SPI, UART, etc).
   */
  [[nodiscard]] CommType& comm() noexcept {
    return comm_;
  }

  /** @brief TMCL module address used on the transport (SPI typically 0). */
  [[nodiscard]] uint8_t tmclAddress() const noexcept { return address_; }

  //============================================================================
  // @name Device Capabilities (board / bootloader topology)
  // @{
  //============================================================================

  /**
   * @brief Per-device feature topology — declares which optional subsystems are
   *        physically wired and bootloader-enabled on **this** TMC9660 instance.
   *
   * Two of the three flags are auto-populated from `BootloaderConfig` if one was
   * passed to the constructor: `spiEncoder` mirrors `bootCfg.spiEnc.enable`, and
   * `stepDir` mirrors `bootCfg.stepDir.enable`. The third flag (`y2Phase`) is
   * **board topology** — there's no bootloader switch for it; the caller must
   * set it (default = true for backward compatibility).
   *
   * When a feature is `false`, the corresponding API surface (e.g. all
   * `StepDir::*` methods, `FeedbackSense::configureAuto(SpiEncoderConfig)`,
   * Y2 arms inside `GateDriver::configurePowerStageProtection`) returns
   * `false` early and emits a single ERROR-level log line via the CRTP
   * `comm.logDebug()` hook. This converts otherwise-confusing chip-level
   * `REPLY_CMD_NOT_AVAILABLE` (0x06) responses into actionable host-side
   * diagnostics that point at the bootloader / board cause.
   *
   * Pass nullptr for `bootCfg` (or use `setCapabilities(...)`) to opt out
   * of all gating — the driver then trusts the caller to know what's wired.
   */
  struct DeviceCapabilities {
    bool spiEncoder = true; ///< SPI encoder feedback subsystem (NRs 181–201). Auto = bootCfg.spiEnc.enable.
    bool stepDir    = true; ///< Step/Dir input subsystem (NRs 205–209). Auto = bootCfg.stepDir.enable.
    bool y2Phase    = true; ///< Y2 power-stage half-bridge wired on board. Caller-set; not bootloader-derived.
  };

  /** @brief Override the capability flags (e.g. set y2Phase=false on 3-phase-only boards). */
  void setCapabilities(const DeviceCapabilities& caps) noexcept { caps_ = caps; }

  /** @brief Read the active capability flags (auto-derived from bootCfg + any subsequent overrides). */
  [[nodiscard]] const DeviceCapabilities& capabilities() const noexcept { return caps_; }

  //============================================================================
  // @name GPIO Helpers (driver-owned control)
  // @{
  //============================================================================

  /**
   * @brief Drive a host-side control pin.
   * @param pin RST / DRV_EN / FAULTN / WAKE.
   * @param signal Active or inactive level for that pin.
   * @return true if the CRTP GPIO hook accepted the write.
   */
  [[nodiscard]] bool GpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept;
  /**
   * @brief Read a host-side control pin.
   * @param pin RST / DRV_EN / FAULTN / WAKE.
   * @param[out] signal Filled on success.
   * @return true if the CRTP GPIO hook accepted the read.
   */
  [[nodiscard]] bool GpioRead(TMC9660CtrlPin pin, GpioSignal& signal) noexcept;
  /**
   * @brief Drive @p pin to its active level.
   * @param pin RST / DRV_EN / FAULTN / WAKE.
   */
  [[nodiscard]] bool GpioSetActive(TMC9660CtrlPin pin) noexcept;
  /**
   * @brief Drive @p pin to its inactive level.
   * @param pin RST / DRV_EN / FAULTN / WAKE.
   */
  [[nodiscard]] bool GpioSetInactive(TMC9660CtrlPin pin) noexcept;

  // @}

  // @}

  //================================================================================
  // @name Bootloader Management
  // @{
  //================================================================================

  /** @brief Bootloader initialization result codes.
   * @details These indicate the outcome of the bootloaderInit() method.
   */
  enum class BootloaderInitResult {
    Success,  ///< Successfully initialized the bootloader
    NoConfig, ///< No bootloader configuration provided
    Failure   ///< Failed to initialize the bootloader
  };

  /** @brief Complete bootloader initialization and transition to parameter mode.
   * @details This method performs the complete initialization sequence:
   *          1. Hardware reset (if performReset =true)
   *          2. Mode detection (bootloader vs parameter)
   *          3. Bootloader configuration (if needed)
   *          4. Motor control startup (if startMotorControl =true)
   *          5. SESSION_START consumption (if startMotorControl =true)
   *          6. TMCL communication verification (if startMotorControl =true)
   *
   * @param cfg Bootloader configuration. MUST set cfg.boot.boot_mode = BootMode::Parameter
   *            for motor control functionality. If nullptr, uses configuration
   *            provided during construction.
   * @param perform_reset If true (default), performs hardware reset sequence (RST pin toggle
   *                     + FAULTN monitoring) to ensure chip enters bootloader mode.
   *                     Set to false if you've already performed reset externally.
   * @param retrieve_bootloader_info If true, retrieves and logs all available bootloader
   *                               information (version, features, git info, etc) for debugging.
   * @param fail_on_verify_error If true (default), initialization fails on read-back verification
   *                          errors. If false, logs warnings but continues despite verification
   *                          failures (useful for debugging or when some configs are expected to
   * fail).
   * @return BootloaderInitResult indicating success, no config, or failure.
   *
   * @warning This method MUST be called successfully before any motor control operations.
   *
   * @note **Complete Initialization (Recommended):**
   *       Set cfg.boot.startMotorControl=true for a fully initialized, communication-verified
   *       chip ready for motor control commands.
   *
   * @note **Bootloader-Only Initialization:**
   *       Set cfg.boot.startMotorControl=false if you need to stay in bootloader mode
   *       (e.g., for firmware flashing or custom configuration).
   *
   * @note Typical usage:
   * @code
   * tmc9660::BootloaderConfig cfg{};
   * cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
   * cfg.boot.startMotorControl = true;  // Start motor control after configuration
   *
   * // Complete initialization (recommended)
   * if (driver.bootloaderInit(&cfg) != TMC9660::BootloaderInitResult::Success) {
   *     // Handle initialization failure
   * }
   * // Driver is now ready for motor control commands!
   *
   * // Bootloader-only initialization
   * cfg.boot.startMotorControl = false;  // Stay in bootloader mode
   * if (driver.bootloaderInit(&cfg) != TMC9660::BootloaderInitResult::Success) {
   *     // Handle initialization failure
   * }
   * // Chip is in bootloader mode, call bootloader_->startMotorControl() later
   * @endcode
   */
  TMC9660::BootloaderInitResult bootloaderInit(const tmc9660::BootloaderConfig* cfg = nullptr,
                                               bool performReset = true,
                                               bool retrieveBootloaderInfo = false,
                                               bool failOnVerifyError = true) noexcept;

  /** @brief Get direct access to the bootloader instance.
   *
   * Allows advanced users to send custom bootloader commands after using
   * applyConfiguration() with startMotorControl =false.
   *
   * @return Pointer to bootloader instance, or nullptr if not initialized or
   *         interface mode doesn't support bootloader (e.g., not SPI/UART).
   *
   * @note Typical usage pattern:
   * @code
   * // 1. Apply configuration without starting motor control
   * tmc9660::BootloaderConfig cfg{};
   * cfg.boot.startMotorControl = false;  // Stay in bootloader
   * driver.bootloaderInit(&cfg);
   *
   * // 2. Get bootloader access for custom operations
   * auto* bootloader = driver.getBootloader();
   * if (bootloader) {
   *     // Program OTP
   *     bootloader->setBank(1);
   *     bootloader->write32Inc(otp_data);
   *     bootloader->otpBurn(0, 4);
   *
   *     // When done, start motor control
   *     bootloader->startMotorControl();
   * }
   * @endcode
   *
   * @warning Only call bootloader methods BEFORE calling start_motor_control().
   *          After start_motor_control(), the bootloader exits and commands will fail.
   */
  tmc9660::TMC9660Bootloader<CommType>* getBootloader() noexcept {
    return bootloader_.get();
  }

  /** @brief Exit parameter mode and return to bootloader mode.
   *
   * This sends the special Boot command (0xF2 / 242) with magic values to trigger
   * the motor control system to exit and return to bootloader mode.
   *
   * @return true if command sent successfully
   * @note After this command, wait 100-200ms for transition to complete
   * @note The chip will be in bootloader mode after transition
   * @note You can then use bootloader commands for reconfiguration
   *
   * @code{.cpp}
   * // Return to bootloader from parameter mode
   * driver.enterBootloaderMode();
   * vTaskDelay(pdMS_TO_TICKS(150));  // Wait for transition
   *
   * // Now use bootloader commands
   * auto* bootloader = driver.getBootloader();
   * bootloader->setBank(5);
   * bootloader->setAddress(0x00020018);
   * bootloader->write32(new_clock_config);
   * bootloader->startMotorControl();  // Return to motor control
   * @endcode
   */
  bool enterBootloaderMode() noexcept {
    // Boot command with magic values per datasheet: TYPE=0x81, MOTOR/BANK=0x92, VALUE=0xA3B4C5D6
    return sendCommand(tmc9660::tmcl::Op::Boot, 0x81, 0x92, 0xA3B4C5D6, nullptr);
  }

  // @}

  //================================================================================
  // @name Core Parameter Access
  // @{
  //================================================================================

  /** @brief Set (write) an axis (motor-specific) parameter on the TMC9660.
   * @param id Parameter ID number (see TMC9660 documentation for the full
   * list).
   * @param value 32-bit value to write to the parameter.
   * @param motor_index Index of the motor/axis (0 or 1). Typically 0 unless the
   * device controls multiple axes.
   * @return true if the parameter was successfully written (acknowledged by the
   * device), false if an error occurred.
   */
  [[nodiscard]] bool writeParameter(tmc9660::tmcl::Parameters id, uint32_t value,
                                    uint8_t motorIndex = 0) noexcept;

  /** @brief Read an axis (motor-specific) parameter from the TMC9660.
   * @param id Parameter ID number to read.
   * @param[out] value Reference to store the 32-bit parameter value read.
   * @param motor_index Index of the motor/axis (0 or 1).
   * @return true if the parameter was successfully read (device responded),
   * false on error.
   */
  [[nodiscard]] bool readParameter(tmc9660::tmcl::Parameters id, uint32_t& value,
                                   uint8_t motorIndex = 0) noexcept;

  /** @brief Read a global parameter via TMCL GGP (TYPE = parameter number, MOTOR/BANK = bank 0/2/3).
   */
  [[nodiscard]] bool readGlobal(uint16_t parameterId, uint8_t bank, uint32_t& value) noexcept;

  /** @brief Write a global parameter via TMCL SGP. */
  [[nodiscard]] bool writeGlobal(uint16_t parameterId, uint8_t bank, uint32_t value) noexcept;

  /** @brief Axis parameter ids are not global parameter ids (removed; was a bank-aliasing trap).
   *  Use readParameter() for axis parameters or a GlobalParamBank0/2/3 id for globals. */
  bool readGlobal(tmc9660::tmcl::Parameters id, uint8_t bank, uint32_t& value) = delete;

  /** @brief Axis parameter ids are not global parameter ids (removed; was a bank-aliasing trap).
   *  Use writeParameter() for axis parameters or a GlobalParamBank0/2/3 id for globals. */
  bool writeGlobal(tmc9660::tmcl::Parameters id, uint8_t bank, uint32_t value) = delete;

  /** @brief Read a global parameter; the bank (0, 2 or 3) follows from the enum type. */
  template <typename BankEnum>
    requires tmc9660::tmcl::IsGlobalParamBank<BankEnum>
  [[nodiscard]] bool readGlobal(BankEnum id, uint32_t& value) noexcept {
    return readGlobal(static_cast<uint16_t>(id), tmc9660::tmcl::kGlobalBankOf<BankEnum>, value);
  }

  /** @brief Write a global parameter; the bank (0, 2 or 3) follows from the enum type. */
  template <typename BankEnum>
    requires tmc9660::tmcl::IsGlobalParamBank<BankEnum>
  [[nodiscard]] bool writeGlobal(BankEnum id, uint32_t value) noexcept {
    return writeGlobal(static_cast<uint16_t>(id), tmc9660::tmcl::kGlobalBankOf<BankEnum>, value);
  }

  // Variant type for global parameter banks (can be index or any bank enum)
  using GlobalParamBankVariant =
      std::variant<uint8_t, tmc9660::tmcl::GlobalParamBank0, tmc9660::tmcl::GlobalParamBank2,
                   tmc9660::tmcl::GlobalParamBank3>;

  /** @brief Set (write) a global parameter on the TMC9660.
   * @param id Global parameter ID: a raw number, or a GlobalParamBank0/2/3 enum value.
   * @param bank Bank number (0, 2 or 3). When @p id is a bank enum, @p bank must be that
   *             enum's bank; a mismatch is rejected without touching the chip.
   * @param value 32-bit value to write.
   * @return true if successfully written, false if an error occurred or the bank mismatched.
   */
  [[nodiscard]] bool writeGlobalParameter(GlobalParamBankVariant id, uint8_t bank,
                                          uint32_t value) noexcept;

  /** @brief Read a global parameter from the TMC9660.
   * @param id Global parameter ID: a raw number, or a GlobalParamBank0/2/3 enum value.
   * @param bank Bank number (0, 2 or 3). When @p id is a bank enum, @p bank must be that
   *             enum's bank; a mismatch is rejected without touching the chip.
   * @param[out] value Reference to store the read 32-bit value.
   * @return true if read successfully, false on error or bank mismatch.
   */
  [[nodiscard]] bool readGlobalParameter(GlobalParamBankVariant id, uint8_t bank,
                                         uint32_t& value) noexcept;

  //--------------------------------------------------------------------------
  // Batched (pipelined) parameter access
  //--------------------------------------------------------------------------

  /** @brief One element of a batched axis-parameter access (SAP/GAP). */
  struct ParameterAccess {
    tmc9660::tmcl::Parameters id{}; ///< Axis parameter.
    uint32_t value = 0;             ///< Write: value to set. Read: value read back.
    uint8_t motor = 0;              ///< Axis index.
    tmc9660::tmcl::ReplyCode status = tmc9660::tmcl::ReplyCode::REPLY_CHKERR; ///< TMCL status.
    [[nodiscard]] bool ok() const noexcept {
      return status == tmc9660::tmcl::ReplyCode::REPLY_OK;
    }
  };

  /** @brief One element of a batched global-parameter access (SGP/GGP).
   *
   * Constructed only from a GlobalParamBank0/2/3 id, so the bank always matches the id. An
   * axis tmcl::Parameters id does not convert (compile error), which keeps axis and global
   * ids on their own banks. */
  struct GlobalParameterAccess {
    uint16_t id = 0;    ///< Global parameter number.
    uint8_t bank = 0;   ///< Bank deduced from the enum type.
    uint32_t value = 0; ///< Write: value to set. Read: value read back.
    tmc9660::tmcl::ReplyCode status = tmc9660::tmcl::ReplyCode::REPLY_CHKERR; ///< TMCL status.

    template <typename BankEnum>
      requires tmc9660::tmcl::IsGlobalParamBank<BankEnum>
    constexpr GlobalParameterAccess(BankEnum p,
                                    uint32_t v = 0) noexcept // NOLINT(google-explicit-constructor)
        : id(static_cast<uint16_t>(p)), bank(tmc9660::tmcl::kGlobalBankOf<BankEnum>), value(v) {}
    GlobalParameterAccess(tmc9660::tmcl::Parameters, uint32_t = 0) = delete;
    [[nodiscard]] bool ok() const noexcept {
      return status == tmc9660::tmcl::ReplyCode::REPLY_OK;
    }
  };

  /** @brief Read several axis parameters in one pipelined batch (GAP).
   *
   * On SPI, N reads cost N + 1 datagrams instead of 2N, and only one handshake wait per
   * element (see SpiCommInterface::transferTMCLBatch and docs "Timing and throughput").
   * Results land in each element's @c value and @c status, in order.
   * @return true if every element was answered with REPLY_OK.
   */
  [[nodiscard]] bool readParameters(std::span<ParameterAccess> items) noexcept;

  /** @brief Write several axis parameters in one pipelined batch (SAP). Same rules as
   *  readParameters(); each element's @c status reports its own result. SAP is idempotent,
   *  which the batch recovery path relies on. */
  [[nodiscard]] bool writeParameters(std::span<ParameterAccess> items) noexcept;

  /** @brief Read several global parameters in one pipelined batch (GGP). */
  [[nodiscard]] bool readGlobalParameters(std::span<GlobalParameterAccess> items) noexcept;

  /** @brief Write several global parameters in one pipelined batch (SGP). */
  [[nodiscard]] bool writeGlobalParameters(std::span<GlobalParameterAccess> items) noexcept;

  /** @brief Send raw TMCL frames as one pipelined batch.
   *
   * Lower-level form of the helpers above. Every request must be idempotent (see
   * SpiCommInterface::transferTMCLBatch). Replies keep their TMCL status.
   * @param[out] completed Optional: leading elements with a valid reply.
   * @return true if every request got a well-formed reply (regardless of TMCL status). */
  [[nodiscard]] bool sendCommandBatch(std::span<const TMCLFrame> requests,
                                      std::span<TMCLReply> replies,
                                      size_t* completed = nullptr) noexcept;

  /// Send a TMCL command. Optionally return the 32-bit reply value.
  /// When @p out_tmcl_status is non-null: set from the TMCL status byte after a successful
  /// transfer; on transport failure set to `REPLY_CHKERR`. (Does not change legacy return.)
  bool sendCommand(tmc9660::tmcl::Op opcode, uint16_t type = 0, uint8_t motor = 0,
                   uint32_t value = 0, uint32_t* reply = nullptr,
                   tmc9660::tmcl::ReplyCode* out_tmcl_status = nullptr) noexcept;

  // @}

  //================================================================================
  // @name Subsystem Interfaces
  // @{
  //================================================================================

  /** @brief Motor configuration and control subsystem.
   * @ingroup TMC9660_Subsystems
   *
   * Provides high-level methods for configuring motor parameters such as
   * motor type, pole pairs, PWM frequency, commutation mode, and control
   * limits. This subsystem abstracts the low-level TMCL parameter access
   * into intuitive, easy-to-use methods.
   *
   * @code{.cpp}
   * // Configure a BLDC motor
   * driver.motor.setType(tmc9660::tmcl::MotorType::BLDC, 7);  // 7 pole pairs
   * driver.motor.setPWMFrequency(20000);  // 20 kHz PWM
   * driver.motor.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_HALL);
   * driver.motor.setMaxTorqueCurrent(2000);  // 2A torque current limit
   * driver.motor.enable();  // Start motor control
   * @endcode
   */
  struct MotorConfig {
    /** @brief Configure the motor type (DC, BLDC, or stepper) and basic motor
     * settings.
     *
     * This sets the MOTOR_TYPE parameter and optionally related parameters like
     * pole pairs for BLDC or microstep settings for steppers. Note that the
     * BLDC option also covers PMSM motors since they use the same three-phase
     * commutation scheme.
     * @param type MotorType (DC, BLDC/PMSM, STEPPER).
     * @param polePairs For BLDC motors, number of pole pairs. For stepper or
     * DC, this can be set to 1.
     * @return true if the motor type was set successfully, false if
     * communication or device error.
     */
    bool setType(tmc9660::tmcl::MotorType type, uint8_t polePairs = 1) noexcept;

    /** @brief Set the motor direction inversion.
     *
     * This configures the MOTOR_DIRECTION parameter which inverts the meaning
     * of "forward" direction for the motor.
     * @param direction MotorDirection (FORWARD or REVERSE).
     * @return true if successfully set.
     */
    bool setDirection(tmc9660::tmcl::MotorDirection direction) noexcept;

    /** @brief Set the PWM frequency for the motor driver.
     * @param frequency_hz PWM frequency in Hertz (allowed range 10kHz to
     * 100kHz).
     * @return true if set successfully, false if an error occurred.
     */
    bool setPWMFrequency(uint32_t frequency_hz) noexcept;

    /** @brief Configure the commutation mode for the motor.
     *
     * This sets how the motor is driven: e.g., open-loop or closed-loop FOC
     * with various sensor feedback options. Typically used for BLDC or stepper
     * motors. For DC motors, commutation modes are not applicable except for
     * sensor feedback in velocity/position control modes.
     *
     * @param mode A CommutationMode value defining the motor control strategy:
     *             - SYSTEM_OFF: Motor disabled (default state after
     * power-on/reset)
     *             - SYSTEM_OFF_LOW_SIDE_FETS_ON: All low-side FETs ON (brake)
     *             - SYSTEM_OFF_HIGH_SIDE_FETS_ON: All high-side FETs ON
     *             - FOC_OPENLOOP_VOLTAGE: Constant duty cycle (voltage) control
     * without feedback
     *             - FOC_OPENLOOP_CURRENT: Constant current control without
     * position feedback
     *             - FOC_ENCODER: Field-oriented control with ABN encoder
     * feedback
     *             - FOC_HALL: Field-oriented control with Hall sensor feedback
     *             - FOC_SPI_ENCODER: Field-oriented control with SPI encoder
     * feedback
     *
     * @return true if the mode was applied successfully.
     *
     * @note When using SYSTEM_OFF, the behavior is controlled by
     * IDLE_MOTOR_PWM_BEHAVIOR parameter. For open-loop modes, additional
     * parameters must be set (OPENLOOP_VOLTAGE or OPENLOOP_CURRENT). For
     * sensor-based modes, the appropriate sensor must be configured before
     * enabling this mode.
     */
    bool setCommutationMode(tmc9660::tmcl::CommutationMode mode) noexcept;

    /** @brief Set the output voltage limit for the FOC controller.
     *
     * This parameter limits the maximum Uq/ Ud output of the FOC controller
     * (PID output circular limiter).
     *
     * @param limit Output voltage limit (0 ... 32767, default 8000).
     * @return true if the parameter was set successfully.
     */
    bool setOutputVoltageLimit(uint16_t limit) noexcept;

    /** @brief Read the output voltage limit for the FOC controller.
     *
     * @param[out] limit Current output voltage limit value.
     * @return true if the parameter was read successfully.
     */
    bool getOutputVoltageLimit(uint16_t& limit) noexcept;

    /** @brief Set the maximum allowed motor current (torque limit).
     *
     * This sets the MAX_TORQUE parameter which limits the peak current/torque
     * that the controller will deliver to the motor.
     * @param milliamps Maximum current in milliamps.
     * @return true on success, false on error.
     *
     * @note This value can be temporarily exceeded marginally due to the
     * operation of the current regulator.
     */
    bool setMaxTorqueCurrent(uint16_t milliamps) noexcept;

    /** @brief Set the maximum allowed flux current for BLDC/stepper motors.
     *
     * This sets the MAX_FLUX parameter which limits the flux-producing current
     * component. Important for field-weakening operation and stepper motor
     * control.
     * @param milliamps Maximum flux current in milliamps.
     * @return true on success, false on error.
     *
     * @note This value can be temporarily exceeded marginally due to the
     * operation of the current regulator.
     */
    bool setMaxFluxCurrent(uint16_t milliamps) noexcept;

    /** @brief Set the PWM switching scheme for the motor driver.
     *
     * This configures how the PWM signals are generated for driving the motor
     * phases. Different schemes offer varying trade-offs between voltage
     * utilization, switching losses, and current measurement windows.
     *
     * @param scheme PWM switching scheme:
     *               0: STANDARD - Standard PWM modulation (86% max voltage for BLDC)
     *               1: SVPWM - Space Vector PWM (100% max voltage for BLDC, balanced load on FETs)
     *               2: FLAT_BOTTOM - Flat bottom PWM (100% max voltage for BLDC, extended current
     * measurement window)
     * @return true if set successfully, false if an error occurred.
     *
     * @note For BLDC motors, SVPWM and Flat Bottom schemes allow full voltage
     * utilization. For Stepper/DC motors, only standard and flat bottom modes
     * are available with 100% duty cycle.
     */
    bool setPWMSwitchingScheme(tmc9660::tmcl::PwmSwitchingScheme scheme) noexcept;

    /**
     * @brief Configure PWM behavior when the motor is idle (System Off mode).
     *
     * Controls whether motor phases are driven or left floating (high-impedance) when commutation
     * is disabled.
     *
     * @param pwmOffWhenIdle
     *        True  → high-Z / disconnected phases
     *        False → all phases driven equally (same PWM output)
     * @return true if the parameter was set successfully.
     *
     * @note Motor-type idle mode selection guide:
     *
     * | Motor Type  | Idle Goal                   | Best Setting   | Notes                           |
     * |-------------|-----------------------------|----------------|----------------------------------|
     * | **Stepper** | Hold shaft position         | PWM **on**     | • Energizing both windings      |
     * |             |                             |                |   provides full static torque   |
     * |             |                             |                | • Prevents back-driving by load |
     * |             |                             |                |   or gravity                    |
     * |             |                             |                | • Always draws current at idle  |
     * |             |                             |                | • Causes heat and power waste—  |
     * |             |                             |                |   disable if holding isn't needed|
     * |             |                             |                |                                  |
     * |-------------|-----------------------------|----------------|----------------------------------|
     * | **BLDC**    | Coast freely (no braking)   | PWM **off**    | • With PWM off (high-Z), stator |
     * | (3-phase)   |                             | (high-Z)       |   phases are floating—not       |
     * |             |                             |                |   connected to power or ground  |
     * |             |                             |                | • No current flows from battery,|
     * |             |                             |                |   eliminating idle power & heat |
     * |             |                             |                | • Rotor coasts with only        |
     * |             |                             |                |   mechanical friction           |
     * |             |                             |                | • No braking torque (this is    |
     * |             |                             |                |   pure coasting, not braking)   |
     * |             |                             |                | • For passive braking (eddy-    |
     * |             |                             |                |   current braking), use         |
     * |             |                             |                |   SYSTEM_OFF_LOW_SIDE_FETS_ON   |
     * |             |                             |                |   or SYSTEM_OFF_HIGH_SIDE_FETS_ON|
     * |             |                             |                |   instead (phases shorted)      |
     * |             |                             |                | • If holding torque is needed   |
     * |             |                             |                |   (rare for sensorless BLDC),   |
     * |             |                             |                |   enable PWM or use active brake|
     * |             |                             |                |                                  |
     * |-------------|-----------------------------|----------------|----------------------------------|
     * | **Brushed** | Coast freely (no braking)   | PWM **off**    | • With PWM off (high-Z), H-bridge|
     * | DC Motor    |                             | (high-Z)       |   terminals float—not connected |
     * |             |                             |                | • No current flows from battery, |
     * |             |                             |                |   eliminating idle power & heat  |
     * |             |                             |                | • Rotor coasts with only        |
     * |             |                             |                |   mechanical friction           |
     * |             |                             |                | • No braking torque (this is    |
     * |             |                             |                |   pure coasting, not braking)   |
     * |             |                             |                | • PWM on = short-circuit braking |
     * |             |                             |                |   (both terminals at same       |
     * |             |                             |                |   voltage) → motor generates     |
     * |             |                             |                |   back-EMF, creating braking     |
     * |             |                             |                |   torque and regenerative       |
     * |             |                             |                |   currents flowing back         |
     * |             |                             |                | • Regenerative currents can      |
     * |             |                             |                |   charge battery or damage      |
     * |             |                             |                |   system if not handled—use with|
     * |             |                             |                |   caution                       |
     *
     * There is no universal best setting—choose based on your **application needs**:
     *
     * • `PWM_OFF_WHEN_MOTOR_IDLE`: All phases set to high-Z. Motor is electrically disconnected.
     * • `PWM_ON_WHEN_MOTOR_IDLE`: All bridge outputs actively driven to same potential.
     *
     * ### Quick Decision Tree
     *
     * 1. **Do you need the shaft locked at idle?**
     *    → Yes: PWM **on**
     *    → No: Continue
     *
     * 2. **Is battery life, idle power, or heat a concern?**
     *    → Yes: PWM **off**
     *
     * 3. **Do you want the rotor to coast freely?**
     *    → Yes: PWM **off** (high-Z)
     *    → No, you want braking: leave PWM **on** or use a dedicated brake feature
     *
     * ### Rule of Thumb
     *
     * • **Steppers**: Keep coils energized only when holding torque is required.
     * • **BLDC & Brushed DC**: Default to PWM off; only energize if braking or holding is needed.
     *
     * Final guidance:
     * • Holding torque needed → **PWM on**
     * • Coasting, low idle power → **PWM off**
     */
    bool setIdleMotorPWMBehavior(
        tmc9660::tmcl::IdleMotorPwmBehavior pwmOffWhenIdle =
            tmc9660::tmcl::IdleMotorPwmBehavior::PWM_OFF_WHEN_MOTOR_IDLE) noexcept;

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------

    /** @brief Configuration structure for auto-configuring motor parameters.
     *
     * This structure captures high-level motor characteristics and automatically
     * derives all necessary low-level configuration parameters.
     */
    struct MotorProfile {
      // Required parameters
      tmc9660::tmcl::MotorType motorType; //!< Motor type (DC, BLDC, STEPPER)
      uint8_t polePairs;        //!< Number of pole pairs (for BLDC/Stepper, typically 1-21)
      uint32_t pwmFrequency_Hz; //!< PWM frequency in Hz (10000-100000, recommended: 20-25kHz for
                                //!< BLDC, 20kHz for stepper)
      float maxPhaseCurrent_A;  //!< Maximum phase current in amperes (used to set MAX_TORQUE and
                                //!< MAX_FLUX)

      // Optional parameters with defaults
      tmc9660::tmcl::MotorDirection direction =
          tmc9660::tmcl::MotorDirection::FORWARD; //!< Motor direction (default: FORWARD)
      tmc9660::tmcl::PwmSwitchingScheme pwmSwitchingScheme =
          tmc9660::tmcl::PwmSwitchingScheme::SVPWM; //!< PWM switching scheme (default: SVPWM for
                                                    //!< BLDC, STANDARD for others)
      float maxFluxCurrent_A =
          std::numeric_limits<float>::quiet_NaN(); //!< Maximum flux current in amperes (optional,
                                                   //!< defaults to maxPhaseCurrent_A * 0.2 for
                                                   //!< BLDC/Stepper)
      uint16_t outputVoltageLimit =
          8000; //!< Output voltage limit for FOC controller (default: 8000)
      tmc9660::tmcl::IdleMotorPwmBehavior idlePwmBehavior =
          tmc9660::tmcl::IdleMotorPwmBehavior::PWM_OFF_WHEN_MOTOR_IDLE; //!< Idle motor PWM behavior
                                                                        //!< (default: PWM_OFF)
      std::optional<tmc9660::tmcl::CommutationMode>
          commutationMode; //!< Commutation mode to apply after configuration (optional, default:
                           //!< remains SYSTEM_OFF). Applied last, after all motor parameters are
                           //!< set.
    };

    /** @brief Auto-configure motor parameters based on high-level motor characteristics.
     *
     * This method automatically configures motor-specific parameters including:
     * - Motor type and pole pairs
     * - Motor direction
     * - PWM frequency and switching scheme
     * - Maximum torque and flux current limits
     * - Idle motor PWM behavior
     * - Output voltage limit
     * - Commutation mode (if specified, applied last after all configuration is complete)
     *
     * @note Current sensing configuration (CSA gain, current scaling factor) should be
     *       configured separately using CurrentSensing::configureAuto().
     *
     * @note The function sets commutation mode to SYSTEM_OFF first (required for motor
     *       type changes), then configures all parameters, and finally applies the
     *       requested commutation mode if specified in the profile.
     *
     * @param profile Motor configuration profile (see MotorProfile)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const MotorProfile& profile) noexcept;

  private:
    friend class TMC9660;
    explicit MotorConfig(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } motorConfig{*this};

  //***************************************************************************
  //**                  SUBSYSTEM: Current Measurement                      **//
  //***************************************************************************

  /** @brief Subsystem for configuring ADC-based current measurement
   */
  struct CurrentSensing {

    /** @brief Set the ADC shunt type (Parameter 12: ADC_SHUNT_TYPE).
     * @param shuntType AdcShuntType enum value
     * @return true if successful
     */
    bool setShuntType(tmc9660::tmcl::AdcShuntType shuntType) noexcept;

    /** @brief Get the ADC shunt type (Parameter 12: ADC_SHUNT_TYPE).
     * @param[out] shuntType AdcShuntType enum value
     * @return true if successful
     */
    bool getShuntType(tmc9660::tmcl::AdcShuntType& shuntType) noexcept;

    /** @brief Read raw ADC values (Parameters 13-16: ADC_I0_RAW ... ADC_I3_RAW).
     * @param[out] adc0 Raw ADC I0
     * @param[out] adc1 Raw ADC I1
     * @param[out] adc2 Raw ADC I2
     * @param[out] adc3 Raw ADC I3
     * @return true if all values were read successfully
     */
    bool readRaw(int16_t& adc0, int16_t& adc1, int16_t& adc2, int16_t& adc3) noexcept;

    /** @brief Set current sense amplifier gain (Parameters 17/18:
     * CSA_GAIN_ADC_I0_TO_ADC_I2, CSA_GAIN_ADC_I3).
     * @param gain012 CsaGain enum value for ADC I0/I1/I2
     * @param gain3 CsaGain enum value for ADC I3
     * @return true if successful
     */
    bool setCSAGain(tmc9660::tmcl::CsaGain gain012, tmc9660::tmcl::CsaGain gain3) noexcept;

    /** @brief Get current sense amplifier gain (Parameters 17/18:
     * CSA_GAIN_ADC_I0_TO_ADC_I2, CSA_GAIN_ADC_I3).
     * @param[out] gain012 CsaGain enum value for ADC I0/I1/I2
     * @param[out] gain3 CsaGain enum value for ADC I3
     * @return true if successful
     */
    bool getCSAGain(tmc9660::tmcl::CsaGain& gain012, tmc9660::tmcl::CsaGain& gain3) noexcept;

    /** @brief Set current sense amplifier filter (Parameters 19/20:
     * CSA_FILTER_ADC_I0_TO_ADC_I2, CSA_FILTER_ADC_I3).
     * @param filter012 CsaFilter enum value for ADC I0/I1/I2
     * @param filter3 CsaFilter enum value for ADC I3
     * @return true if successful
     */
    bool setCSAFilter(tmc9660::tmcl::CsaFilter filter012,
                      tmc9660::tmcl::CsaFilter filter3) noexcept;

    /** @brief Get current sense amplifier filter (Parameters 19/20:
     * CSA_FILTER_ADC_I0_TO_ADC_I2, CSA_FILTER_ADC_I3).
     * @param[out] filter012 CsaFilter enum value for ADC I0/I1/I2
     * @param[out] filter3 CsaFilter enum value for ADC I3
     * @return true if successful
     */
    bool getCSAFilter(tmc9660::tmcl::CsaFilter& filter012,
                      tmc9660::tmcl::CsaFilter& filter3) noexcept;

    /** @brief Set current scaling factor (Parameter 21: CURRENT_SCALING_FACTOR).
     * @param scaling_factor Scaling factor (1...65535)
     * @return true if successful
     */
    bool setScalingFactor(uint16_t scaling_factor) noexcept;

    /** @brief Get current scaling factor (Parameter 21: CURRENT_SCALING_FACTOR).
     * @param[out] scaling_factor Scaling factor (1...65535)
     * @return true if successful
     */
    bool getScalingFactor(uint16_t& scaling_factor) noexcept;

    /** @brief Set ADC mapping for each phase (Parameters 22-25:
     * PHASE_UX1_ADC_MAPPING ... PHASE_Y2_ADC_MAPPING).
     * @param ux1 AdcMapping enum value for UX1
     * @param vx2 AdcMapping enum value for VX2
     * @param wy1 AdcMapping enum value for WY1
     * @param y2 AdcMapping enum value for Y2
     * @return true if all mappings were set successfully
     */
    bool setPhaseAdcMapping(tmc9660::tmcl::AdcMapping ux1, tmc9660::tmcl::AdcMapping vx2,
                            tmc9660::tmcl::AdcMapping wy1, tmc9660::tmcl::AdcMapping y2) noexcept;

    /** @brief Get ADC mapping for each phase (Parameters 22-25:
     * PHASE_UX1_ADC_MAPPING ... PHASE_Y2_ADC_MAPPING).
     * @param[out] ux1 AdcMapping enum value for UX1
     * @param[out] vx2 AdcMapping enum value for VX2
     * @param[out] wy1 AdcMapping enum value for WY1
     * @param[out] y2 AdcMapping enum value for Y2
     * @return true if all mappings were retrieved successfully
     */
    bool getPhaseAdcMapping(tmc9660::tmcl::AdcMapping& ux1, tmc9660::tmcl::AdcMapping& vx2,
                            tmc9660::tmcl::AdcMapping& wy1, tmc9660::tmcl::AdcMapping& y2) noexcept;

    /** @brief Set individual ADC scaling factors (Parameters 26-29: ADC_I0_SCALE
     * ... ADC_I3_SCALE).
     * @param scale0 Scaling factor for ADC I0 (1...32767)
     * @param scale1 Scaling factor for ADC I1 (1...32767)
     * @param scale2 Scaling factor for ADC I2 (1...32767)
     * @param scale3 Scaling factor for ADC I3 (1...32767)
     * @return true if all scales were set successfully
     */
    bool setScalingFactors(uint16_t scale0, uint16_t scale1, uint16_t scale2,
                           uint16_t scale3) noexcept;

    /** @brief Get individual ADC scaling factors (Parameters 26-29: ADC_I0_SCALE
     * ... ADC_I3_SCALE).
     * @param[out] scale0 Scaling factor for ADC I0 (1...32767)
     * @param[out] scale1 Scaling factor for ADC I1 (1...32767)
     * @param[out] scale2 Scaling factor for ADC I2 (1...32767)
     * @param[out] scale3 Scaling factor for ADC I3 (1...32767)
     * @return true if all scales were retrieved successfully
     */
    bool getScalingFactors(uint16_t& scale0, uint16_t& scale1, uint16_t& scale2,
                           uint16_t& scale3) noexcept;

    /** @brief Set ADC inversion (Parameters 30-33: ADC_I0_INVERTED ...
     * ADC_I3_INVERTED).
     * @param inv0 AdcInversion enum value for ADC I0
     * @param inv1 AdcInversion enum value for ADC I1
     * @param inv2 AdcInversion enum value for ADC I2
     * @param inv3 AdcInversion enum value for ADC I3
     * @return true if all inversion flags were set successfully
     */
    bool setInversion(tmc9660::tmcl::AdcInversion inv0, tmc9660::tmcl::AdcInversion inv1,
                      tmc9660::tmcl::AdcInversion inv2, tmc9660::tmcl::AdcInversion inv3) noexcept;

    /** @brief Get ADC inversion (Parameters 30-33: ADC_I0_INVERTED ...
     * ADC_I3_INVERTED).
     * @param[out] inv0 AdcInversion enum value for ADC I0
     * @param[out] inv1 AdcInversion enum value for ADC I1
     * @param[out] inv2 AdcInversion enum value for ADC I2
     * @param[out] inv3 AdcInversion enum value for ADC I3
     * @return true if all inversion flags were retrieved successfully
     */
    bool getInversion(tmc9660::tmcl::AdcInversion& inv0, tmc9660::tmcl::AdcInversion& inv1,
                      tmc9660::tmcl::AdcInversion& inv2,
                      tmc9660::tmcl::AdcInversion& inv3) noexcept;

    /** @brief Set ADC offset (Parameters 34-37: ADC_I0_OFFSET ...
     * ADC_I3_OFFSET).
     * @param offset0 Offset for ADC I0 (-32768...32767)
     * @param offset1 Offset for ADC I1 (-32768...32767)
     * @param offset2 Offset for ADC I2 (-32768...32767)
     * @param offset3 Offset for ADC I3 (-32768...32767)
     * @return true if all offsets were set successfully
     */
    bool setOffsets(int16_t offset0, int16_t offset1, int16_t offset2, int16_t offset3) noexcept;

    /** @brief Get ADC offset (Parameters 34-37: ADC_I0_OFFSET ...
     * ADC_I3_OFFSET).
     * @param[out] offset0 Offset for ADC I0 (-32768...32767)
     * @param[out] offset1 Offset for ADC I1 (-32768...32767)
     * @param[out] offset2 Offset for ADC I2 (-32768...32767)
     * @param[out] offset3 Offset for ADC I3 (-32768...32767)
     * @return true if all offsets were retrieved successfully
     */
    bool getOffsets(int16_t& offset0, int16_t& offset1, int16_t& offset2,
                    int16_t& offset3) noexcept;

    /** @brief Read scaled and offset-compensated ADC values (Parameters 38-41:
     * ADC_I0 ... ADC_I3).
     * @param[out] adc0 Scaled/offset ADC I0
     * @param[out] adc1 Scaled/offset ADC I1
     * @param[out] adc2 Scaled/offset ADC I2
     * @param[out] adc3 Scaled/offset ADC I3
     * @return true if all values were read successfully
     */
    bool readScaledAndOffset(int16_t& adc0, int16_t& adc1, int16_t& adc2, int16_t& adc3) noexcept;

    /** @brief Calibrate the ADC offsets for current measurement.
     *
     * Initiates a calibration sequence for the ADCs. This should be done:
     * 1. With the motor stationary
     * 2. With the commutation mode set to off
     *
     * @param waitForCompletion If true, wait until calibration is completed
     * @param timeoutMs Timeout in milliseconds if waiting for completion
     * @return true if calibration was started (and completed if
     * wait_for_completion is true)
     */
    bool calibrateOffsets(bool waitForCompletion = false, uint32_t timeoutMs = 1000) noexcept;

    /** @brief Check if ADC offset calibration has been completed.
     *
     * @param[out] is_calibrated Set to true if calibration is complete
     * @return true if the status was read successfully
     */
    bool getCalibrationStatus(bool& is_calibrated) noexcept;

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------
    /** @brief Configuration structure for auto-configuring current sensing.
     *
     * This struct contains all parameters needed to configure the TMC9660 current sensing system.
     * Most fields have sensible defaults based on the motor type and datasheet recommendations.
     *
     * @code
     * // Example: Basic BLDC configuration
     * TMC9660::CurrentSensing::AutoConfig config;
     * config.shuntResistance_mOhm = 3.0;
     * config.expectedPeakCurrent_A = 3.0;
     * config.motorType = tmc9660::tmcl::MotorType::BLDC_MOTOR;
     * driver.currentSensing.configureAuto(config);
     *
     * // Example: With custom ADC mapping and inversion
     * config.phaseU_adcMapping = tmc9660::tmcl::AdcMapping::ADC_I0;
     * config.phaseV_adcMapping = tmc9660::tmcl::AdcMapping::ADC_I1;
     * config.phaseW_adcMapping = tmc9660::tmcl::AdcMapping::ADC_I2;
     * config.phaseY2_adcMapping = tmc9660::tmcl::AdcMapping::ADC_I3;
     * config.adc0_inverted = tmc9660::tmcl::AdcInversion::INVERTED;
     * config.adc1_inverted = tmc9660::tmcl::AdcInversion::INVERTED;
     * config.adc2_inverted = tmc9660::tmcl::AdcInversion::INVERTED;
     * config.adc3_inverted = tmc9660::tmcl::AdcInversion::INVERTED;
     * config.autoCalibrate = true;
     * driver.currentSensing.configureAuto(config);
     * @endcode
     */
    struct AutoConfig {
      // Required parameters
      float shuntResistance_mOhm; //!< Nominal shunt resistor value in milliohms (e.g. 3.0 for 3 mΩ)
      float expectedPeakCurrent_A; //!< Expected peak phase current in amperes (e.g., 3.0 for 3A)
      tmc9660::tmcl::MotorType motorType; //!< Motor type (used for default ADC inversion settings)

      // Optional parameters with defaults
      bool usePeakScaling =
          true; //!< If true, use peak scaling (recommended for FOC/BLDC). If false, use RMS.
      tmc9660::tmcl::AdcShuntType shuntType =
          tmc9660::tmcl::AdcShuntType::BOTTOM_SHUNTS; //!< Shunt type configuration
      tmc9660::tmcl::CsaFilter csaFilter =
          tmc9660::tmcl::CsaFilter::T_1_0_MICROSEC; //!< CSA filter time constant

      // Per-ADC actual shunt resistances (for automatic ADC_Ix_SCALE compensation)
      // If NaN or <= 0, uses nominal value (no compensation)
      float actualShuntR_adc0_mOhm =
          std::numeric_limits<float>::quiet_NaN(); //!< Actual shunt resistance for ADC_I0 in mΩ
      float actualShuntR_adc1_mOhm =
          std::numeric_limits<float>::quiet_NaN(); //!< Actual shunt resistance for ADC_I1 in mΩ
      float actualShuntR_adc2_mOhm =
          std::numeric_limits<float>::quiet_NaN(); //!< Actual shunt resistance for ADC_I2 in mΩ
      float actualShuntR_adc3_mOhm =
          std::numeric_limits<float>::quiet_NaN(); //!< Actual shunt resistance for ADC_I3 in mΩ

      // Per-phase ADC mapping (which ADC channel maps to which motor phase)
      // If not set (std::nullopt), uses defaults: U->I0, V->I1, W->I2, Y2->I3
      std::optional<tmc9660::tmcl::AdcMapping>
          phaseU_adcMapping; //!< ADC mapping for phase U (UX1). Default: ADC_I0
      std::optional<tmc9660::tmcl::AdcMapping>
          phaseV_adcMapping; //!< ADC mapping for phase V (VX2). Default: ADC_I1
      std::optional<tmc9660::tmcl::AdcMapping>
          phaseW_adcMapping; //!< ADC mapping for phase W (WY1). Default: ADC_I2
      std::optional<tmc9660::tmcl::AdcMapping>
          phaseY2_adcMapping; //!< ADC mapping for phase Y2. Default: ADC_I3

      // Per-ADC inversion settings
      // If not set (std::nullopt), uses Table 24 defaults based on motor type:
      //   DC:      ADC_I0=INVERTED, ADC_I1=NOT_INVERTED
      //   BLDC:    ADC_I0=INVERTED, ADC_I1=INVERTED, ADC_I2=INVERTED
      //   Stepper: ADC_I0=INVERTED, ADC_I1=NOT_INVERTED, ADC_I2=INVERTED, ADC_I3=NOT_INVERTED
      std::optional<tmc9660::tmcl::AdcInversion>
          adc0_inverted; //!< Inversion for ADC_I0. Default: based on motor type
      std::optional<tmc9660::tmcl::AdcInversion>
          adc1_inverted; //!< Inversion for ADC_I1. Default: based on motor type
      std::optional<tmc9660::tmcl::AdcInversion>
          adc2_inverted; //!< Inversion for ADC_I2. Default: based on motor type
      std::optional<tmc9660::tmcl::AdcInversion>
          adc3_inverted; //!< Inversion for ADC_I3. Default: based on motor type

      // Auto-calibration settings
      bool autoCalibrate =
          false; //!< If true, automatically calibrate ADC offsets after configuration
      uint32_t calibrationTimeoutMs =
          2000; //!< Timeout in milliseconds for auto-calibration verification
    };

    /** @brief Auto-configure current sensing based on shunt resistance and expected current.
     *
     * This method automatically selects the optimal CSA gain and calculates the current
     * scaling factor to enable direct m_a units for torque/flux commands.
     *
     * The function:
     * - Selects the smallest CSA gain that provides ≥1.5x headroom above expected peak current
     * - Calculates CURRENT_SCALING_FACTOR using the formula from the datasheet:
     *   - Peak: Factor = 39.06 / (G_CSA * R_shunt_Ohm)
     *   - RMS:  Factor = 27.62 / (G_CSA * R_shunt_Ohm)
     * - Configures shunt type, CSA gain, filter, scaling factor, and ADC inversion
     * - Optionally calculates per-ADC scaling factors if actual shunt resistances are provided
     *
     * After calling this, torque/flux commands (MAX_TORQUE, TARGET_TORQUE, etc.) can be
     * specified directly in m_a. For example, MAX_TORQUE = 3000 means 3.0 A.
     *
     * ## Two-Level Scaling System
     *
     * The TMC9660 uses a two-level scaling system:
     *
     * 1. **ADC_Ix_SCALE (per-phase trim)**: Compensates for physical mismatch in shunt resistors
     *    and CSA amplifier tolerances. Default: 1024 (1.000×). If actual shunt resistances are
     *    provided, this is automatically calculated to equalize all phases.
     *
     * 2. **CURRENT_SCALING_FACTOR (global unit conversion)**: Converts normalized ADC currents
     *    to m_a units for the torque/flux control system. This is automatically calculated based
     *    on nominal shunt resistance and CSA gain.
     *
     * Signal flow:
     * ```
     * ADC_Ix_RAW → (offset removal) → ADC_Ix → (ADC_Ix_SCALE correction) →
     * normalized phase current → (CURRENT_SCALING_FACTOR) → final torque/flux in m_a
     * ```
     *
     * ## Per-ADC Shunt Resistance Compensation
     *
     * If you have measured the actual shunt resistance for each ADC channel, you can provide
     * them to automatically calculate ADC_Ix_SCALE. The formula used is:
     * ```
     * ADC_Ix_SCALE = 1024 * (R_nominal / R_actual)
     * ```
     *
     * This ensures all phases report the same current for the same real current, compensating
     * for shunt resistor tolerances.
     *
     * @param config Configuration structure containing all current sensing parameters.
     *               See AutoConfig for details on all available options.
     * @return true if configuration was successful (and calibration succeeded if
     * autoCalibrate=true)
     *
     * @note ADC inversion defaults are set according to Table 24 for the specified motor type,
     *       but can be overridden via config.adc0_inverted, etc. Verify in open-loop voltage
     *       mode and adjust if phases are 180° out of phase.
     *
     * @note ADC offset calibration is required before using current sensing. If
     * config.autoCalibrate is false, you must call calibrateOffsets() manually. The motor must be
     * stationary and commutation must be off (SYSTEM_OFF) during calibration.
     *
     * @code
     * // Example 1: Basic configuration (minimal setup)
     * TMC9660::CurrentSensing::AutoConfig config;
     * config.shuntResistance_mOhm = 3.0;
     * config.expectedPeakCurrent_A = 3.0;
     * config.motorType = tmc9660::tmcl::MotorType::BLDC_MOTOR;
     * driver.currentSensing.configureAuto(config);
     *
     * // Example 2: With measured shunt resistances for automatic ADC_Ix_SCALE compensation
     * // Measured: U=2.97 mΩ, V=3.01 mΩ, W=3.02 mΩ (nominal = 3.00 mΩ)
     * config.actualShuntR_adc0_mOhm = 2.97;  // ADC_I0 (Phase U)
     * config.actualShuntR_adc1_mOhm = 3.01;  // ADC_I1 (Phase V)
     * config.actualShuntR_adc2_mOhm = 3.02; // ADC_I2 (Phase W)
     * driver.currentSensing.configureAuto(config);
     *
     * // Example 3: With custom ADC mapping and inversion
     * config.phaseU_adcMapping = tmc9660::tmcl::AdcMapping::ADC_I0;
     * config.phaseV_adcMapping = tmc9660::tmcl::AdcMapping::ADC_I1;
     * config.phaseW_adcMapping = tmc9660::tmcl::AdcMapping::ADC_I2;
     * config.adc0_inverted = tmc9660::tmcl::AdcInversion::INVERTED;
     * config.adc1_inverted = tmc9660::tmcl::AdcInversion::INVERTED;
     * config.adc2_inverted = tmc9660::tmcl::AdcInversion::INVERTED;
     * config.autoCalibrate = true;
     * driver.currentSensing.configureAuto(config);
     *
     * // Now use m_a directly:
     * driver.motorConfig.setMaxTorqueCurrent(3000);  // 3.0 A
     * driver.focControl.setTargetTorque(2500);      // 2.5 A
     * @endcode
     */
    bool configureAuto(const AutoConfig& config) noexcept;

  private:
    friend class TMC9660;
    explicit CurrentSensing(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } currentSensing{*this};

  //***************************************************************************
  //**                  SUBSYSTEM: Gate Driver                              **//
  //***************************************************************************

  /** @brief Subsystem for configuring the MOSFET gate driver
   */
  struct GateDriver {
    /** @brief Set the gate driver output polarity.
     *
     * Configures the PWM_L and PWM_H output polarity.
     * @param low_side Polarity for the low-side gate outputs
     * (tmc9660::tmcl::PwmOutputPolarity enum).
     * @param high_side Polarity for the high-side gate outputs
     * (tmc9660::tmcl::PwmOutputPolarity enum).
     * @return true if the polarity was set successfully.
     */
    bool setOutputPolarity(tmc9660::tmcl::PwmOutputPolarity low_side,
                           tmc9660::tmcl::PwmOutputPolarity high_side) noexcept;

    /** @brief Configure the break-before-make timing for the gate driver (advanced API).
     *
     * Sets the timing for switching between high and low sides of the gate
     * driver using register values.
     *
     * The break-before-make time is applied according to the following equation:
     *    time_ns = value * 8.33
     *
     * Or, conversely, for configuring a time in nanoseconds (ns):
     *    value = time_ns / 8.33
     *
     * @note For most use cases, prefer configureBreakBeforeMakeTiming_ns() which accepts
     *       time values in nanoseconds for a more intuitive API.
     *
     * @param low_side_uvw Break-before-make time for UVW low side (register value: 0-255).
     *        Actual time: t = low_side_uvw * 8.33 ns
     * @param high_side_uvw Break-before-make time for UVW high side (register value: 0-255).
     *        Actual time: t = high_side_uvw * 8.33 ns
     * @param low_side_y2 Break-before-make time for Y2 low side (register value: 0-255).
     *        Actual time: t = low_side_y2 * 8.33 ns
     * @param high_side_y2 Break-before-make time for Y2 high side (register value: 0-255).
     *        Actual time: t = high_side_y2 * 8.33 ns
     * @return true if successfully configured.
     */
    bool configureBreakBeforeMakeTiming(uint8_t low_side_uvw, uint8_t high_side_uvw, uint8_t low_side_y2,
                                        uint8_t high_side_y2) noexcept;

    /**
     * @brief Configure break-before-make timing using nanoseconds (recommended API).
     *
     * Internally computes the register value from the requested ns value.
     * Values outside the valid 0...255 range will be clamped.
     *
     * This is the recommended API for most users as it accepts time values
     * in intuitive units (nanoseconds) rather than register values.
     *
     * @param lowSideUVW_ns Break-before-make time for UVW low side, in nanoseconds.
     * @param highSideUVW_ns Break-before-make time for UVW high side, in nanoseconds.
     * @param lowSideY2_ns Break-before-make time for Y2 low side, in nanoseconds.
     * @param highSideY2_ns Break-before-make time for Y2 high side, in nanoseconds.
     * @return true if successfully configured.
     */
    bool configureBreakBeforeMakeTiming_ns(float lowSideUVW_ns, float highSideUVW_ns,
                                           float lowSideY2_ns, float highSideY2_ns) noexcept;

    /** @brief Enable or disable adaptive drive time for UVW and Y2 phases.
     *
     * Adaptive drive time shortens the discharge cycle by monitoring gate
     * voltages.
     * @param enable_uvw True to enable adaptive drive time for UVW phases.
     * @param enable_y2 True to enable adaptive drive time for Y2 phase.
     * @return true if successfully configured.
     */
    bool enableAdaptiveDriveTime(bool enable_uvw, bool enable_y2) noexcept;

    /** @brief Configure drive times for UVW and Y2 phases (advanced API).
     *
     * Sets the discharge and charge times for the gate driver using register values.
     *
     * The drive time is applied according to the following equation:
     *    time_seconds = (1 / 120,000,000) * (2 * value + 3)
     * For example, input a value between 0...255 (register units).
     *
     * Example conversion to nanoseconds:
     *    time_ns = ((2 * value + 3) * (1e9 / 120'000'000)) = (2*value + 3) * 8.33 ns
     *
     * Or, conversely, for configuring a drive time in nanoseconds (ns):
     *    value = ((desired_time_ns / 8.33) - 3) / 2
     *
     * @note For most use cases, prefer configureDriveTimes_ns() which accepts
     *       time values in nanoseconds for a more intuitive API.
     *
     * @param sink_time_uvw Discharge time for UVW phases (register value: 0 ... 255, default: 255).
     *        Actual time: t = (2*sink_time_uvw + 3) * 8.33 ns
     * @param source_time_uvw Charge time for UVW phases (register value: 0 ... 255, default: 255).
     *        Actual time: t = (2*source_time_uvw + 3) * 8.33 ns
     * @param sink_time_y2 Discharge time for Y2 phase (register value: 0 ... 255, default: 255).
     *        Actual time: t = (2*sink_time_y2 + 3) * 8.33 ns
     * @param source_time_y2 Charge time for Y2 phase (register value: 0 ... 255, default: 255).
     *        Actual time: t = (2*source_time_y2 + 3) * 8.33 ns
     * @return true if successfully configured.
     */
    bool configureDriveTimes(uint8_t sink_time_uvw, uint8_t source_time_uvw, uint8_t sink_time_y2,
                             uint8_t source_time_y2) noexcept;

    /**
     * @brief Configure drive times using nanoseconds (recommended API).
     *
     * Internally computes the register value from the requested ns value.
     * Values outside the valid 0...255 range will be clamped.
     *
     * This is the recommended API for most users as it accepts time values
     * in intuitive units (nanoseconds) rather than register values.
     *
     * @param sinkTimeUVW_ns Discharge time for UVW phases, in nanoseconds.
     * @param sourceTimeUVW_ns Charge time for UVW phases, in nanoseconds.
     * @param sinkTimeY2_ns Discharge time for Y2 phase, in nanoseconds.
     * @param sourceTimeY2_ns Charge time for Y2 phase, in nanoseconds.
     * @return true if successfully configured.
     */
    bool configureDriveTimes_ns(float sinkTimeUVW_ns, float sourceTimeUVW_ns, float sinkTimeY2_ns,
                                float sourceTimeY2_ns) noexcept;

    /** @brief Configure gate driver current limits for UVW and Y2 phases.
     *
     * Sets the maximum sink and source currents for the gate driver.
     * @param sink_current_uvw Sink current for UVW phases (GateCurrentSink enum).
     * @param source_current_uvw Source current for UVW phases (GateCurrentSource
     * enum).
     * @param sink_current_y2 Sink current for Y2 phase (GateCurrentSink enum).
     * @param source_current_y2 Source current for Y2 phase (GateCurrentSource
     * enum).
     * @return true if successfully configured.
     */
    bool configureCurrentLimits(tmc9660::tmcl::GateCurrentSink sink_current_uvw,
                                tmc9660::tmcl::GateCurrentSource source_current_uvw,
                                tmc9660::tmcl::GateCurrentSink sink_current_y2,
                                tmc9660::tmcl::GateCurrentSource source_current_y2) noexcept;

    /** @brief Configure bootstrap current limit.
     *
     * Sets the maximum current for the bootstrap capacitor.
     * @param limit Bootstrap current limit (BootstrapCurrentLimit enum).
     * @return true if successfully configured.
     */
    bool configureBootstrapCurrentLimit(tmc9660::tmcl::BootstrapCurrentLimit limit) noexcept;

    /** @brief Configure undervoltage protection settings.
     *
     * @param supplyLevel Supply voltage (VS) protection level
     * (tmc9660::tmcl::UndervoltageLevel enum).
     * @param enable_vdrv Enable driver voltage (VDRV) protection
     * (tmc9660::tmcl::UndervoltageEnable enum).
     * @param enable_bst_uvw Enable bootstrap capacitor protection for UVW phases
     * (tmc9660::tmcl::UndervoltageEnable enum).
     * @param enable_bst_y2 Enable bootstrap capacitor protection for Y2 phase
     * (tmc9660::tmcl::UndervoltageEnable enum).
     * @return true if successfully configured.
     */
    bool configureUndervoltageProtection(tmc9660::tmcl::UndervoltageLevel supplyLevel,
                                         tmc9660::tmcl::UndervoltageEnable enable_vdrv,
                                         tmc9660::tmcl::UndervoltageEnable enable_bst_uvw,
                                         tmc9660::tmcl::UndervoltageEnable enable_bst_y2) noexcept;

    /** @brief Enable or disable overcurrent protection for UVW and Y2 phases.
     *
     * @param enable_uvw_low_side Enable protection for UVW low side
     * (tmc9660::tmcl::OvercurrentEnable enum).
     * @param enable_uvw_high_side Enable protection for UVW high side
     * (tmc9660::tmcl::OvercurrentEnable enum).
     * @param enable_y2_low_side Enable protection for Y2 low side
     * (tmc9660::tmcl::OvercurrentEnable enum).
     * @param enable_y2_high_side Enable protection for Y2 high side
     * (tmc9660::tmcl::OvercurrentEnable enum).
     * @return true if successfully configured.
     */
    bool enableOvercurrentProtection(tmc9660::tmcl::OvercurrentEnable enable_uvw_low_side,
                                     tmc9660::tmcl::OvercurrentEnable enable_uvw_high_side,
                                     tmc9660::tmcl::OvercurrentEnable enable_y2_low_side,
                                     tmc9660::tmcl::OvercurrentEnable enable_y2_high_side) noexcept;

    /** @brief Configure overcurrent protection thresholds for UVW and Y2 phases.
     *
     * **Threshold Selection:**
     * - **High-Side (HS)**: Always uses VDS sensing thresholds (63mV, 125mV, 187mV, etc.)
     * - **Low-Side (LS)**: Hardware automatically selects threshold based on UVW_LOW_SIDE_USE_VDS /
     *   Y2_LOW_SIDE_USE_VDS setting:
     *   - If VDS enabled: Uses VDS threshold (e.g., V_250_OR_187_MILLIVOLT → 187mV)
     *   - If RSHUNT enabled: Uses RSHUNT threshold (e.g., V_250_OR_187_MILLIVOLT → 250mV)
     *
     * The enum values encode both thresholds (e.g., V_250_OR_187_MILLIVOLT means 250mV for RSHUNT
     * or 187mV for VDS). The hardware automatically applies the correct value based on the sensing
     * method configured via enableVdsMonitoringLow().
     *
     * @param uvw_low_side_threshold Threshold for UVW low side (auto-selected based on VDS/RSHUNT
     * config) (tmc9660::tmcl::OvercurrentThreshold enum).
     * @param uvw_high_side_threshold Threshold for UVW high side (always VDS value)
     * (tmc9660::tmcl::OvercurrentThreshold enum).
     * @param y2LowSideThreshold Threshold for Y2 low side (auto-selected based on VDS/RSHUNT
     * config) (tmc9660::tmcl::OvercurrentThreshold enum).
     * @param y2HighSideThreshold Threshold for Y2 high side (always VDS value)
     * (tmc9660::tmcl::OvercurrentThreshold enum).
     * @return true if successfully configured.
     *
     * @note Configure the sensing method (VDS vs RSHUNT) via enableVdsMonitoringLow() before
     *       calling this function to ensure the correct threshold is applied.
     */
    bool setOvercurrentThresholds(tmc9660::tmcl::OvercurrentThreshold uvw_low_side_threshold,
                                  tmc9660::tmcl::OvercurrentThreshold uvw_high_side_threshold,
                                  tmc9660::tmcl::OvercurrentThreshold y2LowSideThreshold,
                                  tmc9660::tmcl::OvercurrentThreshold y2HighSideThreshold) noexcept;

    /** @brief Configure the overcurrent protection blanking time for UVW and Y2
     * phases.
     *
     * Sets the blanking time for overcurrent protection to filter out transient
     * spikes during switching events.
     * @param uvw_low_side_time Blanking time for the low side of UVW phases
     * (tmc9660::tmcl::OvercurrentTiming enum).
     * @param uvw_high_side_time Blanking time for the high side of UVW phases
     * (tmc9660::tmcl::OvercurrentTiming enum).
     * @param y2LowSideTime Blanking time for the low side of Y2 phase
     * (tmc9660::tmcl::OvercurrentTiming enum).
     * @param y2HighSideTime Blanking time for the high side of Y2 phase
     * (tmc9660::tmcl::OvercurrentTiming enum).
     * @return true if successfully configured.
     */
    bool setOvercurrentBlanking(tmc9660::tmcl::OvercurrentTiming uvw_low_side_time,
                                tmc9660::tmcl::OvercurrentTiming uvw_high_side_time,
                                tmc9660::tmcl::OvercurrentTiming y2LowSideTime,
                                tmc9660::tmcl::OvercurrentTiming y2HighSideTime) noexcept;

    /** @brief Configure the overcurrent protection deglitch time for UVW and Y2
     * phases.
     *
     * Sets how long an overcurrent condition must persist before triggering
     * protection.
     * @param uvw_low_side_time Deglitch time for the low side of UVW phases
     * (tmc9660::tmcl::OvercurrentTiming enum).
     * @param uvw_high_side_time Deglitch time for the high side of UVW phases
     * (tmc9660::tmcl::OvercurrentTiming enum).
     * @param y2LowSideTime Deglitch time for the low side of Y2 phase
     * (tmc9660::tmcl::OvercurrentTiming enum).
     * @param y2HighSideTime Deglitch time for the high side of Y2 phase
     * (tmc9660::tmcl::OvercurrentTiming enum).
     * @return true if successfully configured.
     */
    bool setOvercurrentDeglitch(tmc9660::tmcl::OvercurrentTiming uvw_low_side_time,
                                tmc9660::tmcl::OvercurrentTiming uvw_high_side_time,
                                tmc9660::tmcl::OvercurrentTiming y2LowSideTime,
                                tmc9660::tmcl::OvercurrentTiming y2HighSideTime) noexcept;

    /** @brief Enable or disable VDS monitoring for overcurrent protection on UVW
     * and Y2 low sides.
     *
     * @param uvw_enable True to enable VDS measurement for overcurrent
     * protection on UVW low side (tmc9660::tmcl::VdsUsage enum).
     * @param y2Enable True to enable VDS measurement for overcurrent protection
     * on Y2 low side (tmc9660::tmcl::VdsUsage enum).
     * @return true if successfully configured.
     */
    bool enableVdsMonitoringLow(tmc9660::tmcl::VdsUsage uvw_enable,
                                tmc9660::tmcl::VdsUsage y2Enable) noexcept;

    /** @brief Configure gate-to-source short protection for UVW phases.
     *
     * @param enable_low_side_on Enable protection for ON transition of low side
     * (tmc9660::tmcl::VgsShortEnable enum).
     * @param enable_low_side_off Enable protection for OFF transition of low side
     * (tmc9660::tmcl::VgsShortEnable enum).
     * @param enable_high_side_on Enable protection for ON transition of high side
     * (tmc9660::tmcl::VgsShortEnable enum).
     * @param enable_high_side_off Enable protection for OFF transition of high
     * side (tmc9660::tmcl::VgsShortEnable enum).
     * @return true if successfully configured.
     */
    bool configureVgsShortProtectionUVW(tmc9660::tmcl::VgsShortEnable enable_low_side_on,
                                        tmc9660::tmcl::VgsShortEnable enable_low_side_off,
                                        tmc9660::tmcl::VgsShortEnable enable_high_side_on,
                                        tmc9660::tmcl::VgsShortEnable enable_high_side_off) noexcept;

    /** @brief Configure gate-to-source short protection for Y2 phase.
     *
     * @param enable_low_side_on Enable protection for ON transition of low side
     * (tmc9660::tmcl::VgsShortEnable enum).
     * @param enable_low_side_off Enable protection for OFF transition of low side
     * (tmc9660::tmcl::VgsShortEnable enum).
     * @param enable_high_side_on Enable protection for ON transition of high side
     * (tmc9660::tmcl::VgsShortEnable enum).
     * @param enable_high_side_off Enable protection for OFF transition of high
     * side (tmc9660::tmcl::VgsShortEnable enum).
     * @return true if successfully configured.
     */
    bool configureVgsShortProtectionY2(tmc9660::tmcl::VgsShortEnable enable_low_side_on,
                                       tmc9660::tmcl::VgsShortEnable enable_low_side_off,
                                       tmc9660::tmcl::VgsShortEnable enable_high_side_on,
                                       tmc9660::tmcl::VgsShortEnable enable_high_side_off) noexcept;

    /** @brief Set gate-to-source short protection blanking time.
     *
     * @param uvw_time Blanking time for UVW phases
     * (tmc9660::tmcl::VgsBlankingTime enum).
     * @param y2Time Blanking time for Y2 phase (tmc9660::tmcl::VgsBlankingTime
     * enum).
     * @return true if successfully configured.
     */
    bool setVgsShortBlankingTime(tmc9660::tmcl::VgsBlankingTime uvw_time,
                                 tmc9660::tmcl::VgsBlankingTime y2Time) noexcept;

    /** @brief Set gate-to-source short protection deglitch time.
     *
     * @param uvw_time Deglitch time for UVW phases
     * (tmc9660::tmcl::VgsDeglitchTime enum).
     * @param y2Time Deglitch time for Y2 phase (tmc9660::tmcl::VgsDeglitchTime
     * enum).
     * @return true if successfully configured.
     */
    bool setVgsShortDeglitchTime(tmc9660::tmcl::VgsDeglitchTime uvw_time,
                                 tmc9660::tmcl::VgsDeglitchTime y2Time) noexcept;

    /** @brief Configure fault retry behavior.
     *
     * @param retry_behavior Retry behavior after a fault
     * (tmc9660::tmcl::GdrvRetryBehaviour enum).
     * @return true if successfully configured.
     */
    bool setRetryBehavior(tmc9660::tmcl::GdrvRetryBehaviour retry_behavior) noexcept;

    /** @brief Configure drive fault behavior.
     *
     * @param fault_behavior Behavior after all retries fail
     * (tmc9660::tmcl::DriveFaultBehavior enum).
     * @return true if successfully configured.
     */
    bool setDriveFaultBehavior(tmc9660::tmcl::DriveFaultBehaviour fault_behavior) noexcept;

    /** @brief Set the maximum number of retries for fault handling.
     *
     * @param retries Maximum number of retries (0-255).
     * @return true if successfully configured.
     */
    bool setFaultHandlerRetries(uint8_t retries) noexcept;

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------

    /** @brief Configuration structure for auto-configuring power stage protection.
     *
     * This struct contains high-level physical properties of the power stage that
     * are used to automatically derive all low-level protection timing parameters.
     *
     * Instead of manually configuring dozens of blanking/deglitch registers, users
     * specify only the essential physical properties:
     * - MOSFET Rds_on: Used to compute overcurrent trip thresholds
     * - Gate charge (Qg): Determines switching speed → blanking/deglitch times
     * - Bus voltage: Used to estimate di/dt during switching
     * - PWM frequency: Limits maximum blanking time (must be < 10-20% of PWM period)
     * - Expected peak current: Validates trip thresholds are above normal operation
     * - Motor inductance (optional): For more accurate di/dt estimation
     *
     * @code
     * // Example: Basic configuration
     * TMC9660::GateDriver::PowerStageProfile profile;
     * profile.mosfet_RdsOn_mOhm = 3.8f;      // 3.8 mΩ at operating temperature
     * profile.mosfet_gateCharge_nC = 14.0f;  // 14 n_c gate charge
     * profile.shuntResistance_mOhm = 3.0f;   // 3 mΩ shunt resistors (for low-side sensing)
     * profile.busVoltage_V = 24.0f;          // 24V bus
     * profile.pwmFrequency_Hz = 32000.0f;    // 32 kHz PWM
     * profile.expectedPeakCurrent_A = 5.0f;   // 5A peak phase current
     * profile.motorInductance_uH = 50.0f;     // 50 µH phase inductance (optional, for gear motors)
     * driver.gateDriver.configurePowerStageProtection(profile);
     *
     * // Example: Custom switching speed (faster turn-on for lower EMI, slower turn-off for safety)
     * TMC9660::GateDriver::PowerStageProfile fastProfile;
     * // ... set required parameters ...
     * fast_profile.targetTurnOnTime_ns = 150.0f;   // Faster turn-on: 150ns (default: 200ns)
     * fast_profile.targetTurnOffTime_ns = 180.0f;  // Slower turn-off: 180ns (default: 135ns)
     * driver.gateDriver.configurePowerStageProtection(fast_profile);
     * @endcode
     */
    struct PowerStageProfile {
      // Required parameters
      float mosfet_RdsOn_mOhm; //!< MOSFET on-resistance in milliohms (at operating junction temp)
      float mosfet_gateCharge_nC; //!< MOSFET gate charge in nanocoulombs (Qg from datasheet)
      float shuntResistance_mOhm; //!< Low-side shunt resistor value in milliohms (R_SHUNT, used for
                                  //!< primary current sensing)
      float busVoltage_V;         //!< DC bus voltage in volts
      float pwmFrequency_Hz;      //!< PWM switching frequency in Hz
      float expectedPeakCurrent_A; //!< Expected peak phase current in amperes

      // Optional parameters with defaults
      float motorInductance_uH =
          std::numeric_limits<float>::quiet_NaN(); //!< Motor phase inductance in microhenries
                                                   //!< (optional, for accurate di/dt)

      // Safety margin multipliers (optional, with conservative defaults)
      float overcurrentMargin =
          1.5f; //!< Overcurrent threshold margin above expected peak (default: 1.5x)
      float blankingMargin = 1.2f; //!< Blanking time safety margin multiplier (default: 1.2x)

      // Fault handling behavior (optional, with safe defaults)
      tmc9660::tmcl::GdrvRetryBehaviour retryBehaviour =
          tmc9660::tmcl::GdrvRetryBehaviour::OPEN_CIRCUIT; //!< Behavior after gate driver fault
                                                           //!< (default: OPEN_CIRCUIT)
      tmc9660::tmcl::DriveFaultBehaviour faultBehaviour =
          tmc9660::tmcl::DriveFaultBehaviour::OPEN_CIRCUIT; //!< Behavior after all retries fail
                                                            //!< (default: OPEN_CIRCUIT)
      uint8_t faultHandlerRetries =
          5; //!< Maximum number of retries per detected fault [0-255] (default: 5)

      // Gate driver timing overrides (optional, for advanced tuning)
      std::optional<float>
          targetTurnOnTime_ns; //!< Override target turn-on time in nanoseconds (default: 200ns).
                               //!< Use for custom switching speed requirements.
      std::optional<float>
          targetTurnOffTime_ns; //!< Override target turn-off time in nanoseconds (default: 135ns).
                                //!< Use for custom switching speed requirements.

      // Gate driver interface configuration (optional, with safe defaults)
      tmc9660::tmcl::PwmOutputPolarity pwmLowPolarity =
          tmc9660::tmcl::PwmOutputPolarity::ACTIVE_HIGH; //!< PWM_L output polarity (default:
                                                         //!< ACTIVE_HIGH). Critical for hardware
                                                         //!< compatibility.
      tmc9660::tmcl::PwmOutputPolarity pwmHighPolarity =
          tmc9660::tmcl::PwmOutputPolarity::ACTIVE_HIGH; //!< PWM_H output polarity (default:
                                                         //!< ACTIVE_HIGH). Critical for hardware
                                                         //!< compatibility.

      // Undervoltage protection configuration (optional, with safe defaults)
      tmc9660::tmcl::UndervoltageLevel supplyLevel =
          tmc9660::tmcl::UndervoltageLevel::DISABLED; //!< Supply voltage (VS) undervoltage
                                                      //!< protection level (default: DISABLED).
                                                      //!< 0=disabled, 1-16 map to HW levels 0-15.

      /** When false, configurePowerStageProtection() programs only UVW (3-phase); all Y2 gate
       *  driver and Y2 overcurrent SAP writes are skipped. Use for 3-phase BLDC hardware or
       *  firmware builds where Y2 parameters are absent or return TMCL errors. Default true. */
      bool configure_y2_phase = true;

      /** When set, UVW gate sink current uses this enum instead of the Qg-derived selection. */
      std::optional<tmc9660::tmcl::GateCurrentSink> uvw_gate_current_sink;
      /** When set, UVW gate source current uses this enum instead of the Qg-derived selection. */
      std::optional<tmc9660::tmcl::GateCurrentSource> uvw_gate_current_source;
      /** When false, UVW_SINK_CURRENT / UVW_SOURCE_CURRENT (and Y2 pair when configure_y2_phase)
       *  are not written. Some firmware builds reject all values until a different init state. */
      bool program_gate_current_limits = true;

      /** When false, configurePowerStageProtection skips Part 2 (OC/VGS) and Part 3 (GDRV retry /
       *  drive fault / fault handler TMCL parameters 286–288). Use when firmware rejects those SAP
       *  writes; timing, bootstrap, and UV from Part 1 still run. Default true. */
      bool configure_gate_oc_vgs_protection = true;
    };

    /** @brief Auto-configure complete power stage based on physical properties.
     *
     * This method automatically configures the entire power stage including:
     * - Gate driver interface (PWM polarity)
     * - Gate driver timing (drive times, gate currents, dead time, bootstrap)
     * - Protection parameters (overcurrent thresholds, blanking, deglitch, VGS protection,
     * bootstrap UVP)
     * - Fault handling (retry behavior, fault behavior, retry count)
     *
     * All parameters are derived from high-level physical properties, eliminating
     * the need to manually configure dozens of low-level registers.
     *
     * @note This function does NOT enable the gate driver. After calling this function, you must:
     *   - Assert the DRV_EN hardware pin (if used)
     *   - Set COMMUTATION_MODE to a value other than SYSTEM_OFF (e.g., via setCommutationMode())
     *
     * @note Phase selection (3-phase vs 4-phase) is determined by boot configuration and
     * MOTOR_TYPE. This function configures all phases that are enabled by the system.
     * @note Set PowerStageProfile::configure_y2_phase to false to omit Y2 programming (pure
     *       3-phase stacks / firmware without Y2 SAP support).
     *
     * **Automatic Derivation Logic:**
     *
     * **PART 0: Gate Driver Interface Configuration**
     *
     * 0. **PWM Output Polarity**: Configurable via profile (default: ACTIVE_HIGH/ACTIVE_HIGH)
     *    - Critical for hardware compatibility - must match PCB layout and external gate driver
     * stages
     *    - Wrong polarity will prevent gate driver from working correctly
     *
     * **PART 1: Gate Driver Timing Configuration**
     *
     * 2. **Gate Current Limits**: Calculated from gate charge (Qg) and target switching times
     *    - Source current: I = Qg / target_turn_on_time (default: ~200ns, can be overridden)
     *    - Sink current: I = Qg / target_turn_off_time (default: ~135ns, can be overridden)
     *    - Automatically selects closest available enum values
     *    - Use `targetTurnOnTime_ns` and `targetTurnOffTime_ns` in profile to override defaults
     *
     * 3. **Drive Times**: Set to target switching times (default: 200ns turn-on, 135ns turn-off)
     *    - With adaptive drive time enabled, these are maximum times
     *    - Driver automatically optimizes down based on actual gate voltage monitoring
     *    - Can be customized via optional `targetTurnOnTime_ns` and `targetTurnOffTime_ns` profile
     * members
     *
     * 4. **Adaptive Drive Time**: Automatically enabled for efficiency
     *    - Monitors gate voltage and shortens drive times automatically
     *    - DRIVE_TIME_SINK acts as upper bound when adaptive mode is enabled
     *
     * 5. **Break-Before-Make (Dead Time)**: Set to 0 per documentation recommendation
     *    - Driver uses internal optimized timing
     *    - Can be overridden in future via PowerStageProfile if needed for special cases
     *
     * 6. **Bootstrap Current Limit**: Physics-based calculation from PWM frequency and gate charge
     *    - I_avg = (Qg × 3_phases × f_PWM) / duty_cycle × safety_margin
     *    - Accounts for 3 high-side FETs, 50% average duty cycle, 2.5× safety margin
     *    - Automatically selects closest available enum value
     *
     * 7. **Undervoltage Protection**: Configured from profile
     *    - Supply level: From profile.supplyLevel (0=disabled, 1-16 map to HW levels 0-15)
     *    - VDRV protection: Enabled by default for safety
     *    - Bootstrap UVP: Enabled by default for safety (prevents gate drive failure)
     *    - Enabled for both UVW and Y2 phases
     *
     * **PART 2: Protection Parameter Configuration**
     *
     * 8. **Overcurrent Threshold**: Computed from Rds_on and expected peak current
     *    - Threshold = expectedPeakCurrent_A * overcurrentMargin * Rds_on_mOhm (VDS)
     *    - Threshold = expectedPeakCurrent_A * overcurrentMargin * R_shunt_mOhm (RSHUNT)
     *    - Automatically selects appropriate register values for both sensing methods
     *
     * 9. **Blanking Time**: Derived from di/dt estimation
     *    - di/dt ≈ busVoltage_V / motorInductance_uH (or 20µH conservative estimate if unknown)
     *    - Blanking time selected based on di/dt range:
     *      - < 5 A/µs → 0.25 µs
     *      - 5-15 A/µs → 0.5 µs
     *      - 15-30 A/µs → 1.0 µs
     *      - 30-60 A/µs → 2.0 µs
     *      - > 60 A/µs → 4-8 µs (capped at 10-15% of PWM period, minimum 5% or 0.25µs)
     *    - Applied blankingMargin safety factor
     *
     * 10. **Deglitch Time**: Based on gate charge (Qg) and switching speed
     *    - Low Qg (< 15 n_c): Fast switching → longer deglitch (1-4 µs)
     *    - Medium Qg (15-40 n_c): Normal → moderate deglitch (0.5-1 µs)
     *    - High Qg (> 40 n_c): Slow switching → shorter deglitch (0.25-0.5 µs)
     *    - Applied blankingMargin safety factor
     *
     * 11. **VGS Short Protection**: Based on gate charge
     *    - Fast FETs (low Qg): Longer blanking/deglitch
     *    - Slow FETs (high Qg): Shorter blanking/deglitch
     *    - Automatically enabled for all transitions
     *
     * 12. **VDS Monitoring**: Automatically enabled if Rds_on < 10 mΩ for reliable sensing
     *    - Otherwise uses RSHUNT-based sensing (more accurate, recommended)
     *
     * **PART 3: Fault Handling Configuration**
     *
     * 13. **Retry Behavior**: Configurable behavior after gate driver fault
     *    - OPEN_CIRCUIT (default): Motor spins freely
     *    - ELECTRICAL_BRAKING: Enable braking if possible
     *
     * 14. **Drive Fault Behavior**: Configurable behavior after all retries fail
     *    - OPEN_CIRCUIT (default): Motor spins freely
     *    - ELECTRICAL_BRAKING: Enable braking if possible
     *    - MECHANICAL_BRAKING_AND_OPEN_CIRCUIT: Engage mechanical brake if configured
     *    - MECHANICAL_AND_ELECTRICAL_BRAKING: Both electrical and mechanical braking
     *
     * 15. **Fault Handler Retries**: Maximum number of retries per detected fault
     *    - Default: 5 retries (range: 0-255)
     *
     * @param profile Power stage physical properties (see PowerStageProfile)
     * @return true if configuration was successful
     *
     * @note All protection features are enabled by default. The method configures
     *       timing parameters only - protection enables are set to safe defaults.
     *
     * @note If motor inductance is not provided (NaN), conservative di/dt estimates
     *       are used based on typical motor sizes.
     */
    bool configurePowerStageProtection(const PowerStageProfile& profile) noexcept;

  private:
    friend class TMC9660;
    explicit GateDriver(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } gateDriver{*this};

  //***************************************************************************
  //**                  SUBSYSTEM: Feedback Sensors                        **//
  //***************************************************************************

  /** @brief Subsystem for feedback sensor configuration
   */
  struct FeedbackSense {
    // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    //  HALL sensors (digital Hall)
    // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

    /** @brief Configure digital Hall sensors for BLDC commutation.
     *
     * This enables Hall sensor inputs as the feedback for commutation.
     * Typically used with tmc9660::tmcl::CommutationMode::FOC_HALL.
     * @param sectorOffset Hall sensor 60-degree/sector offset
     * (tmc9660::tmcl::HallSectorOffset):
     *                     tmc9660::tmcl::HallSectorOffset::DEG_0, DEG_60,
     * DEG_120, DEG_180, DEG_240, DEG_300 This combines both the 120° order
     * offset and 180° polarity offset.
     * @param inverted If true, invert the interpretation of hall sensor signals
     * (tmc9660::tmcl::Direction).
     * @param enableExtrapolation If true, enable hall extrapolation for higher
     * resolution position signal (tmc9660::tmcl::EnableDisable).
     * @param filterLength Digital filter length (0-255) for hall sensor inputs.
     * @return true if Hall sensor feedback is configured successfully.
     */
    bool configureHall(
        tmc9660::tmcl::HallSectorOffset sectorOffset = tmc9660::tmcl::HallSectorOffset::DEG_0,
        tmc9660::tmcl::Direction inverted = tmc9660::tmcl::Direction::NOT_INVERTED,
        tmc9660::tmcl::EnableDisable enableExtrapolation = tmc9660::tmcl::EnableDisable::DISABLED,
        uint8_t filterLength = 0) noexcept;

    /** @brief Set Hall sensor position offsets for improved accuracy.
     *
     * Compensates for Hall sensor mounting tolerances by setting precise
     * electrical angle offsets.
     *
     * @param offset0 Offset for 0° Hall position (-32768 to 32767)
     * @param offset60 Offset for 60° Hall position (-32768 to 32767)
     * @param offset120 Offset for 120° Hall position (-32768 to 32767)
     * @param offset180 Offset for 180° Hall position (-32768 to 32767)
     * @param offset240 Offset for 240° Hall position (-32768 to 32767)
     * @param offset300 Offset for 300° Hall position (-32768 to 32767)
     * @param globalOffset Additional global offset applied to all positions
     * (-32768 to 32767)
     * @return true if Hall position offsets were set successfully.
     */
    bool setHallPositionOffsets(int16_t offset0 = 0, int16_t offset60 = 10922,
                                int16_t offset120 = 21845, int16_t offset180 = -32768,
                                int16_t offset240 = -21846, int16_t offset300 = -10923,
                                int16_t globalOffset = 0) noexcept;

    /** @brief Set Hall sensor position offsets using degrees.
     *
     * Convenience function that converts degrees to the internal 16-bit format.
     * The electrical angle is represented as: value = (degrees * 65536) / 360.
     *
     * @param offset0Deg Offset for 0° Hall position in degrees
     * @param offset60Deg Offset for 60° Hall position in degrees
     * @param offset120Deg Offset for 120° Hall position in degrees
     * @param offset180Deg Offset for 180° Hall position in degrees
     * @param offset240Deg Offset for 240° Hall position in degrees
     * @param offset300Deg Offset for 300° Hall position in degrees
     * @param globalOffsetDeg Additional global offset in degrees
     * @return true if Hall position offsets were set successfully.
     */
    bool setHallPositionOffsetsDegrees(float offset0Deg = 0.0f, float offset60Deg = 60.0f,
                                       float offset120Deg = 120.0f, float offset180Deg = 180.0f,
                                       float offset240Deg = 240.0f, float offset300Deg = 300.0f,
                                       float globalOffsetDeg = 0.0f) noexcept;

    /** @brief Set Hall sensor position offsets using radians.
     *
     * Convenience function that converts radians to the internal 16-bit format.
     * The electrical angle is represented as: value = (radians * 65536) / (2 * π).
     *
     * @param offset0Rad Offset for 0° Hall position in radians
     * @param offset60Rad Offset for 60° Hall position in radians
     * @param offset120Rad Offset for 120° Hall position in radians
     * @param offset180Rad Offset for 180° Hall position in radians
     * @param offset240Rad Offset for 240° Hall position in radians
     * @param offset300Rad Offset for 300° Hall position in radians
     * @param globalOffsetRad Additional global offset in radians
     * @return true if Hall position offsets were set successfully.
     */
    bool setHallPositionOffsetsRadians(float offset0Rad = 0.0f, float offset60Rad = 1.04719755f,
                                       float offset120Rad = 2.09439510f,
                                       float offset180Rad = 3.14159265f,
                                       float offset240Rad = 4.18879020f,
                                       float offset300Rad = 5.23598775f,
                                       float globalOffsetRad = 0.0f) noexcept;

    /** @brief Convert electrical angle from degrees to 16-bit format.
     *
     * Converts degrees to the internal representation: value = (degrees * 65536) / 360.
     * The result wraps around the 16-bit signed integer range.
     *
     * @param degrees Electrical angle in degrees
     * @return 16-bit signed integer value (-32768 to 32767)
     */
    static int16_t degreesToHallOffset(float degrees) noexcept;

    /** @brief Convert electrical angle from radians to 16-bit format.
     *
     * Converts radians to the internal representation: value = (radians * 65536) / (2 * π).
     * The result wraps around the 16-bit signed integer range.
     *
     * @param radians Electrical angle in radians
     * @return 16-bit signed integer value (-32768 to 32767)
     */
    static int16_t radiansToHallOffset(float radians) noexcept;

    /** @brief Read the electrical angle (phi_e) calculated from Hall feedback.
     * @param[out] phi_e Electrical angle (-32768 to 32767).
     * @return true if the value was read successfully.
     */
    bool getHallPhiE(int16_t& phi_e) noexcept;

    // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    //  ABN encoders (ABN1, ABN2)
    // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

    /** @brief Configure an ABN incremental encoder for feedback.
     *
     * Sets up an incremental quadrature encoder with optional index (N) channel
     * for position and velocity feedback.
     * @param counts_per_rev Encoder resolution (counts per revolution,
     * 0-16777215).
     * @param inverted If true, invert the encoder direction
     * (tmc9660::tmcl::Direction).
     * @param nChannelInverted If true, invert the N-channel signal (active low
     * instead of active high) (tmc9660::tmcl::EnableDisable).
     * @return true if encoder parameters were set successfully.
     */
    bool configureABNEncoder(
        uint32_t counts_per_rev,
        tmc9660::tmcl::Direction inverted = tmc9660::tmcl::Direction::NOT_INVERTED,
        tmc9660::tmcl::EnableDisable nChannelInverted =
            tmc9660::tmcl::EnableDisable::DISABLED) noexcept;

    /** @brief Configure ABN encoder initialization method.
     *
     * Sets the method used to align the ABN encoder with the rotor's absolute
     * position.
     *
     * @param initMethod Initialization method (tmc9660::tmcl::AbnInitMethod):
     *                   FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING,
     * FORCED_PHI_E_90_ZERO, USE_HALL, USE_N_CHANNEL_OFFSET
     * @param initDelay Delay in milliseconds to wait for mechanical
     * oscillations to stop (1000-10000)
     * @param initVelocity Velocity used during N-channel initialization
     * (-200000 to 200000)
     * @param nChannelOffset Offset between phi_e zero and encoder index pulse
     * position (-32768 to 32767)
     * @return true if ABN initialization parameters were set successfully.
     */
    bool configureABNInitialization(
        tmc9660::tmcl::AbnInitMethod initMethod =
            tmc9660::tmcl::AbnInitMethod::FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING,
        uint16_t initDelay = 1000, int32_t initVelocity = 5, int16_t nChannelOffset = 0) noexcept;

    /** @brief Read the current state of ABN encoder initialization.
     * @param[out] state Current initialization state
     * (tmc9660::tmcl::AbnInitState): IDLE, BUSY, WAIT, DONE
     * @return true if the state was read successfully.
     */
    bool getABNInitializationState(tmc9660::tmcl::AbnInitState& state) noexcept;

    /** @brief Read the electrical angle (phi_e) calculated from ABN feedback.
     * @param[out] phi_e Electrical angle (-32768 to 32767).
     * @return true if the value was read successfully.
     */
    bool getABNPhiE(int16_t& phi_e) noexcept;

    /** @brief Read the raw ABN encoder internal counter value.
     * @param[out] value Raw counter value (0-16777215).
     * @return true if the value was read successfully.
     */
    bool getABNRawValue(uint32_t& value) noexcept;

    /** @brief Configure N-channel filtering for ABN encoder.
     *
     * Sets up filtering for the N-channel (index pulse) to handle imprecise
     * encoders.
     *
     * @param filterMode N-channel filtering mode
     * (tmc9660::tmcl::AbnNChannelFiltering): FILTERING_OFF,
     * N_EVENT_ON_A_HIGH_B_HIGH, N_EVENT_ON_A_HIGH_B_LOW,
     * N_EVENT_ON_A_LOW_B_HIGH, N_EVENT_ON_A_LOW_B_LOW
     * @param clearOnNextNull If true, clear position counter on next N-channel
     * event (tmc9660::tmcl::EnableDisable).
     * @return true if N-channel settings were applied successfully.
     */
    bool configureABNNChannel(tmc9660::tmcl::AbnNChannelFiltering filterMode =
                                  tmc9660::tmcl::AbnNChannelFiltering::FILTERING_OFF,
                              tmc9660::tmcl::EnableDisable clearOnNextNull =
                                  tmc9660::tmcl::EnableDisable::DISABLED) noexcept;

    // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

    /** @brief Configure the secondary ABN encoder input.
     *
     * This allows the use of a second incremental encoder or a geared
     * encoder setup. It writes ABN_2_* parameters to set the resolution,
     * direction and optional gear ratio.
     *
     * @param counts_per_rev Encoder resolution in counts per revolution.
     * @param inverted     True to invert the encoder direction
     * (tmc9660::tmcl::Direction).
     * @param gearRatio    Gear ratio between the second encoder and the
     *                     motor shaft. Use 1 if directly coupled.
     * @return true if all parameters were written successfully.
     */
    bool configureSecondaryABNEncoder(
        uint32_t counts_per_rev,
        tmc9660::tmcl::Direction inverted = tmc9660::tmcl::Direction::NOT_INVERTED,
        uint8_t gearRatio = 1) noexcept;

    // ABN-2 (secondary encoder) getters
    /** @brief Read ABN_2_STEPS (encoder steps per rotation, 0…16777215).
     * @param[out] counts CPR value.
     * @return true if read successful.
     */
    bool getSecondaryABNCountsPerRev(uint32_t& counts) noexcept;

    /** @brief Read ABN_2_DIRECTION (normal/inverted).
     * @param[out] dir Direction (tmc9660::tmcl::Direction).
     * @return true if read successful.
     */
    bool getSecondaryABNDirection(tmc9660::tmcl::Direction& dir) noexcept;

    /** @brief Read ABN_2_GEAR_RATIO (1…255).
     * @param[out] ratio Gear ratio.
     * @return true if read successful.
     */
    bool getSecondaryABNGearRatio(uint8_t& ratio) noexcept;

    /** @brief Enable or disable the secondary ABN encoder.
     * @param enable True to enable, false to disable.
     * @return true if the operation was successful.
     */
    bool setSecondaryABNEncoderEnabled(bool enable) noexcept;

    /** @brief Read the raw ABN2 encoder internal counter value.
     * @param[out] value Raw counter value (0-4294967295).
     * @return true if the value was read successfully.
     */
    bool getSecondaryABNEncoderValue(uint32_t& value) noexcept;

    // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

    // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    //  SPI encoder timing & frame size
    // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

    /** @brief Configure a SPI-based encoder for feedback.
     *
     * Sets up a digital SPI encoder (e.g., absolute magnetic encoder) for
     * position feedback.
     *
     * @param cmd_size Size of SPI transfer frame (1-16 bytes).
     * @param cs_settle_time_ns CS settle time in nanoseconds (0-6375).
     * @param cs_idle_time_us CS idle time between frames in microseconds (0-102).
     * @return true if configured successfully.
     */
    bool configureSPIEncoder(uint8_t cmd_size, uint16_t cs_settle_time_ns,
                             uint8_t cs_idle_time_us) noexcept;

    /** @brief Configure SPI encoder data format and processing.
     *
     * Sets up how the position data is extracted from the SPI encoder response.
     *
     * @param position_mask Bit mask to extract position from SPI response.
     * @param positionShift Right shift value to apply to position counter.
     * @param invertDirection If true, invert the direction of the SPI encoder
     * (tmc9660::tmcl::Direction).
     * @return true if configuration was successful.
     */
    bool configureSPIEncoderDataFormat(
        uint32_t position_mask, uint8_t positionShift = 0,
        tmc9660::tmcl::Direction invertDirection = tmc9660::tmcl::Direction::NOT_INVERTED) noexcept;

    /** @brief Set up SPI encoder request data for continuous transfer mode.
     *
     * Sets the data to be sent to the SPI encoder during position acquisition.
     *
     * @param requestData Array of data bytes to send to the encoder.
     * @param size Size of the request data (1-16 bytes).
     * @return true if transfer data was set successfully.
     */
    bool setSPIEncoderRequestData(const uint8_t* requestData, uint8_t size) noexcept;

    /** @brief Configure SPI encoder initialization method.
     *
     * Sets how the SPI encoder is initialized for commutation.
     *
     * @param init_method Initialization method (tmc9660::tmcl::SpiInitMethod):
     *                   FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING,
     * FORCED_PHI_E_90_ZERO, USE_OFFSET
     * @param offset Manual offset value if using offset-based initialization.
     * @return true if initialization method was set successfully.
     */
    bool configureSPIEncoderInitialization(tmc9660::tmcl::SpiInitMethod init_method,
                                           int16_t offset = 0) noexcept;

    /** @brief Enable or disable SPI encoder lookup table correction.
     *
     * Enables the lookup table-based correction for encoder nonlinearity.
     *
     * @param enable If true, enable LUT correction
     * (tmc9660::tmcl::EnableDisable).
     * @param shiftFactor Common shift factor for all LUT entries.
     * @return true if LUT settings were applied successfully.
     */
    bool setSPIEncoderLUTCorrection(tmc9660::tmcl::EnableDisable enable,
                                    int8_t shiftFactor = 0) noexcept;

    /** @brief Upload a single entry to the SPI encoder correction lookup table.
     *
     * @param index Index in the LUT (0-255).
     * @param value Correction value (-128 to 127).
     * @return true if the entry was uploaded successfully.
     */
    bool uploadSPIEncoderLUTEntry(uint8_t index, int8_t value) noexcept;

    // SPI encoder timing & frame size
    /** @brief Read SPI_ENCODE_CS_SETTLE_DELAY_TIME (0…6375 ns).
     */
    bool getSPIEncoderCSSettleDelay(uint16_t& timeNs) noexcept;

    /** @brief Read SPI_ENCODER_CS_IDLE_DELAY_TIME (0…102 µs).
     */
    bool getSPIEncoderCSIdleDelay(uint8_t& timeUs) noexcept;

    /** @brief Read SPI_ENCODER_MAIN_TRANSFER_CMD_SIZE (1…16 bytes).
     */
    bool getSPIEncoderMainCmdSize(uint8_t& size) noexcept;

    /** @brief Read SPI_ENCODER_SECONDARY_TRANSFER_CMD_SIZE (0…15 bytes).
     */
    bool getSPIEncoderSecondaryCmdSize(uint8_t& size) noexcept;

    // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    //  SPI encoder data & status
    /** @brief Read SPI_ENCODER_POSITION_COUNTER_MASK.
     */
    bool getSPIEncoderPositionMask(uint32_t& mask) noexcept;

    /** @brief Read SPI_ENCODER_POSITION_COUNTER_SHIFT.
     */
    bool getSPIEncoderPositionShift(uint8_t& shift) noexcept;

    /** @brief Read SPI_ENCODER_POSITION_COUNTER_VALUE.
     */
    bool getSPIEncoderPositionValue(uint32_t& value) noexcept;

    /** @brief Read SPI_ENCODER_COMMUTATION_ANGLE (-32768…32767).
     */
    bool getSPIEncoderCommutationAngle(int16_t& angle) noexcept;

    // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    //  SPI encoder initialization & offset
    /** @brief Read SPI_ENCODER_INITIALIZATION_METHOD and SPI_ENCODER_OFFSET.
     */
    bool getSPIEncoderInitialization(tmc9660::tmcl::SpiInitMethod& method,
                                     int16_t& offset) noexcept;

    /** @brief Read SPI_ENCODER_DIRECTION.
     */
    bool getSPIEncoderDirection(tmc9660::tmcl::Direction& dir) noexcept;

    // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    //  SPI LUT
    /** @brief Read SPI_LUT_ADDRESS_SELECT.
     */
    bool getSPIEncoderLUTAddress(uint8_t& address) noexcept;

    /** @brief Read SPI_LUT_DATA.
     */
    bool getSPIEncoderLUTData(int8_t& data) noexcept;

    /** @brief Read SPI_LUT_COMMON_SHIFT_FACTOR.
     */
    bool getSPIEncoderLUTShiftFactor(int8_t& shift_factor) noexcept;

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------

    /** @brief Configuration structure for Hall sensor auto-configuration.
     */
    struct HallConfig {
      // Required parameters
      tmc9660::tmcl::HallSectorOffset sectorOffset =
          tmc9660::tmcl::HallSectorOffset::DEG_0; //!< Hall sensor 60-degree/sector offset (default:
                                                  //!< DEG_0)

      // Optional parameters with defaults
      tmc9660::tmcl::Direction direction =
          tmc9660::tmcl::Direction::NOT_INVERTED; //!< Hall sensor direction (default: NOT_INVERTED)
      tmc9660::tmcl::EnableDisable extrapolation =
          tmc9660::tmcl::EnableDisable::DISABLED; //!< Enable Hall extrapolation for higher
                                                  //!< resolution (default: DISABLED)
      uint8_t filterLength =
          0; //!< Digital filter length for Hall inputs [0-255] (default: 0, no filtering)

      // Position offsets (optional, use NaN to skip)
      std::optional<float>
          offset0Deg; //!< Offset for 0° Hall position in degrees (optional, default: 0°)
      std::optional<float>
          offset60Deg; //!< Offset for 60° Hall position in degrees (optional, default: 60°)
      std::optional<float>
          offset120Deg; //!< Offset for 120° Hall position in degrees (optional, default: 120°)
      std::optional<float>
          offset180Deg; //!< Offset for 180° Hall position in degrees (optional, default: 180°)
      std::optional<float>
          offset240Deg; //!< Offset for 240° Hall position in degrees (optional, default: 240°)
      std::optional<float>
          offset300Deg; //!< Offset for 300° Hall position in degrees (optional, default: 300°)
      std::optional<float> globalOffsetDeg; //!< Global offset in degrees (optional, default: 0°)
    };

    /** @brief Configuration structure for ABN encoder auto-configuration.
     */
    struct AbnConfig {
      // Required parameters
      uint32_t countsPerRev; //!< Encoder resolution in counts per revolution (CPR) [0-16777215]

      // Optional parameters with defaults
      tmc9660::tmcl::Direction direction =
          tmc9660::tmcl::Direction::NOT_INVERTED; //!< Encoder direction (default: NOT_INVERTED)
      tmc9660::tmcl::EnableDisable nChannelInverted =
          tmc9660::tmcl::EnableDisable::DISABLED; //!< N-channel (index) inversion (default:
                                                  //!< DISABLED, active high)

      // Initialization parameters (optional)
      tmc9660::tmcl::AbnInitMethod initMethod = tmc9660::tmcl::AbnInitMethod::
          FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING; //!< Initialization method (default:
                                               //!< FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING)
      uint16_t initDelay = 1000;  //!< Initialization delay in ms [1000-10000] (default: 1000ms)
      int32_t initVelocity = 5;   //!< Initialization velocity [-200000 to 200000] (default: 5)
      int16_t nChannelOffset = 0; //!< N-channel offset [-32768 to 32767] (default: 0)

      // Open-loop current used during FORCED_PHI_E_* alignment (param 46, mA).
      // The chip-level OPENLOOP_CURRENT parameter is what physically drives the
      // rotor to a known electrical angle during ABN init — NOT
      // OPENLOOP_VOLTAGE. Default chip value is 1000 mA which can be too low
      // for gearboxed or high-stiction motors and silently leave the rotor
      // static; init then "completes" with a bogus phi_e offset and FOC_ABN
      // produces vibration with near-zero net torque. Set this explicitly when
      // using FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING or FORCED_PHI_E_90_ZERO.
      // 0 = leave OPENLOOP_CURRENT untouched (use whatever was previously set).
      uint16_t initOpenloopCurrent_mA =
          0; //!< OPENLOOP_CURRENT for FORCED_PHI_E_* init [0-65535 mA]; 0 = do not modify (default: 0)

      // N-channel filtering (optional)
      tmc9660::tmcl::AbnNChannelFiltering nChannelFiltering =
          tmc9660::tmcl::AbnNChannelFiltering::FILTERING_OFF; //!< N-channel filtering mode
                                                              //!< (default: FILTERING_OFF)
      tmc9660::tmcl::EnableDisable clearOnNextNull =
          tmc9660::tmcl::EnableDisable::DISABLED; //!< Clear position on next N-channel event
                                                  //!< (default: DISABLED)
    };

    /** @brief Configuration structure for ABN2 (secondary) encoder auto-configuration.
     */
    struct Abn2Config {
      // Required parameters
      uint32_t countsPerRev; //!< Encoder resolution in counts per revolution (CPR) [0-16777215]

      // Optional parameters with defaults
      tmc9660::tmcl::Direction direction =
          tmc9660::tmcl::Direction::NOT_INVERTED; //!< Encoder direction (default: NOT_INVERTED)
      uint8_t gearRatio =
          1; //!< Gear ratio between encoder and motor shaft [1-255] (default: 1, directly coupled)
      bool enable = true; //!< Enable the ABN2 encoder (default: true)
    };

    /** @brief Configuration structure for SPI encoder auto-configuration.
     */
    struct SpiEncoderConfig {
      // Required parameters
      uint8_t cmdSize;       //!< Size of SPI transfer frame [1-16 bytes]
      uint32_t positionMask; //!< Bit mask to extract position from SPI response [0-4294967295]

      // Optional parameters with defaults
      uint16_t csSettleTimeNs = 0; //!< CS settle delay time [0-6375 ns] (default: 0)
      uint8_t csIdleTimeUs = 0;    //!< CS idle time between frames [0-102 µs] (default: 0)
      uint8_t positionShift = 0;   //!< Right shift for position counter [0-127] (default: 0)
      tmc9660::tmcl::Direction direction =
          tmc9660::tmcl::Direction::NOT_INVERTED; //!< SPI encoder direction (default: NOT_INVERTED)

      // Initialization parameters (optional)
      tmc9660::tmcl::SpiInitMethod initMethod = tmc9660::tmcl::SpiInitMethod::
          FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING; //!< Initialization method (default:
                                               //!< FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING)
      int16_t offset = 0; //!< Manual offset for USE_OFFSET initialization method (default: 0)

      // Request data for continuous transfer (optional)
      std::optional<std::array<uint8_t, 16>>
          requestData; //!< Request data bytes to send to encoder (optional, for continuous transfer
                       //!< mode)

      // LUT correction (optional)
      tmc9660::tmcl::EnableDisable lutCorrection =
          tmc9660::tmcl::EnableDisable::DISABLED; //!< Enable lookup table correction (default:
                                                  //!< DISABLED)
      int8_t lutShiftFactor = 0;                  //!< LUT common shift factor [0-4] (default: 0)
    };

    /** @brief Auto-configure Hall sensor feedback.
     *
     * Configures all Hall sensor parameters including offsets, filtering, and extrapolation.
     *
     * @param config Hall sensor configuration (see HallConfig)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const HallConfig& config) noexcept;

    /** @brief Auto-configure ABN encoder feedback.
     *
     * Configures ABN encoder including initialization method, N-channel settings, and filtering.
     *
     * @warning When `config.initMethod` is `FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING`
     *          or `FORCED_PHI_E_90_ZERO`, the chip aligns the rotor by driving
     *          a constant current set by parameter `OPENLOOP_CURRENT` (param 46, mA),
     *          NOT by `OPENLOOP_VOLTAGE`. The chip default is 1000 mA which can
     *          be insufficient for gearboxed / high-stiction motors and will
     *          silently leave the rotor static while reporting init "DONE" with
     *          a bogus phi_e offset — FOC_ABN will then produce vibration only.
     *          Set `config.initOpenloopCurrent_mA` to the alignment current you
     *          need (must be ≤ MAX_TORQUE), or write OPENLOOP_CURRENT yourself
     *          before engaging FOC_ABN.
     *
     * @param config ABN encoder configuration (see AbnConfig)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const AbnConfig& config) noexcept;

    /** @brief Auto-configure ABN2 (secondary) encoder feedback.
     *
     * Configures the secondary ABN encoder for position/velocity feedback (cannot be used for
     * commutation).
     *
     * @param config ABN2 encoder configuration (see Abn2Config)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const Abn2Config& config) noexcept;

    /** @brief Auto-configure SPI encoder feedback.
     *
     * Configures SPI encoder including timing, data format, initialization, and optional LUT
     * correction.
     *
     * @param config SPI encoder configuration (see SpiEncoderConfig)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const SpiEncoderConfig& config) noexcept;

    // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

  private:
    friend class TMC9660;
    explicit FeedbackSense(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } feedbackSense{*this};

  //***************************************************************************
  //**                  SUBSYSTEM: Torque/Flux Control (FOC)                **//
  //***************************************************************************

  /** @brief Subsystem for torque and flux current control (FOC inner loop).
   *
   * Controls the motor currents (torque and flux) using hardware PI controllers.
   * Covers TMCL parameters 104–120, 308, 310.
   *
   * This is the innermost control loop, operating at PWM frequency.
   * Torque control is used for direct current control, while flux control enables
   * field weakening for high-speed operation.
   */
  struct TorqueFluxControl {
    //-------------------------------------------------------------------------
    // Torque / Flux control (104–120)
    //-------------------------------------------------------------------------
    /** @brief Stop torque/flux control (SYSTEM_OFF).
     * @return true on success.
     */
    bool stop() noexcept;

    /** @brief Set desired torque.
     * @param milliamps Target torque in m_a.
     * @return true if written.
     */
    bool setTargetTorque(int16_t milliamps) noexcept;
    /** @brief Read actual torque.
     * @param[out] milliamps Actual torque in m_a.
     * @return true if read.
     */
    bool getActualTorque(int16_t& milliamps) noexcept;

    /** @brief Set desired flux current.
     * @param milliamps Target flux in m_a.
     * @return true if written.
     */
    bool setTargetFlux(int16_t milliamps) noexcept;
    /** @brief Read actual flux current.
     * @param[out] milliamps Actual flux in m_a.
     * @return true if read.
     */
    bool getActualFlux(int32_t& milliamps) noexcept;

    /** @brief Set torque offset (feed-forward).
     * @param milliamps Offset in m_a.
     * @return true if written.
     */
    bool setTorqueOffset(int16_t milliamps) noexcept;
    /** @brief Read torque offset.
     * @param[out] milliamps Offset in m_a.
     * @return true if read.
     */
    bool getTorqueOffset(int16_t& milliamps) noexcept;

    /** @brief Set flux offset (feed-forward).
     * @param milliamps Offset in m_a.
     * @return true if written.
     */
    bool setFluxOffset(int16_t milliamps) noexcept;
    /** @brief Read flux offset.
     * @param[out] milliamps Offset in m_a.
     * @return true if read.
     */
    bool getFluxOffset(int16_t& milliamps) noexcept;

    /** @brief Configure current-loop PI gains.
     * @param p          Proportional gain for torque (and flux if not
     * separate).
     * @param i          Integral gain for torque (and flux if not separate).
     * @param separate   true to use separate flux gains.
     * @param fluxP      Proportional gain for flux loop.
     * @param fluxI      Integral gain for flux loop.
     * @return true if written.
     */
    bool setCurrentLoopGains(uint16_t p, uint16_t i, bool separate = false, uint16_t fluxP = 0,
                             uint16_t fluxI = 0) noexcept;
    /** @brief Select combined or separate torque/flux PI parameters.
     * @param sep Separation mode.
     * @return true if written.
     */
    bool setTorqueFluxPiSeparation(tmc9660::tmcl::TorqueFluxPiSeparation sep) noexcept;

    /** @brief Set normalization for current-PI outputs.
     * @param p_norm Normalization for P-term.
     * @param i_norm Normalization for I-term.
     * @return true if written.
     */
    bool setCurrentNormalization(tmc9660::tmcl::CurrentPiNormalization p_norm,
                                 tmc9660::tmcl::CurrentPiNormalization i_norm) noexcept;

    /** @brief Read torque PI error.
     * @param[out] error Current torque-PI error.
     * @return true if read.
     */
    bool getTorquePiError(int32_t& error) noexcept;
    /** @brief Read flux PI error.
     * @param[out] error Current flux-PI error.
     * @return true if read.
     */
    bool getFluxPiError(int32_t& error) noexcept;
    /** @brief Read torque-PI integrator state.
     * @param[out] integrator Integrator value.
     * @return true if read.
     */
    bool getTorquePiIntegrator(int32_t& integrator) noexcept;
    /** @brief Read flux-PI integrator state.
     * @param[out] integrator Integrator value.
     * @return true if read.
     */
    bool getFluxPiIntegrator(int32_t& integrator) noexcept;

    //-------------------------------------------------------------------------
    // Open‐loop support (45–47)
    //-------------------------------------------------------------------------
    /** @brief Read open-loop angle.
     * @param[out] angle Electrical angle.
     * @return true if read.
     */
    bool getOpenloopAngle(int16_t& angle) noexcept;

    /** @brief Set open-loop current.
     * @param milliamps Current in m_a.
     * @return true if written.
     */
    bool setOpenloopCurrent(uint16_t milliamps) noexcept;
    /** @brief Read open-loop current.
     * @param[out] milliamps Current in m_a.
     * @return true if read.
     */
    bool getOpenloopCurrent(uint16_t& milliamps) noexcept;

    /** @brief Set open-loop voltage (PWM duty cycle, relative to V_BUS).
     * @param voltage Voltage register value (0..16383, 14-bit; per parameter
     *                NR 47 OPENLOOP_VOLTAGE). 16383 = 100% duty. The chip does
     *                **not** current-limit voltage mode, so use a low value
     *                (≈5% = 800) for unloaded startup.
     * @return true if written.
     */
    bool setOpenloopVoltage(uint16_t voltage) noexcept;
    /** @brief Read open-loop voltage.
     * @param[out] voltage Voltage unit.
     * @return true if read.
     */
    bool getOpenloopVoltage(uint16_t& voltage) noexcept;

    //-------------------------------------------------------------------------
    // Field-weakening (308, 310)
    //-------------------------------------------------------------------------
    /// @name Field-weakening (read/write)
    ///@{
    bool setFieldWeakeningI(uint16_t milliamps) noexcept;
    bool getFieldWeakeningI(uint16_t& milliamps) noexcept;

    bool setFieldWeakeningVoltageThreshold(uint16_t voltage) noexcept;
    bool getFieldWeakeningVoltageThreshold(uint16_t& voltage) noexcept;
    ///@}

    /// @name Target Torque Biquad Filter (read/write)
    ///@{
    bool setTargetTorqueBiquadFilterEnable(bool enable) noexcept;
    bool getTargetTorqueBiquadFilterEnable(bool& enable) noexcept;

    bool setTargetTorqueBiquadFilterACoeff1(int32_t coeff) noexcept;
    bool getTargetTorqueBiquadFilterACoeff1(int32_t& coeff) noexcept;

    bool setTargetTorqueBiquadFilterACoeff2(int32_t coeff) noexcept;
    bool getTargetTorqueBiquadFilterACoeff2(int32_t& coeff) noexcept;

    bool setTargetTorqueBiquadFilterBCoeff0(int32_t coeff) noexcept;
    bool getTargetTorqueBiquadFilterBCoeff0(int32_t& coeff) noexcept;

    bool setTargetTorqueBiquadFilterBCoeff1(int32_t coeff) noexcept;
    bool getTargetTorqueBiquadFilterBCoeff1(int32_t& coeff) noexcept;

    bool setTargetTorqueBiquadFilterBCoeff2(int32_t coeff) noexcept;
    bool getTargetTorqueBiquadFilterBCoeff2(int32_t& coeff) noexcept;
    ///@}

    /// @name Intermediate FOC Voltages (read-only)
    ///@{
    bool getFocVoltageUx(int16_t& voltage) noexcept;
    bool getFocVoltageWy(int16_t& voltage) noexcept;
    bool getFocVoltageV(int16_t& voltage) noexcept;
    bool getFocVoltageUq(int16_t& voltage) noexcept;
    ///@}

    /// @name Intermediate FOC Currents (read-only)
    ///@{
    bool getFocCurrentUx(int16_t& milliamps) noexcept;
    bool getFocCurrentV(int16_t& milliamps) noexcept;
    bool getFocCurrentWy(int16_t& milliamps) noexcept;
    bool getFocCurrentIq(int16_t& milliamps) noexcept;
    ///@}

    /// @name Combined & integrated raw measurements (read-only)
    ///@{
    bool getTorqueFluxCombinedTargetValues(uint32_t& value) noexcept;
    bool getTorqueFluxCombinedActualValues(uint32_t& value) noexcept;
    bool getVoltageDqCombinedActualValues(uint32_t& value) noexcept;
    bool getIntegratedActualTorqueValue(uint32_t& value) noexcept;
    ///@}

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------

    /** @brief Configuration structure for torque/flux control auto-configuration.
     *
     * This structure provides high-level, user-friendly parameters for configuring
     * the torque and flux current control loops. Complex low-level parameters are
     * automatically derived from these intuitive settings.
     */
    struct TorqueFluxConfig {
      // ========================================================================
      // PI Controller Configuration
      // ========================================================================

      /** @brief Direct PI gain configuration (optional).
       *
       * If provided, these values are used directly. If not provided, default
       * values are used (P=50, I=100). Use this for fine-tuning or when you know
       * the exact gains you need.
       *
       * Typical values:
       * - P: 30-100 (higher = faster response)
       * - I: 50-200 (higher = better steady-state accuracy)
       */
      std::optional<uint16_t>
          torqueP; //!< Torque P gain [0-32767] (optional, defaults to 50 if not provided)
      std::optional<uint16_t>
          torqueI; //!< Torque I gain [0-32767] (optional, defaults to 100 if not provided)

      /** @brief Use separate PI parameters for torque and flux loops.
       *
       * When false (default): Torque and flux share the same PI gains.
       * When true: Torque and flux have independent PI gains (flux_p, flux_i).
       *
       * Separate loops are useful when torque and flux have different response
       * requirements, but most applications work fine with combined loops.
       */
      bool separateTorqueFluxLoops =
          false; //!< Use separate PI parameters for torque and flux (default: false, combined)

      /** @brief Flux PI gains (only used if separateTorqueFluxLoops = true).
       *
       * If separate_torque_flux_loops is true and these are not provided, they
       * default to the same values as torqueP/torqueI.
       */
      std::optional<uint16_t>
          fluxP; //!< Flux P gain [0-32767] (optional, only used if separateTorqueFluxLoops = true)
      std::optional<uint16_t>
          fluxI; //!< Flux I gain [0-32767] (optional, only used if separateTorqueFluxLoops = true)

      /** @brief PI normalization format (optional).
       *
       * These control how the PI gains are normalized internally. If not provided,
       * defaults are used (SHIFT_8_BIT for P, SHIFT_16_BIT for I) which match the
       * datasheet defaults and work well for most applications.
       *
       * - SHIFT_8_BIT: Standard normalization, good balance
       * - SHIFT_16_BIT: Higher precision, better for fine control
       *
       * Typically, P uses 8-bit (faster, standard) and I uses 16-bit (better precision).
       * Only override if you have specific requirements.
       */
      std::optional<tmc9660::tmcl::CurrentPiNormalization>
          pNormalization; //!< P-term normalization format (optional, defaults to SHIFT_8_BIT)
      std::optional<tmc9660::tmcl::CurrentPiNormalization>
          iNormalization; //!< I-term normalization format (optional, defaults to SHIFT_16_BIT)

      // ========================================================================
      // Field Weakening Configuration
      // ========================================================================

      /** @brief Enable field weakening for high-speed operation.
       *
       * Field weakening allows the motor to operate beyond its base speed by
       * reducing the back-EMF through negative flux current. This is essential
       * for high-speed BLDC/stepper motors.
       *
       * When enabled, field weakening activates automatically when the motor
       * voltage exceeds field_weakening_voltage_threshold_percent.
       */
      bool enableFieldWeakening = false; //!< Enable field weakening (default: false)

      /** @brief Field weakening voltage threshold as percentage of OUTPUT_VOLTAGE_LIMIT.
       *
       * Field weakening activates when motor voltage exceeds this threshold.
       * Expressed as a fraction (0.0 to 1.0) of the OUTPUT_VOLTAGE_LIMIT.
       *
       * Typical values:
       * - 0.80-0.90: Standard field weakening (activates at 80-90% of voltage limit)
       * - 0.85: Recommended starting point
       *
       * Example: If OUTPUT_VOLTAGE_LIMIT = 8000 and threshold = 0.85,
       * field weakening activates at 6800 (85% of 8000).
       */
      float fieldWeakeningVoltageThresholdPercent =
          0.85f; //!< Voltage threshold for field weakening [0.0-1.0] as fraction of
                 //!< OUTPUT_VOLTAGE_LIMIT (default: 0.85 = 85%)

      /** @brief Field weakening I-controller gain.
       *
       * Higher values = more aggressive field weakening (faster response to
       * voltage limit, but may cause instability).
       * Lower values = gentler field weakening (slower response, more stable).
       *
       * Typical values:
       * - 50-200: Standard field weakening
       * - 100: Recommended starting point
       * - >200: Aggressive (may cause oscillations)
       */
      uint16_t fieldWeakeningI = 100; //!< Field weakening I gain [0-32767] (default: 100, only used
                                      //!< if enableFieldWeakening = true)

      // ========================================================================
      // Advanced Configuration (Optional)
      // ========================================================================

      /** @brief Current offset compensation.
       *
       * These offsets are added to the torque/flux commands to compensate for
       * measurement errors or motor asymmetries. Usually set to 0 unless
       * calibration reveals systematic offsets.
       */
      int16_t torqueOffset_mA = 0; //!< Torque offset in m_a [-4700 to 4700] (default: 0)
      int16_t fluxOffset_mA = 0;   //!< Flux offset in m_a [-4700 to 4700] (default: 0)

      /** @brief Enable biquad filter on target torque command.
       *
       * The biquad filter can be used to smooth the torque command, reducing
       * high-frequency noise or oscillations. This is particularly useful when
       * using velocity or position control, where the velocity/position controller
       * output is used as the target torque.
       *
       * When enabled, you can optionally provide filter coefficients below.
       * If coefficients are not provided, the filter is enabled but uses default
       * coefficient values (BCOEFF_0 = 1048576, others = 0).
       *
       * Default: Disabled (filter is off by default).
       * Most users should leave this disabled unless experiencing torque
       * command oscillations or need to smooth controller outputs.
       */
      bool enableTorqueBiquadFilter =
          false; //!< Enable biquad filter on target torque (default: false)

      /** @brief Biquad filter coefficients (optional, only used if enableTorqueBiquadFilter =
       * true).
       *
       * These coefficients define the biquad filter difference equation:
       * Y(n) = X(n)*b0 + X(n-1)*b1 + X(n-2)*b2 - Y(n-1)*a1 - Y(n-2)*a2
       *
       * Or in z-domain transfer function form:
       * H(z) = (B0 + B1*z^-1 + B2*z^-2) / (1 + A1*z^-1 + A2*z^-2)
       *
       * **Coefficient Format:**
       * All coefficients are 24-bit values in Q4.20 fixed-point format:
       * - 4 integer bits (range: -8 to +7)
       * - 20 fractional bits (resolution: 1/1048576)
       * - To convert a floating-point value to Q4.20: `coeff = (float_value * 1048576.0f)`
       * - Example: 1.0 → 1048576, 0.5 → 524288, -0.25 → -262144
       *
       * **Default Values:**
       * If not provided, the filter uses default coefficients:
       * - BCOEFF_0 = 1048576 (1.0 in Q4.20, unity gain)
       * - All other coefficients = 0 (no filtering, passes signal through)
       *
       * **Typical Use Cases:**
       * - Low-pass filter: Smooth high-frequency noise from velocity/position controllers
       * - Notch filter: Remove specific oscillation frequencies
       * - Custom filtering: Based on motor/system characteristics
       *
       * **Note:** For most applications, leave these unset unless you have specific
       * filtering requirements. The default values provide a simple pass-through filter.
       */
      std::optional<int32_t> biquadACoeff1; //!< Biquad A coefficient 1 (a1) in Q4.20 format
                                            //!< [-2147483648 to 2147483647] (optional, default: 0)
      std::optional<int32_t> biquadACoeff2; //!< Biquad A coefficient 2 (a2) in Q4.20 format
                                            //!< [-2147483648 to 2147483647] (optional, default: 0)
      std::optional<int32_t>
          biquadBCoeff0; //!< Biquad B coefficient 0 (b0) in Q4.20 format [-2147483648 to
                         //!< 2147483647] (optional, default: 1048576 = 1.0)
      std::optional<int32_t> biquadBCoeff1; //!< Biquad B coefficient 1 (b1) in Q4.20 format
                                            //!< [-2147483648 to 2147483647] (optional, default: 0)
      std::optional<int32_t> biquadBCoeff2; //!< Biquad B coefficient 2 (b2) in Q4.20 format
                                            //!< [-2147483648 to 2147483647] (optional, default: 0)
    };

    /** @brief Auto-configure torque/flux control parameters.
     *
     * Configures PI gains, normalization, field weakening, and offsets based on
     * high-level control characteristics.
     *
     * @param config Torque/flux control configuration (see TorqueFluxConfig)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const TorqueFluxConfig& config) noexcept;

  private:
    friend class TMC9660;
    explicit TorqueFluxControl(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } torqueFluxControl{*this};

  //***************************************************************************
  //**                  SUBSYSTEM: Velocity Control                           **//
  //***************************************************************************

  /** @brief Subsystem for velocity control (FOC middle loop).
   *
   * Controls motor velocity using a PI controller that outputs torque commands.
   * Covers TMCL parameters 123–139.
   *
   * This control loop operates at a downsampled frequency relative to PWM.
   * Velocity feedback can come from various sensors (ABN, Hall, SPI encoder).
   */
  struct VelocityControl {
    //-------------------------------------------------------------------------
    // Velocity control (123–139)
    //-------------------------------------------------------------------------
    /** @brief Stop velocity control (SYSTEM_OFF).
     * @return true on success.
     */
    bool stop() noexcept;

    /** @brief Select velocity feedback sensor.
     * @param sel Sensor selection.
     * @return true if written.
     */
    bool setVelocitySensor(tmc9660::tmcl::VelocitySensorSelection sel) noexcept;
    /** @brief Read velocity feedback sensor.
     * @param[out] sel Sensor selection.
     * @return true if read.
     */
    bool getVelocitySensor(tmc9660::tmcl::VelocitySensorSelection& sel) noexcept;

    /** @brief Set target velocity in any engineering unit (canonical API).
     *
     * Canonical motor-shaft velocity setter. The `(value, unit, ctx)` form
     * forces the caller to be explicit about which unit the number is in;
     * thanks to the dedicated `VelocityUnit` enum class, mixing positions
     * or accelerations here is a compile error.
     *
     * @param value Target velocity expressed in `unit`.
     * @param unit  Engineering unit of `value` (RPM / RPS / rad·s⁻¹ / deg·s⁻¹ / Internal).
     * @param ctx   Motor context (pole pairs, sensor selection, encoder CPR)
     *              used to derive `k_RPM`. Acquire once via `getMotorContext()`.
     * @return true if the SAP succeeded.
     */
    bool setTargetVelocity(double value,
                           ::tmc9660::units::VelocityUnit unit,
                           ::tmc9660::units::MotorContext const& ctx) noexcept;

    /// Convenience: set target velocity in mechanical RPM.
    bool setTargetVelocityRpm(double rpm, ::tmc9660::units::MotorContext const& ctx) noexcept {
        return setTargetVelocity(rpm, ::tmc9660::units::VelocityUnit::Rpm, ctx);
    }
    /// Convenience: set target velocity in rad/s (mechanical).
    bool setTargetVelocityRadPerSec(double rad_per_s,
                                    ::tmc9660::units::MotorContext const& ctx) noexcept {
        return setTargetVelocity(rad_per_s, ::tmc9660::units::VelocityUnit::RadPerSec, ctx);
    }

    /** @brief Set target velocity in raw chip internal units (escape hatch).
     *
     * Prefer the unit-aware overload above. Use this only when integrating
     * with code that already speaks internal units or when bypassing the
     * `MotorContext`-based scaling.
     */
    bool setTargetVelocityRaw(int32_t velocity_internal) noexcept;

    /** @brief Read actual velocity in raw chip internal units.
     * @param[out] velocity Measured velocity (signed internal units).
     * @return true if read.
     */
    bool getActualVelocity(int32_t& velocity) noexcept;

    /** @brief Read actual velocity in any engineering unit.
     * @param[out] value Measured velocity expressed in `unit`.
     * @param unit       Engineering unit to convert into.
     * @param ctx        Motor context (must be valid for unit ≠ `Internal`).
     * @return true if the underlying SAP succeeded.
     */
    bool getActualVelocity(double& value,
                           ::tmc9660::units::VelocityUnit unit,
                           ::tmc9660::units::MotorContext const& ctx) noexcept;

    /// Convenience: read actual velocity in mechanical RPM.
    bool getActualVelocityRpm(double& rpm,
                              ::tmc9660::units::MotorContext const& ctx) noexcept {
        return getActualVelocity(rpm, ::tmc9660::units::VelocityUnit::Rpm, ctx);
    }

    /** @brief Set velocity offset (feed-forward bias) in any engineering unit.
     * @note  The previous implementation's docstring claimed "RPM" but in
     *        fact wrote raw internal units to the chip. This canonical
     *        overload removes that ambiguity by requiring an explicit unit.
     */
    bool setVelocityOffset(double value,
                           ::tmc9660::units::VelocityUnit unit,
                           ::tmc9660::units::MotorContext const& ctx) noexcept;

    /// Set velocity offset in raw chip internal units (escape hatch).
    bool setVelocityOffsetRaw(int32_t offset_internal) noexcept;

    /// Read velocity offset in raw chip internal units.
    bool getVelocityOffset(int32_t& offset_internal) noexcept;

    /** @brief Configure velocity PI gains.
     * @param p P gain.
     * @param i I gain.
     * @return true if written.
     */
    bool setVelocityLoopGains(uint16_t p, uint16_t i) noexcept;
    /** @brief Set velocity PI normalization.
     * @param p_norm P-term norm.
     * @param i_norm I-term norm.
     * @return true if written.
     */
    bool setVelocityNormalization(tmc9660::tmcl::VelocityPiNorm p_norm,
                                  tmc9660::tmcl::VelocityPiNorm i_norm) noexcept;

    /** @brief Read velocity-PI integrator.
     * @param[out] integrator Integrator value.
     * @return true if read.
     */
    bool getVelocityPiIntegrator(int32_t& integrator) noexcept;
    /** @brief Read velocity-PI error.
     * @param[out] error PI error.
     * @return true if read.
     */
    bool getVelocityPiError(int32_t& error) noexcept;

    /** @brief Set velocity scaling factor.
     * @param factor Scale factor.
     * @return true if written.
     */
    bool setVelocityScalingFactor(uint16_t factor) noexcept;
    /** @brief Read velocity scaling factor.
     * @param[out] factor Scale factor.
     * @return true if read.
     */
    bool getVelocityScalingFactor(uint16_t& factor) noexcept;

    /** @brief Configure stop-on-velocity-deviation.
     * @param max_error Max allowed deviation.
     * @param softStop true for ramp down, false for hard stop.
     * @return true if written.
     */
    bool setStopOnVelocityDeviation(uint32_t max_error, bool softStop = true) noexcept;
    /** @brief Read stop-on-velocity-deviation settings.
     * @param[out] max_error Configured max deviation.
     * @param[out] soft_stop Soft/hard stop flag.
     * @return true if read.
     */
    bool getStopOnVelocityDeviation(uint32_t& max_error, bool& soft_stop) noexcept;

    /** @brief Set velocity loop downsampling.
     * @param divider Downsample factor.
     * @return true if written.
     */
    bool setVelocityLoopDownsampling(uint8_t divider) noexcept;
    /** @brief Read velocity loop downsampling.
     * @param[out] divider Factor.
     * @return true if read.
     */
    bool getVelocityLoopDownsampling(uint8_t& divider) noexcept;

    // Note: VELOCITY_REACHED_THRESHOLD is not a configurable parameter in TMC9660.
    // The VELOCITY_REACHED flag is set when both actual and target velocity are below
    // an internal threshold. Parameter 134 is STOP_ON_VELOCITY_DEVIATION, not
    // VELOCITY_REACHED_THRESHOLD.

    /** @brief Set velocity meter switch threshold.
     * @param threshold Threshold value.
     * @return true if written.
     */
    bool setVelocityMeterSwitchThreshold(uint32_t threshold) noexcept;
    /** @brief Read velocity meter switch threshold.
     * @param[out] threshold Threshold.
     * @return true if read.
     */
    bool getVelocityMeterSwitchThreshold(uint32_t& threshold) noexcept;

    /** @brief Set velocity meter hysteresis.
     * @param hysteresis Hysteresis value.
     * @return true if written.
     */
    bool setVelocityMeterSwitchHysteresis(uint16_t hysteresis) noexcept;
    /** @brief Read velocity meter hysteresis.
     * @param[out] hysteresis Hysteresis.
     * @return true if read.
     */
    bool getVelocityMeterSwitchHysteresis(uint16_t& hysteresis) noexcept;

    /** @brief Read current velocity meter mode.
     * @param[out] mode Current mode.
     * @return true if read.
     */
    bool getVelocityMeterMode(tmc9660::tmcl::VelocityMeterMode& mode) noexcept;

    /// @name Actual Velocity Biquad Filter (read/write)
    ///@{
    bool setActualVelocityBiquadFilterEnable(bool enable) noexcept;
    bool getActualVelocityBiquadFilterEnable(bool& enable) noexcept;
    bool setActualVelocityBiquadFilterACoeff1(int32_t coeff) noexcept;
    bool getActualVelocityBiquadFilterACoeff1(int32_t& coeff) noexcept;
    bool setActualVelocityBiquadFilterACoeff2(int32_t coeff) noexcept;
    bool getActualVelocityBiquadFilterACoeff2(int32_t& coeff) noexcept;
    bool setActualVelocityBiquadFilterBCoeff0(int32_t coeff) noexcept;
    bool getActualVelocityBiquadFilterBCoeff0(int32_t& coeff) noexcept;
    bool setActualVelocityBiquadFilterBCoeff1(int32_t coeff) noexcept;
    bool getActualVelocityBiquadFilterBCoeff1(int32_t& coeff) noexcept;
    bool setActualVelocityBiquadFilterBCoeff2(int32_t coeff) noexcept;
    bool getActualVelocityBiquadFilterBCoeff2(int32_t& coeff) noexcept;
    ///@}

    /// @name Combined & integrated raw measurements (read-only)
    ///@{
    bool getIntegratedActualVelocityValue(uint32_t& value) noexcept;
    ///@}

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------

    /** @brief Configuration structure for velocity control auto-configuration.
     */
    struct VelocityConfig {
      // Required parameters
      tmc9660::tmcl::VelocitySensorSelection
          sensorSelection; //!< Velocity feedback sensor selection (required)

      // Optional parameters with defaults
      /** @brief Velocity P gain [0-32767] (optional, default: 800).
       *
       * Direct P gain for the velocity PI controller. Higher values = faster response
       * but potentially less stable. Typical range: 400-1600 for most applications.
       */
      std::optional<uint16_t> velocityP; //!< Velocity P gain [0-32767] (optional, default: 800)

      /** @brief Velocity I gain [0-32767] (optional, default: 1).
       *
       * Direct I gain for the velocity PI controller. Higher values = better
       * steady-state accuracy but potentially more overshoot. Typical range: 1-10.
       */
      std::optional<uint16_t> velocityI; //!< Velocity I gain [0-32767] (optional, default: 1)

      tmc9660::tmcl::VelocityPiNorm pNormalization =
          tmc9660::tmcl::VelocityPiNorm::SHIFT_16_BIT; //!< P-term normalization (default:
                                                       //!< SHIFT_16_BIT)
      tmc9660::tmcl::VelocityPiNorm iNormalization =
          tmc9660::tmcl::VelocityPiNorm::SHIFT_16_BIT; //!< I-term normalization (default:
                                                       //!< SHIFT_16_BIT)

      // Velocity scaling (optional, can be auto-calculated from encoder CPR and motor parameters)
      /** @brief Velocity scaling factor [1-2047] (optional).
       *
       * This factor converts internal velocity units to real-world RPM.
       * Formula: v_RPM = v_internal / k_RPM, where k_RPM = VELOCITY_SCALING_FACTOR
       *
       * **Auto-calculation:**
       * If not provided, the scaling factor is automatically calculated from:
       * - `encoderCountsPerRev` (or derived from sensor type and `motor_polePairs`)
       * - `motor_polePairs` (for SAME_AS_COMMUTATION or DIGITAL_HALL sensors)
       * - Formula: k_RPM = (CPR × 2^24) / (40MHz × 60)
       *
       * **CPR calculation by sensor type:**
       * - SAME_AS_COMMUTATION: CPR = 2^16 × motor_polePairs
       * - DIGITAL_HALL: CPR = 6 × motor_polePairs
       * - ABN1_ENCODER, ABN2_ENCODER, SPI_ENCODER: CPR = encoderCountsPerRev (from datasheet)
       *
       * **Note:** It's recommended to leave this at default (1) and handle scaling
       * externally for better resolution. Internal scaling reduces resolution.
       */
      std::optional<uint16_t>
          velocityScalingFactor; //!< Velocity scaling factor [1-2047] (optional, auto-calculated if
                                 //!< not provided)

      /** @brief Encoder counts per mechanical revolution (CPR) for auto-calculating velocity
       * scaling.
       *
       * Required for ABN1_ENCODER, ABN2_ENCODER, or SPI_ENCODER sensor types.
       * For SAME_AS_COMMUTATION or DIGITAL_HALL, this is calculated from motor_polePairs.
       *
       * If not provided and sensor type requires it, scaling factor defaults to 1.
       */
      std::optional<uint32_t>
          encoderCountsPerRev; //!< Encoder CPR (optional, required for encoder-based sensors if
                               //!< velocityScalingFactor not provided)

      /** @brief Motor pole pairs (optional, used for auto-calculating velocity scaling).
       *
       * Required for SAME_AS_COMMUTATION or DIGITAL_HALL sensor types.
       * If not provided, the value is read from MotorConfig (MOTOR_POLE_PAIRS parameter).
       * For encoder-based sensors, this is not used.
       */
      std::optional<uint8_t>
          motor_polePairs; //!< Motor pole pairs (optional, read from MotorConfig if not provided)

      /** @brief PWM frequency in Hz (optional, used for velocity meter threshold calculation).
       *
       * If not provided, the value is read from MotorConfig (MOTOR_PWM_FREQUENCY parameter).
       * Used to calculate velocity loop frequency for meter switch threshold.
       */
      std::optional<uint32_t> pwmFrequency_Hz; //!< PWM frequency in Hz (optional, read from
                                               //!< MotorConfig if not provided)

      // Velocity loop downsampling (optional)
      /** @brief Velocity loop downsampling factor [0-127] (default: 5).
       *
       * **Clock Distribution:**
       * The velocity control loop frequency is derived from the PWM frequency:
       * - Velocity loop frequency = PWM frequency / (loop_downsampling + 1)
       * - Example: 25kHz PWM, downsampling=5 → velocity loop runs at 25kHz/6 = 4.17kHz
       *
       * **Effect on PI Gains:**
       * Lower loop frequencies (higher downsampling) require proportionally higher
       * PI gains to maintain the same response. The integrator speed depends on
       * both the PWM frequency and this downsampling factor.
       *
       * **Typical Values:**
       * - 0-2: Fast response (high-frequency velocity control)
       * - 3-5: Standard (good balance, default: 5)
       * - 6-10: Slower response (for heavy loads or stability)
       * - >10: Very slow (rarely needed)
       */
      uint8_t loopDownsampling = 5; //!< Velocity loop downsampling factor [0-127] (default: 5)

      // Velocity reached threshold (optional)
      uint32_t velocityReachedThreshold = 1000; //!< Velocity reached threshold (default: 1000)

      // Stop on deviation (optional)
      std::optional<uint32_t>
          stopOnDeviationMaxError; //!< Max allowed velocity deviation for stop condition (optional,
                                   //!< disabled if not provided)
      bool stopOnDeviationSoftStop =
          true; //!< Use soft stop (ramp down) for deviation stop (default: true)

      // Velocity meter (optional, can be auto-calculated for optimal performance)
      /** @brief Velocity meter switch threshold (optional, auto-calculated if not provided).
       *
       * Threshold for switching from period-based to frequency-based velocity measurement.
       * The optimal switchover point is calculated to minimize measurement noise.
       *
       * **Auto-calculation:**
       * If not provided, the threshold is calculated using:
       * - `encoderCountsPerRev` (or derived from sensor type and `motor_polePairs`)
       * - `pwmFrequency_Hz` and `loop_downsampling` (to calculate velocity loop frequency)
       * - Formulas:
       *   - v_PerLim_RPM = 0.9 × (40MHz) / (CPR × 60 × 53)
       *   - v_COP_RPM = 60 × (f_Velo + sqrt(f_Velo² + f_Velo × 40MHz × 8)) / (4 × CPR)
       *   - v_THR_RPM = min(v_COP_RPM, v_PerLim_RPM)
       *   - threshold = v_THR_RPM × k_RPM (where k_RPM is the velocity scaling factor)
       *
       * **Note:** If auto-calculation is not possible (missing parameters), default value (2000) is
       * used.
       */
      std::optional<uint32_t> meterSwitchThreshold; //!< Velocity meter switch threshold (optional,
                                                    //!< auto-calculated if not provided)
      uint16_t meterHysteresis = 500;               //!< Velocity meter hysteresis (default: 500)

      // Velocity offset (optional)
      int32_t velocityOffset = 0; //!< Velocity offset [-200000 to 200000] (default: 0)

      // Biquad filter (optional)
      /** @brief Enable/disable biquad filter on actual velocity feedback.
       *
       * The velocity biquad filter filters the measured velocity before it's used
       * as input to the velocity controller. This filter is **enabled by default**
       * in hardware because measured velocity is usually quite noisy.
       *
       * When enabled, you can optionally provide filter coefficients below.
       * If coefficients are not provided, the filter uses default coefficient
       * values optimized for typical velocity measurement noise reduction.
       *
       * Default: Enabled (filter is on by default for noise reduction).
       * Most users should leave this enabled unless they have specific requirements.
       */
      std::optional<bool>
          enableVelocityBiquadFilter; //!< Enable/disable biquad filter on actual velocity
                                      //!< (optional, default: enabled in hardware)

      /** @brief Velocity biquad filter coefficients (optional, only used if
       * enableVelocityBiquadFilter is explicitly set).
       *
       * These coefficients define the biquad filter difference equation:
       * Y(n) = X(n)*b0 + X(n-1)*b1 + X(n-2)*b2 - Y(n-1)*a1 - Y(n-2)*a2
       *
       * **Coefficient Format:**
       * All coefficients are 24-bit values in Q4.20 fixed-point format:
       * - 4 integer bits (range: -8 to +7)
       * - 20 fractional bits (resolution: 1/1048576)
       * - To convert a floating-point value to Q4.20: `coeff = (float_value * 1048576.0f)`
       * - Example: 1.0 → 1048576, 0.5 → 524288, -0.25 → -262144
       *
       * **Default Values (if not provided):**
       * The hardware uses optimized default coefficients for velocity noise reduction:
       * - ACOEFF_1 = 1849195
       * - ACOEFF_2 = 15961938
       * - BCOEFF_0 = 3665
       * - BCOEFF_1 = 7329
       * - BCOEFF_2 = 3665
       *
       * **Note:** For most applications, leave these unset to use the optimized
       * hardware defaults. Only override if you have specific filtering requirements.
       */
      std::optional<int32_t> velocityBiquadACoeff1; //!< Velocity biquad A coefficient 1 (a1) in
                                                    //!< Q4.20 format (optional, default: 1849195)
      std::optional<int32_t> velocityBiquadACoeff2; //!< Velocity biquad A coefficient 2 (a2) in
                                                    //!< Q4.20 format (optional, default: 15961938)
      std::optional<int32_t> velocityBiquadBCoeff0; //!< Velocity biquad B coefficient 0 (b0) in
                                                    //!< Q4.20 format (optional, default: 3665)
      std::optional<int32_t> velocityBiquadBCoeff1; //!< Velocity biquad B coefficient 1 (b1) in
                                                    //!< Q4.20 format (optional, default: 7329)
      std::optional<int32_t> velocityBiquadBCoeff2; //!< Velocity biquad B coefficient 2 (b2) in
                                                    //!< Q4.20 format (optional, default: 3665)
    };

    /** @brief Auto-configure velocity control parameters.
     *
     * Configures velocity sensor selection, PI gains, scaling, and other velocity
     * control parameters based on high-level characteristics.
     *
     * @param config Velocity control configuration (see VelocityConfig)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const VelocityConfig& config) noexcept;

  private:
    friend class TMC9660;
    explicit VelocityControl(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } velocityControl{*this};

  //***************************************************************************
  //**                  SUBSYSTEM: Position Control                          **//
  //***************************************************************************

  /** @brief Subsystem for position control (FOC outer loop).
   *
   * Controls motor position using a PI controller that outputs velocity commands.
   * Covers TMCL parameters 142–157, 161–170.
   *
   * This control loop operates at a downsampled frequency relative to velocity loop.
   * Position feedback can come from various sensors (ABN, SPI encoder).
   */
  struct PositionControl {
    //-------------------------------------------------------------------------
    // Position control (142–157)
    //-------------------------------------------------------------------------
    /** @brief Stop position control (SYSTEM_OFF).
     * @return true on success.
     */
    bool stop() noexcept;

    /** @brief Select position feedback sensor.
     * @param sel Sensor selection.
     * @return true if written.
     */
    bool setPositionSensor(tmc9660::tmcl::PositionSensorSelection sel) noexcept;
    /** @brief Read position feedback sensor.
     * @param[out] sel Sensor selection.
     * @return true if read.
     */
    bool getPositionSensor(tmc9660::tmcl::PositionSensorSelection& sel) noexcept;

    /** @brief Set target position in any engineering unit (canonical API).
     *
     * @param value Target position expressed in `unit`.
     * @param unit  Engineering unit of `value` (mech revs / deg-mech / rad-mech / Counts).
     * @param ctx   Motor context (provides CPR for Counts ↔ rev mapping).
     * @return true if the SAP succeeded.
     */
    bool setTargetPosition(double value,
                           ::tmc9660::units::PositionUnit unit,
                           ::tmc9660::units::MotorContext const& ctx) noexcept;

    /// Convenience: set target position in mechanical degrees.
    bool setTargetPositionDegMech(double deg,
                                  ::tmc9660::units::MotorContext const& ctx) noexcept {
        return setTargetPosition(deg, ::tmc9660::units::PositionUnit::DegMech, ctx);
    }
    /// Convenience: set target position in mechanical revolutions.
    bool setTargetPositionMechRevs(double revs,
                                   ::tmc9660::units::MotorContext const& ctx) noexcept {
        return setTargetPosition(revs, ::tmc9660::units::PositionUnit::MechRevs, ctx);
    }

    /** @brief Set target position in raw chip counts (escape hatch).
     * @param position_counts Desired position in raw counts.
     */
    bool setTargetPositionRaw(int32_t position_counts) noexcept;

    /** @brief Read actual position in raw chip counts.
     * @param[out] position Measured position in counts.
     * @return true if read.
     */
    bool getActualPosition(int32_t& position) noexcept;

    /** @brief Read actual position in any engineering unit.
     * @param[out] value Measured position expressed in `unit`.
     * @param unit       Engineering unit to convert into.
     * @param ctx        Motor context (provides CPR for Counts ↔ rev mapping).
     * @return true if the underlying SAP succeeded.
     */
    bool getActualPosition(double& value,
                           ::tmc9660::units::PositionUnit unit,
                           ::tmc9660::units::MotorContext const& ctx) noexcept;

    /// Convenience: read actual position in mechanical degrees.
    bool getActualPositionDegMech(double& deg,
                                  ::tmc9660::units::MotorContext const& ctx) noexcept {
        return getActualPosition(deg, ::tmc9660::units::PositionUnit::DegMech, ctx);
    }

    /** @brief Set position scaling factor.
     * @param factor Scale factor.
     * @return true if written.
     */
    bool setPositionScalingFactor(uint16_t factor) noexcept;
    /** @brief Read position scaling factor.
     * @param[out] factor Scale factor.
     * @return true if read.
     */
    bool getPositionScalingFactor(uint16_t& factor) noexcept;

    /** @brief Configure position PI gains.
     * @param p P gain.
     * @param i I gain.
     * @return true if written.
     */
    bool setPositionLoopGains(uint16_t p, uint16_t i) noexcept;
    /** @brief Set position PI normalization.
     * @param p_norm P-term norm.
     * @param i_norm I-term norm.
     * @return true if written.
     */
    bool setPositionNormalization(tmc9660::tmcl::VelocityPiNorm p_norm,
                                  tmc9660::tmcl::VelocityPiNorm i_norm) noexcept;

    /** @brief Read position-PI integrator.
     * @param[out] integrator Integrator value.
     * @return true if read.
     */
    bool getPositionPiIntegrator(int32_t& integrator) noexcept;
    /** @brief Read position-PI error.
     * @param[out] error PI error.
     * @return true if read.
     */
    bool getPositionPiError(int32_t& error) noexcept;

    /** @brief Configure stop-on-position-deviation.
     * @param max_error Max allowed deviation.
     * @param softStop true for ramp down, false for hard stop.
     * @return true if written.
     */
    bool setStopOnPositionDeviation(uint32_t max_error, bool softStop = true) noexcept;
    /** @brief Read stop-on-position-deviation settings.
     * @param[out] max_error Configured max deviation.
     * @param[out] soft_stop Soft/hard stop flag.
     * @return true if read.
     */
    bool getStopOnPositionDeviation(uint32_t& max_error, bool& soft_stop) noexcept;

    /** @brief Set position loop downsampling.
     * @param divider Downsample factor.
     * @return true if written.
     */
    bool setPositionLoopDownsampling(uint8_t divider) noexcept;
    /** @brief Read position loop downsampling.
     * @param[out] divider Factor.
     * @return true if read.
     */
    bool getPositionLoopDownsampling(uint8_t& divider) noexcept;

    /** @brief Set low position limit.
     * @param limit Minimum allowed position.
     * @return true if written.
     */
    bool setPositionLimitLow(int32_t limit) noexcept;
    /** @brief Read low position limit.
     * @param[out] limit Minimum allowed position.
     * @return true if read.
     */
    bool getPositionLimitLow(int32_t& limit) noexcept;

    /** @brief Set high position limit.
     * @param limit Maximum allowed position.
     * @return true if written.
     */
    bool setPositionLimitHigh(int32_t limit) noexcept;
    /** @brief Read high position limit.
     * @param[out] limit Maximum allowed position.
     * @return true if read.
     */
    bool getPositionLimitHigh(int32_t& limit) noexcept;

    /** @brief Set position reached threshold.
     * @param threshold Latch threshold.
     * @return true if written.
     */
    bool setPositionReachedThreshold(uint32_t threshold) noexcept;
    /** @brief Read position reached threshold.
     * @param[out] threshold Latch threshold.
     * @return true if read.
     */
    bool getPositionReachedThreshold(uint32_t& threshold) noexcept;

    //-------------------------------------------------------------------------
    // Ref switch & stop-event (161–170)
    //-------------------------------------------------------------------------
    /** @brief Enable/disable reference switch stops.
     * @param enable Bitmask of switch stops.
     * @return true if written.
     */
    bool setReferenceSwitchEnable(tmc9660::tmcl::ReferenceSwitchEnable enable) noexcept;
    /** @brief Read reference switch enable mask.
     * @param[out] enable Mask.
     * @return true if read.
     */
    bool getReferenceSwitchEnable(tmc9660::tmcl::ReferenceSwitchEnable& enable) noexcept;

    /** @brief Configure switch polarity and swap.
     * @param config Polarity/swap config.
     * @return true if written.
     */
    bool setReferenceSwitchPolaritySwap(tmc9660::tmcl::ReferenceSwitchPolaritySwap config) noexcept;
    /** @brief Read switch polarity/swap config.
     * @param[out] config Config.
     * @return true if read.
     */
    bool getReferenceSwitchPolaritySwap(
        tmc9660::tmcl::ReferenceSwitchPolaritySwap& config) noexcept;

    /** @brief Configure switch latch settings.
     * @param setting Latch behavior.
     * @return true if written.
     */
    bool setReferenceSwitchLatchSettings(
        tmc9660::tmcl::ReferenceSwitchLatchSettings setting) noexcept;
    /** @brief Read switch latch settings.
     * @param[out] setting Latch behavior.
     * @return true if read.
     */
    bool getReferenceSwitchLatchSettings(
        tmc9660::tmcl::ReferenceSwitchLatchSettings& setting) noexcept;

    /** @brief Configure event-stop settings.
     * @param settings Stop conditions.
     * @return true if written.
     */
    bool setEventStopSettings(tmc9660::tmcl::EventStopSettings settings) noexcept;
    /** @brief Read event-stop settings.
     * @param[out] settings Stop conditions.
     * @return true if read.
     */
    bool getEventStopSettings(tmc9660::tmcl::EventStopSettings& settings) noexcept;

    /** @brief Set reference search mode.
     * @param mode Search sequence.
     * @return true if written.
     */
    bool setReferenceSwitchSearchMode(tmc9660::tmcl::ReferenceSwitchSearchMode mode) noexcept;
    /** @brief Read reference search mode.
     * @param[out] mode Search sequence.
     * @return true if read.
     */
    bool getReferenceSwitchSearchMode(tmc9660::tmcl::ReferenceSwitchSearchMode& mode) noexcept;

    /** @brief Set reference search speed.
     * @param speed Search velocity.
     * @return true if written.
     */
    bool setReferenceSwitchSearchSpeed(int32_t speed) noexcept;
    /** @brief Read reference search speed.
     * @param[out] speed Search velocity.
     * @return true if read.
     */
    bool getReferenceSwitchSearchSpeed(int32_t& speed) noexcept;

    /** @brief Set reference positioning speed.
     * @param speed Approach speed.
     * @return true if written.
     */
    bool setReferenceSwitchSpeed(int32_t speed) noexcept;
    /** @brief Read reference positioning speed.
     * @param[out] speed Approach speed.
     * @return true if read.
     */
    bool getReferenceSwitchSpeed(int32_t& speed) noexcept;

    /** @brief Read right-limit-switch position.
     * @param[out] position Position value.
     * @return true if read.
     */
    bool getRightLimitSwitchPosition(int32_t& position) noexcept;

    /** @brief Read home-switch position.
     * @param[out] position Position value.
     * @return true if read.
     */
    bool getHomeSwitchPosition(int32_t& position) noexcept;

    /** @brief Read last reference position.
     * @param[out] position Position value.
     * @return true if read.
     */
    bool getLastReferencePosition(int32_t& position) noexcept;

    /// @name Raw Inputs (read-only)
    ///@{
    // Raw inputs for ABN, hall, reference switches, driver enabled,
    // hall filtered and ABN2 or Step/Dir (Parameter 304: MCC_INPUTS_RAW)
    bool getMccInputsRaw(uint16_t& inputs) noexcept;
    ///@}

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------

    /** @brief Configuration structure for position control auto-configuration.
     */
    struct PositionConfig {
      // Required parameters
      tmc9660::tmcl::PositionSensorSelection
          sensorSelection; //!< Position feedback sensor selection (required)

      // Optional parameters with defaults
      /** @brief Position P gain [0-32767] (optional, default: 2000).
       *
       * Direct P gain for the position PI controller. Higher values = faster response
       * but potentially less stable. Typical range: 1000-4000 for most applications.
       */
      std::optional<uint16_t> positionP; //!< Position P gain [0-32767] (optional, default: 2000)

      /** @brief Position I gain [0-32767] (optional, default: 100).
       *
       * Direct I gain for the position PI controller. Higher values = better
       * steady-state accuracy but potentially more overshoot. Typical range: 50-200.
       */
      std::optional<uint16_t> positionI; //!< Position I gain [0-32767] (optional, default: 100)

      tmc9660::tmcl::VelocityPiNorm pNormalization =
          tmc9660::tmcl::VelocityPiNorm::SHIFT_16_BIT; //!< P-term normalization (default:
                                                       //!< SHIFT_16_BIT)
      tmc9660::tmcl::VelocityPiNorm iNormalization =
          tmc9660::tmcl::VelocityPiNorm::SHIFT_16_BIT; //!< I-term normalization (default:
                                                       //!< SHIFT_16_BIT)

      // Position scaling (optional, can be auto-calculated from encoder CPR)
      std::optional<uint16_t>
          positionScalingFactor; //!< Position scaling factor [1-2047] (optional, auto-calculated
                                 //!< from encoder CPR if not provided)
      std::optional<uint32_t>
          encoderCountsPerRev; //!< Encoder CPR for auto-calculating position scaling (optional,
                               //!< required if positionScalingFactor not provided)

      // Position loop downsampling (optional)
      /** @brief Position loop downsampling factor [0-127] (default: 1).
       *
       * **Clock Distribution (Cascading):**
       * The position control loop frequency is derived from the **velocity loop frequency**:
       * - Velocity loop frequency = PWM frequency / (VELOCITY_LOOP_DOWNSAMPLING + 1)
       * - Position loop frequency = Velocity loop frequency / (loop_downsampling + 1)
       * - Example: 25kHz PWM, velocity downsampling=5, position downsampling=1
       *   → velocity loop = 25kHz/6 = 4.17kHz
       *   → position loop = 4.17kHz/2 = 2.08kHz
       *
       * **Effect on PI Gains:**
       * Lower loop frequencies (higher downsampling) require proportionally higher
       * PI gains to maintain the same response. The integrator speed depends on
       * the PWM frequency, velocity loop downsampling, and this position loop
       * downsampling factor.
       *
       * **Typical Values:**
       * - 0-1: Fast response (high-frequency position control, default: 1)
       * - 2-3: Standard (good balance for most applications)
       * - 4-6: Slower response (for heavy loads or stability)
       * - >6: Very slow (rarely needed)
       *
       * **Note:** Position loop must be slower than velocity loop (typically 5-10× slower).
       */
      uint8_t loopDownsampling = 1; //!< Position loop downsampling factor [0-127] (default: 1)

      // Position limits (optional)
      std::optional<int32_t>
          positionLimitLow; //!< Low position limit (optional, disabled if not provided)
      std::optional<int32_t>
          positionLimitHigh; //!< High position limit (optional, disabled if not provided)

      // Position reached threshold (optional)
      uint32_t positionReachedThreshold = 100; //!< Position reached threshold (default: 100)

      // Stop on deviation (optional)
      std::optional<uint32_t>
          stopOnDeviationMaxError; //!< Max allowed position deviation for stop condition (optional,
                                   //!< disabled if not provided)
      bool stopOnDeviationSoftStop =
          true; //!< Use soft stop (ramp down) for deviation stop (default: true)
    };

    /** @brief Auto-configure position control parameters.
     *
     * Configures position sensor selection, PI gains, scaling, limits, and other
     * position control parameters based on high-level characteristics.
     *
     * @param config Position control configuration (see PositionConfig)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const PositionConfig& config) noexcept;

  private:
    friend class TMC9660;
    explicit PositionControl(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } positionControl{*this};

  //***************************************************************************
  //**                  SUBSYSTEM: Motion Ramp                             **//
  //***************************************************************************

  /** @brief Hardware 8-segment acceleration/dec-acc profile controller.
   *
   * Covers all ramp generator and feedforward parameters:
   * - ACCELERATION_FF_GAIN (50)
   * - ACCELERATION_FF_SHIFT (51)
   * - RAMP_ENABLE (52)
   * - DIRECT_VELOCITY_MODE (53)
   * - RAMP_AMAX/A1/A2 (54/55/56)
   * - RAMP_DMAX/D1/D2 (57/58/59)
   * - RAMP_VMAX/V1/V2/VSTART/VSTOP (60/61/62/63/64)
   * - RAMP_TVMAX/TZEROWAIT (65/66)
   * - ACCELERATION_FEEDFORWARD_ENABLE (67)
   * - VELOCITY_FEEDFORWARD_ENABLE (68)
   * - RAMP_VELOCITY (69)
   * - RAMP_POSITION (70)
   */
  struct Ramp {
    /** @brief Enable or disable the ramp generator block.
     * @param on True to enable, false to disable (RAMP_ENABLE).
     */
    bool enable(bool on) noexcept;

    /** @brief Set acceleration segments A1, A2, Amax (µ units/s²).
     * @param a1 First acceleration (RAMP_A1)
     * @param a2 Second acceleration (RAMP_A2)
     * @param a_max Top acceleration (RAMP_AMAX)
     */
    bool setAcceleration(uint32_t a1, uint32_t a2, uint32_t a_max) noexcept;

    /** @brief Set deceleration segments D1, D2, Dmax (µ units/s²).
     * @param d1 Second deceleration (RAMP_D1)
     * @param d2 First deceleration (RAMP_D2)
     * @param d_max Top deceleration (RAMP_DMAX)
     */
    bool setDeceleration(uint32_t d1, uint32_t d2, uint32_t d_max) noexcept;

    /** @brief Configure velocity thresholds and limits.
     * @param v_start Start velocity (RAMP_VSTART)
     * @param v_stop Stop velocity (RAMP_VSTOP)
     * @param v1 Velocity threshold 1 (RAMP_V1)
     * @param v2 Velocity threshold 2 (RAMP_V2)
     * @param v_max Maximum velocity (RAMP_VMAX)
     */
    bool setVelocities(uint32_t v_start, uint32_t v_stop, uint32_t v1, uint32_t v2,
                       uint32_t v_max) noexcept;

    /** @brief Timing constraints at Vmax and between moves.
     * @param t_vmax_cycles Minimum time at VMAX (RAMP_TVMAX)
     * @param t_zero_wait_cycles Wait time at end of ramp (RAMP_TZEROWAIT)
     */
    bool setTiming(uint16_t t_vmax_cycles, uint16_t t_zero_wait_cycles) noexcept;

    /** @brief Enable hardware feed-forward terms and set gain/shift.
     *
     * This allows the ramp generator to use feed-forward terms for velocity and
     * acceleration.
     * @param enable_vel_ff Enable the VELOCITY_FEEDFORWARD feature
     * (VELOCITY_FEEDFORWARD_ENABLE)
     * @param enable_accel_ff Enable the ACCELERATION_FEEDFORWARD feature
     * (ACCELERATION_FEEDFORWARD_ENABLE)
     * @param accel_ff_gain  ACCELERATION_FF_GAIN (0…65535)
     * @param accel_ff_shift ACCELERATION_FF_SHIFT enum
     * (tmc9660::tmcl::AccelerationFFShift)
     */
    bool enableFeedForward(bool enable_vel_ff, bool enable_accel_ff, uint16_t accel_ff_gain,
                           tmc9660::tmcl::AccelerationFFShift accel_ff_shift) noexcept;

    /** @brief Select what the velocity PI tracks (param 53 DIRECT_VELOCITY_MODE).
     *
     * @details This does **not** disable the velocity PI. Measured polarity:
     *          - `true`  (1, chip default): PI tracks `TARGET_VELOCITY` (124).
     *            The chip ramp is bypassed; acceleration is bounded by
     *            `MAX_TORQUE`. Use this for closed-loop Hall / encoder FOC.
     *          - `false` (0): PI tracks `RAMP_VELOCITY` (69). In
     *            `FOC_OPENLOOP_*` the ramper also generates φe — required
     *            there so the field angle follows the ramp, not a sensor.
     *
     *          Writing 0 under `FOC_HALL_SENSOR` makes the PI close on ramp
     *          following error (near zero while the shaft is hundreds of RPM
     *          behind). Do not infer polarity from the parameter name.
     *
     * @param enable True → write 1 (direct / TARGET_VELOCITY).
     */
    bool setDirectVelocityMode(bool enable) noexcept;

    /** @brief Get the current target velocity calculated by the ramp controller.
     * @param[out] velocity The current ramp target velocity (RAMP_VELOCITY,
     * param 69)
     * @return true if the value was read successfully.
     */
    bool getRampVelocity(int32_t& velocity) noexcept;

    /** @brief Get the current target position calculated by the ramp controller.
     * @param[out] position The current ramp target position (RAMP_POSITION,
     * param 70)
     * @return true if the value was read successfully.
     */
    bool getRampPosition(int32_t& position) noexcept;

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------

    /** @brief Configuration structure for ramp auto-configuration.
     */
    struct RampConfig {
      // Required parameters
      /** @brief Maximum velocity in internal units (required).
       *
       * Maximum velocity that the ramp generator can command.
       * Range: 0-134217727 (default: 134217727 = unlimited)
       */
      uint32_t maxVelocity = 134217727; //!< Maximum velocity [0-134217727] (default: unlimited)

      // Optional parameters with defaults
      /** @brief Maximum acceleration in µ units/s² (optional, default: 1000).
       *
       * Top acceleration value (RAMP_AMAX). Higher values = faster acceleration.
       * Typical range: 500-5000 for most applications.
       */
      std::optional<uint32_t>
          maxAcceleration; //!< Maximum acceleration [µ units/s²] (optional, default: 1000)

      /** @brief First acceleration segment in µ units/s² (optional, default: 8000).
       *
       * Acceleration at low velocities (RAMP_A1). Typically higher than A2/AMAX
       * for smooth startup.
       */
      std::optional<uint32_t>
          acceleration1; //!< First acceleration segment [µ units/s²] (optional, default: 8000)

      /** @brief Second acceleration segment in µ units/s² (optional, default: 4000).
       *
       * Acceleration at medium velocities (RAMP_A2). Intermediate value between
       * A1 and AMAX.
       */
      std::optional<uint32_t>
          acceleration2; //!< Second acceleration segment [µ units/s²] (optional, default: 4000)

      /** @brief Maximum deceleration in µ units/s² (optional, default: 1000).
       *
       * Top deceleration value (RAMP_DMAX). Can be different from acceleration
       * for asymmetric profiles.
       */
      std::optional<uint32_t>
          maxDeceleration; //!< Maximum deceleration [µ units/s²] (optional, default: 1000)

      /** @brief First deceleration segment in µ units/s² (optional, default: 8000).
       *
       * Deceleration at medium velocities (RAMP_D1).
       */
      std::optional<uint32_t>
          deceleration1; //!< First deceleration segment [µ units/s²] (optional, default: 8000)

      /** @brief Second deceleration segment in µ units/s² (optional, default: 8000).
       *
       * Deceleration at low velocities (RAMP_D2). Typically higher for smooth stopping.
       */
      std::optional<uint32_t>
          deceleration2; //!< Second deceleration segment [µ units/s²] (optional, default: 8000)

      /** @brief Velocity threshold 1 in internal units (optional, default: 0).
       *
       * Velocity threshold for switching from A1/D1 to A2/D2 (RAMP_V1).
       */
      std::optional<uint32_t>
          velocityThreshold1; //!< Velocity threshold 1 [internal units] (optional, default: 0)

      /** @brief Velocity threshold 2 in internal units (optional, default: 0).
       *
       * Velocity threshold for switching from A2/D2 to AMAX/DMAX (RAMP_V2).
       */
      std::optional<uint32_t>
          velocityThreshold2; //!< Velocity threshold 2 [internal units] (optional, default: 0)

      /** @brief Start velocity in internal units (optional, default: 0).
       *
       * Initial velocity when ramp starts (RAMP_VSTART).
       */
      std::optional<uint32_t>
          startVelocity; //!< Start velocity [internal units] (optional, default: 0)

      /** @brief Stop velocity in internal units (optional, default: 1).
       *
       * Velocity at which ramp considers motion stopped (RAMP_VSTOP).
       */
      std::optional<uint32_t>
          stopVelocity; //!< Stop velocity [internal units] (optional, default: 1)

      /** @brief Minimum time at VMAX before deceleration (optional, default: 0).
       *
       * Minimum time to maintain maximum velocity before starting deceleration (RAMP_TVMAX).
       * Units: velocity loop cycles.
       */
      std::optional<uint16_t> timeAtVmax; //!< Minimum time at VMAX [cycles] (optional, default: 0)

      /** @brief Wait time at end of ramp (optional, default: 0).
       *
       * Time to wait after ramp completes before next move (RAMP_TZEROWAIT).
       * Units: velocity loop cycles.
       */
      std::optional<uint16_t> timeZeroWait; //!< Wait time at end [cycles] (optional, default: 0)

      /** @brief Enable ramp generator (optional, default: false).
       *
       * When enabled, the ramp generator controls acceleration/deceleration.
       * When disabled, direct velocity commands are used.
       */
      std::optional<bool> enableRamp; //!< Enable ramp generator (optional, default: false)

      /** @brief DIRECT_VELOCITY_MODE (optional, default: true = 1).
       *
       * `true`  (1): velocity PI tracks TARGET_VELOCITY — closed-loop FOC.
       * `false` (0): velocity PI tracks RAMP_VELOCITY; open-loop φe comes
       * from the ramper. Required `false` for FOC_OPENLOOP_*_MODE.
       */
      std::optional<bool>
          enableDirectVelocityMode; //!< Enable direct velocity mode (optional, default: true)

      /** @brief Enable velocity feedforward (optional, default: false).
       *
       * When enabled, velocity feedforward term is added to improve tracking.
       */
      std::optional<bool>
          enableVelocityFeedForward; //!< Enable velocity feedforward (optional, default: false)

      /** @brief Enable acceleration feedforward (optional, default: false).
       *
       * When enabled, acceleration feedforward term is added to improve tracking.
       */
      std::optional<bool> enableAccelerationFeedForward; //!< Enable acceleration feedforward
                                                         //!< (optional, default: false)

      /** @brief Acceleration feedforward gain (optional, default: 8).
       *
       * Gain for acceleration feedforward term (ACCELERATION_FF_GAIN).
       * Range: 0-65535.
       */
      std::optional<uint16_t>
          accelerationFeedForwardGain; //!< Acceleration FF gain [0-65535] (optional, default: 8)

      /** @brief Acceleration feedforward shift (optional, default: SHIFT_4).
       *
       * Shift for acceleration feedforward term (ACCELERATION_FF_SHIFT).
       */
      std::optional<tmc9660::tmcl::AccelerationFFShift>
          accelerationFeedForwardShift; //!< Acceleration FF shift (optional, default: SHIFT_4_BIT)
    };

    /** @brief Auto-configure ramp parameters.
     *
     * Configures acceleration, deceleration, velocity thresholds, and feedforward
     * parameters based on high-level motion profile requirements.
     *
     * @param config Ramp configuration (see RampConfig)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const RampConfig& config) noexcept;

    /** @brief Build a `RampConfig` from engineering units.
     *
     * Convenience builder that converts a max velocity (any `VelocityUnit`)
     * and a max acceleration (any `AccelerationUnit`) into the chip's raw
     * internal units, populating the principal `RampConfig` fields. The
     * `enableRamp` / `enableDirectVelocityMode` / advanced segment fields
     * are left at default; tweak them on the returned struct before
     * passing to `configureAuto()`.
     *
     * The result is a fully-populated single-segment trapezoidal profile:
     *  - `maxAcceleration` (AMAX), `acceleration1` (A1), `acceleration2` (A2)
     *    are all set to the same value derived from `max_acceleration`.
     *  - `maxDeceleration` (DMAX), `deceleration1` (D1), `deceleration2` (D2)
     *    mirror the acceleration value (symmetric ramp).
     *  - `velocityThreshold1` / `velocityThreshold2` are pinned to 0, which
     *    selects single-segment AMAX/DMAX commutation (A1/A2/D1/D2 unused).
     *  - `startVelocity` = 0, `stopVelocity` = 1 (lowest non-zero raw value;
     *    motion is considered stopped at <= 1 internal unit ≈ 0.0003 RPM).
     *  - `timeAtVmax` = 0, `timeZeroWait` = 0.
     *
     * Callers wanting an asymmetric, segmented, or jerk-shaped profile should
     * override the relevant fields on the returned struct before calling
     * `configureAuto()`. Callers wanting feedforward should set
     * `enableVelocityFeedForward` / `enableAccelerationFeedForward` separately.
     */
    [[nodiscard]] static RampConfig buildRampConfig(
        double max_velocity, ::tmc9660::units::VelocityUnit vel_unit,
        double max_acceleration, ::tmc9660::units::AccelerationUnit accel_unit,
        ::tmc9660::units::MotorContext const& ctx) noexcept {
        RampConfig rc{};
        const int32_t v = ::tmc9660::units::velocityToInternal(max_velocity, vel_unit, ctx);
        rc.maxVelocity = static_cast<uint32_t>(v < 0 ? -v : v);
        const uint32_t a = ::tmc9660::units::accelerationToInternal(max_acceleration, accel_unit, ctx);
        rc.maxAcceleration = a;
        rc.acceleration1   = a;
        rc.acceleration2   = a;
        rc.maxDeceleration = a;
        rc.deceleration1   = a;
        rc.deceleration2   = a;
        rc.velocityThreshold1 = 0u;  // single-segment: AMAX/DMAX active across full velocity range
        rc.velocityThreshold2 = 0u;
        rc.startVelocity      = 0u;
        rc.stopVelocity       = 1u;  // chip default; lowest non-zero "stopped" threshold
        rc.timeAtVmax         = uint16_t{0};
        rc.timeZeroWait       = uint16_t{0};
        return rc;
    }

  private:
    friend class TMC9660;
    explicit Ramp(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } ramp{*this};

  //***************************************************************************
  //**              SUBSYSTEM: Step/Dir Input Extrapolation                **//
  //***************************************************************************

  /** @brief Subsystem for controlling the STEP/DIR pulse input interface.
   *
   * Enables stepper-style control using external STEP and DIR pulses, with
   * support for:
   * - Signal extrapolation to interpolate between pulses
   * - Velocity feed-forward
   * - Microstep resolution configuration
   *
   * Refer to Parameter IDs #205–#209 (Table 48). See datasheet page 95 for
   * details on the STEP/DIR interface.
   */
  struct StepDir {

    /** @brief Configure microstep resolution for each STEP pulse.
     *
     * @param µSteps Microsteps per full step (e.g. 256 = 1/256 resolution)
     * @return true on success
     *
     * - Parameter: `STEPDIR_STEP_DIVIDER_SHIFT`
     *   (shift of incoming step pulse count)
     */
    bool setMicrostepResolution(tmc9660::tmcl::StepDirStepDividerShift µSteps) noexcept;

    /** @brief Enable or disable the STEP/DIR interface.
     *
     * @param on true = enable, false = disable
     * @return true on success
     *
     * - Parameter: `STEPDIR_ENABLE`
     * - Boot option: see datasheet Table 11
     */
    bool enableInterface(bool on) noexcept;

    /** @brief Enable signal extrapolation between STEP pulses.
     *
     * @param enable true = enable extrapolation
     * @return true on success
     *
     * - Parameter: `STEPDIR_EXTRAPOLATE`
     * - Behavior described on p. 96, Fig. 27
     */
    bool enableExtrapolation(bool enable) noexcept;

    /** @brief Timeout before extrapolated motion stops after last pulse.
     *
     * @param timeout_ms Timeout in milliseconds
     * @return true on success
     *
     * - Parameter: `STEPDIR_STEP_SIGNAL_TIMEOUT_LIMIT`
     */
    bool setSignalTimeout(uint16_t timeout_ms) noexcept;

    /** @brief Set maximum allowed extrapolation velocity.
     *
     * @param e_rpm Max electrical RPM before extrapolation is disabled
     * @return true on success
     *
     * - Parameter: `STEPDIR_MAXIMUM_EXTRAPOLATION_VELOCITY`
     */
    bool setMaxExtrapolationVelocity(uint32_t e_rpm) noexcept;

    /** @brief Enable hardware feed-forward terms and set gain/shift.
     *
     * This allows the ramp generator to use feed-forward terms for velocity and
     * acceleration.
     * @param enable_vel_ff Enable the VELOCITY_FEEDFORWARD feature
     * (VELOCITY_FEEDFORWARD_ENABLE)
     * @return true on success
     */
    bool enableVelocityFeedForward(bool enable_vel_ff) noexcept;

  private:
    friend class TMC9660;
    explicit StepDir(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } stepDir{*this};

  //***********************************************************************
  //**                    SUBSYSTEM: Reference Search                   **//
  //***********************************************************************

  /** @brief Subsystem for executing a reference search (homing) routine. */
  struct ReferenceSearch {
    /// Start the reference search procedure.
    bool start() noexcept;
    /// Abort an ongoing reference search.
    bool stop() noexcept;
    /// Query the current reference search status code.
    bool getStatus(tmc9660::tmcl::ReferenceSearchStatus& status) noexcept;

  private:
    friend class TMC9660;
    explicit ReferenceSearch(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } referenceSearch{*this};

  //***************************************************************************
  //**                      SUBSYSTEM: Brake Chopper                       **//
  //***************************************************************************

  /** @brief Subsystem controlling the brake chopper and mechanical brake
   * features.
   */
  struct Brake {
    /** @brief Enable or disable the brake chopper functionality.
     * @param enable True to enable the brake chopper, false to disable it.
     * @return True if the command was sent and acknowledged.
     */
    bool enableChopper(bool enable) noexcept;

    /** @brief Set the overvoltage threshold for the brake chopper.
     * @param voltage Threshold voltage in volts (5.0 to 100.0 V).
     * @return True if the parameter was written successfully.
     */
    bool setVoltageLimit(float voltage) noexcept;

    /** @brief Set the hysteresis for the brake chopper threshold.
     * @param voltage Hysteresis in volts (0.0 to 5.0 V).
     * @return True if the parameter was written successfully.
     */
    bool setHysteresis(float voltage) noexcept;

    /** @brief Trigger a release of the mechanical brake.
     * @return True if the command was sent successfully.
     */
    bool release() noexcept;

    /** @brief Engage (lock) the mechanical brake.
     * @return True if the command was sent successfully.
     */
    bool engage() noexcept;

    /** @brief Set the PWM duty cycle for releasing the brake.
     * @param percent Duty cycle (0 to 99%).
     * @return True if the parameter was written successfully.
     */
    bool setReleasingDutyCycle(uint8_t percent) noexcept;

    /** @brief Set the PWM duty cycle for holding the brake released.
     * @param percent Duty cycle (0 to 99%).
     * @return True if the parameter was written successfully.
     */
    bool setHoldingDutyCycle(uint8_t percent) noexcept;

    /** @brief Set the duration of the brake release initial phase.
     * @param milliseconds Duration in ms (0 to 65535).
     * @return True if the parameter was written successfully.
     */
    bool setReleasingDuration(uint16_t milliseconds) noexcept;

    /** @brief Invert or normalize the brake output signal polarity.
     * @param invert True to invert the brake output, false for normal.
     * @return True if the parameter was written successfully.
     */
    bool invertOutput(bool invert) noexcept;

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------

    /** @brief Configuration structure for brake auto-configuration.
     */
    struct BrakeConfig {
      // Brake chopper configuration (optional)
      /** @brief Enable brake chopper (optional, default: false).
       *
       * When enabled, the brake chopper activates when bus voltage exceeds
       * the threshold, dumping excess energy into a resistor.
       */
      std::optional<bool> enableChopper; //!< Enable brake chopper (optional, default: false)

      /** @brief Brake chopper voltage threshold in volts (optional, default: 30.0V for 24V
       * systems).
       *
       * Voltage at which brake chopper activates to dump excess energy.
       * Typical values:
       * - 24V systems: 28-32V (default: 30.0V)
       * - 48V systems: 56-60V
       * - 12V systems: 14-16V
       */
      std::optional<float> chopperVoltageThreshold_V; //!< Brake chopper voltage threshold [V]
                                                      //!< (optional, default: 30.0V)

      /** @brief Brake chopper hysteresis in volts (optional, default: 2.0V).
       *
       * Hysteresis for brake chopper threshold to prevent oscillation.
       * Typical values: 1.0-5.0V (default: 2.0V)
       */
      std::optional<float>
          chopperHysteresis_V; //!< Brake chopper hysteresis [V] (optional, default: 2.0V)

      // Mechanical brake configuration (optional)
      /** @brief PWM duty cycle for brake release in percent (optional, default: 50%).
       *
       * Duty cycle used during initial brake release phase.
       * Range: 0-99% (default: 50%)
       */
      std::optional<uint8_t>
          releasingDutyCycle; //!< Brake release duty cycle [0-99%] (optional, default: 50%)

      /** @brief PWM duty cycle for brake holding in percent (optional, default: 30%).
       *
       * Duty cycle used to hold brake in released state.
       * Range: 0-99% (default: 30%)
       */
      std::optional<uint8_t>
          holdingDutyCycle; //!< Brake holding duty cycle [0-99%] (optional, default: 30%)

      /** @brief Brake release duration in milliseconds (optional, default: 100ms).
       *
       * Duration of initial high-duty-cycle phase for brake release.
       * Range: 0-65535ms (default: 100ms)
       */
      std::optional<uint16_t>
          releasingDuration_ms; //!< Brake release duration [ms] (optional, default: 100ms)

      /** @brief Invert brake output polarity (optional, default: false).
       *
       * When true, inverts the brake output signal polarity.
       */
      std::optional<bool> invertOutput; //!< Invert brake output (optional, default: false)
    };

    /** @brief Auto-configure brake parameters.
     *
     * Configures brake chopper and mechanical brake parameters based on
     * high-level requirements.
     *
     * @param config Brake configuration (see BrakeConfig)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const BrakeConfig& config) noexcept;

  private:
    friend class TMC9660;
    explicit Brake(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } brake{*this};

  //***************************************************************************
  //**                   SUBSYSTEM: I²t Overload Protection                **//
  //***************************************************************************

  /** @brief Subsystem for motor thermal overload protection via I²t integration.
   *
   * Configures two independent I²t windows that monitor integrated current over
   * time (in A²·ms) to detect thermal overloads. If either limit is exceeded, a
   * fault is triggered.
   *
   * - Refer to: Parameters #224–#228 (Table 41)
   * - Manual: “IIT” section, p. 86
   * - Related fault flags: `IIT_1_EXCEEDED`, `IIT_2_EXCEEDED`
   */
  struct IIT {
    /** @brief Configure the two I²t monitoring windows.
     *
     * @param timeConstant1_ms Time constant for window 1 [ms]
     * @param continuousCurrent1_A Current limit for window 1 [A]
     * @param timeConstant2_ms Time constant for window 2 [ms]
     * @param continuousCurrent2_A Current limit for window 2 [A]
     * @return true if parameters written successfully
     *
     * - THERMAL_WINDING_TIME_CONSTANT_1/2
     * - IIT_LIMIT_1/2
     */
    bool configure(uint16_t timeConstant1_ms, float continuousCurrent1_A, uint16_t timeConstant2_ms,
                   float continuousCurrent2_A) noexcept;

    /** @brief Reset both I²t accumulators to zero.
     * - RESET_IIT_SUMS
     */
    bool resetIntegralState() noexcept;

    /** @brief Set the winding time constant for window 1.
     * - THERMAL_WINDING_TIME_CONSTANT_1
     */
    bool setThermalWindingTimeConstant1(uint16_t ms) noexcept;
    /** @brief Get the winding time constant for window 1.
     */
    bool getThermalWindingTimeConstant1(uint16_t& ms) noexcept;

    /** @brief Set the I²t limit for window 1.
     * - IIT_LIMIT_1
     */
    bool setLimit1(uint32_t limit) noexcept;
    /** @brief Get the I²t limit for window 1.
     */
    bool getLimit1(uint32_t& limit) noexcept;

    /** @brief Set the winding time constant for window 2.
     * - THERMAL_WINDING_TIME_CONSTANT_2
     */
    bool setThermalWindingTimeConstant2(uint16_t ms) noexcept;
    /** @brief Get the winding time constant for window 2.
     */
    bool getThermalWindingTimeConstant2(uint16_t& ms) noexcept;

    /** @brief Set the I²t limit for window 2.
     * - IIT_LIMIT_2
     */
    bool setLimit2(uint32_t limit) noexcept;
    /** @brief Get the I²t limit for window 2.
     */
    bool getLimit2(uint32_t& limit) noexcept;

    /** @brief Read the total motor current (torque+flux).
     * - ACTUAL_TOTAL_MOTOR_CURRENT
     */
    bool getActualTotalMotorCurrent(uint32_t& current, uint8_t motorIndex = 0) noexcept;

    /** @brief Read the current integrated sum of window 1.
     * - IIT_SUM_1
     */
    bool getSum1(uint32_t& sum) noexcept;

    /** @brief Read the current integrated sum of window 2.
     * - IIT_SUM_2
     */
    bool getSum2(uint32_t& sum) noexcept;

  private:
    friend class TMC9660;
    explicit IIT(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } iit{*this};

  //===========================================================================
  //==                  SUBSYSTEM: Telemetry & Status                       ==//
  //===========================================================================

  /** @brief Subsystem for reading various telemetry and status information from
   * the driver.
   */
  struct Telemetry {
    /** @brief Read the GENERAL_STATUS_FLAGS register.
     * @param[out] flags Bit mask of current status flags.
     * @return true if the flags were read successfully.
     */
    bool getGeneralStatusFlags(uint32_t& flags) noexcept;

    /** @brief Read the current supply (bus) voltage.
     * @return Supply voltage in volts. Returns a negative value if unable to
     * read.
     */
    float getSupplyVoltage() noexcept;

    /** @brief Read the internal chip temperature.
     * @return Chip temperature in degrees Celsius.
     *         Returns a negative value (e.g. -273) on read error.
     *         Formula: T(°C) = raw * 0.01615 - 268.15
     */
    float getChipTemperature() noexcept;

    /** @brief Read the current motor current (torque current).
     * @return Motor current in milliamps (m_a). Returns 0 if unable to read.
     */
    int16_t getMotorCurrent() noexcept;

    /** @brief Read the measured actual velocity of the motor.
     * @return The actual velocity in internal units. Returns 0 if not
     * available.
     */
    int32_t getActualVelocity() noexcept;

    /** @brief Read the measured actual position of the motor.
     * @return The actual position in internal units. Returns 0 if not
     * available.
     */
    int32_t getActualPosition() noexcept;

    /** @brief Read the GENERAL_ERROR_FLAGS register.
     * @param[out] flags Bit mask of current error flags.
     * @return true if the flags were read successfully.
     */
    bool getGeneralErrorFlags(uint32_t& flags) noexcept;

    /** @brief Read the GDRV_ERROR_FLAGS register.
     * @param[out] flags Bit mask of current gate driver error flags.
     * @return true if the flags were read successfully.
     */
    bool getGateDriverErrorFlags(uint32_t& flags) noexcept;

    /** @brief Clear bits in the GENERAL_ERROR_FLAGS register.
     * @param mask Bit mask of flags to clear (write-1-to-clear).
     * @return true if the mask was written successfully.
     */
    bool clearGeneralErrorFlags(uint32_t mask) noexcept;

    /** @brief Clear bits in the GDRV_ERROR_FLAGS register.
     * @param mask Bit mask of flags to clear (write-1-to-clear).
     * @return true if the mask was written successfully.
     */
    bool clearGateDriverErrorFlags(uint32_t mask) noexcept;

    /** @brief Read the ADC_STATUS_FLAGS register (clipped ADC channels).
     * @param[out] flags Bit mask of clipping status.
     * @return true if read successfully.
     */
    bool getADCStatusFlags(uint32_t& flags) noexcept;

    /** @brief Clear bits in the ADC_STATUS_FLAGS register (write-1-to-clear).
     * @param mask Bit mask of clipping flags to clear.
     * @return true if the mask was written successfully.
     */
    bool clearADCStatusFlags(uint32_t mask) noexcept;

    /** @brief Read the external temperature sensor raw value.
     * @return Raw external temperature. Returns 0 if unable to read.
     */
    uint16_t getExternalTemperature() noexcept;

  private:
    friend class TMC9660;
    explicit Telemetry(TMC9660& parent) noexcept;
    TMC9660& driver;
  } telemetry{*this};

  //***************************************************************************
  //**                  SUBSYSTEM: Diagnostics                             **//
  //***************************************************************************

  /** @brief One-shot diagnostic snapshots for upstream health reporting.
   *
   * Two flavours:
   *   - `summary(MotorSummary&)` — small, single-screen health (vbus, temp,
   *     mode, target/actual RPM, regulating flag, fault summary). Cheap.
   *   - `snapshot(MotorSnapshot&, MotorContext const&)` — full chip state
   *     in both raw and engineering-unit form. Use for logging,
   *     bench expectation-checking, or upstream telemetry packets.
   *
   * Both methods are best-effort: a failed individual SAP leaves the
   * corresponding field at its struct default. The methods themselves
   * always succeed (return value is reserved for future use).
   */
  struct Diagnostics {
    /** @brief Populate a compact motor health summary.
     *
     * @param[out] out Summary struct to fill. All fields are zero-initialised
     *                 first; per-field `valid_*` bits flip true on success.
     * @param ctx      Motor context used to report velocity in RPM.
     * @return true (always — included for API symmetry / future error paths).
     */
    bool summary(::tmc9660::diagnostics::MotorSummary& out,
                 ::tmc9660::units::MotorContext const& ctx) noexcept;

    /** @brief Populate a full motor state snapshot.
     *
     * @param[out] out Snapshot struct to fill. All fields are zero-initialised
     *                 first.
     * @param ctx      Motor context used to convert raw fields to engineering
     *                 units. Stored on `out.context` so the consumer can
     *                 reproduce conversions later.
     * @return true (always — included for API symmetry / future error paths).
     */
    bool snapshot(::tmc9660::diagnostics::MotorSnapshot& out,
                  ::tmc9660::units::MotorContext const& ctx) noexcept;

  private:
    friend class TMC9660;
    explicit Diagnostics(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } diagnostics{*this};

  //***************************************************************************
  //**                  Motor context (chip-side facts)                    **//
  //***************************************************************************

  /** @brief Read a `MotorContext` (pole pairs, sensor selection, encoder CPR)
   *         live from the chip.
   *
   * Reads the bootloader-set/parameter-mode authoritative values:
   *   - NR  8 MOTOR_TYPE
   *   - NR  9 MOTOR_POLE_PAIRS  (for steppers, this is `full_steps_per_rev/4`)
   *   - NR 132 VELOCITY_SENSOR_SELECTION
   *   - For ABN encoder feedback: ABN_1_STEPS / ABN_2_STEPS
   *
   * @param[out] ctx The populated context. Fields default-init to safe values
   *                 if the corresponding read fails (caller should check
   *                 `ctx.valid()` for unit-conversion correctness).
   * @return true if every required parameter read succeeded.
   *
   * @note  After changing the motor type, pole pairs, sensor selection or
   *        encoder CPR via the bootloader or parameter-mode SAPs, call this
   *        again to refresh any cached `MotorContext` you keep host-side.
   */
  bool getMotorContext(::tmc9660::units::MotorContext& ctx) noexcept;

  /** @brief Convenience overload returning the context by value. */
  [[nodiscard]] ::tmc9660::units::MotorContext getMotorContext() noexcept {
    ::tmc9660::units::MotorContext c{};
    (void)getMotorContext(c);
    return c;
  }

  //***************************************************************************
  //**                  SUBSYSTEM: Stop / Event                            **//
  //***************************************************************************

  /** @brief Configure automatic stop/latch behaviour for deviation, switches.
   */
  struct StopEvents {
    /** @brief Stop when ramp target deviates from actual > thresholds.
     */
    /** @brief Stop when ramp target deviates from actual values beyond the
     *        allowed thresholds.
     *
     * @param max_vel_error Maximum allowed velocity error.
     * @param max_pos_error Maximum allowed position error.
     * @param softStop Use soft stop instead of immediate stop when true.
     * @return true on success
     *
     * @see EVENT_STOP_SETTINGS and STOP_ON_*_DEVIATION in the datasheet.
     */
    bool enableDeviationStop(
        uint32_t max_vel_error, uint32_t max_pos_error,
        bool softStop = true) noexcept; ///< STOP_ON_*_DEVIATION + EVENT_STOP_SETTINGS
                                        ///< @see Datasheet EVENT_STOP_SETTINGS

    /** @brief Configure reference and limit switch inputs.
     *
     * @param mask    Bit mask of switches to enable (see REFERENCE_SWITCH_ENABLE).
     * @param invert_l Invert left switch polarity.
     * @param invert_r Invert right switch polarity.
     * @param invert_h Invert home switch polarity.
     * @param swap_lr  Swap left and right wiring.
     * @return true on success
     *
     * @see REFERENCE_SWITCH_* parameters in the datasheet.
     */
    bool configureReferenceSwitches(
        uint8_t mask, bool invert_l, bool invert_r, bool invert_h,
        bool swap_lr) noexcept; ///< REFERENCE_SWITCH_*
                               ///< @see Datasheet REFERENCE_SWITCH_ENABLE

    /** @brief Read and clear the latched position from a switch event.
     *
     * @param[out] pos Latched position value.
     * @return true on success
     *
     * @see LATCH_POSITION / RAMPER_LATCHED in the datasheet.
     */
    bool getAndClearLatchedPosition(int32_t& pos) noexcept; ///< LATCH_POSITION / RAMPER_LATCHED
                                                            ///< @see Datasheet LATCH_POSITION

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------

    /** @brief Configuration structure for stop events auto-configuration.
     */
    struct StopEventsConfig {
      // Deviation stop (optional)
      /** @brief Maximum allowed velocity deviation for stop condition (optional, disabled if not
       * provided).
       *
       * When actual velocity deviates from target velocity by more than this value,
       * a stop event is triggered.
       * Range: 0-200000 (default: disabled if not provided)
       */
      std::optional<uint32_t> maxVelocityDeviation; //!< Max velocity deviation [0-200000]
                                                    //!< (optional, disabled if not provided)

      /** @brief Maximum allowed position deviation for stop condition (optional, disabled if not
       * provided).
       *
       * When actual position deviates from target position by more than this value,
       * a stop event is triggered.
       * Range: 0-2147483647 (default: disabled if not provided)
       */
      std::optional<uint32_t> maxPositionDeviation; //!< Max position deviation [0-2147483647]
                                                    //!< (optional, disabled if not provided)

      /** @brief Use soft stop (ramp down) for deviation stop (optional, default: true).
       *
       * When true, uses soft stop (ramp down) instead of immediate hard stop.
       */
      std::optional<bool>
          deviationSoftStop; //!< Use soft stop for deviation (optional, default: true)

      // Reference switches (optional)
      /** @brief Reference switch enable mask (optional, default: 0 = all disabled).
       *
       * Bit mask of switches to enable:
       * - Bit 0: Left switch
       * - Bit 1: Right switch
       * - Bit 2: Home switch
       * Range: 0-7 (default: 0 = all disabled)
       */
      std::optional<uint8_t>
          referenceSwitchMask; //!< Reference switch enable mask [0-7] (optional, default: 0)

      /** @brief Invert left switch polarity (optional, default: false).
       */
      std::optional<bool> invertLeftSwitch; //!< Invert left switch (optional, default: false)

      /** @brief Invert right switch polarity (optional, default: false).
       */
      std::optional<bool> invertRightSwitch; //!< Invert right switch (optional, default: false)

      /** @brief Invert home switch polarity (optional, default: false).
       */
      std::optional<bool> invertHomeSwitch; //!< Invert home switch (optional, default: false)

      /** @brief Swap left and right switch wiring (optional, default: false).
       */
      std::optional<bool> swapLeftRight; //!< Swap left/right wiring (optional, default: false)
    };

    /** @brief Auto-configure stop events parameters.
     *
     * Configures deviation stop thresholds and reference switch settings based on
     * high-level requirements.
     *
     * @param config Stop events configuration (see StopEventsConfig)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const StopEventsConfig& config) noexcept;

  private:
    friend class TMC9660;
    explicit StopEvents(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } stopEvents{*this};

  //===========================================================================
  //==                  SUBSYSTEM: Protection                               ==//
  //===========================================================================

  /** @brief Subsystem for motor protection features
   */
  struct Protection {
    /** @brief Configure overvoltage and undervoltage protection thresholds.
     * @param over_volt_threshold Over-voltage warning threshold in units of 0.1V.
     * @param under_volt_threshold Under-voltage warning threshold in units of
     * 0.1V.
     * @return true if thresholds were set successfully.
     */
    bool configureVoltage(uint16_t over_volt_threshold, uint16_t under_volt_threshold) noexcept;

    /** @brief Configure over-temperature protection thresholds.
     *
     * The TMC9660 has an internal temperature sensor and supports an external
     * analog temperature sensor. This function sets warning and shutdown
     * thresholds for the internal sensor.
     * @param warning_deg_c Warning threshold in °C for chip temperature.
     * @param shutdown_deg_c Shutdown (fault) threshold in °C for chip
     * temperature.
     * @return true if set successfully.
     */
    bool configureTemperature(float warning_deg_c, float shutdown_deg_c) noexcept;

    /** @brief Enable or disable overcurrent protection on the driver outputs.
     *
     * This controls the internal gate driver overcurrent detection (e.g.,
     * comparators) for various FETs.
     * @param enabled True to enable overcurrent protection (shut down drivers
     * on overcurrent), false to disable.
     * @return true if command was sent successfully.
     */
    bool setOvercurrentEnabled(bool enabled) noexcept;

    /** @brief Configure the two I²t monitoring windows for motor current.
     * @param timeConstant1_ms Time constant for first window in milliseconds.
     * @param continuousCurrent1_A Continuous current limit for first window in
     * amps.
     * @param timeConstant2_ms Time constant for second window in milliseconds.
     * @param continuousCurrent2_A Continuous current limit for second window in
     * amps.
     * @return true if configuration was successful.
     */
    bool configureI2t(uint16_t timeConstant1_ms, float continuousCurrent1_A,
                      uint16_t timeConstant2_ms, float continuousCurrent2_A) noexcept;

    /** @brief Reset the integrated I²t sum accumulators.
     * @return true if reset was successful.
     */
    bool resetI2tState() noexcept;

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------

    /** @brief Configuration structure for protection auto-configuration.
     */
    struct ProtectionConfig {
      // Voltage protection (optional, with defaults)
      /** @brief Overvoltage threshold in volts (optional, default: 28.0V for 24V systems).
       *
       * Threshold at which overvoltage warning is triggered.
       * Typical values:
       * - 24V systems: 28-30V (default: 28.0V)
       * - 48V systems: 56-60V
       * - 12V systems: 14-15V
       */
      std::optional<float>
          overvoltageThreshold_V; //!< Overvoltage threshold in volts (optional, default: 28.0V)

      /** @brief Undervoltage threshold in volts (optional, default: 20.0V for 24V systems).
       *
       * Threshold at which undervoltage warning is triggered.
       * Typical values:
       * - 24V systems: 18-20V (default: 20.0V)
       * - 48V systems: 40-44V
       * - 12V systems: 10-11V
       */
      std::optional<float>
          undervoltageThreshold_V; //!< Undervoltage threshold in volts (optional, default: 20.0V)

      // Temperature protection (optional, with defaults)
      /** @brief Temperature warning threshold in °C (optional, default: 80.0°C).
       *
       * Temperature at which warning is triggered.
       * Typical values:
       * - Standard: 80-85°C (default: 80.0°C)
       * - High-temp: 90-95°C
       */
      std::optional<float>
          temperatureWarning_C; //!< Temperature warning threshold in °C (optional, default: 80.0°C)

      /** @brief Temperature shutdown threshold in °C (optional, default: 100.0°C).
       *
       * Temperature at which motor is shut down for protection.
       * Typical values:
       * - Standard: 100-105°C (default: 100.0°C)
       * - High-temp: 110-115°C
       */
      std::optional<float> temperatureShutdown_C; //!< Temperature shutdown threshold in °C
                                                  //!< (optional, default: 100.0°C)

      // Overcurrent protection (optional, with default)
      /** @brief Enable overcurrent protection (optional, default: true).
       *
       * When enabled, the gate driver overcurrent detection will shut down
       * drivers on overcurrent conditions.
       */
      std::optional<bool>
          enableOvercurrent; //!< Enable overcurrent protection (optional, default: true)

      // I²t thermal protection (optional, with defaults)
      /** @brief I²t window 1 time constant in milliseconds (optional, default: 100ms).
       *
       * Time constant for the first I²t monitoring window.
       * Typical values: 50-200ms (default: 100ms)
       */
      std::optional<uint16_t>
          i2tTimeConstant1_ms; //!< I²t window 1 time constant [ms] (optional, default: 100ms)

      /** @brief I²t window 1 continuous current limit in Amps (optional, default: 1.5A).
       *
       * Continuous current limit for the first I²t monitoring window.
       * Should be set based on motor continuous current rating.
       */
      std::optional<float> i2tContinuousCurrent1_A; //!< I²t window 1 continuous current limit [A]
                                                    //!< (optional, default: 1.5A)

      /** @brief I²t window 2 time constant in milliseconds (optional, default: 1000ms).
       *
       * Time constant for the second I²t monitoring window (longer-term protection).
       * Typical values: 500-2000ms (default: 1000ms)
       */
      std::optional<uint16_t>
          i2tTimeConstant2_ms; //!< I²t window 2 time constant [ms] (optional, default: 1000ms)

      /** @brief I²t window 2 continuous current limit in Amps (optional, default: 1.25A).
       *
       * Continuous current limit for the second I²t monitoring window.
       * Typically set slightly lower than window 1 for longer-term protection.
       */
      std::optional<float> i2tContinuousCurrent2_A; //!< I²t window 2 continuous current limit [A]
                                                    //!< (optional, default: 1.25A)

      /** When false, configureAuto skips gateDriver.enableOvercurrentProtection (UVW/Y2 OC SAPs).
       *  When true, uses SAP on all four enables; if the full cluster fails (e.g. Y2 unsupported),
       *  `setOvercurrentEnabled` retries UVW-only. */
      bool program_gate_driver_overcurrent_enable = true;

      /** When false, configureAuto skips I²t time/limit SAP writes. */
      bool program_i2t_limits = true;

      /** When false, configureAuto skips configureVoltage (supply OV/UV warning SAPs). */
      bool program_supply_voltage_warnings = true;

      /** When false, configureAuto skips configureTemperature (chip temperature threshold SAPs). */
      bool program_chip_temperature_warnings = true;
    };

    /** @brief Auto-configure protection parameters.
     *
     * Configures voltage, temperature, overcurrent, and I²t protection based on
     * high-level protection requirements.
     *
     * @param config Protection configuration (see ProtectionConfig)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const ProtectionConfig& config) noexcept;

  private:
    friend class TMC9660;
    explicit Protection(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } protection{*this};

  //===========================================================================
  //==                  SUBSYSTEM: Script                                   ==//
  //===========================================================================

  /** @brief Subsystem for TMCL script execution control
   */
  struct Script {
    /** @brief Upload a TMCL script to the TMC9660's internal memory.
     *
     * This enters download mode, writes a series of instructions to the device
     * memory, and exits download mode. The script will typically run on device
     * startup or when triggered.
     * @param script_data Vector of 32-bit instructions representing the TMCL
     * script.
     * @return true if the script was uploaded successfully.
     */
    bool upload(const std::vector<uint32_t>& script_data) noexcept;

    /** @brief Start or restart execution of the stored script.
     * @param address The address from which to start execution (usually 0 for
     * beginning of script).
     * @return true if the command to start the script was sent.
     */
    bool start(uint16_t address = 0) noexcept;

    /** @brief Stop execution of the running script.
     * @return true if the stop command was sent successfully.
     */
    bool stop() noexcept;

    /** @brief Execute a single TMCL instruction of the loaded script.
     *
     * Uses the `ApplStep` opcode which advances the interpreter by one
     * instruction. Useful for debugging scripts in real time.
     */
    bool step() noexcept;

    /** @brief Reset the TMCL program counter.
     *
     * Sends the `ApplReset` command which stops execution and resets the
     * script to the beginning.
     */
    bool reset() noexcept;

    /** @brief Query the execution state of the running script.
     * @param[out] status Raw status value returned by `GetStatusScript`.
     * @return true if the status value was retrieved successfully.
     */
    bool getStatus(uint32_t& status) noexcept;

    /** @brief Read a 32-bit instruction from script memory.
     * @param address Instruction address to read.
     * @param[out] value Returned 32-bit TMCL instruction word.
     * @return true on success.
     */
    bool readMemory(uint16_t address, uint32_t& value) noexcept;

    /** @brief Add a breakpoint at the given address.
     * @param address Instruction address where execution should break.
     * @return true if the breakpoint was set.
     */
    bool addBreakpoint(uint16_t address) noexcept;

    /** @brief Remove a previously set breakpoint.
     * @param address Address of the breakpoint to remove.
     * @return true if removed successfully.
     */
    bool removeBreakpoint(uint16_t address) noexcept;

    /** @brief Remove all breakpoints from the script.
     * @return true if the command succeeded.
     */
    bool clearBreakpoints() noexcept;

    /** @brief Query the maximum number of supported breakpoints.
     * @param[out] count Maximum breakpoint count reported by the device.
     * @return true on success.
     */
    bool getMaxBreakpointCount(uint32_t& count) noexcept;

  private:
    friend class TMC9660;
    explicit Script(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } script{*this};

  //***************************************************************************
  //**                  SUBSYSTEM: RamDebug                                **//
  //***************************************************************************
  /** @brief Subsystem for debug and data logging features
   */
  struct RamDebug {
    /** @brief Initialize and configure the RAMDebug feature (data logging).
     *
     * This sends a command to initialize the RAM debug system and reset any
     * previous configurations.
     * @param sample_count Number of samples to collect in the buffer.
     * @return true if the RAM debug was initialized properly.
     */
    bool init(uint32_t sample_count) noexcept;

    /** @brief Start capturing data using RAMDebug.
     * @return true if the capture started successfully.
     */
    bool startCapture() noexcept;

    /** @brief Read a captured sample from RAMDebug buffer.
     * @param index Sample index to read (0-based).
     * @param[out] data Variable to store the 32-bit sample data.
     * @return true if the data was read successfully.
     */
    bool readData(uint32_t index, uint32_t& data) noexcept;

    /** @brief Get the current state of the RAM debug engine.
     * @param[out] is_running Will be set to true if capture is ongoing, false if
     * stopped or idle.
     * @return true if the status was retrieved successfully.
     */
    bool getStatus(bool& is_running) noexcept;

  private:
    friend class TMC9660;
    explicit RamDebug(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } ramDebug{*this};

  //===========================================================================
  //**                SUBSYSTEM: FLASH STORAGE                             ==//
  //===========================================================================

  /** @brief Subsystem for storing and recalling parameters from nonvolatile
   * flash.
   *
   * This API wraps STAP (0xFFF) and FactoryDefault commands. Use it to
   * persist all RWE parameters across power cycles, matching the functionality
   * of the “Save to Flash” button in GUI tools.
   *
   * WARNING: Requires that external memory is configured via BOOT_CONFIG.
   * Refer to the "Storing System Settings" section in the datasheet
   * (see page 15).
   */
  struct NvmStorage {
    /** @brief Store all writable parameters to flash or EEPROM.
     *
     * @return true if the operation completed without error.
     *
     * Internally sends `STAP` (type=0xFFF) to save parameters.
     */
    bool storeToFlash() noexcept;

    /** @brief Restore parameters previously saved to NVM.
     *
     * @return true if configuration was successfully recalled.
     *
     * Sets CONFIG_LOADED flag in GENERAL_STATUS_FLAGS if success.
     */
    bool recallFromFlash() noexcept;

    /** @brief Erase a configuration bank from external memory.
     *
     * @param n Index of the flash bank to erase (typically 0)
     * @return true if erase command sent successfully.
     *
     * Use before re-storing to flash if stale config causes issues.
     */
    bool eraseFlashBank(uint8_t n) noexcept;

  private:
    friend class TMC9660;
    explicit NvmStorage(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } nvmStorage{*this};

  //===========================================================================
  //==                SUBSYSTEM: Heartbeat (Watchdog)                       ==//
  //===========================================================================

  /** @brief Subsystem for configuring the communication watchdog (heartbeat).
   *
   * If enabled, the TMC9660 monitors the time since the last command.
   * If no communication occurs before timeout expires, the chip faults or
   * disables motor outputs.
   *
   * Global bank 0 parameters 3 (`HEARTBEAT_MONITORING_CONFIG`) and 4
   * (`HEARTBEAT_MONITORING_TIMEOUT`), written with SGP — never with SAP, where ids 3/4 are
   * MOTOR_PWM_FREQUENCY / COMMUTATION_MODE (Parameter Mode Reference Manual, global
   * parameters bank 0). The watchdog only sees datagrams on the interface(s) it monitors, so
   * it must be armed for the interface the host actually uses: SPI on SPI-connected boards
   * such as Vortex V1.
   */
  struct Heartbeat {
    /** @brief Arm or disarm the heartbeat monitor on an explicit interface.
     *
     * @param mode Interface(s) to monitor, or DISABLED.
     * @param timeout_ms Timeout in milliseconds (1 … 2^32−1).
     * @return true if both values were written successfully.
     *
     * The timeout is written before the interface, so arming never runs with a stale
     * timeout. Both writes go to global bank 0.
     */
    bool configure(tmc9660::tmcl::HeartbeatMonitoringConfig mode, uint32_t timeout_ms) noexcept;

    /** @brief Interface value matching the driver's own transport: SPI_INTERFACE for an
     *  SPI CommType, TMCL_UART_INTERFACE for UART. */
    [[nodiscard]] tmc9660::tmcl::HeartbeatMonitoringConfig transportInterface() const noexcept;

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------

    /** @brief Configuration structure for heartbeat auto-configuration.
     */
    struct HeartbeatConfig {
      /** @brief Enable heartbeat monitoring (optional, default: false).
       *
       * When enabled, the TMC9660 monitors communication timeout and faults
       * if no commands are received within the timeout period.
       */
      std::optional<bool> enable; //!< Enable heartbeat monitoring (optional, default: false)

      /** @brief Heartbeat timeout in milliseconds (optional, default: 1000ms).
       *
       * Timeout period after which the chip faults if no communication occurs.
       * Typical values: 500-5000ms (default: 1000ms)
       */
      std::optional<uint32_t> timeoutMs; //!< Heartbeat timeout [ms] (optional, default: 1000ms)

      /** @brief Interface the watchdog monitors when enabled (optional).
       *
       * Default: the transport this driver instance talks over (transportInterface()), i.e.
       * SPI_INTERFACE on SPI boards. Releases before 1.1.0 always armed TMCL_UART_INTERFACE,
       * which on an SPI board never sees traffic. Set TMCL_UART_AND_SPI_INTERFACE to watch
       * both. Ignored when @ref enable is false.
       */
      std::optional<tmc9660::tmcl::HeartbeatMonitoringConfig> interface;
    };

    /** @brief Configure heartbeat monitoring from a HeartbeatConfig.
     *
     * Disabled unless @c enable is set. When enabled, the monitored interface is
     * @c interface if given, otherwise the driver's own transport (no guessing).
     *
     * @param config Heartbeat configuration (see HeartbeatConfig)
     * @return true if configuration succeeded, false otherwise
     */
    bool configureAuto(const HeartbeatConfig& config) noexcept;

  private:
    friend class TMC9660;
    explicit Heartbeat(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } heartbeat{*this};

  //***************************************************************************
  //**                   SUBSYSTEM: Global Parameter Access                **//
  //***************************************************************************
  /**
   * @brief Convenience helpers for reading and writing global parameters.
   *
   * The TMC9660 groups non-motion parameters into banks. This subsystem
   * provides simple wrappers to access them using the enum classes defined in
   * `tmc9660_param_mode_tmcl.hpp`.
   */
  struct Globals {
    /// Write a value to bank 0 (system settings).
    bool writeBank0(tmc9660::tmcl::GlobalParamBank0 param, uint32_t value) noexcept;

    /// Read a value from bank 0 (system settings).
    bool readBank0(tmc9660::tmcl::GlobalParamBank0 param, uint32_t& value) noexcept;

    /// Write a signed user variable in bank 2.
    bool writeBank2(tmc9660::tmcl::GlobalParamBank2 param, int32_t value) noexcept;

    /// Read a signed user variable from bank 2.
    bool readBank2(tmc9660::tmcl::GlobalParamBank2 param, int32_t& value) noexcept;

    /// Write a value to bank 3 (interrupt configuration).
    bool writeBank3(tmc9660::tmcl::GlobalParamBank3 param, uint32_t value) noexcept;

    /// Read a value from bank 3 (interrupt configuration).
    bool readBank3(tmc9660::tmcl::GlobalParamBank3 param, uint32_t& value) noexcept;

    // High level helpers --------------------------------------------------
    /// Set module serial address (bank0:SERIAL_ADDRESS).
    bool setSerialAddress(uint8_t address) noexcept;

    /// Get module serial address (bank0:SERIAL_ADDRESS).
    bool getSerialAddress(uint8_t& address) noexcept;

    /// Set host serial address (bank0:SERIAL_HOST_ADDRESS).
    bool setHostAddress(uint8_t address) noexcept;

    /// Get host serial address (bank0:SERIAL_HOST_ADDRESS).
    bool getHostAddress(uint8_t& address) noexcept;

    /// Configure heartbeat monitoring interface and timeout.
    bool configureHeartbeat(tmc9660::tmcl::HeartbeatMonitoringConfig iface,
                            uint32_t timeout_ms) noexcept;

    /// Read heartbeat monitoring configuration.
    bool getHeartbeat(tmc9660::tmcl::HeartbeatMonitoringConfig& iface,
                      uint32_t& timeout_ms) noexcept;

    /// Set GPIO direction mask (bank0:IO_DIRECTION_MASK).
    bool setIODirectionMask(uint32_t mask) noexcept;

    /// Get GPIO direction mask (bank0:IO_DIRECTION_MASK).
    bool getIODirectionMask(uint32_t& mask) noexcept;

    /// Set pull-up/down enable mask (bank0:IO_INPUT_PULLUP_PULLDOWN_ENABLE_MASK).
    bool setPullEnableMask(uint32_t mask) noexcept;

    /// Get pull-up/down enable mask.
    bool getPullEnableMask(uint32_t& mask) noexcept;

    /// Set pull direction mask (bank0:IO_INPUT_PULLUP_PULLDOWN_DIRECTION_MASK).
    bool setPullDirectionMask(uint32_t mask) noexcept;

    /// Get pull direction mask.
    bool getPullDirectionMask(uint32_t& mask) noexcept;

    /// Enable or disable auto-start of stored program (bank0:AUTO_START_ENABLE).
    bool setAutoStart(bool enable) noexcept;

    /// Read auto-start enable flag.
    bool getAutoStart(bool& enable) noexcept;

    /// Configure clearing of user variables on startup (bank0:CLEAR_USER_VARIABLES).
    bool setClearUserVariables(bool clear) noexcept;

    /// Get clear-user-variable flag.
    bool getClearUserVariables(bool& clear) noexcept;

    /// Set user variable by index (bank2).
    bool setUserVariable(uint8_t index, int32_t value) noexcept;

    /// Read user variable by index (bank2).
    bool getUserVariable(uint8_t index, int32_t& value) noexcept;

    /// Set interrupt timer period (bank3).
    bool setTimerPeriod(uint8_t timer, uint32_t period_ms) noexcept;

    /// Get interrupt timer period (bank3).
    bool getTimerPeriod(uint8_t timer, uint32_t& period_ms) noexcept;

    /// Set trigger transition for digital input n (bank3 INPUT_n_TRIGGER_TRANSITION).
    bool setInputTrigger(uint8_t index, tmc9660::tmcl::TriggerTransition transition) noexcept;

    /// Get trigger transition for digital input n.
    bool getInputTrigger(uint8_t index, tmc9660::tmcl::TriggerTransition& transition) noexcept;

  private:
    friend class TMC9660;
    explicit Globals(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } globals{*this};

  //***************************************************************************
  //**        SUBSYSTEM: General-purpose GPIO (Digital/Analog I/O)          **//
  //***************************************************************************

  /** @brief Subsystem for configuring general-purpose IOs (GPIOs).
   *
   * Pins can be configured as digital inputs, digital outputs, or analog
   * inputs. The input pull-up/down resistors and output state can also be
   * controlled.
   *
   * Refer to the GPIO section and TMCL commands SIO / GIO (Table 18).
   * See datasheet page 19.
   */
  struct GPIO {
    /** @brief Configure a GPIO pin as input or output.
     *
     * @param pin GPIO index (e.g. GPIO0..GPIO18)
     * @param output Set to true to make the pin output, false = input
     * @param pullEnable Enable pull resistor
     * @param pullUp true = pull-up, false = pull-down
     * @return true if configuration applied successfully
     */
    bool setMode(uint8_t pin, bool output, bool pullEnable = false, bool pullUp = true) noexcept;

    /** @brief Write a digital value to a configured output pin.
     * @param pin GPIO pin index
     * @param value true = high, false = low
     * @return true if write succeeded
     */
    bool writePin(uint8_t pin, bool value) noexcept;

    /** @brief Read a digital input pin.
     * @param pin GPIO pin index
     * @param[out] value Logic level read from the pin
     * @return true on successful read
     */
    bool readDigital(uint8_t pin, bool& value) noexcept;

    /** @brief Read an analog input (e.g. external temperature or potentiometer).
     * @param pin ADC input index
     * @param[out] value Raw ADC value (typically 0–65535)
     * @return true on success
     *
     * - Analog reads typically apply to AIN3, used with external thermistors.
     */
    bool readAnalog(uint8_t pin, uint16_t& value) noexcept;

  private:
    friend class TMC9660;
    explicit GPIO(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } gpio{*this};

  //***************************************************************************
  //**               SUBSYSTEM: Power Management                          **//
  //***************************************************************************

  /** @brief Subsystem for entering low-power hibernation mode and configuring
   * wake.
   *
   * The TMC9660 supports timed power-down and wake-on-pin. These features
   * reduce power when idle.
   *
   * - See the "Hibernation and Wakeup" section (datasheet page 101).
   * - Wake behavior depends on BOOT_CONFIG and external wiring.
   */
  struct Power {
    /** @brief Enable or disable the external wake-up pin.
     * @param enable true = enable pin
     * @return true on success
     *
     * - Parameter: `ENABLE_WAKE_PIN`
     */
    bool enableWakePin(bool enable) noexcept;

    /** @brief Put the chip into power-down mode for a set duration.
     * @param period Enum selecting one of 6 durations (e.g. PERIOD_1 = 250 ms)
     * @return true on success
     *
     * - Parameter: `GO_TO_TIMEOUT_POWER_DOWN_STATE`
     */
    bool enterPowerDown(tmc9660::tmcl::PowerDownTimeout period) noexcept;

    //-------------------------------------------------------------------------
    // Auto-Configuration
    //-------------------------------------------------------------------------

    /** @brief Configuration structure for power management auto-configuration.
     */
    struct PowerConfig {
      /** @brief Enable external wake-up pin (optional, default: false).
       *
       * When enabled, the external wake-up pin can be used to wake the chip
       * from power-down mode.
       */
      std::optional<bool> enableWakePin; //!< Enable wake-up pin (optional, default: false)

      /** @brief Power-down timeout period (optional, default: disabled).
       *
       * Duration after which the chip enters power-down mode if no activity.
       * If not provided, power-down is disabled.
       */
      std::optional<tmc9660::tmcl::PowerDownTimeout>
          powerDownTimeout; //!< Power-down timeout (optional, disabled if not provided)
    };

    /** @brief Auto-configure power management parameters.
     *
     * Configures wake-up pin and power-down timeout based on high-level requirements.
     *
     * @param config Power configuration (see PowerConfig)
     * @return true if all configurations succeeded, false otherwise
     */
    bool configureAuto(const PowerConfig& config) noexcept;

  private:
    friend class TMC9660;
    explicit Power(TMC9660& parent) noexcept : driver(parent) {}
    TMC9660& driver;
  } power{*this};

  // ===========================================================================
  // Driver Version
  // ===========================================================================

  /** @brief Get the compiled driver version string. */
  static constexpr const char* GetDriverVersion() noexcept {
    return HF_TMC9660_VERSION_STRING;
  }

  /** @brief Get the compiled driver major version number. */
  static constexpr uint8_t GetDriverVersionMajor() noexcept {
    return HF_TMC9660_VERSION_MAJOR;
  }

  /** @brief Get the compiled driver minor version number. */
  static constexpr uint8_t GetDriverVersionMinor() noexcept {
    return HF_TMC9660_VERSION_MINOR;
  }

  /** @brief Get the compiled driver patch version number. */
  static constexpr uint8_t GetDriverVersionPatch() noexcept {
    return HF_TMC9660_VERSION_PATCH;
  }

  //==================================================
  // PRIVATE MEMBERS
  //==================================================
private:
  /// Chunked driver for the batch helpers: build frames, run sendCommandBatch(), store results.
  template <typename Item, typename MakeFrame, typename Store>
  bool runBatch(std::span<Item> items, MakeFrame make_frame, Store store) noexcept;

  CommType& comm_; ///< Communication interface (transport) for
                               ///< sending/receiving data.
  uint8_t address_;            ///< Module address (0-127). Used primarily for UART
                               ///< multi-drop addressing.
  std::unique_ptr<tmc9660::TMC9660Bootloader<CommType>> bootloader_; ///< Bootloader helper
  const tmc9660::BootloaderConfig* bootCfg_;
  DeviceCapabilities caps_{}; ///< Feature topology — auto-derived from bootCfg_ in ctor; mutable via setCapabilities().

#ifdef TMC_API_EXTERNAL_CRC_TABLE
  extern const uint8_t tmcCRCTable_Poly7Reflected[256];
#else
  const uint8_t tmcCRCTable_Poly7Reflected[256] = {
      0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA,
      0x7B, 0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84,
      0xF6, 0x67, 0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31,
      0xA0, 0xD2, 0x43, 0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58,
      0x2D, 0xBC, 0xCE, 0x5F, 0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D,
      0x0C, 0x79, 0xE8, 0x9A, 0x0B, 0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3,
      0x81, 0x10, 0x65, 0xF4, 0x86, 0x17, 0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46,
      0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33, 0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21,
      0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F, 0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04,
      0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B, 0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A,
      0x18, 0x89, 0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87, 0xD8, 0x49, 0x3B, 0xAA, 0xDF,
      0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3, 0xC4, 0x55, 0x27, 0xB6,
      0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF, 0x90, 0x01, 0x73,
      0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB, 0x8C, 0x1D,
      0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7, 0xA8,
      0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
      0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E,
      0xCF,
  };
#endif
};

// Include template implementation
#define TMC9660_HEADER_INCLUDED
// NOLINTNEXTLINE(bugprone-suspicious-include) - Intentional: template implementation file
#include "../src/tmc9660.ipp"
#undef TMC9660_HEADER_INCLUDED

/** @brief Get the TMC9660 driver version string (namespace-level). */
inline const char* GetDriverVersion() noexcept {
  return HF_TMC9660_VERSION_STRING;
}

} // namespace tmc9660
