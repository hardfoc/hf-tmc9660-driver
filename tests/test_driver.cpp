/**
 * @file test_driver.cpp
 * @brief Driver-level behaviour on the mock chip: batch helpers, global banks, heartbeat
 *        interface selection and RAMDebug command encoding.
 */
#include "test_harness.hpp"

#include "mock_tmc9660.hpp"

using tmc9660::TMC9660;
using tmc9660::tmcl::GlobalParamBank0;
using tmc9660::tmcl::GlobalParamBank2;
using tmc9660::tmcl::HeartbeatMonitoringConfig;
using tmc9660::tmcl::Parameters;
using tmc9660::tmcl::RamDebugState;
using tmc9660_test::MockTmc9660Spi;
using tmc9660_test::MockTmc9660Uart;
using tmc9660_test::SeenFrame;

using SpiDriver = TMC9660<MockTmc9660Spi>;
using UartDriver = TMC9660<MockTmc9660Uart>;

// ---------------------------------------------------------------------------------------
// Compile-time bank checks
// ---------------------------------------------------------------------------------------

// Heartbeat/hibernation are not axis parameters any more (they aliased ids 3/4/10/11).
template <typename T>
concept HasAxisHeartbeat = requires { T::HEARTBEAT_MONITORING_CONFIG; };
static_assert(!HasAxisHeartbeat<Parameters>);
template <typename T>
concept HasAxisWakePin = requires { T::WAKE_PIN_CONTROL_ENABLE; };
static_assert(!HasAxisWakePin<Parameters>);
static_assert(static_cast<uint16_t>(Parameters::MOTOR_PWM_FREQUENCY) == 3);
static_assert(static_cast<uint16_t>(Parameters::COMMUTATION_MODE) == 4);
static_assert(static_cast<uint16_t>(GlobalParamBank0::HEARTBEAT_MONITORING_CONFIG) == 3);
static_assert(static_cast<uint16_t>(GlobalParamBank0::HEARTBEAT_MONITORING_TIMEOUT) == 4);
static_assert(tmc9660::tmcl::detail::axisParameterIdsUnique());

// Banks follow from the enum type.
static_assert(tmc9660::tmcl::kGlobalBankOf<GlobalParamBank0> == 0);
static_assert(tmc9660::tmcl::kGlobalBankOf<GlobalParamBank2> == 2);
static_assert(tmc9660::tmcl::kGlobalBankOf<tmc9660::tmcl::GlobalParamBank3> == 3);
static_assert(!tmc9660::tmcl::IsGlobalParamBank<Parameters>);

// Axis ids cannot be sent with SGP/GGP; global ids cannot be sent with SAP/GAP.
template <typename D>
concept CanWriteGlobalWithAxisId =
    requires(D d, uint32_t v) { d.writeGlobal(Parameters::COMMUTATION_MODE, uint8_t{0}, v); };
static_assert(!CanWriteGlobalWithAxisId<SpiDriver>);
template <typename D>
concept CanReadGlobalWithAxisId =
    requires(D d, uint32_t& v) { d.readGlobal(Parameters::COMMUTATION_MODE, uint8_t{0}, v); };
static_assert(!CanReadGlobalWithAxisId<SpiDriver>);
template <typename D>
concept CanSapGlobalId =
    requires(D d) { d.writeParameter(GlobalParamBank0::HEARTBEAT_MONITORING_TIMEOUT, 20U); };
static_assert(!CanSapGlobalId<SpiDriver>);
static_assert(!std::is_constructible_v<SpiDriver::GlobalParameterAccess, Parameters>);
static_assert(std::is_constructible_v<SpiDriver::GlobalParameterAccess, GlobalParamBank0>);

namespace {
bool sameFrame(const SeenFrame& f, uint8_t op, uint16_t type, uint8_t motor, uint32_t value) {
  return f.opcode == op && f.type == type && f.motor == motor && f.value == value;
}
} // namespace

// ---------------------------------------------------------------------------------------
// Batch helpers
// ---------------------------------------------------------------------------------------

TEST_CASE("readParameters returns values in request order with N+1 datagrams") {
  MockTmc9660Spi bus;
  SpiDriver drv(bus);
  bus.axis[MockTmc9660Spi::key(0, static_cast<uint16_t>(Parameters::ACTUAL_VELOCITY))] = 111;
  bus.axis[MockTmc9660Spi::key(0, static_cast<uint16_t>(Parameters::ACTUAL_POSITION))] = 222;
  bus.axis[MockTmc9660Spi::key(0, static_cast<uint16_t>(Parameters::GENERAL_STATUS_FLAGS))] = 333;
  SpiDriver::ParameterAccess items[] = {{Parameters::ACTUAL_VELOCITY},
                                        {Parameters::ACTUAL_POSITION},
                                        {Parameters::GENERAL_STATUS_FLAGS}};
  REQUIRE(drv.readParameters(items));
  CHECK_EQ(items[0].value, 111U);
  CHECK_EQ(items[1].value, 222U);
  CHECK_EQ(items[2].value, 333U);
  CHECK(items[0].ok() && items[1].ok() && items[2].ok());
  CHECK_EQ(bus.datagrams, 4U);
}

TEST_CASE("readParameters spans several chunks without losing order") {
  MockTmc9660Spi bus;
  SpiDriver drv(bus);
  std::vector<SpiDriver::ParameterAccess> items;
  for (uint16_t i = 0; i < 40; ++i) {
    bus.axis[MockTmc9660Spi::key(0, static_cast<uint16_t>(100 + i))] = 7000U + i;
    items.push_back({static_cast<Parameters>(100 + i)});
  }
  REQUIRE(drv.readParameters(items));
  for (uint16_t i = 0; i < 40; ++i) {
    CHECK_EQ(items[i].value, 7000U + i);
  }
  CHECK_EQ(bus.datagrams, 40U + 3U); // 16 + 16 + 8 per chunk, + 1 reply-request each
}

TEST_CASE("writeParameters writes each value once") {
  MockTmc9660Spi bus;
  SpiDriver drv(bus);
  SpiDriver::ParameterAccess items[] = {{Parameters::MOTOR_PWM_FREQUENCY, 25000},
                                        {Parameters::COMMUTATION_MODE, 0}};
  REQUIRE(drv.writeParameters(items));
  CHECK_EQ(bus.axis[MockTmc9660Spi::key(0, 3)], 25000U);
  CHECK_EQ(bus.executionsOf(5, 3), 1);
  CHECK_EQ(bus.executionsOf(5, 4), 1);
}

TEST_CASE("global batches carry the bank from the enum type") {
  MockTmc9660Spi bus;
  SpiDriver drv(bus);
  SpiDriver::GlobalParameterAccess w[] = {{GlobalParamBank0::HEARTBEAT_MONITORING_TIMEOUT, 20},
                                          {GlobalParamBank2::USER_VARIABLE_15, 0xC0FFEE}};
  REQUIRE(drv.writeGlobalParameters(w));
  const auto acc = bus.accepted();
  REQUIRE(acc.size() == 2);
  CHECK(sameFrame(acc[0], 9, 4, 0, 20));
  CHECK(sameFrame(acc[1], 9, 15, 2, 0xC0FFEE));
  SpiDriver::GlobalParameterAccess r[] = {{GlobalParamBank2::USER_VARIABLE_15}};
  REQUIRE(drv.readGlobalParameters(r));
  CHECK_EQ(r[0].value, 0xC0FFEEU);
  CHECK_EQ(bus.executionsOf(5, 4), 0); // no SAP of COMMUTATION_MODE
}

TEST_CASE("bank-mismatched global access is refused before it reaches the chip") {
  MockTmc9660Spi bus;
  SpiDriver drv(bus);
  CHECK(!drv.writeGlobalParameter(GlobalParamBank0::HEARTBEAT_MONITORING_TIMEOUT, 2, 20));
  uint32_t v = 0;
  CHECK(!drv.readGlobalParameter(GlobalParamBank0::MAIN_LOOPS, 3, v));
  CHECK_EQ(bus.datagrams, 0U);
  CHECK(drv.writeGlobalParameter(uint8_t{4}, 0, 20)); // raw number: caller owns the bank
  CHECK(drv.writeGlobal(GlobalParamBank0::HEARTBEAT_MONITORING_TIMEOUT, 21U));
  CHECK_EQ(bus.globals[MockTmc9660Spi::key(0, 4)], 21U);
}

TEST_CASE("UART transport runs batches as plain request/reply") {
  MockTmc9660Uart bus;
  UartDriver drv(bus, 1);
  bus.axis[tmc9660_test::Tmc9660Model::key(0, 125)] = 9;
  bus.axis[tmc9660_test::Tmc9660Model::key(0, 144)] = 10;
  UartDriver::ParameterAccess items[] = {{Parameters::ACTUAL_VELOCITY},
                                         {Parameters::ACTUAL_POSITION}};
  REQUIRE(drv.readParameters(items));
  CHECK_EQ(items[0].value, 9U);
  CHECK_EQ(items[1].value, 10U);
}

// ---------------------------------------------------------------------------------------
// Heartbeat
// ---------------------------------------------------------------------------------------

TEST_CASE("heartbeat on an SPI driver arms the SPI watchdog in global bank 0") {
  MockTmc9660Spi bus;
  SpiDriver drv(bus);
  SpiDriver::Heartbeat::HeartbeatConfig cfg{};
  cfg.enable = true;
  cfg.timeoutMs = 20;
  REQUIRE(drv.heartbeat.configureAuto(cfg));
  const auto acc = bus.accepted();
  REQUIRE(acc.size() == 2);
  CHECK(sameFrame(acc[0], 9, 4, 0, 20)); // SGP bank 0 #4 timeout first
  CHECK(sameFrame(acc[1], 9, 3, 0, 2));  // SGP bank 0 #3 = SPI_INTERFACE
  CHECK_EQ(bus.executionsOf(5, 3), 0);   // never SAP MOTOR_PWM_FREQUENCY
  CHECK_EQ(bus.executionsOf(5, 4), 0);   // never SAP COMMUTATION_MODE
  CHECK(drv.heartbeat.transportInterface() == HeartbeatMonitoringConfig::SPI_INTERFACE);
}

TEST_CASE("heartbeat on a UART driver arms the UART watchdog; explicit override wins") {
  MockTmc9660Uart bus;
  UartDriver drv(bus, 1);
  UartDriver::Heartbeat::HeartbeatConfig cfg{};
  cfg.enable = true;
  cfg.timeoutMs = 50;
  REQUIRE(drv.heartbeat.configureAuto(cfg));
  REQUIRE(bus.seen.size() == 2);
  CHECK(sameFrame(bus.seen[1], 9, 3, 0, 1));

  bus.seen.clear();
  cfg.interface = HeartbeatMonitoringConfig::TMCL_UART_AND_SPI_INTERFACE;
  REQUIRE(drv.heartbeat.configureAuto(cfg));
  CHECK(sameFrame(bus.seen[1], 9, 3, 0, 3));
}

TEST_CASE("heartbeat defaults to disabled") {
  MockTmc9660Spi bus;
  SpiDriver drv(bus);
  REQUIRE(drv.heartbeat.configureAuto({}));
  const auto acc = bus.accepted();
  REQUIRE(acc.size() == 2);
  CHECK(sameFrame(acc[0], 9, 4, 0, 1000));
  CHECK(sameFrame(acc[1], 9, 3, 0, 0));
}

// ---------------------------------------------------------------------------------------
// RAMDebug (Table 16 encoding)
// ---------------------------------------------------------------------------------------

TEST_CASE("RAMDebug configure() sends the Table 16 sequence with correct fields") {
  MockTmc9660Spi bus;
  SpiDriver drv(bus);
  using RD = SpiDriver::RamDebug;
  RD::CaptureConfig cfg{};
  cfg.channels[0] = RD::Channel::axis(Parameters::ACTUAL_VELOCITY);
  cfg.channels[1] = RD::Channel::global(GlobalParamBank0::MAIN_LOOPS);
  cfg.channelCount = 2;
  cfg.samplesPerChannel = 256;
  cfg.pretriggerSamplesPerChannel = 64;
  cfg.prescaler = 4;
  cfg.trigger.source = RD::Channel::axis(Parameters::GENERAL_ERROR_FLAGS, 0);
  cfg.trigger.type = RD::TriggerType::RISING_EDGE_UNSIGNED;
  cfg.trigger.threshold = 0;
  cfg.trigger.mask = 0x00FF0000;
  cfg.trigger.shift = 15;
  REQUIRE(drv.ramDebug.configure(cfg));
  REQUIRE(drv.ramDebug.start(cfg.trigger));

  const uint16_t err = static_cast<uint16_t>(Parameters::GENERAL_ERROR_FLAGS);
  const auto acc = bus.accepted();
  REQUIRE(acc.size() == 9);
  CHECK(sameFrame(acc[0], 142, 0, 0, 0));           // reset
  CHECK(sameFrame(acc[1], 142, 1, 0, 512));         // total samples
  CHECK(sameFrame(acc[2], 142, 3, 0, 4));           // prescaler
  CHECK(sameFrame(acc[3], 142, 4, 1, 125));         // axis param 125, motor 0
  CHECK(sameFrame(acc[4], 142, 4, 3, 12));          // global bank 0 #12
  CHECK(sameFrame(acc[5], 142, 13, 0, 128));        // total pretrigger samples
  CHECK(sameFrame(acc[6], 142, 6, 15, 0x00FF0000)); // shift in MOTOR, mask in VALUE
  CHECK(sameFrame(acc[7], 142, 5, 1, err));         // trigger channel
  CHECK(sameFrame(acc[8], 142, 7, 4, 0));           // enable trigger: rising unsigned
}

TEST_CASE("RAMDebug channel address packs motor/bank in the top byte") {
  using RD = SpiDriver::RamDebug;
  constexpr auto c = RD::Channel::axis(Parameters::ACTUAL_POSITION, 1);
  static_assert(c.address() == ((1U << 24) | 144U));
  constexpr auto g = RD::Channel::global(GlobalParamBank2::USER_VARIABLE_3);
  static_assert(g.address() == ((2U << 24) | 3U));
  static_assert(static_cast<uint8_t>(g.type) == 3);
  CHECK(true);
}

TEST_CASE("RAMDebug compatibility calls use the corrected sub-commands") {
  MockTmc9660Spi bus;
  SpiDriver drv(bus);
  REQUIRE(drv.ramDebug.init(100));
  REQUIRE(drv.ramDebug.startCapture());
  bus.ramDebugCaptured = 10;
  uint32_t v = 0;
  REQUIRE(drv.ramDebug.readData(7, v));
  CHECK_EQ(v, 0xA007U);
  const auto acc = bus.accepted();
  REQUIRE(acc.size() == 4);
  CHECK(sameFrame(acc[0], 142, 0, 0, 0));
  CHECK(sameFrame(acc[1], 142, 1, 0, 100));
  CHECK(sameFrame(acc[2], 142, 7, 0, 0)); // ENABLE_TRIGGER, not SET_TRIGGER_MASK_SHIFT (6)
  CHECK(sameFrame(acc[3], 142, 9, 0, 7)); // READ_SAMPLE with the index in VALUE, not ReadMem
}

TEST_CASE("RAMDebug state decoding: COMPLETE is not running") {
  MockTmc9660Spi bus;
  SpiDriver drv(bus);
  bool running = true;
  RamDebugState st{};
  const struct {
    uint8_t raw;
    bool running;
  } cases[] = {{0, false}, {1, true}, {2, true}, {3, false}, {4, true}};
  for (const auto& c : cases) {
    bus.ramDebugState = c.raw;
    REQUIRE(drv.ramDebug.getStatus(running));
    CHECK_EQ(running, c.running);
    REQUIRE(drv.ramDebug.getState(st));
    CHECK_EQ(static_cast<uint8_t>(st), c.raw);
  }
  bus.ramDebugState = 9;
  CHECK(!drv.ramDebug.getState(st));
}

TEST_CASE("RAMDebug readSamples pipelines READ_SAMPLE and stops at the captured count") {
  MockTmc9660Spi bus;
  SpiDriver drv(bus);
  bus.ramDebugCaptured = 20;
  std::vector<uint32_t> out(20);
  size_t n = 0;
  REQUIRE(drv.ramDebug.readSamples(0, out, &n));
  CHECK_EQ(n, 20U);
  for (uint32_t i = 0; i < 20; ++i) {
    CHECK_EQ(out[i], 0xA000U + i);
  }
  CHECK_EQ(bus.datagrams, 20U + 2U); // two chunks (16 + 4), one reply-request each

  std::vector<uint32_t> more(8);
  CHECK(!drv.ramDebug.readSamples(16, more, &n));
  CHECK_EQ(n, 4U);
}

TEST_CASE("RAMDebug rejects configs it cannot encode") {
  MockTmc9660Spi bus;
  SpiDriver drv(bus);
  using RD = SpiDriver::RamDebug;
  CHECK(!drv.ramDebug.setTriggerMaskShift(0xFFFF, 16));
  RD::CaptureConfig cfg{};
  CHECK(!drv.ramDebug.configure(cfg)); // no channels
  cfg.channelCount = 5;
  cfg.samplesPerChannel = 10;
  CHECK(!drv.ramDebug.configure(cfg)); // too many channels
  cfg.channelCount = 1;
  cfg.pretriggerSamplesPerChannel = 11;
  CHECK(!drv.ramDebug.configure(cfg)); // pretrigger > samples
  CHECK_EQ(bus.datagrams, 0U);
}
