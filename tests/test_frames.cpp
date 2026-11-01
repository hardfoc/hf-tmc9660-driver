/**
 * @file test_frames.cpp
 * @brief TMCL frame encode/decode, checksum and reply classification.
 */
#include "test_harness.hpp"

#include "tmc9660.hpp"

using tmc9660::classifyTmclSpiReply;
using tmc9660::TMCLFrame;
using tmc9660::TMCLReply;
using tmc9660::TmclSpiRxClass;

namespace {
std::array<uint8_t, 8> withChecksum(std::array<uint8_t, 8> b) {
  b[7] = tmc9660::tmclChecksum(b.data(), 7);
  return b;
}
} // namespace

TEST_CASE("checksum is the 8-bit sum of the leading bytes") {
  static constexpr uint8_t bytes[] = {0x06, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK_EQ(tmc9660::tmclChecksum(bytes, 7), 0x83);
  const uint8_t wrap[] = {0xFF, 0xFF, 0x03};
  CHECK_EQ(tmc9660::tmclChecksum(wrap, 3), 0x01);
  static_assert(tmc9660::TMCLFrame::calculateChecksum(bytes, 7) == 0x83);
}

TEST_CASE("SPI request packs TYPE[11:8] in the high nibble and MOTOR in the low nibble") {
  TMCLFrame f{};
  f.opcode = 5; // SAP
  f.type = 0x12B;
  f.motor = 0x3;
  f.value = 0xDEADBEEF;
  std::array<uint8_t, 8> b{};
  f.toSpi(b);
  CHECK_EQ(b[0], 0x05);
  CHECK_EQ(b[1], 0x2B);
  CHECK_EQ(b[2], 0x13);
  CHECK_EQ(b[3], 0xDE);
  CHECK_EQ(b[4], 0xAD);
  CHECK_EQ(b[5], 0xBE);
  CHECK_EQ(b[6], 0xEF);
  CHECK_EQ(b[7], tmc9660::tmclChecksum(b.data(), 7));
  const TMCLFrame back = TMCLFrame::fromSpi(b);
  CHECK_EQ(back.opcode, 5);
  CHECK_EQ(back.type, 0x12B);
  CHECK_EQ(back.motor, 3);
  CHECK_EQ(back.value, 0xDEADBEEFU);
}

TEST_CASE("UART request carries sync bit, address and the same TYPE/MOTOR packing") {
  TMCLFrame f{};
  f.opcode = 6;
  f.type = 0x14B; // 331
  f.motor = 0;
  f.value = 0;
  std::array<uint8_t, 9> b{};
  f.toUart(0x02, b);
  CHECK_EQ(b[0], 0x03);
  CHECK_EQ(b[1], 0x06);
  CHECK_EQ(b[2], 0x4B);
  CHECK_EQ(b[3], 0x10);
  CHECK_EQ(b[8], tmc9660::tmclChecksum(b.data(), 8));
}

TEST_CASE("SPI reply decode: OK, bad checksum, NOT_READY, CHECKSUM_ERROR") {
  TMCLReply r{};
  const auto ok = withChecksum({0xFF, 100, 6, 0x00, 0x00, 0x01, 0x02, 0});
  CHECK(TMCLReply::fromSpi(ok, r));
  CHECK(r.isOK());
  CHECK_EQ(r.opcode, 6);
  CHECK_EQ(r.value, 0x0102U);

  auto bad = ok;
  bad[7] ^= 0x01;
  CHECK(!TMCLReply::fromSpi(bad, r));

  const auto busy = withChecksum({0xF0, 0, 0, 0, 0, 0, 0, 0});
  CHECK(!TMCLReply::fromSpi(busy, r));

  const std::array<uint8_t, 8> zeros{};
  CHECK(!TMCLReply::fromSpi(zeros, r));

  const auto wrong_type = withChecksum({0xFF, 3, 6, 0, 0, 0, 0, 0});
  CHECK(TMCLReply::fromSpi(wrong_type, r));
  CHECK(!r.isOK());
}

TEST_CASE("reply classification follows the NOT_READY / echo rules") {
  const auto ok = withChecksum({0xFF, 100, 6, 0, 0, 0, 7, 0});
  CHECK(classifyTmclSpiReply(ok, 6, true) == TmclSpiRxClass::VALID);
  CHECK(classifyTmclSpiReply(ok, 5, true) == TmclSpiRxClass::STALE);
  CHECK(classifyTmclSpiReply(ok, 5, false) == TmclSpiRxClass::VALID);

  const auto busy = withChecksum({0xF0, 0, 0, 0, 0, 0, 0, 0});
  CHECK(classifyTmclSpiReply(busy, 6, true) == TmclSpiRxClass::NOT_READY);
  CHECK(classifyTmclSpiReply(busy, 136, true) == TmclSpiRxClass::NOT_READY);

  const std::array<uint8_t, 8> zeros{};
  CHECK(classifyTmclSpiReply(zeros, 6, true) == TmclSpiRxClass::STALE);

  const auto first = withChecksum({0x0C, 0, 0, 0, 0, 0, 0, 0});
  CHECK(classifyTmclSpiReply(first, 6, true) == TmclSpiRxClass::STALE);

  auto corrupt = ok;
  corrupt[4] ^= 0x10;
  CHECK(classifyTmclSpiReply(corrupt, 6, true) == TmclSpiRxClass::STALE);

  // GetVersion type 0 replies with an ASCII string and no TMCL checksum.
  const std::array<uint8_t, 8> version{0x02, '0', '0', '5', '1', 'V', '1', '0'};
  CHECK(classifyTmclSpiReply(version, 136, true) == TmclSpiRxClass::VALID);
}

TEST_CASE("default timing: no blind gap, bounded doubling back-off") {
  constexpr tmc9660::TmclSpiTiming t{};
  static_assert(t.frameGapUs == 0);
  static_assert(t.pollWaitUs(0) == 20 && t.pollWaitUs(1) == 40 && t.pollWaitUs(3) == 160);
  static_assert(t.pollWaitUs(4) == 200 && t.pollWaitUs(50) == 200);
  CHECK_EQ(t.worstCasePollWaitUs(), 20U + 40U + 80U + 160U + 56U * 200U);

  constexpr auto legacy = tmc9660::TmclSpiTiming::legacy();
  static_assert(legacy.frameGapUs == 1000);
  CHECK_EQ(legacy.worstCasePollWaitUs(), 50U * 200U);
}
