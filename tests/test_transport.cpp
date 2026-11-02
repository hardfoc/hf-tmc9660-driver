/**
 * @file test_transport.cpp
 * @brief SPI TMCL handshake: gap/poll configuration, NOT_READY handling and pipelined batches.
 */
#include "test_harness.hpp"

#include "mock_tmc9660.hpp"

#include <algorithm>

using tmc9660::TMCLFrame;
using tmc9660::TMCLReply;
using tmc9660::TmclSpiTiming;
using tmc9660_test::MockTmc9660Spi;

namespace {
TMCLFrame gap(uint16_t param, uint8_t motor = 0) {
  TMCLFrame f{};
  f.opcode = 6;
  f.type = param;
  f.motor = motor;
  return f;
}
TMCLFrame sap(uint16_t param, uint32_t value, uint8_t motor = 0) {
  TMCLFrame f{};
  f.opcode = 5;
  f.type = param;
  f.motor = motor;
  f.value = value;
  return f;
}
size_t count(const std::vector<uint32_t>& v, uint32_t x) {
  return static_cast<size_t>(std::count(v.begin(), v.end(), x));
}
} // namespace

TEST_CASE("single access: two datagrams, reply-request is FF 00..00 FF, no delay when idle") {
  MockTmc9660Spi bus;
  bus.axis[MockTmc9660Spi::key(0, 125)] = 0x12345678;
  TMCLReply r{};
  REQUIRE(bus.transferTMCL(gap(125), r, 0, nullptr, nullptr));
  CHECK(r.isOK());
  CHECK_EQ(r.value, 0x12345678U);
  CHECK_EQ(bus.datagrams, 2U);
  CHECK(bus.delays.empty());
  REQUIRE(bus.seen.size() == 2);
  CHECK_EQ(bus.seen[1].opcode, 0xFF);
  CHECK_EQ(bus.seen[1].value, 0U);
}

TEST_CASE("single access polls NOT_READY instead of waiting blind") {
  MockTmc9660Spi bus;
  bus.processUs = 50; // chip needs 50 µs; frames cost 8 µs
  bus.axis[MockTmc9660Spi::key(0, 289)] = 0xCAFE;
  TMCLReply r{};
  REQUIRE(bus.transferTMCL(gap(289), r, 0, nullptr, nullptr));
  CHECK_EQ(r.value, 0xCAFEU);
  // Polls: 20, 40 (8 µs frame + 20 + 8 + 40 > 50).
  CHECK_EQ(bus.tmclStats().notReady, 2U);
  CHECK(bus.delays == (std::vector<uint32_t>{20, 40}));
  CHECK_EQ(bus.executionsOf(6, 289), 1);
}

TEST_CASE("configured frame gap is honoured and removes the polls") {
  MockTmc9660Spi bus;
  bus.processUs = 50;
  TmclSpiTiming t{};
  t.frameGapUs = 60;
  bus.setTmclTiming(t);
  TMCLReply r{};
  REQUIRE(bus.transferTMCL(gap(3), r, 0, nullptr, nullptr));
  CHECK(bus.delays == (std::vector<uint32_t>{60}));
  CHECK_EQ(bus.tmclStats().notReady, 0U);
  CHECK_EQ(bus.datagrams, 2U);
}

TEST_CASE("legacy timing reproduces the old fixed 1000 us gap") {
  MockTmc9660Spi bus;
  bus.setTmclTiming(TmclSpiTiming::legacy());
  TMCLReply r{};
  REQUIRE(bus.transferTMCL(gap(3), r, 0, nullptr, nullptr));
  CHECK(bus.delays == (std::vector<uint32_t>{1000}));
}

TEST_CASE("legacy retry setters map onto the timing struct") {
  MockTmc9660Spi bus;
  bus.setSpiRetryMaxCount(7);
  bus.setSpiRetryInterval(150);
  CHECK_EQ(bus.tmclTiming().maxPolls, 7);
  CHECK_EQ(bus.tmclTiming().pollIntervalUs, 150U);
  CHECK_EQ(bus.tmclTiming().pollIntervalMaxUs, 150U);
  CHECK_EQ(bus.getSpiRetryMaxCount(), 7);
  CHECK_EQ(bus.getSpiRetryInterval(), 150U);
}

TEST_CASE("request answered NOT_READY is re-sent (it was ignored), never duplicated") {
  MockTmc9660Spi bus;
  bus.processUs = 30;
  TMCLReply r{};
  REQUIRE(bus.transferTMCL(sap(3, 25000), r, 0, nullptr, nullptr));
  // The next request arrives while the chip is still busy with the reply-request.
  bus.processUs = 0;
  REQUIRE(bus.transferTMCL(sap(4, 8), r, 0, nullptr, nullptr));
  CHECK(r.isOK());
  CHECK_EQ(bus.executionsOf(5, 3), 1);
  CHECK_EQ(bus.executionsOf(5, 4), 1);
  CHECK_EQ(bus.axis[MockTmc9660Spi::key(0, 4)], 8U);
}

TEST_CASE("poll budget is bounded and reported") {
  MockTmc9660Spi bus;
  bus.processUs = 1000000; // never ready in practice
  TmclSpiTiming t{};
  t.maxPolls = 5;
  bus.setTmclTiming(t);
  TMCLReply r{};
  CHECK(!bus.transferTMCL(gap(125), r, 0, nullptr, nullptr));
  CHECK_EQ(bus.tmclStats().failures, 1U);
  CHECK(bus.datagrams <= 2U + 5U);
  CHECK(r.spi_status == tmc9660::SPIStatus::NOT_READY);
}

TEST_CASE("stale reply to a single access is re-polled, never accepted") {
  MockTmc9660Spi bus;
  bus.axis[MockTmc9660Spi::key(0, 125)] = 77;
  TMCLReply r{};
  // Zeros on the request datagram only: that reply is discarded anyway.
  bus.zeroReplies = 1;
  REQUIRE(bus.transferTMCL(gap(125), r, 0, nullptr, nullptr));
  CHECK_EQ(r.value, 77U);

  // Zeros on the reply-request too: the mock loses the reply, the chip then answers the
  // reply-request itself (echo 0xFF). The driver must not take that as the GAP reply and
  // fails fast instead of polling out the budget.
  bus.clearObservations();
  bus.zeroReplies = 2;
  CHECK(!bus.transferTMCL(gap(125), r, 0, nullptr, nullptr));
  CHECK_EQ(bus.tmclStats().staleReplies, 2U);
  CHECK_EQ(bus.datagrams, 3U);

  // Same fault with the opcode check off (legacy): the wrong reply is accepted as parsed.
  bus.clearObservations();
  TmclSpiTiming t = TmclSpiTiming::legacy();
  t.frameGapUs = 0;
  bus.setTmclTiming(t);
  bus.zeroReplies = 2;
  CHECK(bus.transferTMCL(gap(125), r, 0, nullptr, nullptr));
  CHECK(!r.isOK()); // status 2 / opcode 0xFF from the reply-request, not the GAP
  CHECK_EQ(r.opcode, 0xFF);
}

TEST_CASE("pipelined batch: N reads in N+1 datagrams, replies in request order") {
  MockTmc9660Spi bus;
  const uint16_t ids[] = {125, 105, 289, 331, 299, 300};
  for (size_t i = 0; i < 6; ++i) {
    bus.axis[MockTmc9660Spi::key(0, ids[i])] = 1000U + static_cast<uint32_t>(i);
  }
  // Leave an unrelated reply pending so the first element must ignore it.
  TMCLReply prior{};
  REQUIRE(bus.transferTMCL(sap(3, 1), prior, 0, nullptr, nullptr));
  bus.clearObservations();

  std::array<TMCLFrame, 6> rq{};
  std::array<TMCLReply, 6> rp{};
  for (size_t i = 0; i < 6; ++i) {
    rq[i] = gap(ids[i]);
  }
  size_t done = 0;
  REQUIRE(bus.transferTMCLBatch(rq, rp, 0, &done));
  CHECK_EQ(done, 6U);
  CHECK_EQ(bus.datagrams, 7U);
  for (size_t i = 0; i < 6; ++i) {
    CHECK(rp[i].isOK());
    CHECK_EQ(rp[i].opcode, 6);
    CHECK_EQ(rp[i].value, 1000U + i);
  }
  CHECK_EQ(bus.seen.back().opcode, 0xFF); // last element collected by the reply-request
  CHECK_EQ(bus.tmclStats().accesses, 6U);
}

TEST_CASE("pipelined batch with a processing delay: NOT_READY re-sends, no duplicates") {
  MockTmc9660Spi bus;
  bus.processUs = 25;
  std::array<TMCLFrame, 4> rq{sap(3, 20000), sap(4, 8), sap(5, 3), sap(6, 4)};
  std::array<TMCLReply, 4> rp{};
  REQUIRE(bus.transferTMCLBatch(rq, rp, 0, nullptr));
  for (const auto& f : rq) {
    CHECK_EQ(bus.executionsOf(5, f.type), 1);
    CHECK_EQ(bus.axis[MockTmc9660Spi::key(0, f.type)], f.value);
  }
  for (const auto& r : rp) {
    CHECK(r.isOK());
  }
  CHECK(bus.tmclStats().notReady > 0U);
  CHECK_EQ(bus.tmclStats().recoveries, 0U);
}

TEST_CASE("pipelined batch: frame gap applies before every reply-carrying datagram") {
  MockTmc9660Spi bus;
  TmclSpiTiming t{};
  t.frameGapUs = 30;
  bus.setTmclTiming(t);
  std::array<TMCLFrame, 3> rq{gap(1), gap(2), gap(3)};
  std::array<TMCLReply, 3> rp{};
  REQUIRE(bus.transferTMCLBatch(rq, rp, 0, nullptr));
  CHECK_EQ(count(bus.delays, 30), 3U);
  CHECK_EQ(bus.delays.size(), 3U);
}

TEST_CASE("pipelined batch recovers from a stale reply by re-issuing that element") {
  MockTmc9660Spi bus;
  for (uint16_t id = 10; id < 15; ++id) {
    bus.axis[MockTmc9660Spi::key(0, id)] = id * 3U;
  }
  std::array<TMCLFrame, 5> rq{gap(10), gap(11), gap(12), gap(13), gap(14)};
  std::array<TMCLReply, 5> rp{};
  size_t done = 0;

  // Zeros on datagram 0 only: its reply belongs to whatever preceded the batch.
  bus.zeroReplies = 1;
  REQUIRE(bus.transferTMCLBatch(rq, rp, 0, &done));
  CHECK_EQ(done, 5U);
  for (size_t i = 0; i < 5; ++i) {
    CHECK_EQ(rp[i].value, (10U + i) * 3U);
  }
  CHECK_EQ(bus.tmclStats().recoveries, 0U);

  // Zeros on datagrams 0 and 1: datagram 1 carried the reply to element 0, so element 0
  // is re-issued as a single access and the pipeline restarts at element 1.
  bus.clearObservations();
  bus.zeroReplies = 2;
  REQUIRE(bus.transferTMCLBatch(rq, rp, 0, &done));
  CHECK_EQ(done, 5U);
  for (size_t i = 0; i < 5; ++i) {
    CHECK(rp[i].isOK());
    CHECK_EQ(rp[i].value, (10U + i) * 3U);
  }
  CHECK_EQ(bus.tmclStats().recoveries, 1U);
  CHECK_EQ(bus.executionsOf(6, 10), 2); // datagram 0 + recovery
  CHECK_EQ(bus.executionsOf(6, 11), 2); // datagram 1 (outcome unknown) + re-send
  CHECK_EQ(bus.executionsOf(6, 12), 1);
}

TEST_CASE("pipelined batch: first and last element with a single-element batch") {
  MockTmc9660Spi bus;
  bus.axis[MockTmc9660Spi::key(1, 144)] = 0xFFFFFFFE;
  std::array<TMCLFrame, 1> rq{gap(144, 1)};
  std::array<TMCLReply, 1> rp{};
  REQUIRE(bus.transferTMCLBatch(rq, rp, 0, nullptr));
  CHECK_EQ(rp[0].value, 0xFFFFFFFEU);
  CHECK_EQ(bus.datagrams, 2U);
}

TEST_CASE("pipelined batch stops on a bus error and reports progress") {
  MockTmc9660Spi bus;
  std::array<TMCLFrame, 4> rq{gap(1), gap(2), gap(3), gap(4)};
  std::array<TMCLReply, 4> rp{};
  bus.failTransfersAfter = 3; // datagrams 0..2 succeed
  size_t done = 99;
  CHECK(!bus.transferTMCLBatch(rq, rp, 0, &done));
  CHECK_EQ(done, 2U);
  CHECK_EQ(bus.tmclStats().failures, 1U);
}

TEST_CASE("empty batch and short reply span") {
  MockTmc9660Spi bus;
  size_t done = 5;
  CHECK(bus.transferTMCLBatch({}, {}, 0, &done));
  CHECK_EQ(done, 0U);
  std::array<TMCLFrame, 2> rq{gap(1), gap(2)};
  std::array<TMCLReply, 1> rp{};
  CHECK(!bus.transferTMCLBatch(rq, rp, 0, nullptr));
  CHECK_EQ(bus.datagrams, 0U);
}
