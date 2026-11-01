/**
 * @file tmcl_timing.cpp
 * @brief Measures the cost of one TMCL parameter access over SPI: single vs pipelined,
 *        across frame-gap settings, with a latency histogram per configuration.
 *
 * What it does (chip in parameter mode, motor control idle; nothing moves):
 *  1. Writes known patterns into global bank-2 user variables 0..7 so every read can be
 *     checked against an expected value (detects stale or shifted replies).
 *  2. For each timing profile (legacy 1000 us gap, then a gap sweep down to 0 us with
 *     NOT_READY polling) runs single GGP reads and reports min / p50 / p99 / max, a
 *     histogram, NOT_READY / stale / failure counts and value mismatches.
 *  3. Runs pipelined batches of 6 and 8 reads (readGlobalParameters) with the same
 *     profiles and reports per-batch and per-element time.
 *  4. Reads G0:12 MAIN_LOOPS so the result can be related to the firmware main-loop rate.
 *
 * Timing uses esp_timer_get_time() (1 us resolution) around each access. Build with
 * `./scripts/build_app.sh tmcl_timing Release` and flash as for the other examples.
 * Results are printed on the console; copy them into the driver's "Timing and throughput"
 * docs section when they replace the derived numbers there.
 *
 * Board wiring and SPI clock come from Esp32Tmc9660BusConfig (esp32_tmc9660_bus.hpp);
 * change kSpiClockHz below to measure at another clock (the TMC9660 accepts up to 10 MHz).
 */

#include "../../../inc/tmc9660.hpp"
#include "esp32_tmc9660_bus.hpp"

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstdio>
#include <vector>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using tmc9660::TMC9660;
using tmc9660::TmclSpiTiming;
using tmc9660::tmcl::GlobalParamBank0;
using tmc9660::tmcl::GlobalParamBank2;

namespace {

constexpr const char* TAG = "TMCL_TIMING";

/// SPI clock for the measurement. 1 MHz is the example default wiring; Vortex runs 10 MHz.
constexpr uint32_t kSpiClockHz = 1000000;
/// Single accesses per profile.
constexpr size_t kSingleIterations = 2000;
/// Batches per profile and batch size.
constexpr size_t kBatchIterations = 500;
/// Frame gaps swept (us). The first profile is the pre-1.1.0 legacy timing.
constexpr std::array<uint32_t, 8> kGapSweepUs = {1000, 500, 250, 100, 50, 25, 10, 0};

using Driver = TMC9660<Esp32Tmc9660SpiBus>;

/// Histogram bucket upper bounds (us); the last bucket is open-ended.
constexpr std::array<uint32_t, 10> kBucketUs = {25,  50,   100,  200,  400,
                                                800, 1600, 3200, 6400, 12800};

struct Summary {
  uint32_t min = 0;
  uint32_t p50 = 0;
  uint32_t p99 = 0;
  uint32_t max = 0;
  std::array<uint32_t, kBucketUs.size() + 1> buckets{};
};

Summary summarize(std::vector<uint32_t>& samples) {
  Summary s{};
  if (samples.empty()) {
    return s;
  }
  std::sort(samples.begin(), samples.end());
  s.min = samples.front();
  s.max = samples.back();
  s.p50 = samples[samples.size() / 2];
  s.p99 = samples[std::min(samples.size() - 1, (samples.size() * 99) / 100)];
  for (uint32_t v : samples) {
    size_t b = 0;
    while (b < kBucketUs.size() && v >= kBucketUs[b]) {
      ++b;
    }
    ++s.buckets[b];
  }
  return s;
}

void printHistogram(const Summary& s, size_t total) {
  uint32_t lower = 0;
  for (size_t b = 0; b < s.buckets.size(); ++b) {
    char label[24];
    if (b < kBucketUs.size()) {
      std::snprintf(label, sizeof(label), "%5" PRIu32 "-%5" PRIu32 " us", lower, kBucketUs[b]);
      lower = kBucketUs[b];
    } else {
      std::snprintf(label, sizeof(label), "%5" PRIu32 "+      us", lower);
    }
    if (s.buckets[b] == 0) {
      continue;
    }
    const size_t width = (static_cast<size_t>(s.buckets[b]) * 50U + total - 1U) / total;
    char bar[52];
    std::fill(bar, bar + width, '#');
    bar[width] = '\0';
    ESP_LOGI(TAG, "    %s | %6" PRIu32 " %s", label, s.buckets[b], bar);
  }
}

void printStats(const char* what, const Summary& s, const tmc9660::TmclSpiStats& st,
                uint32_t mismatches, size_t total) {
  ESP_LOGI(TAG,
           "  %-26s min %5" PRIu32 "  p50 %5" PRIu32 "  p99 %5" PRIu32 "  max %6" PRIu32
           " us | frames %" PRIu32 " notReady %" PRIu32 " stale %" PRIu32 " recov %" PRIu32
           " fail %" PRIu32 " mismatch %" PRIu32,
           what, s.min, s.p50, s.p99, s.max, st.frames, st.notReady, st.staleReplies, st.recoveries,
           st.failures, mismatches);
  printHistogram(s, total);
}

constexpr uint32_t patternFor(size_t index) {
  return 0x5A000000U | (static_cast<uint32_t>(index) * 0x01010101U & 0x00FFFFFFU);
}

constexpr std::array<GlobalParamBank2, 8> kUserVars = {
    GlobalParamBank2::USER_VARIABLE_0, GlobalParamBank2::USER_VARIABLE_1,
    GlobalParamBank2::USER_VARIABLE_2, GlobalParamBank2::USER_VARIABLE_3,
    GlobalParamBank2::USER_VARIABLE_4, GlobalParamBank2::USER_VARIABLE_5,
    GlobalParamBank2::USER_VARIABLE_6, GlobalParamBank2::USER_VARIABLE_7};

bool writePatterns(Driver& drv) {
  for (size_t i = 0; i < kUserVars.size(); ++i) {
    if (!drv.writeGlobal(kUserVars[i], patternFor(i))) {
      ESP_LOGE(TAG, "SGP user variable %u failed", static_cast<unsigned>(i));
      return false;
    }
  }
  return true;
}

void measureSingle(Driver& drv, Esp32Tmc9660SpiBus& bus, const char* label) {
  std::vector<uint32_t> samples;
  samples.reserve(kSingleIterations);
  uint32_t mismatches = 0;
  bus.resetTmclStats();
  for (size_t i = 0; i < kSingleIterations; ++i) {
    const size_t idx = i % kUserVars.size();
    uint32_t value = 0;
    const int64_t t0 = esp_timer_get_time();
    const bool ok = drv.readGlobal(kUserVars[idx], value);
    const int64_t t1 = esp_timer_get_time();
    samples.push_back(static_cast<uint32_t>(t1 - t0));
    if (!ok || value != patternFor(idx)) {
      ++mismatches;
    }
  }
  const tmc9660::TmclSpiStats st = bus.tmclStats();
  Summary s = summarize(samples);
  printStats(label, s, st, mismatches, samples.size());
}

void measureBatch(Driver& drv, Esp32Tmc9660SpiBus& bus, size_t batch, const char* label) {
  std::vector<uint32_t> samples;
  samples.reserve(kBatchIterations);
  uint32_t mismatches = 0;
  bus.resetTmclStats();
  std::array<Driver::GlobalParameterAccess, kUserVars.size()> items{
      kUserVars[0], kUserVars[1], kUserVars[2], kUserVars[3],
      kUserVars[4], kUserVars[5], kUserVars[6], kUserVars[7]};
  for (size_t i = 0; i < kBatchIterations; ++i) {
    for (auto& it : items) {
      it.value = 0;
    }
    const int64_t t0 = esp_timer_get_time();
    const bool ok = drv.readGlobalParameters(std::span(items.data(), batch));
    const int64_t t1 = esp_timer_get_time();
    samples.push_back(static_cast<uint32_t>(t1 - t0));
    for (size_t k = 0; k < batch; ++k) {
      if (!ok || items[k].value != patternFor(k)) {
        ++mismatches;
      }
    }
  }
  const tmc9660::TmclSpiStats st = bus.tmclStats();
  Summary s = summarize(samples);
  printStats(label, s, st, mismatches, samples.size());
  ESP_LOGI(TAG, "    per element: p50 %" PRIu32 " us (batch of %u)",
           s.p50 / static_cast<uint32_t>(batch), static_cast<unsigned>(batch));
}

void runProfile(Driver& drv, Esp32Tmc9660SpiBus& bus, const TmclSpiTiming& timing,
                const char* name) {
  bus.setTmclTiming(timing);
  ESP_LOGI(TAG, "%s", "");
  ESP_LOGI(TAG, "Profile %s: gap %" PRIu32 " us, poll %" PRIu32 "..%" PRIu32 " us x %u, echo %s",
           name, timing.frameGapUs, timing.pollIntervalUs, timing.pollIntervalMaxUs,
           static_cast<unsigned>(timing.maxPolls), timing.verifyReplyOpcode ? "on" : "off");
  measureSingle(drv, bus, "single GGP");
  measureBatch(drv, bus, 6, "pipelined batch of 6");
  measureBatch(drv, bus, 8, "pipelined batch of 8");
}

void timingTask(void*) {
  Esp32Tmc9660BusConfig config{};
  config.spi.clock_speed_hz = kSpiClockHz;
  auto bus = CreateEsp32Tmc9660SpiBus(config);
  if (!bus) {
    ESP_LOGE(TAG, "SPI bus init failed");
    vTaskDelete(nullptr);
    return;
  }
  Driver drv(*bus, 0);

  tmc9660::BootloaderConfig cfg{};
  cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
  cfg.boot.start_motor_control = true;
  if (drv.bootloaderInit(&cfg, true, true, false) != Driver::BootloaderInitResult::Success) {
    ESP_LOGE(TAG, "bootloaderInit failed");
    vTaskDelete(nullptr);
    return;
  }

  // Start from the bench-proven timing for setup writes.
  bus->setTmclTiming(TmclSpiTiming::legacy());
  if (!writePatterns(drv)) {
    vTaskDelete(nullptr);
    return;
  }
  uint32_t main_loops = 0;
  if (drv.readGlobal(GlobalParamBank0::MAIN_LOOPS, main_loops)) {
    ESP_LOGI(TAG, "G0:12 MAIN_LOOPS = %" PRIu32 " /s (%.1f us per loop)", main_loops,
             main_loops ? 1e6 / static_cast<double>(main_loops) : 0.0);
  }
  ESP_LOGI(TAG, "SPI clock %" PRIu32 " Hz, %u single reads and %u batches per profile", kSpiClockHz,
           static_cast<unsigned>(kSingleIterations), static_cast<unsigned>(kBatchIterations));

  runProfile(drv, *bus, TmclSpiTiming::legacy(), "legacy (pre-1.1.0)");
  for (uint32_t gap : kGapSweepUs) {
    TmclSpiTiming t{};
    t.frameGapUs = gap;
    char name[32];
    std::snprintf(name, sizeof(name), "gap %" PRIu32 " us + poll", gap);
    runProfile(drv, *bus, t, name);
  }

  // Leave the user variables cleared.
  bus->setTmclTiming(TmclSpiTiming{});
  for (auto v : kUserVars) {
    (void)drv.writeGlobal(v, 0U);
  }
  ESP_LOGI(TAG, "%s", "");
  ESP_LOGI(TAG, "Done. A profile is usable when mismatch and fail are 0 over all runs.");
  vTaskDelete(nullptr);
}

} // namespace

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "TMC9660 TMCL timing (driver %s)", tmc9660::GetDriverVersion());
  esp_log_level_set("SPI_TMCL", ESP_LOG_WARN);
  esp_log_level_set("TMC9660", ESP_LOG_WARN);
  xTaskCreate(timingTask, "tmcl_timing", 8192, nullptr, 5, nullptr);
}
