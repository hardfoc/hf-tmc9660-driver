---
layout: default
title: "⏱️ Timing and Throughput"
description: "Cost of one TMCL access over SPI, how to configure it, and pipelined batches"
nav_order: 9
parent: "📚 Documentation"
permalink: /docs/timing_and_throughput/
---

# Timing and Throughput

This page explains what one TMCL parameter access costs over SPI, which settings change it,
and how pipelined batches cut the cost. It separates **what the vendor specifies**, **what is
derived** from that, and **what still has to be measured** on hardware.

## What the chip specifies

From the TMC9660 Parameter Mode Reference Manual (Rev. 0, "Serial-Peripheral Interface",
Tables 3–5) and the TMC9660 datasheet ("SPI Peripheral — Electrical Characteristics"). Both
PDFs are in [`docs/datasheet/`](datasheet/).

| Fact | Source |
|---|---|
| A 64-bit request's reply is clocked out during the **next** datagram | Reference manual, SPI section |
| `SPI_STATUS_NOT_READY` (0xF0): the chip is busy, the reply in this datagram is not the reply you want, **and the command in this datagram was ignored**. Re-send the datagram until the status goes away | Reference manual, SPI section and Table 5 |
| fSCK ≤ 10 MHz, SPI mode 3 | Datasheet, SPI peripheral |
| `CS_` high time tCSH ≥ 4 × `T_CLK` (about 100 ns) | Datasheet, SPI peripheral |
| Minimum request processing time / turnaround | **Not specified** |

So the protocol has a ready indication (NOT_READY) and no documented minimum delay. Releases
up to 1.0.0 nevertheless waited a fixed **1000 µs** between the request and the reply-request
datagram, and ignored NOT_READY on the request datagram. That second point also explains the
old "`REPLY_INVALID_CMD` on back-to-back transfers" symptom (see
[Communication Protocols](special_feature_protocols.md)): a request that arrived while the
chip was still busy was dropped, and the reply-request then returned the reply to the
previous reply-request (status 2, invalid command).

## What the driver does now

`SpiCommInterface::transferTMCL()` (one access):

1. Send the request. If the answer is NOT_READY, the request was ignored: back off and send it
   again.
2. Wait `TmclSpiTiming::frameGapUs` (default **0 µs**).
3. Send the reply-request `FF 00 00 00 00 00 00 FF`. While the answer is NOT_READY, has status
   0x00, a bad checksum, or (with `verifyReplyOpcode`) does not echo the request's opcode,
   back off and send it again. A well-formed reply that echoes 0xFF means the real reply was
   lost, and the access fails at once instead of returning the wrong frame.

The back-off starts at `pollIntervalUs` (20 µs) and doubles up to `pollIntervalMaxUs`
(200 µs), for at most `maxPolls` (60) re-sends: about **11.5 ms** worst case. That bound is
for thread context. Lower `maxPolls` in a high-priority control task.

`SpiCommInterface::transferTMCLBatch()` and the driver helpers `readParameters()`,
`writeParameters()`, `readGlobalParameters()` and `writeGlobalParameters()` **pipeline**:
datagram *k* carries request *k* and clocks out the reply to request *k* − 1, and a final
reply-request collects the last reply. N accesses cost N + 1 datagrams.

- The reply carried by the first datagram belongs to whatever ran before and is discarded.
- NOT_READY means "this datagram was ignored": it is re-sent, nothing is skipped or doubled.
- A stale reply (0x00, bad checksum, wrong echo) leaves it unknown whether the request in that
  datagram ran. The previous element is re-issued as a single access and the pipeline restarts
  at the current element, which may then run twice. Batches therefore accept only idempotent
  requests (SAP, GAP, SGP, GGP, RAMDebug reads). `TmclSpiStats::recoveries` counts this.

## Configuring it

```cpp
tmc9660::TmclSpiTiming t{};      // datasheet defaults: no gap, 20→200 µs back-off, 60 polls
t.frameGapUs = 30;               // only if a measurement shows the chip needs it
t.maxPolls = 4;                  // e.g. in a 1 kHz control task: fail fast, mark the sample stale
comm.setTmclTiming(t);

comm.setTmclTiming(tmc9660::TmclSpiTiming::legacy()); // behaviour of releases up to 1.0.0

const auto& s = comm.tmclStats(); // frames, notReady, staleReplies, recoveries, failures
```

The older `setSpiRetryMaxCount()` / `setSpiRetryInterval()` still work. They set `maxPolls`, and
a fixed interval (no back-off).

Batched reads:

```cpp
using Drv = tmc9660::TMC9660<MySpi>;
Drv::ParameterAccess rd[] = {{tmcl::Parameters::ACTUAL_VELOCITY},
                             {tmcl::Parameters::ACTUAL_POSITION},
                             {tmcl::Parameters::GENERAL_STATUS_FLAGS}};
if (driver.readParameters(rd)) { /* rd[i].value, rd[i].status */ }

Drv::GlobalParameterAccess g[] = {{tmcl::GlobalParamBank0::MAIN_LOOPS}}; // bank 0 from the type
(void)driver.readGlobalParameters(g);
```

Defining `TMC9660_DISABLE_DEBUG_LOGGING` also removes the per-access parameter-name lookup and
string formatting in `sendCommand()`. Batches never run that code.

## Cost per access: before and after

Notation: T_f = one 64-bit datagram including the host driver's per-transaction overhead,
T_p = the chip's processing time for a request (not specified, to be measured).

| | Formula | 10 MHz, T_f ≈ 20 µs | 1 MHz, T_f ≈ 80 µs |
|---|---|---:|---:|
| Before (1.0.0 + hf-core handler: 1000 µs gap, 200 µs post-delay) | 2·T_f + 1000 + 200 | ≈ 1.24 ms | ≈ 1.36 ms |
| Before, 6 accesses | 6 × above | ≈ 7.4 ms | ≈ 8.2 ms |
| After, single, chip keeps up (T_p ≤ T_f) | 2·T_f | ≈ 40 µs | ≈ 160 µs |
| After, pipelined batch of 6 (T_p ≤ T_f) | 7·T_f | ≈ 140 µs | ≈ 560 µs |
| After, pipelined batch of 8 (T_p ≤ T_f) | 9·T_f | ≈ 180 µs | ≈ 720 µs |
| After, single, T_p = 50 µs (one or two polls) | ≈ 3·T_f + 20 | ≈ 80 µs | (T_p < T_f) |
| After, batch of 6, T_p = 50 µs, no gap | ≈ 6·(2·T_f + 20) + T_f | ≈ 380 µs | (T_p < T_f) |
| After, batch of 6, T_p = 50 µs, `frameGapUs = 30` | ≈ 6·(T_f + 30) + T_f | ≈ 320 µs | (T_p < T_f) |

**Every number in this table is derived, not measured.** T_f ≈ 20 µs at 10 MHz assumes 6.4 µs
of clocking plus about 10–15 µs of ESP-IDF transaction overhead. The "before" row matches the
Vortex observability design's estimate (scenario A, 1.24 ms). The host unit tests check the
datagram counts and waits of each formula on a mock chip with a virtual clock, which is not a
timing measurement.

## Measuring it

The ESP32 example [`tmcl_timing`](../examples/esp32/main/tmcl_timing.cpp) measures it on
hardware:

```bash
cd examples/esp32
./scripts/build_app.sh tmcl_timing Release
./scripts/flash_app.sh flash_monitor tmcl_timing Release
```

For the legacy profile and for frame gaps of 1000, 500, 250, 100, 50, 25, 10 and 0 µs (all with
NOT_READY polling), it prints single-read and pipelined batch-of-6 and batch-of-8 latency
(min / p50 / p99 / max and a histogram), the transport counters, value mismatches against known
patterns in bank-2 user variables 0–7, and G0:12 `MAIN_LOOPS`. A profile is usable when
`mismatch` and `fail` are both 0. Repeat with the motor in open-loop and FOC under load before
picking a default for a control loop. Firmware-serviced SPI replies have been seen to stall for
milliseconds under FOC load on the bench.
