# Changelog

All notable changes to this driver are listed here. Versions follow
[Semantic Versioning](https://semver.org/). Entries marked **BREAKING** need source changes in
code that used the affected API; together they call for a major version bump at release.

## Unreleased

### Breaking changes

- **BREAKING:** `tmcl::Parameters` no longer contains `HEARTBEAT_MONITORING_CONFIG`,
  `HEARTBEAT_MONITORING_TIMEOUT`, `WAKE_PIN_CONTROL_ENABLE` or `GO_TO_TIMEOUT_POWER_DOWN_STATE`.
  They are global bank-0 parameters and aliased axis ids 3/4/10/11 (3 and 4 are
  `MOTOR_PWM_FREQUENCY` and `COMMUTATION_MODE`), so `writeParameter(HEARTBEAT_MONITORING_TIMEOUT,
  …)` wrote the commutation mode. Use `tmcl::GlobalParamBank0::…` with `writeGlobal()`,
  `globals.writeBank0()` or the `heartbeat` / `power` subsystems. A `static_assert` now rejects
  duplicate axis ids.
- **BREAKING:** `readGlobal(tmcl::Parameters, bank, …)` and `writeGlobal(tmcl::Parameters, bank, …)`
  are deleted; they sent axis ids with SGP/GGP. Use the new `readGlobal(GlobalParamBankN id, …)` /
  `writeGlobal(GlobalParamBankN id, …)`, which deduce the bank from the enum type, or the raw
  `uint16_t` overloads.
- **BREAKING (behaviour):** `writeGlobalParameter()` / `readGlobalParameter()` return false without
  touching the chip when a bank enum id is passed with a different bank number.
- **BREAKING (behaviour):** the SPI handshake no longer waits a fixed 1000 µs between the request
  and the reply-request; it polls NOT_READY (see "Changed"). Restore the old timing with
  `comm.setTmclTiming(tmc9660::TmclSpiTiming::legacy())`.
- **BREAKING (behaviour):** `heartbeat.configureAuto({.enable = true})` arms the watchdog on the
  driver's own transport (SPI for an SPI comm type) instead of always on UART. Set
  `HeartbeatConfig::interface` to choose explicitly.

### Fixed

- SPI requests answered `NOT_READY` were silently dropped (the reference manual says the command in
  such a datagram is ignored); they are now re-sent. This was the cause of the
  "`REPLY_INVALID_CMD` on back-to-back transfers" symptom.
- SPI replies are checked against the request's opcode; a reply that belongs to another datagram
  is no longer returned as the answer.
- `RamDebug::startCapture()` sent type 6 (`SET_TRIGGER_MASK_SHIFT`); it now sends type 7
  (`ENABLE_TRIGGER`). `readData()` used opcode `ReadMem`; it now uses `RamDebug` type 9
  (`READ_SAMPLE`). `getStatus()` reported `COMPLETE` as running.
- `Heartbeat` docs named bank-0 ids 10/11; they are 3/4. `configureHeartbeat()` writes the timeout
  before the interface.

### Added

- `TmclSpiTiming` (frame gap, poll back-off, poll budget, opcode check) with
  `setTmclTiming()` / `tmclTiming()`, and `TmclSpiStats` counters.
- Pipelined batches: `SpiCommInterface::transferTMCLBatch()` (N accesses in N + 1 datagrams) and
  `TMC9660::readParameters()`, `writeParameters()`, `readGlobalParameters()`,
  `writeGlobalParameters()`, `sendCommandBatch()`, `RamDebug::readSamples()`.
- Full RAMDebug API: one call per Table 16 type code, `Channel` / `Trigger` / `CaptureConfig`,
  `configure()` / `start()`, `getState()`, `getInfo()`.
- `Heartbeat::HeartbeatConfig::interface` and `Heartbeat::transportInterface()`.
- Host unit tests (`-DHF_TMC9660_BUILD_TESTS=ON`) with mock SPI/UART transports, and a CI workflow.
- ESP32 example `tmcl_timing` (single vs pipelined access latency, gap sweep, histogram).
- [Timing and Throughput](docs/timing_and_throughput.md) documentation.

### Changed

- `setSpiRetryMaxCount()` / `setSpiRetryInterval()` now set `TmclSpiTiming::maxPolls` and a fixed
  poll interval. The default budget is 60 re-sends with a 20 → 200 µs back-off (about 11.5 ms).
- With `TMC9660_DISABLE_DEBUG_LOGGING`, `sendCommand()` skips its per-access name lookup and
  formatting.
