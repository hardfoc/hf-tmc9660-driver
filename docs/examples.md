# Examples

Complete ESP32 applications that exercise the driver on the TMC9660 evaluation kits, one feature area each.

They run on an ESP32-C6 DevKit wired to a TMC9660-3PH-EVKIT (BLDC) or TMC9660-STP-EVKIT (stepper); the pins are
in [`esp32_tmc9660_test_config.hpp`](../examples/esp32/main/esp32_tmc9660_test_config.hpp) and the port in
[`esp32_tmc9660_bus.hpp`](../examples/esp32/main/esp32_tmc9660_bus.hpp). CI builds every app below on each pull
request.

## The apps

<!-- generated: examples -->
| App | What it shows | Source | CI |
| --- | --- | --- | --- |
| `bldc_comprehensive_test` :material-star:{ title="featured" } | Comprehensive BLDC motor testing suite with Hall sensors, ABN encoders, and velocity control | [`bldc_comprehensive_test.cpp`](../examples/esp32/main/bldc_comprehensive_test.cpp) | yes |
| `stepper_comprehensive_test` :material-star:{ title="featured" } | Comprehensive Stepper motor testing suite with FOC and step/dir control | [`stepper_comprehensive_test.cpp`](../examples/esp32/main/stepper_comprehensive_test.cpp) | yes |
| `dc_comprehensive_test` :material-star:{ title="featured" } | Comprehensive DC motor testing suite with current control and velocity control | [`dc_comprehensive_test.cpp`](../examples/esp32/main/dc_comprehensive_test.cpp) | yes |
| `telemetry_comprehensive_test` :material-star:{ title="featured" } | Comprehensive telemetry monitoring test suite for temperature, current, voltage, and status | [`telemetry_comprehensive_test.cpp`](../examples/esp32/main/telemetry_comprehensive_test.cpp) | yes |
| `bootloader_comprehensive_test` :material-star:{ title="featured" } | Comprehensive bootloader testing suite for initialization and configuration validation | [`bootloader_comprehensive_test.cpp`](../examples/esp32/main/bootloader_comprehensive_test.cpp) | yes |
| `tmcl_timing` | TMCL SPI access timing: single vs pipelined, frame-gap sweep, latency histogram | [`tmcl_timing.cpp`](../examples/esp32/main/tmcl_timing.cpp) | yes |

Build and flash one (from `examples/esp32`, ESP-IDF environment loaded):

```bash
./scripts/build_app.sh bldc_comprehensive_test Release
./scripts/flash_app.sh flash_monitor bldc_comprehensive_test Release
```

Source of this table: [`examples/esp32/app_config.yml`](../examples/esp32/app_config.yml).
<!-- /generated -->

## Set up once

```bash
git submodule update --init examples/esp32/scripts   # the HardFOC ESP-IDF build scripts
cd examples/esp32
. $IDF_PATH/export.sh                                 # ESP-IDF v5.5
./scripts/build_app.sh list                           # every app and build type
```

The scripts read [`app_config.yml`](../examples/esp32/app_config.yml) (the same file this page's table comes
from) and build each app as its own firmware image.

## What to read in an example

| You want to | Read |
| --- | --- |
| See a full BLDC bring-up: gate driver, current sensing, Hall, FOC loops | `bldc_comprehensive_test.cpp`, `configure*` functions |
| Run a stepper in FOC with step/dir | `stepper_comprehensive_test.cpp` |
| Read telemetry and decode faults in a loop | `telemetry_comprehensive_test.cpp` |
| Check every bootloader option | `bootloader_comprehensive_test.cpp` |
| Measure TMCL access latency on your board | `tmcl_timing.cpp` ([Timing and throughput](timing.md#measuring-it)) |

The [example folder's README](../examples/esp32/README.md) has the evaluation-kit wiring, the per-test
checklists and the expected serial output.
