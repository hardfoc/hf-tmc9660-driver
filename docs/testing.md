# Testing and CI

How the driver is checked: host unit tests against a model of the chip, compiled documentation, ESP32 builds and
static analysis, on every pull request.

## Host unit tests

The tests build the real driver against mock transports that model the TMC9660's parameter-mode behaviour: the
SPI reply arrives one datagram late, a busy chip answers NOT_READY and ignores the datagram, and faults can be
injected. Time is virtual, so tests check the exact number of datagrams and the waits between them.

```bash
cmake -S . -B build-host -DHF_TMC9660_BUILD_TESTS=ON
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

| File | Covers |
| --- | --- |
| [`tests/test_frames.cpp`](../tests/test_frames.cpp) | TMCL frame encoding, checksums, reply classification |
| [`tests/test_transport.cpp`](../tests/test_transport.cpp) | NOT_READY polling, back-off, stale replies, pipelined batches |
| [`tests/test_driver.cpp`](../tests/test_driver.cpp) | Driver calls end to end over the mocks |
| [`tests/mock_tmc9660.hpp`](../tests/mock_tmc9660.hpp) | The SPI and UART chip models |

## Documentation snippets

Every C++ block on this site is included from [`tests/docs/`](../tests/docs/my_spi.hpp), which the host build
compiles (target `hf_tmc9660_doc_snippets`) with the driver instantiated over the documented ports. A change to
the API that breaks a page breaks the build first.

## CI workflows

| Workflow | What it checks |
| --- | --- |
| Host unit tests | GCC and Clang build and run the tests and the doc snippets |
| ESP32 examples build | Every app in `examples/esp32/app_config.yml`, for each listed ESP-IDF version and build type |
| C++ lint, C++ analysis | clang-format, clang-tidy and cppcheck |
| Docs | Strict site build, figure and generated-table drift, Doxygen, link check, Lighthouse budget |
| Markdown lint, YAML lint, link check | Repository text files |

The workflows call the shared, versioned workflows of
[hf-general-ci-tools](https://github.com/hardfoc/hf-general-ci-tools) and
[hf-espidf-ci-tools](https://github.com/hardfoc/hf-espidf-ci-tools).

## On hardware

The [ESP32 examples](examples.md) are the hardware test suite: each prints a pass / fail summary over the
serial monitor. Run `tmcl_timing` after any transport change.
