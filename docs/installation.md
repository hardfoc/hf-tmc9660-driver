# Installation

Add the driver to a CMake or ESP-IDF project. It is a header-mostly C++20 library with no dependencies.

## Requirements

- A C++20 compiler: GCC 10+, Clang 12+ or a current MSVC.
- No RTOS, heap or exceptions: every call is `noexcept` and returns `bool` or a result enum.

## Get the source

A git submodule keeps the driver pinned to a version you chose:

```bash
git submodule add https://github.com/hardfoc/hf-tmc9660-driver.git external/hf-tmc9660-driver
git submodule update --init external/hf-tmc9660-driver
```

Only `inc/`, `src/`, `cmake/` and `CMakeLists.txt` are needed to build. The examples have their own submodule
(`examples/esp32/scripts`); you do not need it to use the driver.

## CMake

```cmake
add_subdirectory(external/hf-tmc9660-driver)
target_link_libraries(my_app PRIVATE hf_tmc9660)
```

The `hf_tmc9660` target carries the include directories and requires C++20 (`cxx_std_20`), so your target does
not need to set either.

| Option | Default | Effect |
| --- | --- | --- |
| `HF_TMC9660_ENABLE_WARNINGS` | `OFF` | `-Wall -Wextra -Wpedantic` on the driver sources |
| `HF_TMC9660_BUILD_TESTS` | `OFF` | Builds the host unit tests in `tests/` ([Testing and CI](testing.md)) |

## ESP-IDF

The ESP-IDF component wrapper is [`examples/esp32/components/hf_tmc9660/`](../examples/esp32/components/hf_tmc9660/CMakeLists.txt).
Copy that folder into your project's `components/` (or point `EXTRA_COMPONENT_DIRS` at it) and set the
`HF_TMC9660_ROOT` cache variable to the driver checkout; the wrapper stops with a clear message when the path is
wrong. Your component then declares `REQUIRES hf_tmc9660`.

## How the build is organised

| Layer | File | Role |
| --- | --- | --- |
| Settings | [`cmake/hf_tmc9660_build_settings.cmake`](../cmake/hf_tmc9660_build_settings.cmake) | Sources, include directories, version: the single source of truth |
| Library | [`CMakeLists.txt`](../CMakeLists.txt) | Static library for host builds and CI |
| ESP-IDF | [`examples/esp32/components/hf_tmc9660/CMakeLists.txt`](../examples/esp32/components/hf_tmc9660/CMakeLists.txt) | IDF component wrapper over the same settings |

The version is set once in the settings file and written into `tmc9660_version.h` at configure time. Read it at
run time:

```cpp
const char* v = tmc9660::TMC9660<MySpi>::GetDriverVersion();   // "1.0.0"
```

## Compile-time switches

| Define | Effect |
| --- | --- |
| `TMC9660_DISABLE_DEBUG_LOGGING` | Removes every `logDebug()` call and the parameter-name lookup behind it |
| `TMC9660_LOG_TMCL_RAW_FRAMES=1` | Logs every raw SPI TMCL datagram (bring-up only; slow) |

## Next

[Quickstart](quickstart.md) wires the chip and spins a motor; [Porting](porting.md) writes the communication
interface for your MCU.
