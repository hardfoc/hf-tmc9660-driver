# Quickstart

Spin a BLDC motor from your MCU: wire six signals, write a small port class, call five driver functions.

!!! warning "Before you power the motor"
    Use a current-limited supply for the first run and keep the motor free to turn. The example commands
    1000 RPM as soon as the chip is up.

## 1. Wire it

| MCU | TMC9660 | Notes |
| --- | --- | --- |
| SPI SCLK, MOSI, MISO, CS | SPI0_SCK, SPI0_MOSI, SPI0_MISO, SPI0_CS0 | SPI mode 3, up to 10 MHz; start at 1 MHz |
| GPIO out | RST | Reset; the driver pulses it in `bootloaderInit()` |
| GPIO out | DRV_EN | Enables the gate driver outputs |
| GPIO out | WAKE | Wakes the chip from hibernation |
| GPIO in | FAULTN | Open drain: pull it up |
| GND | GND | Common ground between the MCU and the motor supply |

The [Hardware](hardware.md) page has the full pin list, the evaluation-kit headers and the electrical limits.

## 2. Add the driver

Add the repository as a submodule and link the `hf_tmc9660` CMake target (or include `inc/` and build with
C++20). [Installation](installation.md) covers CMake and ESP-IDF.

```bash
git submodule add https://github.com/hardfoc/hf-tmc9660-driver.git external/hf-tmc9660-driver
```

## 3. Write the port

The port is the only platform code. It moves bytes over SPI, sets and reads the four control pins and waits.
Replace the four `platform_*` calls with your SDK's.

```cpp title="my_spi.hpp"
--8<-- "tests/docs/my_spi.hpp:port"
```

The constructor arguments are the physical level of RST, DRV_EN, WAKE and FAULTN when each is active. Get them
from your schematic: inverters and level shifters change them. [Porting](porting.md) explains each method and
shows the UART variant.

## 4. Spin the motor

```cpp title="main.cpp"
--8<-- "tests/docs/quickstart.cpp:quickstart"
```

What each step does:

1. `bootloaderInit()` resets the chip, sends the bootloader configuration and starts the motor-control firmware
   in parameter mode. Nothing else works until it returns `Success`.
2. `setType()` and `setPWMFrequency()` tell the chip what it drives. Use the pole-pair count from the motor's
   datasheet.
3. `getMotorContext()` reads the settings the unit conversions need (pole pairs, sensor, scaling) from the chip.
4. `setTargetVelocityRpm()` converts RPM to the chip's internal unit and writes the target velocity.

This example is built by the host CI, so it compiles against the current headers.

## If it does not work

| Symptom | First thing to check |
| --- | --- |
| `bootloaderInit()` returns `Failure` | SPI mode 3, CS held low for the whole 8-byte frame, RST active level |
| Every reply is `0x00` or `0xFF` | MISO wiring, a common ground, the chip's supply |
| Motor hums but does not turn | Pole pairs, phase wiring, commutation mode and feedback ([API guide](api-guide.md#configure-the-motor)) |
| FAULTN stays active | Supply under-voltage, gate-driver fault ([Troubleshooting](troubleshooting.md)) |

## Next

- [API guide](api-guide.md): configure feedback and control loops, read telemetry, handle errors.
- [Examples](examples.md): complete ESP32 test applications for BLDC, stepper, DC and telemetry.
- [Bootloader](bootloader.md): every boot option, including encoders and step/dir.
