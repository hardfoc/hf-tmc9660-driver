---
hide:
  - navigation
  - toc
---

<div class="hf-hero" markdown>
<span class="hf-kicker">HardFOC driver</span>

# TMC9660 Driver

<p>A C++20 driver for the Analog Devices (Trinamic) TMC9660 smart motor controller. It boots the chip into parameter
mode, configures BLDC, stepper and DC motors, runs field-oriented control and reads telemetry, on any MCU with SPI
or UART.</p>

[Quickstart](quickstart.md){ .md-button .md-button--primary }
[API guide](api-guide.md){ .md-button }
[Source on GitHub](https://github.com/hardfoc/hf-tmc9660-driver){ .md-button }
</div>

The driver has no platform code. You give it a small class that moves bytes over SPI or UART and toggles four
pins; it does the rest: the bootloader handshake, TMCL framing, retries, unit conversion and fault decoding.

<div class="hf-chip" markdown>
<div class="hf-chip__head">
<span class="hf-chip__kicker">Smart gate driver with on-chip FOC</span>
<span class="hf-chip__name">TMC9660</span>
<span class="hf-chip__vendor">Analog Devices (Trinamic)</span>
</div>

Motors
:   3-phase BLDC / PMSM, 2-phase stepper, brushed DC

Control
:   Field-oriented control on chip: torque / flux, velocity and position loops, ramps

Feedback
:   Digital Hall, ABN incremental encoder (two), SPI absolute encoder

Host interface
:   SPI mode 3 up to 10 MHz (64-bit TMCL datagrams) or UART (72-bit frames)

Control pins
:   RST, DRV_EN, WAKE (inputs), FAULTN (open-drain output)

Driver modes
:   Parameter mode (TMCL), which this driver uses; register-mode headers are included

[Datasheet](datasheet/tmc9660.pdf) ·
[Parameter-mode reference](datasheet/tmc9660-parameter-mode-reference-manual.pdf) ·
[Register-mode reference](datasheet/tmc9660-register-mode-reference-manual.pdf) ·
[Configuration and bootstrapping](datasheet/tmc9660-configuration-and-bootstrapping.pdf)
</div>

## What you get

- **One object per chip.** `TMC9660<YourPort>` exposes the chip as named subsystems: `motorConfig`,
  `feedbackSense`, `velocityControl`, `positionControl`, `torqueFluxControl`, `telemetry`, `protection`,
  `diagnostics` and more.
- **Boot handled.** `bootloaderInit()` resets the chip, writes the bootloader configuration and starts motor
  control in parameter mode. See [Bootloader](bootloader.md).
- **Engineering units.** RPM, degrees, rad/s and amperes in and out, converted with the motor's real pole pairs
  and sensor settings. See [Units and diagnostics](units-and-diagnostics.md).
- **A fast, correct SPI transport.** NOT_READY polling instead of fixed delays, and pipelined batches: N
  parameter reads cost N + 1 datagrams. See [Timing and throughput](timing.md).
- **Zero-overhead portability.** The port is a CRTP class: no virtual calls, no heap, no RTOS dependency.
  See [Porting](porting.md).
- **Tested without hardware.** Host unit tests run the driver against a model of the chip's SPI and UART
  behaviour. See [Testing and CI](testing.md).

## Where to go next

<div class="grid cards" markdown>

-   :material-rocket-launch-outline: **[Quickstart](quickstart.md)**

    ---

    Wire the chip, write the port, spin a motor in about 15 lines.

-   :material-chip: **[Hardware](hardware.md)**

    ---

    Pins, bus settings, control-pin levels and the evaluation kits.

-   :material-layers-outline: **[Architecture](architecture.md)**

    ---

    The three layers and what each one owns.

-   :material-code-braces: **[API guide](api-guide.md)**

    ---

    Configure, command, read and handle errors, task by task.

-   :material-swap-horizontal: **[Porting](porting.md)**

    ---

    Implement the communication interface for your MCU.

-   :material-book-open-variant: **[API reference](api-reference.md)**

    ---

    Every class and member, generated from the headers.

</div>
