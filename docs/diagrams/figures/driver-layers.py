"""The three layers of the driver: hardware-agnostic core, communication interface, platform port.

Reads: From top to bottom. Your application talks to one TMC9660<Comm> object, whose public members are the chip's
subsystems (listed from inc/tmc9660.hpp). Each subsystem turns a call into TMCL parameter reads and writes. The
communication interface (SpiCommInterface or UartCommInterface in inc/tmc9660_comm_interface.hpp) frames them,
polls NOT_READY, pipelines batches and drives the control pins by name. The platform port is the only code you
write: a class that moves bytes over SPI or UART, sets and reads four GPIOs and waits. Below it is the hardware.
"""

import re
from pathlib import Path

from hardfoc_mkdocs_theme.figkit import Fig

ROOT = Path(__file__).resolve().parents[3]
DRIVER = "inc/tmc9660.hpp"
COMM = "inc/tmc9660_comm_interface.hpp"


def data():
    drv = (ROOT / DRIVER).read_text()
    subsystems = re.findall(r"^  \} (\w+)\{\*this\};", drv, re.M)
    comm = (ROOT / COMM).read_text()
    hooks = []
    for name in ("spiTransferTMCL", "spiTransferBootloader", "gpioSet", "gpioRead", "delayUs", "delayMs"):
        if re.search(rf"\b{name}\(", comm):
            hooks.append(name)
    return {
        "subsystems": subsystems,
        "port_hooks": hooks,
        "sources": [f"{DRIVER}:TMC9660 members", f"{COMM}:SpiCommInterface"],
    }


def build(d):
    f = Fig(
        "driver-layers",
        "Three layers, one of them yours",
        alt="Layer diagram: the application calls the hardware-agnostic TMC9660 driver and its subsystems, which use "
        "the SPI or UART communication interface, which calls the platform port you write, which drives the SPI bus "
        "and four control pins of the TMC9660.",
        width=1200,
        kicker="Architecture",
        subtitle="The driver and the interface are the same on every MCU. The port is a small class with six methods.",
        profile="overview",
        legend=[("brand", "this library"), ("teal", "you write"), ("slate", "hardware")],
    )
    W = f.body_w
    app = f.card(0, 0, W, "Your application", ["control loop, state machine, CLI ..."], tone="slate")
    top = app.b + 40
    chips_h = f.chips(16, f.region_inner_top(top), W - 32, d["subsystems"], tone="brand")
    reg = f.region(0, top, W, f.region_inner_top(top) - top + chips_h + 18, "Hardware-agnostic core", tone="brand",
                   sub="TMC9660<Comm>, inc/tmc9660.hpp")
    comm = f.card(0, reg.b + 40, W, "Communication interface",
                  ["SpiCommInterface / UartCommInterface: TMCL frames, NOT_READY polling, pipelined batches,",
                   "control pins by name (RST, DRV_EN, WAKE, FAULTN) with per-board active levels"],
                  tone="brand", accent=True)
    port = f.card(0, comm.b + 40, W, "Platform port (your class)", ["  ".join(h + "()" for h in d["port_hooks"])],
                  tone="teal", accent=True, mono=True)
    hw = f.card(0, port.b + 40, W, "TMC9660", ["SPI mode 3 up to 10 MHz, or UART  /  RST, DRV_EN, WAKE, FAULTN"],
                tone="slate")
    for a, b in ((app, reg), (reg, comm), (comm, port), (port, hw)):
        f.connect(a, b, "bottom", "top", tone="rail")
    return f
