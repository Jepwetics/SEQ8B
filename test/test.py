# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

# SPDX-License-Identifier: Apache-2.0
"""
Tests for the SEQ8 sequencer.

Run them with:   cd test && make
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

# ui_in bit positions
SCK, MOSI, CSN, RUN = 0, 1, 2, 3
IN0, IN1 = 4, 5
PS0, PS1 = 6, 7          # tick speed select: 00=1, 01=16, 10=256, 11=4096 cycles

# Opcodes (2 bits)
OUT, WAIT, JMP, NOP = range(4)

# Jump conditions (imm[7:6])
ALWAYS, IF_IN0, IF_IN1, STOP = range(4)


def start_clock(dut):
    """Start a 100 MHz clock. Works with both older and newer cocotb."""
    try:
        clock = Clock(dut.clk, 10, unit="ns")
    except TypeError:
        clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())


def enc(op, imm=0):
    """Encode one 10-bit instruction."""
    return ((op & 0x3) << 8) | (imm & 0xFF)


def jmp(addr, cond=ALWAYS):
    return enc(JMP, (cond << 6) | (addr & 0x7))


HALT = enc(JMP, STOP << 6)
WORD_BITS = 10


class Pins:
    """Keeps track of the ui_in value."""

    def __init__(self, dut):
        self.dut = dut
        self.value = 1 << CSN  # CS_N idles high, everything else low
        dut.ui_in.value = self.value

    def set(self, bit, level):
        if level:
            self.value |= 1 << bit
        else:
            self.value &= ~(1 << bit)
        self.dut.ui_in.value = self.value

    def set_speed(self, sel):
        self.set(PS0, sel & 1)
        self.set(PS1, (sel >> 1) & 1)


async def reset(dut):
    dut.ena.value = 1
    dut.uio_in.value = 0
    pins = Pins(dut)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    return pins


async def load_program(dut, pins, words):
    """Shift a list of 10-bit instructions in over the SPI-like port."""
    pins.set(RUN, 0)  # core stays in reset while loading
    await ClockCycles(dut.clk, 3)

    pins.set(CSN, 0)
    await ClockCycles(dut.clk, 4)

    for word in words:
        for i in range(WORD_BITS - 1, -1, -1):
            pins.set(MOSI, (word >> i) & 1)
            pins.set(SCK, 0)
            await ClockCycles(dut.clk, 4)
            pins.set(SCK, 1)
            await ClockCycles(dut.clk, 4)

    pins.set(SCK, 0)
    await ClockCycles(dut.clk, 4)
    pins.set(CSN, 1)
    await ClockCycles(dut.clk, 4)


async def start(dut, pins):
    pins.set(RUN, 1)
    await ClockCycles(dut.clk, 4)  # let the synchronisers catch up


async def gap_cycles(dut, sel, n):
    """Cycles between OUT 1 and OUT 2 for a WAIT n at a given tick speed."""
    pins = await reset(dut)
    pins.set_speed(sel)
    await load_program(dut, pins, [
        enc(OUT, 0x01),
        enc(WAIT, n),
        enc(OUT, 0x02),
        HALT,
    ])
    await start(dut, pins)

    while dut.uo_out.value != 0x01:
        await RisingEdge(dut.clk)
    cycles = 0
    while dut.uo_out.value != 0x02:
        await RisingEdge(dut.clk)
        cycles += 1
    return cycles


@cocotb.test()
async def test_reset_state(dut):
    """After reset the outputs are zero and the bidirectional pins are not driven."""
    start_clock(dut)
    await reset(dut)

    assert dut.uo_out.value == 0
    assert dut.uio_oe.value == 0


@cocotb.test()
async def test_out_then_halt(dut):
    """OUT drives the output pins, then HALT freezes them."""
    start_clock(dut)
    pins = await reset(dut)

    await load_program(dut, pins, [
        enc(OUT, 0xA5),
        enc(OUT, 0x3C),
        HALT,
    ])
    await start(dut, pins)
    await ClockCycles(dut.clk, 10)
    assert dut.uo_out.value == 0x3C, f"uo_out = {dut.uo_out.value}"

    await ClockCycles(dut.clk, 200)
    assert dut.uo_out.value == 0x3C, "output changed after HALT"


@cocotb.test()
async def test_run_low_resets_core(dut):
    """Dropping RUN clears the outputs and rewinds to address 0."""
    start_clock(dut)
    pins = await reset(dut)

    await load_program(dut, pins, [enc(OUT, 0xFF), HALT])
    await start(dut, pins)
    await ClockCycles(dut.clk, 10)
    assert dut.uo_out.value == 0xFF

    pins.set(RUN, 0)
    await ClockCycles(dut.clk, 6)
    assert dut.uo_out.value == 0x00

    pins.set(RUN, 1)
    await ClockCycles(dut.clk, 10)
    assert dut.uo_out.value == 0xFF


@cocotb.test()
async def test_wait_timing(dut):
    """WAIT n takes n + 3 clock cycles at the fastest tick speed."""
    start_clock(dut)
    for n in (0, 3, 9):
        cycles = await gap_cycles(dut, 0, n)
        assert cycles == n + 3, f"WAIT {n} took {cycles} cycles, expected {n + 3}"


@cocotb.test()
async def test_tick_speed_select(dut):
    """ui_in[7:6] = 01 makes each tick 16 clock cycles."""
    start_clock(dut)
    n = 3
    fast = await gap_cycles(dut, 0, n)
    slow = await gap_cycles(dut, 1, n)
    dut._log.info(f"fast = {fast}, slow = {slow}")
    assert 16 * n <= slow <= 16 * (n + 2), f"slow = {slow}"
    assert slow > fast * 5


@cocotb.test()
async def test_jump_loops(dut):
    """JMP makes a program repeat."""
    start_clock(dut)
    pins = await reset(dut)

    await load_program(dut, pins, [
        enc(OUT, 0x11),
        enc(WAIT, 2),
        enc(OUT, 0x22),
        enc(WAIT, 2),
        jmp(0),
    ])
    await start(dut, pins)

    seen = []
    for _ in range(200):
        await RisingEdge(dut.clk)
        v = int(dut.uo_out.value)
        if not seen or seen[-1] != v:
            seen.append(v)

    body = [v for v in seen if v != 0]
    assert body[:4] == [0x11, 0x22, 0x11, 0x22], f"saw {seen}"


@cocotb.test()
async def test_conditional_jumps(dut):
    """JMP if IN0 / IN1 branches only while that input is high."""
    start_clock(dut)

    for pin, cond in ((IN0, IF_IN0), (IN1, IF_IN1)):
        for level, expected in ((0, 0xAA), (1, 0x55)):
            pins = await reset(dut)
            await load_program(dut, pins, [
                jmp(3, cond),        # 0: branch to 3 if the pin is high
                enc(OUT, 0xAA),      # 1: pin low path
                HALT,                # 2
                enc(OUT, 0x55),      # 3: pin high path
                HALT,                # 4
            ])
            pins.set(pin, level)
            await start(dut, pins)
            await ClockCycles(dut.clk, 20)
            assert dut.uo_out.value == expected, (
                f"pin bit {pin} level {level}: got {dut.uo_out.value}, "
                f"expected {expected:#x}"
            )


@cocotb.test()
async def test_program_counter_wraps(dut):
    """A program that fills all 8 words loops on its own, with no JMP."""
    start_clock(dut)
    pins = await reset(dut)

    await load_program(dut, pins, [
        enc(OUT, 0x01), enc(WAIT, 1),
        enc(OUT, 0x02), enc(WAIT, 1),
        enc(OUT, 0x04), enc(WAIT, 1),
        enc(OUT, 0x08), enc(WAIT, 1),
    ])
    await start(dut, pins)

    seen = []
    for _ in range(100):
        await RisingEdge(dut.clk)
        v = int(dut.uo_out.value)
        if v != 0 and (not seen or seen[-1] != v):
            seen.append(v)

    assert seen[:8] == [1, 2, 4, 8, 1, 2, 4, 8], f"saw {seen}"


@cocotb.test()
async def test_traffic_light(dut):
    """The traffic-light program (exactly 8 words) cycles through all four phases."""
    start_clock(dut)
    pins = await reset(dut)

    A_GREEN = 0b00001100    # A green, B red
    A_YELLOW = 0b00001010   # A yellow, B red
    B_GREEN = 0b00100001    # A red, B green
    B_YELLOW = 0b00010001   # A red, B yellow

    await load_program(dut, pins, [
        enc(OUT, A_GREEN), enc(WAIT, 9),
        enc(OUT, A_YELLOW), enc(WAIT, 4),
        enc(OUT, B_GREEN), enc(WAIT, 9),
        enc(OUT, B_YELLOW), enc(WAIT, 4),
    ])
    await start(dut, pins)

    order = []
    for _ in range(400):
        await RisingEdge(dut.clk)
        v = int(dut.uo_out.value)
        if v != 0 and (not order or order[-1] != v):
            order.append(v)

    expected = [A_GREEN, A_YELLOW, B_GREEN, B_YELLOW] * 2
    assert order[:8] == expected, f"phase order was {order[:8]}"

    # Safety: never conflicting lights
    for v in order:
        a_red, a_grn = v & 0b000001, v & 0b000100
        b_red, b_grn = v & 0b001000, v & 0b100000
        assert not (a_red and a_grn), "A red and green together"
        assert not (b_red and b_grn), "B red and green together"
        assert not (a_grn and b_grn), "both directions green"
