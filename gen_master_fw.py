#!/usr/bin/env python3
"""Generate the position-independent master firmware images for new_soc_v2.

The same RV32IMC program is emitted as SRAM and flash $readmemh images.  Its
reset trampoline and trap-vector setup are PC-relative, so it also runs after
boot-copy.  All words are 32-bit, one hex word per line.
"""
from pathlib import Path

# RV32I encodings -------------------------------------------------------------
OP_LUI, OP_AUIPC, OP_JAL, OP_JALR = 0x37, 0x17, 0x6F, 0x67
OP_BRANCH, OP_LOAD, OP_STORE, OP_IMM, OP_REG, OP_SYSTEM = 0x63, 0x03, 0x23, 0x13, 0x33, 0x73


def enc_i(op, f3, rd, rs1, imm):
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op


def enc_s(f3, rs2, rs1, imm):
    return (((imm >> 5) & 0x7F) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | ((imm & 0x1F) << 7) | OP_STORE


def enc_b(f3, rs2, rs1, imm):
    assert (imm & 1) == 0 and -4096 <= imm < 4096, imm
    return (((imm >> 12) & 1) << 31) | (((imm >> 5) & 0x3F) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (((imm >> 1) & 0xF) << 8) | (((imm >> 11) & 1) << 7) | OP_BRANCH


def enc_j(rd, imm):
    assert (imm & 1) == 0 and -(1 << 20) <= imm < (1 << 20), imm
    return (((imm >> 20) & 1) << 31) | (((imm >> 1) & 0x3FF) << 21) | (((imm >> 11) & 1) << 20) | (((imm >> 12) & 0xFF) << 12) | (rd << 7) | OP_JAL


def enc_u(op, rd, imm20):
    return ((imm20 & 0xFFFFF) << 12) | (rd << 7) | op


class Asm:
    def __init__(self):
        self.pc = 0
        self.words = {}
        self.labels = {}
        self.fixups = []
        self.serial = 0

    def org(self, addr):
        assert addr % 4 == 0
        self.pc = addr

    def label(self, name):
        if name in self.labels:
            raise ValueError("duplicate label " + name)
        self.labels[name] = self.pc

    def fresh(self, prefix="L"):
        self.serial += 1
        return f"{prefix}_{self.serial}"

    def emit(self, word):
        self.words[self.pc // 4] = word & 0xFFFFFFFF
        self.pc += 4

    def lui(self, rd, imm20): self.emit(enc_u(OP_LUI, rd, imm20))
    def auipc(self, rd, imm20): self.emit(enc_u(OP_AUIPC, rd, imm20))
    def addi(self, rd, rs1, imm): self.emit(enc_i(OP_IMM, 0, rd, rs1, imm))
    def andi(self, rd, rs1, imm): self.emit(enc_i(OP_IMM, 7, rd, rs1, imm))
    def ori(self, rd, rs1, imm): self.emit(enc_i(OP_IMM, 6, rd, rs1, imm))
    def slli(self, rd, rs1, shamt): self.emit(enc_i(OP_IMM, 1, rd, rs1, shamt))
    def jalr(self, rd, imm, rs1): self.emit(enc_i(OP_JALR, 0, rd, rs1, imm))
    def lw(self, rd, off, rs1): self.emit(enc_i(OP_LOAD, 2, rd, rs1, off))
    def lbu(self, rd, off, rs1): self.emit(enc_i(OP_LOAD, 4, rd, rs1, off))
    def sw(self, rs2, off, rs1): self.emit(enc_s(2, rs2, rs1, off))
    def add(self, rd, rs1, rs2): self.emit((rs2 << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | OP_REG)
    def sub(self, rd, rs1, rs2): self.emit((0x20 << 25) | (rs2 << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | OP_REG)
    def orr(self, rd, rs1, rs2): self.emit((rs2 << 20) | (rs1 << 15) | (6 << 12) | (rd << 7) | OP_REG)

    def li(self, rd, value):
        value &= 0xFFFFFFFF
        signed = value if value < 0x80000000 else value - 0x100000000
        hi = (signed + 0x800) >> 12
        lo = signed - (hi << 12)
        if hi:
            self.lui(rd, hi)
        if lo or not hi:
            self.addi(rd, rd if hi else 0, lo)

    def branch(self, name, rs1, rs2, label):
        f3 = {"beq": 0, "bne": 1, "blt": 4, "bge": 5}[name]
        self.fixups.append((self.pc, "b", f3, rs1, rs2, label))
        self.emit(0)

    def jal(self, rd, label):
        self.fixups.append((self.pc, "j", rd, label))
        self.emit(0)

    def csr(self, f3, rd, csr_addr, rs1):
        self.emit(((csr_addr & 0xFFF) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | OP_SYSTEM)

    def csrrw(self, rd, csr_addr, rs1): self.csr(1, rd, csr_addr, rs1)
    def csrrs(self, rd, csr_addr, rs1): self.csr(2, rd, csr_addr, rs1)
    def csrrc(self, rd, csr_addr, rs1): self.csr(3, rd, csr_addr, rs1)
    def ecall(self): self.emit(0x00000073)
    def ebreak(self): self.emit(0x00100073)
    def mret(self): self.emit(0x30200073)
    def illegal(self): self.emit(0xFFFFFFFF)

    def resolve(self):
        for f in self.fixups:
            if f[1] == "b":
                pc, _, f3, rs1, rs2, label = f
                if label not in self.labels: raise ValueError("undefined " + label)
                self.words[pc // 4] = enc_b(f3, rs2, rs1, self.labels[label] - pc)
            else:
                pc, _, rd, label = f
                if label not in self.labels: raise ValueError("undefined " + label)
                self.words[pc // 4] = enc_j(rd, self.labels[label] - pc)
        return self.words


# Register aliases used by the hand-built program.
X0, RA, SP = 0, 1, 2
T0, T1, T2, T3 = 5, 6, 7, 28
S0, S1, A0, A1, A2 = 8, 9, 10, 11, 12
A3, A4, A5, A6, A7 = 13, 14, 15, 16, 17
S2, S3, S4, S5, S6, S7 = 18, 19, 20, 21, 22, 23
S8, S9, S10, S11 = 24, 25, 26, 27

# SoC address map and SRAM test data.
UART = 0x80000000
SPI = 0x80001000
I2C = 0x80002000
I2S = 0x80003000
WDG = 0x80004000
CRC = 0x80005000
DMA0 = 0x80006000
DMA1 = 0x80007000
INTC = 0x80008000
SRC0, DST0 = 0x10000, 0x12000
SRC1, DST1 = 0x11000, 0x13000
COUNTERS = 0x14000
RXBUF = 0x14100
TXBUF = 0x14120
# Keep DMA-written result buffers on separate cache lines from preceding
# CPU/DMA data. The simple write-through D-cache is not snooped by DMA.
SPI_TXBUF, SPI_RXBUF = 0x14160, 0x14180
I2C_TXBUF, I2C_RXBUF = 0x141A0, 0x141C0
I2S_TXBUF, I2S_RXBUF = 0x141E0, 0x14200
WDG_DST = 0x14220


def emit_poll_mask(a, base_reg, off, mask, wait_label):
    a.label(wait_label)
    a.lw(T0, off, base_reg)
    a.andi(T0, T0, mask)
    a.branch("bne", T0, X0, wait_label)


def putc(a, ch, uart_reg=S0):
    wait = a.fresh("putc")
    a.label(wait)
    a.lw(T0, 20, uart_reg)
    a.andi(T0, T0, 0x20)
    a.branch("beq", T0, X0, wait)
    a.li(T1, ch)
    a.sw(T1, 0, uart_reg)


def print_text(a, text, uart_reg=S0):
    for c in text:
        putc(a, ord(c), uart_reg)


def dma_setup(a, base, stream, src, dest, count, cfg):
    a.li(T0, base + stream * 16)
    a.li(T1, src); a.sw(T1, 0, T0)
    a.li(T1, dest); a.sw(T1, 4, T0)
    a.li(T1, count); a.sw(T1, 8, T0)
    a.li(T1, cfg); a.sw(T1, 12, T0)


def dma_wait(a, base, stream, label=None):
    if label is None: label = a.fresh("dma_wait")
    a.li(T0, base + stream * 16 + 12)
    a.label(label)
    a.lw(T1, 0, T0)
    a.andi(T1, T1, 1)
    a.branch("bne", T1, X0, label)


def make_firmware():
    a = Asm()

    # Position-independent reset trampoline: SRAM and flash both jump forward
    # by 0x2000 bytes, so the same image also works after flash-to-SRAM copy.
    a.org(0)
    a.jal(X0, "main")
    a.emit(0x00000013)  # nop

    # MTVEC is 256-byte aligned (0x1000 relative to the selected boot base).
    a.org(0x1000)
    a.label("trap_vector_base")
    a.jal(X0, "trap_dispatch")  # synchronous exceptions enter at base
    for cause in range(1, 32):
        a.jal(X0, "trap_dispatch")

    # Main firmware, common for SRAM, XIP, and boot-copy.
    a.org(0x2000)
    a.label("main")
    a.li(SP, 0x17F00)
    a.li(T0, COUNTERS)
    a.li(T1, 20)
    a.label("clear_counters")
    a.sw(X0, 0, T0)
    a.addi(T0, T0, 4)
    a.addi(T1, T1, -1)
    a.branch("bne", T1, X0, "clear_counters")

    # Runtime mtvec = the position-relative vector base at image offset 0x1000.
    # Compute the signed displacement from this AUIPC's PC, so the same words
    # work in SRAM, flash XIP, and after boot-copy.
    mtvec_delta = 0x1000 - a.pc
    mtvec_hi = (mtvec_delta + 0x800) >> 12
    mtvec_lo = mtvec_delta - (mtvec_hi << 12)
    a.auipc(T0, mtvec_hi)
    if mtvec_lo:
        a.addi(T0, T0, mtvec_lo)
    a.ori(T0, T0, 1)
    a.csrrw(X0, 0x305, T0)   # mtvec

    # Enable interrupt controller sources 0..6 and set priorities.
    a.li(T0, INTC)
    a.li(T1, 0x7F)
    a.sw(T1, 0, T0)
    a.li(T1, 0x76543210)
    a.sw(T1, 8, T0)

    # Enable E40P custom IRQ IDs 16..23 in mie; global MIE is enabled after
    # peripheral setup below.  Sources 0..7 map to custom IDs 16..23.
    a.li(T0, 0x00FF0000)
    a.csrrw(X0, 0x304, T0)   # mie

    # Set up UART0 for 8N1, fast simulation divisor 4 (640 ns bit at 100 MHz).
    a.li(S0, UART)
    a.li(T0, 0x83); a.sw(T0, 12, S0)
    a.li(T0, 4);    a.sw(T0, 0, S0)
    a.li(T0, 0);    a.sw(T0, 4, S0)
    a.li(T0, 0x03); a.sw(T0, 12, S0)
    a.li(T0, 0);    a.sw(T0, 4, S0)  # IER disabled initially

    a.li(T0, 8)
    a.csrrs(X0, 0x300, T0)  # set mstatus.MIE

    # Exercise machine-mode synchronous exceptions; handler skips each word.
    a.ecall()
    a.ebreak()
    a.illegal()

    # CRC-32/MPEG-2 single-word known vector.
    a.li(S1, CRC)
    a.li(T0, 1); a.sw(T0, 0, S1)
    a.li(T0, 0x12345678); a.sw(T0, 4, S1)
    crc_wait = "crc_wait"
    a.label(crc_wait)
    a.lw(T1, 12, S1); a.andi(T1, T1, 1)
    a.branch("bne", T1, X0, crc_wait)
    a.lw(T1, 8, S1)
    a.li(T2, 0xDF8A8A2B)
    a.branch("bne", T1, T2, "fail")

    # Seed two 8-stream source banks with distinct, deterministic patterns.
    a.li(A0, SRC0); a.li(A1, 0x13579BDF); a.li(A2, 32)
    a.label("seed_dma0")
    a.sw(A1, 0, A0); a.addi(A0, A0, 4); a.addi(A1, A1, 1)
    a.addi(A2, A2, -1); a.branch("bne", A2, X0, "seed_dma0")
    a.li(A0, SRC1); a.li(A1, 0x2468ACE0); a.li(A2, 32)
    a.label("seed_dma1")
    a.sw(A1, 0, A0); a.addi(A0, A0, 4); a.addi(A1, A1, 1)
    a.addi(A2, A2, -1); a.branch("bne", A2, X0, "seed_dma1")

    # Launch all eight memory-to-memory streams on both DMA controllers.
    for ctrl, src, dst, label in ((DMA0, SRC0, DST0, "cfg_dma0"), (DMA1, SRC1, DST1, "cfg_dma1")):
        a.li(S0, ctrl); a.li(S1, src); a.li(S2, dst); a.li(S3, 8)
        a.label(label)
        a.sw(S1, 0, S0); a.sw(S2, 4, S0)
        a.li(T0, 4); a.sw(T0, 8, S0)
        a.li(T0, 1); a.sw(T0, 12, S0)
        a.addi(S0, S0, 16); a.addi(S1, S1, 16); a.addi(S2, S2, 16)
        a.addi(S3, S3, -1); a.branch("bne", S3, X0, label)

    # Wait for all 16 streams to auto-clear START.
    for ctrl, label in ((DMA0, "wait_dma0_all"), (DMA1, "wait_dma1_all")):
        a.li(S0, ctrl + 12); a.li(S1, 8)
        a.label(label)
        a.lw(T0, 0, S0); a.andi(T0, T0, 1)
        poll = a.fresh("one_stream")
        a.branch("beq", T0, X0, poll + "_done")
        a.label(poll)
        a.lw(T0, 0, S0); a.andi(T0, T0, 1)
        a.branch("bne", T0, X0, poll)
        a.label(poll + "_done")
        a.addi(S0, S0, 16); a.addi(S1, S1, -1)
        a.branch("bne", S1, X0, label)

    # Verify all words copied by DMA0 and DMA1.
    for src, dst, label in ((SRC0, DST0, "verify_dma0"), (SRC1, DST1, "verify_dma1")):
        a.li(S0, src); a.li(S1, dst); a.li(S2, 32)
        a.label(label)
        a.lw(T0, 0, S0); a.lw(T1, 0, S1)
        a.branch("bne", T0, T1, "fail")
        a.addi(S0, S0, 4); a.addi(S1, S1, 4); a.addi(S2, S2, -1)
        a.branch("bne", S2, X0, label)

    # UART RX through DMA0 stream 0. The testbench sends bytes after 'R'.
    a.li(S0, UART); a.li(T0, 1); a.sw(T0, 4, S0)
    dma_setup(a, DMA0, 0, UART, RXBUF, 4, 0x13)  # START|SRC_FIXED|REQ_SEL=UART
    putc(a, ord('R'), S0)
    dma_wait(a, DMA0, 0)
    a.li(S1, RXBUF)
    for i, byte in enumerate((0xA5, 0x5A, 0xC3, 0x3C)):
        a.lw(T0, i * 4, S1); a.li(T1, byte)
        a.branch("bne", T0, T1, "fail")
    a.li(T0, 0); a.sw(T0, 4, S0)

    # UART TX through DMA1 stream 0 (fixed THR destination, THRE pacing).
    # The test bytes live in TXBUF.
    a.li(A0, TXBUF)
    for i, byte in enumerate((0x31, 0x32, 0x33, 0x34)):
        a.li(T0, byte); a.sw(T0, i * 4, A0)
    a.li(T0, 2); a.sw(T0, 4, S0)  # UART THRE interrupt enable
    dma_setup(a, DMA1, 0, TXBUF, UART, 4, 0x15)  # START|DEST_FIXED|REQ_SEL=UART
    dma_wait(a, DMA1, 0)
    temt = "uart_temt_wait"
    a.label(temt)
    a.lw(T0, 20, S0); a.andi(T0, T0, 0x40)
    a.branch("beq", T0, X0, temt)
    a.li(T0, 0); a.sw(T0, 4, S0)

    # SPI full-duplex DMA transaction. Testbench loops MOSI back to MISO.
    a.li(S1, SPI)
    a.li(T0, 0); a.sw(T0, 4, S1)               # CLKDIV
    a.li(T0, 1); a.sw(T0, 8, S1)               # CMD EOT
    a.li(T0, 0x00080000); a.sw(T0, 16, S1)    # 8 data bits
    a.li(T0, 0xA5); a.li(T1, SPI_TXBUF); a.sw(T0, 0, T1)
    dma_setup(a, DMA1, 2, SPI_TXBUF, SPI + 0x18, 1, 0x05) # DMA TXFIFO
    dma_wait(a, DMA1, 2)
    dma_setup(a, DMA0, 2, SPI + 0x20, SPI_RXBUF, 1, 0x23) # RXFIFO paced by SPI event
    a.li(T0, 0x103); a.sw(T0, 0, S1)           # CS0 + read + write
    spi_idle = "spi_idle_wait"
    a.label(spi_idle)
    a.lw(T0, 0, S1); a.andi(T0, T0, 1)
    a.branch("bne", T0, X0, spi_idle)
    dma_wait(a, DMA0, 2)
    a.li(T1, SPI_RXBUF); a.lw(T0, 0, T1); a.li(T2, 0xA5)
    a.branch("bne", T0, T2, "fail")

    # I2C slave transaction: address/data write plus one-byte read via DMA.
    a.li(S1, I2C)
    a.li(T0, 3); a.sw(T0, 0, S1)              # prescaler
    a.li(T0, 0xC0); a.sw(T0, 4, S1)          # EN + IEN
    a.li(T0, 0xA0); a.sw(T0, 16, S1)
    a.li(T0, 0x90); a.sw(T0, 20, S1)         # START | WR address
    a.jal(RA, "i2c_wait_iack")
    a.li(T0, 0x5A); a.li(T1, I2C_TXBUF); a.sw(T0, 0, T1)
    dma_setup(a, DMA1, 3, I2C_TXBUF, I2C + 0x10, 1, 0x05) # fixed I2C TX
    dma_wait(a, DMA1, 3)
    a.li(T0, 0x10); a.sw(T0, 20, S1)         # WR data byte
    a.jal(RA, "i2c_wait_iack")
    a.li(T0, 0x40); a.sw(T0, 20, S1)         # STOP
    a.jal(RA, "i2c_wait_iack")

    # Repeated transaction for a one-byte read. DMA waits on I2C IRQ source.
    a.li(T0, 0xA1); a.sw(T0, 16, S1)
    a.li(T0, 0x90); a.sw(T0, 20, S1)
    a.jal(RA, "i2c_wait_iack")
    dma_setup(a, DMA0, 3, I2C + 0x08, I2C_RXBUF, 1, 0x33) # SRC_FIXED + REQ_SEL=I2C
    a.li(T0, 0x20); a.sw(T0, 20, S1)         # RD one byte
    # Wait TIP low first; DMA then consumes RX while IRQ remains asserted.
    i2c_rd_tip = "i2c_rd_tip"
    a.label(i2c_rd_tip)
    a.lw(T0, 12, S1); a.andi(T0, T0, 2)
    a.branch("bne", T0, X0, i2c_rd_tip)
    dma_wait(a, DMA0, 3)
    a.li(T0, 1); a.sw(T0, 20, S1)            # IACK
    a.li(T1, I2C_RXBUF); a.lw(T0, 0, T1); a.li(T2, 0xC3)
    a.andi(T0, T0, 0xFF); a.branch("bne", T0, T2, "fail")
    a.li(T0, 0x40); a.sw(T0, 20, S1)
    a.jal(RA, "i2c_wait_iack")

    # I2S TX DMA then external SD RX DMA. i2s_sd_i is held high by the TB,
    # making the captured input sample 0xFFFF_FFFF.
    a.li(T0, 0x89ABCDEF); a.li(T1, I2S_TXBUF); a.sw(T0, 0, T1)
    dma_setup(a, DMA1, 4, I2S_TXBUF, I2S + 0x08, 1, 0x45) # TXDATA, REQ_SEL=I2S TX
    dma_wait(a, DMA1, 4)
    a.li(S1, I2S)
    a.li(T0, 1); a.sw(T0, 4, S1)             # DIV
    a.li(T0, 0x13); a.sw(T0, 0, S1)          # EN|TX_EN|TX_IRQ_EN
    a.li(T0, 0x2000)
    a.label("i2s_tx_delay")
    a.addi(T0, T0, -1); a.branch("bne", T0, X0, "i2s_tx_delay")
    a.li(T0, 0); a.sw(T0, 0, S1)
    a.li(T0, 1); a.sw(T0, 20, S1)            # clear TX_DONE

    dma_setup(a, DMA0, 4, I2S + 0x0C, I2S_RXBUF, 1, 0x53) # RXDATA, REQ_SEL=I2S RX
    a.li(T0, 0x25); a.sw(T0, 0, S1)          # EN|RX_EN|RX_IRQ_EN
    dma_wait(a, DMA0, 4)
    a.li(T0, 0); a.sw(T0, 0, S1)
    a.li(T1, I2S_RXBUF); a.lw(T0, 0, T1)
    a.li(T2, 0xFFFFFFFF); a.branch("bne", T0, T2, "fail")
    a.li(T0, 2); a.sw(T0, 20, S1)            # clear RX_AVAIL

    # Watchdog timeout IRQ and request-paced DMA stream (REQ_SEL=6).
    dma_setup(a, DMA0, 5, SRC0, WDG_DST, 1, 0x61)
    a.li(S1, WDG)
    a.li(T0, 40); a.sw(T0, 0, S1)
    a.li(T0, 1); a.sw(T0, 8, S1)             # EN
    dma_wait(a, DMA0, 5)
    a.li(T0, 2); a.sw(T0, 8, S1)             # disable + W1C timeout
    a.li(T1, WDG_DST); a.lw(T0, 0, T1)
    a.li(T2, 0x13579BDF); a.branch("bne", T0, T2, "fail")

    # Verify the trap table handled all requested synchronous exceptions and
    # every connected interrupt source (UART, SPI, I2C, I2S, WDG, DMA0, DMA1).
    for i, minimum in enumerate((4, 1, 1, 1, 1, 1, 1)):
        a.li(T0, COUNTERS + 4 * i); a.lw(T1, 0, T0); a.li(T2, minimum)
        a.branch("blt", T1, T2, "fail")
    for addr in (COUNTERS + 0x40, COUNTERS + 0x44, COUNTERS + 0x48):
        a.li(T0, addr); a.lw(T1, 0, T0); a.branch("beq", T1, X0, "fail")

    # Success/failure string is the master TB's final criterion.
    a.li(S0, UART)
    print_text(a, "PASS")
    a.label("pass_halt")
    a.jal(X0, "pass_halt")

    a.label("fail")
    a.li(S0, UART)
    print_text(a, "FAIL")
    a.label("fail_halt")
    a.jal(X0, "fail_halt")

    # Shared I2C command completion + interrupt clear helper.  It is a local
    # subroutine with JALR return; main's RA is preserved by the caller.
    a.label("i2c_wait_iack")
    a.addi(S4, RA, 0)                        # save return address
    i2c_tip = "i2c_tip_wait"
    a.label(i2c_tip)
    a.lw(T0, 12, S1); a.andi(T0, T0, 2)
    a.branch("bne", T0, X0, i2c_tip)
    i2c_irq = "i2c_irq_wait"
    a.label(i2c_irq)
    a.lw(T0, 12, S1); a.andi(T0, T0, 1)
    a.branch("beq", T0, X0, i2c_irq)
    a.li(T0, 1); a.sw(T0, 20, S1)
    a.jalr(X0, 0, S4)

    # Trap handler (common vectored entry).  It preserves all registers it
    # uses, counts causes, advances MEPC for 32-bit synchronous exceptions,
    # and returns with MRET. Peripheral servicing is done by the main test
    # sequence after the associated event.
    a.org(0x3000)
    a.label("trap_dispatch")
    a.addi(SP, SP, -32)
    a.sw(T0, 0, SP); a.sw(T1, 4, SP); a.sw(T2, 8, SP); a.sw(A0, 12, SP)
    a.csrrs(T0, 0x342, X0)                    # mcause
    a.branch("blt", T0, X0, "irq_dispatch")
    a.andi(T1, T0, 0x1F)
    a.li(T2, 2); a.branch("beq", T1, T2, "exc_illegal")
    a.li(T2, 3); a.branch("beq", T1, T2, "exc_break")
    a.li(T2, 11); a.branch("beq", T1, T2, "exc_ecall")
    a.jal(X0, "exc_other")

    # Exception counters: +0x40 illegal, +0x44 ebreak, +0x48 ecall.
    for label, off in (("exc_illegal", 0x40), ("exc_break", 0x44), ("exc_ecall", 0x48), ("exc_other", 0x4C)):
        a.label(label)
        a.li(T2, COUNTERS + off)
        a.lw(A0, 0, T2); a.addi(A0, A0, 1); a.sw(A0, 0, T2)
        a.csrrs(T1, 0x341, X0)               # mepc
        a.addi(T1, T1, 4)
        a.csrrw(X0, 0x341, T1)
        a.jal(X0, "trap_return")

    a.label("irq_dispatch")
    a.andi(T1, T0, 0x1F)
    for source in range(8):
        a.li(T2, 16 + source)
        a.branch("beq", T1, T2, f"irq_source_{source}")
    a.jal(X0, "trap_return")
    for source in range(8):
        a.label(f"irq_source_{source}")
        a.li(T2, COUNTERS + 4 * source)
        a.lw(A0, 0, T2); a.addi(A0, A0, 1); a.sw(A0, 0, T2)
        a.jal(X0, "trap_return")

    a.label("trap_return")
    a.lw(T0, 0, SP); a.lw(T1, 4, SP); a.lw(T2, 8, SP); a.lw(A0, 12, SP)
    a.addi(SP, SP, 32)
    a.mret()

    words = a.resolve()
    return words, a.labels


def write_image(words, filename):
    top = max(words)
    path = Path(__file__).resolve().parent / filename
    with path.open("w", encoding="ascii") as f:
        for i in range(top + 1):
            f.write(f"{words.get(i, 0):08x}\n")
    print(f"{path.name}: {top + 1} words, top offset 0x{top*4:04X}")


if __name__ == "__main__":
    image, labels = make_firmware()
    # Identical position-independent content; each memory model maps line 0
    # to its own base. fw_flash is also the source image for BOOT_MODE=10.
    write_image(image, "fw_sram.hex")
    write_image(image, "fw_flash.hex")
    write_image(image, "fw_copy.hex")
    print("master firmware assembled; main=0x%04X trap=0x%04X" % (labels["main"], labels["trap_dispatch"]))
