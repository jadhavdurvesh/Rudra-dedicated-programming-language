"""An ATmega328P emulator for testing generated machine code: CPU with exact flags and cycle counts, SRAM,
Timer0 (overflow interrupt), the UART (TX captured, RX fed), and ports B/C/D (every pin change recorded with its time).

It implements the instructions rudrac emits (see asm.py), and raises on anything else rather than guessing.
It models a 16 MHz clock. It is NOT a substitute for the real chip: it is the thing generated code is checked
against first, and the first real-hardware results decide what it gets wrong.
"""
from typing import Dict, List, Optional, Tuple

F_CPU = 16_000_000
RAM_START, RAM_END = 0x100, 0x8FF
SREG_C, SREG_Z, SREG_N, SREG_V, SREG_S, SREG_H, SREG_T, SREG_I = range(8)

# data-space addresses
PINB, DDRB, PORTB = 0x23, 0x24, 0x25
PINC, DDRC, PORTC = 0x26, 0x27, 0x28
PIND, DDRD, PORTD = 0x29, 0x2A, 0x2B
TIFR0, TCCR0A, TCCR0B, TCNT0, OCR0A = 0x35, 0x44, 0x45, 0x46, 0x47
TIMSK0 = 0x6E
UCSR0A, UCSR0B, UCSR0C, UBRR0L, UBRR0H, UDR0 = 0xC0, 0xC1, 0xC2, 0xC4, 0xC5, 0xC6
SPL, SPH, SREG = 0x5D, 0x5E, 0x5F
TIMER0_COMPA_VECTOR = 0x1C     # word addresses of the ATmega328P vectors (2 words each)
TIMER0_OVF_VECTOR = 0x20
USART_RX_VECTOR = 0x24
PRESCALE0 = {1: 1, 2: 8, 3: 64, 4: 256, 5: 1024}


class EmuError(Exception):
    pass


class AVR:
    def __init__(self, words: Dict[int, int]):
        self.flash = [0xFFFF] * 0x4000                        # 32 KB
        for a, w in words.items():
            if a >= 0x4000:
                raise EmuError("program is larger than the 32 KB flash")
            self.flash[a] = w
        self.r = [0] * 32
        self.mem = [0] * 0x900                                # data space 0..0x8FF
        self.pc = 0
        self.sreg = 0
        self.cycles = 0
        self.t0_phase = 0                                     # cycles since the last Timer0 tick
        self.pins: Dict[int, int] = {}                        # last level written, by port-bit key
        self.pin_log: List[Tuple[float, str, int]] = []       # (ms, "B5", level)
        self.tx = bytearray()
        self.tx_log: List[Tuple[float, int]] = []             # (ms, byte)
        self.rx: List[int] = []
        self.sleeping = False
        self.input_pins = {"B": 0, "C": 0, "D": 0}            # levels the outside world drives (read via PINx)

    # ------------------------------------------------------------------ memory-mapped I/O
    @property
    def ms(self) -> float:
        return self.cycles * 1000.0 / F_CPU

    def rd(self, a: int) -> int:
        if a < 32:
            return self.r[a]
        if a == SREG:
            return self.sreg
        if a in (PINB, PINC, PIND):
            port = {PINB: "B", PINC: "C", PIND: "D"}[a]
            ddr, out = self.mem[a + 1], self.mem[a + 2]
            return (out & ddr) | (self.input_pins[port] & ~ddr & 0xFF)
        if a == UCSR0A:
            v = self.mem[a] & ~0x80 & 0xFF
            if self.rx:
                v |= 0x80                                     # RXC0
            return v | 0x20                                   # UDRE0: always ready to send
        if a == UDR0:
            return self.rx.pop(0) if self.rx else 0
        if 0 <= a <= RAM_END:
            return self.mem[a]
        raise EmuError(f"read from unmapped data address 0x{a:04x} at pc=0x{self.pc * 2:04x}")

    def wr(self, a: int, v: int):
        v &= 0xFF
        if a < 32:
            self.r[a] = v
        elif a == SREG:
            self.sreg = v
        elif a in (PORTB, PORTC, PORTD):
            port = {PORTB: "B", PORTC: "C", PORTD: "D"}[a]
            old, ddr = self.mem[a], self.mem[a - 1]
            self.mem[a] = v
            for bit in range(8):
                if ddr >> bit & 1 and (v ^ old) >> bit & 1:
                    self.pin_log.append((self.ms, f"{port}{bit}", v >> bit & 1))
        elif a in (DDRB, DDRC, DDRD):
            port = {DDRB: "B", DDRC: "C", DDRD: "D"}[a]
            old, out = self.mem[a], self.mem[a + 1]
            self.mem[a] = v
            for bit in range(8):                              # a pin starting to drive a 1 goes high now
                if (v ^ old) >> bit & 1 and out >> bit & 1:
                    self.pin_log.append((self.ms, f"{port}{bit}", v >> bit & 1))
        elif a in (PINB, PINC, PIND):                         # writing PINx toggles PORTx
            self.wr(a + 2, self.mem[a + 2] ^ v)
        elif a == UDR0:
            self.tx.append(v)
            self.tx_log.append((self.ms, v))
        elif a == TIFR0:
            self.mem[a] &= ~v & 0xFF                          # flags clear by writing a one
        elif a == TCNT0:
            self.mem[a] = v
            self.t0_phase = 0
        elif a in (UCSR0B, UCSR0C, UBRR0L, UBRR0H, UCSR0A, TCCR0A, TCCR0B, TIMSK0, OCR0A, SPL, SPH):
            self.mem[a] = v
        elif RAM_START <= a <= RAM_END:
            self.mem[a] = v
        else:
            raise EmuError(f"write to unmapped data address 0x{a:04x} at pc=0x{self.pc * 2:04x}")

    def sp(self) -> int:
        return self.mem[SPL] | (self.mem[SPH] << 8)

    def set_sp(self, v: int):
        self.mem[SPL], self.mem[SPH] = v & 0xFF, (v >> 8) & 0xFF

    def push8(self, v: int):
        sp = self.sp()
        if sp < RAM_START or sp > RAM_END:
            raise EmuError(f"stack pointer 0x{sp:04x} outside SRAM at pc=0x{self.pc * 2:04x}")
        self.mem[sp] = v & 0xFF
        self.set_sp(sp - 1)

    def pop8(self) -> int:
        sp = self.sp() + 1
        if sp > RAM_END:
            raise EmuError(f"stack underflow at pc=0x{self.pc * 2:04x}")
        self.set_sp(sp)
        return self.mem[sp]

    def push_pc(self, pc: int):
        self.push8(pc & 0xFF)
        self.push8(pc >> 8)

    def pop_pc(self) -> int:
        hi = self.pop8()
        return (hi << 8) | self.pop8()

    # ------------------------------------------------------------------ flags
    def flag(self, bit: int) -> int:
        return (self.sreg >> bit) & 1

    def setf(self, bit: int, v):
        if v:
            self.sreg |= 1 << bit
        else:
            self.sreg &= ~(1 << bit) & 0xFF

    def _nzvs(self, res: int, v: int):
        self.setf(SREG_N, res & 0x80)
        self.setf(SREG_V, v)
        self.setf(SREG_S, bool(res & 0x80) != bool(v))
        self.setf(SREG_Z, (res & 0xFF) == 0)

    def add(self, a, b, c=0):
        res = a + b + c
        r = res & 0xFF
        self.setf(SREG_H, ((a & 0xF) + (b & 0xF) + c) > 0xF)
        self.setf(SREG_C, res > 0xFF)
        self._nzvs(r, ((a ^ r) & (b ^ r) & 0x80) != 0)
        return r

    def sub(self, a, b, c=0, keep_z=False):
        res = a - b - c
        r = res & 0xFF
        self.setf(SREG_H, (a & 0xF) - (b & 0xF) - c < 0)
        self.setf(SREG_C, res < 0)
        z = self.flag(SREG_Z)
        self._nzvs(r, ((a ^ b) & (a ^ r) & 0x80) != 0)
        if keep_z:                                            # sbc / sbci / cpc: Z only stays set if it already was
            self.setf(SREG_Z, r == 0 and z)
        return r

    def logic(self, r):
        self._nzvs(r, 0)
        return r

    # ------------------------------------------------------------------ run
    def pending_irq(self) -> int:
        """Word address of the vector to take now (lowest vector number first), or 0."""
        if not self.flag(SREG_I):
            return 0
        f, m = self.mem[TIFR0], self.mem[TIMSK0]
        if (f & 2) and (m & 2):
            return TIMER0_COMPA_VECTOR
        if (f & 1) and (m & 1):
            return TIMER0_OVF_VECTOR
        if self.rx and (self.mem[UCSR0B] & 0x80):
            return USART_RX_VECTOR
        return 0

    def advance(self, n: int):
        self.cycles += n
        presc = PRESCALE0.get(self.mem[TCCR0B] & 7, 0)
        if presc:
            self.t0_phase += n
            ctc = (self.mem[TCCR0A] & 3) == 2                 # WGM01: clear timer on compare match
            while self.t0_phase >= presc:
                self.t0_phase -= presc
                t = self.mem[TCNT0]
                if ctc:
                    t = 0 if t >= self.mem[OCR0A] else t + 1
                    if t == self.mem[OCR0A]:
                        self.mem[TIFR0] |= 2
                else:
                    t = (t + 1) & 0xFF
                    if t == 0:
                        self.mem[TIFR0] |= 1
                self.mem[TCNT0] = t

    def step(self):
        vec = self.pending_irq()
        if vec:
            self.push_pc(self.pc)
            self.setf(SREG_I, 0)
            if vec == TIMER0_COMPA_VECTOR:
                self.mem[TIFR0] &= ~2 & 0xFF
            elif vec == TIMER0_OVF_VECTOR:
                self.mem[TIFR0] &= ~1 & 0xFF
            self.pc = vec
            self.advance(4)
            return
        op = self.flash[self.pc]
        if op == 0xFFFF:
            raise EmuError(f"executing erased flash at pc=0x{self.pc * 2:04x}")
        self.pc += 1
        n = self.exec(op)
        self.advance(n)

    def skip_len(self) -> int:
        nxt = self.flash[self.pc]
        two = (nxt & 0xFE0F) in (0x9000, 0x9200) or (nxt & 0xFE0E) in (0x940C, 0x940E)
        self.pc += 2 if two else 1
        return 2 if two else 1

    def exec(self, op: int) -> int:
        r = self.r
        d5 = (op >> 4) & 0x1F
        r5 = ((op >> 5) & 0x10) | (op & 0xF)
        top = op & 0xFC00
        # --- two-register group
        if top == 0x0C00:
            r[d5] = self.add(r[d5], r[r5]); return 1
        if top == 0x1C00:
            r[d5] = self.add(r[d5], r[r5], self.flag(SREG_C)); return 1
        if top == 0x1800:
            r[d5] = self.sub(r[d5], r[r5]); return 1
        if top == 0x0800:
            r[d5] = self.sub(r[d5], r[r5], self.flag(SREG_C), True); return 1
        if top == 0x1400:
            self.sub(r[d5], r[r5]); return 1
        if top == 0x0400:
            self.sub(r[d5], r[r5], self.flag(SREG_C), True); return 1
        if top == 0x2000:
            r[d5] = self.logic(r[d5] & r[r5]); return 1
        if top == 0x2800:
            r[d5] = self.logic(r[d5] | r[r5]); return 1
        if top == 0x2400:
            r[d5] = self.logic(r[d5] ^ r[r5]); return 1
        if top == 0x2C00:
            r[d5] = r[r5]; return 1
        if top == 0x1000:                                       # cpse
            if r[d5] == r[r5]:
                return 1 + self.skip_len()
            return 1
        if top == 0x9C00:                                       # mul
            p = r[d5] * r[r5]
            r[0], r[1] = p & 0xFF, p >> 8
            self.setf(SREG_C, p & 0x8000)
            self.setf(SREG_Z, (p & 0xFFFF) == 0)
            return 2
        if (op & 0xFF00) == 0x0100:                             # movw
            d, s = ((op >> 4) & 0xF) * 2, (op & 0xF) * 2
            r[d], r[d + 1] = r[s], r[s + 1]; return 1
        # --- register-immediate group
        hi = op & 0xF000
        if hi in (0x3000, 0x4000, 0x5000, 0x6000, 0x7000, 0xE000):
            d = 16 + ((op >> 4) & 0xF)
            k = ((op >> 4) & 0xF0) | (op & 0xF)
            if hi == 0xE000:
                r[d] = k
            elif hi == 0x3000:
                self.sub(r[d], k)
            elif hi == 0x4000:
                r[d] = self.sub(r[d], k, self.flag(SREG_C), True)
            elif hi == 0x5000:
                r[d] = self.sub(r[d], k)
            elif hi == 0x6000:
                r[d] = self.logic(r[d] | k)
            else:
                r[d] = self.logic(r[d] & k)
            return 1
        # --- one-register group (0x9400..0x95xx) and friends
        if (op & 0xFE0F) == 0x9400:                             # com
            v = 0xFF - r[d5]
            r[d5] = v
            self._nzvs(v, 0)
            self.setf(SREG_C, 1); return 1
        if (op & 0xFE0F) == 0x9401:                             # neg
            a = r[d5]
            res = (-a) & 0xFF
            r[d5] = res
            self.setf(SREG_H, (res & 8) or (a & 8))
            self._nzvs(res, res == 0x80)
            self.setf(SREG_C, res != 0); return 1
        if (op & 0xFE0F) == 0x9402:
            a = r[d5]; r[d5] = ((a << 4) | (a >> 4)) & 0xFF; return 1
        if (op & 0xFE0F) == 0x9403:                             # inc
            res = (r[d5] + 1) & 0xFF
            self._nzvs(res, r[d5] == 0x7F)
            r[d5] = res; return 1
        if (op & 0xFE0F) == 0x940A:                             # dec
            res = (r[d5] - 1) & 0xFF
            self._nzvs(res, r[d5] == 0x80)
            r[d5] = res; return 1
        if (op & 0xFE0F) in (0x9405, 0x9406, 0x9407):           # asr / lsr / ror
            a = r[d5]
            kind = op & 0xF
            res = (a >> 1) | ((a & 0x80) if kind == 5 else (self.flag(SREG_C) << 7 if kind == 7 else 0))
            self.setf(SREG_C, a & 1)
            self.setf(SREG_N, res & 0x80)
            self.setf(SREG_Z, res == 0)
            self.setf(SREG_V, bool(res & 0x80) != bool(a & 1))
            self.setf(SREG_S, self.flag(SREG_N) != self.flag(SREG_V))
            r[d5] = res; return 1
        if (op & 0xFE0F) == 0x920F:
            self.push8(r[d5]); return 2
        if (op & 0xFE0F) == 0x900F:
            r[d5] = self.pop8(); return 2
        if (op & 0xFF00) in (0x9600, 0x9700):                   # adiw / sbiw
            d = 24 + ((op >> 4) & 3) * 2
            k = ((op >> 2) & 0x30) | (op & 0xF)
            v = r[d] | (r[d + 1] << 8)
            res = (v + k if (op & 0xFF00) == 0x9600 else v - k) & 0xFFFF
            if (op & 0xFF00) == 0x9600:
                self.setf(SREG_V, (~v & res) & 0x8000)
                self.setf(SREG_C, (~res & v) & 0x8000)
            else:
                self.setf(SREG_V, (v & ~res) & 0x8000)
                self.setf(SREG_C, (res & ~v) & 0x8000)
            self.setf(SREG_N, res & 0x8000)
            self.setf(SREG_Z, res == 0)
            self.setf(SREG_S, self.flag(SREG_N) != self.flag(SREG_V))
            r[d], r[d + 1] = res & 0xFF, res >> 8; return 2
        # --- jumps and branches
        if (op & 0xF000) == 0xC000:
            k = op & 0xFFF
            self.pc += k - 0x1000 if k & 0x800 else k; return 2
        if (op & 0xF000) == 0xD000:
            k = op & 0xFFF
            self.push_pc(self.pc)
            self.pc += k - 0x1000 if k & 0x800 else k; return 3
        if (op & 0xFC08) == 0xFC00:                             # sbrc / sbrs
            bit = (r[d5] >> (op & 7)) & 1
            if bit == (1 if op & 0x0200 else 0):
                return 1 + self.skip_len()
            return 1
        if (op & 0xFC00) in (0xF000, 0xF400):
            k = (op >> 3) & 0x7F
            k = k - 128 if k & 0x40 else k
            bit = op & 7
            taken = bool(self.flag(bit)) == ((op & 0x0400) == 0)
            if taken:
                self.pc += k
                return 2
            return 1
        if op == 0x9508:
            self.pc = self.pop_pc(); return 4
        if op == 0x9518:
            self.pc = self.pop_pc(); self.setf(SREG_I, 1); return 4
        if op == 0x9478:
            self.setf(SREG_I, 1); return 1
        if op == 0x94F8:
            self.setf(SREG_I, 0); return 1
        if op == 0x9408:
            self.setf(SREG_C, 1); return 1
        if op == 0x9488:
            self.setf(SREG_C, 0); return 1
        if op == 0x0000:
            return 1
        if op == 0x9509:
            self.push_pc(self.pc); self.pc = r[30] | (r[31] << 8); return 3
        if op == 0x9409:
            self.pc = r[30] | (r[31] << 8); return 2
        if (op & 0xFE0E) in (0x940C, 0x940E):
            k = ((((op >> 3) & 0x3E) | (op & 1)) << 16) | self.flash[self.pc]
            self.pc += 1
            if (op & 0xFE0E) == 0x940E:
                self.push_pc(self.pc)
                self.pc = k; return 4
            self.pc = k; return 3
        # --- I/O
        if (op & 0xF800) == 0xB000:
            a = ((op >> 5) & 0x30) | (op & 0xF)
            r[d5] = self.rd(a + 0x20); return 1
        if (op & 0xF800) == 0xB800:
            a = ((op >> 5) & 0x30) | (op & 0xF)
            self.wr(a + 0x20, r[d5]); return 1
        if (op & 0xFC00) == 0x9800:
            a, b = (op >> 3) & 0x1F, op & 7
            cur = self.rd(a + 0x20)
            kind = op & 0x0300
            if kind == 0x0000:
                self.wr(a + 0x20, cur & ~(1 << b))
                return 2
            if kind == 0x0200:
                self.wr(a + 0x20, cur | (1 << b))
                return 2
            bit = (cur >> b) & 1
            if bit == (1 if kind == 0x0300 else 0):
                return 1 + self.skip_len()
            return 1
        # --- memory
        if (op & 0xFE0F) == 0x9000:
            r[d5] = self.rd(self.flash[self.pc]); self.pc += 1; return 2
        if (op & 0xFE0F) == 0x9200:
            self.wr(self.flash[self.pc], r[d5]); self.pc += 1; return 2
        if (op & 0xFE0E) == 0x9004:
            z = r[30] | (r[31] << 8)
            f = self.flash[(z >> 1) & 0x3FFF]
            r[d5] = (f >> 8) if z & 1 else (f & 0xFF)
            if op & 1:
                z = (z + 1) & 0xFFFF
                r[30], r[31] = z & 0xFF, z >> 8
            return 3
        if (op & 0xD000) == 0x8000 or (op & 0xFC00) == 0x9000:
            return self.ldst(op, d5)
        raise EmuError(f"instruction 0x{op:04x} at word 0x{self.pc - 1:04x} is not implemented")

    def ldst(self, op: int, d5: int) -> int:
        store = bool(op & 0x0200)
        r = self.r
        if (op & 0xD000) == 0x8000:                             # ldd/std with displacement (and plain Y / Z)
            q = ((op >> 8) & 0x20) | ((op >> 7) & 0x18) | (op & 7)
            base = 28 if op & 8 else 30
            addr = (r[base] | (r[base + 1] << 8)) + q
        else:
            mode = op & 0xF
            if mode not in (0x1, 0x2, 0x9, 0xA, 0xC, 0xD, 0xE):
                raise EmuError(f"load/store form 0x{op:04x} not implemented")
            base = {0x1: 30, 0x2: 30, 0x9: 28, 0xA: 28, 0xC: 26, 0xD: 26, 0xE: 26}[mode]
            p = r[base] | (r[base + 1] << 8)
            if mode in (0x2, 0xA, 0xE):
                p = (p - 1) & 0xFFFF
                r[base], r[base + 1] = p & 0xFF, p >> 8
            addr = p
            if mode in (0x1, 0x9, 0xD):
                q = (p + 1) & 0xFFFF
                r[base], r[base + 1] = q & 0xFF, q >> 8
        addr &= 0xFFFF
        if store:
            self.wr(addr, r[d5])
        else:
            r[d5] = self.rd(addr)
        return 2

    # ------------------------------------------------------------------ driving it
    def run_ms(self, ms: float, rx_events: Optional[List[Tuple[float, bytes]]] = None, max_steps: int = 500_000_000):
        end = self.cycles + int(ms * F_CPU / 1000)
        ev = sorted(rx_events or [], key=lambda e: e[0])
        k = 0
        steps = 0
        while self.cycles < end:
            while k < len(ev) and ev[k][0] <= self.ms:
                self.rx.extend(ev[k][1])
                k += 1
            self.step()
            steps += 1
            if steps > max_steps:
                raise EmuError("step limit reached")

    def tx_text(self) -> str:
        return self.tx.decode("latin-1")
