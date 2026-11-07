"""A small AVR assembler: exactly the instructions rudrac emits, nothing else.

Syntax (one statement per line, ';' starts a comment):
    label:                 ldi r16, 0xFF          lds r24, 0x0100         rjmp label
    .org 0x20   (word address)      .word 0x1234, label      .byte 1, 2, 3 (padded to a word)
    .equ NAME, value        lo8(x) / hi8(x) / lo8(pm(label)) in operands
Registers r0..r31 (and the names X, Y, Z for the pointer pairs). Data addresses and I/O addresses are plain numbers.
Every instruction word is checked for range; an operand that does not fit is an error, never silently truncated.
"""
import re
from typing import Dict, List, Optional, Tuple


class AsmError(Exception):
    pass


BRANCH = {  # mnemonic -> (base, bit)
    "breq": (0xF000, 1), "brne": (0xF400, 1), "brcs": (0xF000, 0), "brlo": (0xF000, 0), "brcc": (0xF400, 0),
    "brsh": (0xF400, 0), "brmi": (0xF000, 2), "brpl": (0xF400, 2), "brvs": (0xF000, 3), "brvc": (0xF400, 3),
    "brlt": (0xF000, 4), "brge": (0xF400, 4), "brhs": (0xF000, 5), "brhc": (0xF400, 5), "brts": (0xF000, 6),
    "brtc": (0xF400, 6), "brie": (0xF000, 7), "brid": (0xF400, 7),
}
TWO_REG = {"add": 0x0C00, "adc": 0x1C00, "sub": 0x1800, "sbc": 0x0800, "and": 0x2000, "or": 0x2800, "eor": 0x2400,
           "mov": 0x2C00, "cp": 0x1400, "cpc": 0x0400, "cpse": 0x1000, "mul": 0x9C00}
IMM = {"subi": 0x5000, "sbci": 0x4000, "andi": 0x7000, "ori": 0x6000, "cpi": 0x3000, "ldi": 0xE000}
ONE_REG = {"com": 0x9400, "neg": 0x9401, "swap": 0x9402, "inc": 0x9403, "asr": 0x9405, "lsr": 0x9406, "ror": 0x9407,
           "dec": 0x940A, "push": 0x920F, "pop": 0x900F}
ALIAS2 = {"lsl": "add", "rol": "adc", "tst": "and", "clr": "eor"}      # op rd  ==  op rd, rd
NOARG = {"ret": 0x9508, "reti": 0x9518, "sei": 0x9478, "cli": 0x94F8, "nop": 0x0000, "icall": 0x9509,
         "ijmp": 0x9409, "sec": 0x9408, "clc": 0x9488, "sleep": 0x9588, "wdr": 0x95A8}
LDST = {  # (is_store, pointer-form) -> base
    (0, "X"): 0x900C, (0, "X+"): 0x900D, (0, "-X"): 0x900E, (0, "Y+"): 0x9009, (0, "-Y"): 0x900A,
    (0, "Z+"): 0x9001, (0, "-Z"): 0x9002,
    (1, "X"): 0x920C, (1, "X+"): 0x920D, (1, "-X"): 0x920E, (1, "Y+"): 0x9209, (1, "-Y"): 0x920A,
    (1, "Z+"): 0x9201, (1, "-Z"): 0x9202,
}
TWO_WORD = {"lds", "sts", "jmp", "call"}


def _split(line: str) -> Tuple[Optional[str], List[str]]:
    line = line.split(";", 1)[0].strip()
    if not line:
        return None, []
    m = re.match(r"([A-Za-z_.][\w.]*)\s*(.*)$", line)
    if not m:
        raise AsmError(f"cannot read: {line!r}")
    op, rest = m.group(1).lower(), m.group(2).strip()
    ops, depth, cur = [], 0, ""
    for ch in rest:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        if ch == "," and depth == 0:
            ops.append(cur.strip())
            cur = ""
        else:
            cur += ch
    if cur.strip():
        ops.append(cur.strip())
    return op, ops


class Assembler:
    def __init__(self):
        self.syms: Dict[str, int] = {}          # label -> word address, .equ -> value
        self.words: Dict[int, int] = {}         # word address -> 16-bit word
        self.lines: List[Tuple[int, str, List[str], int]] = []   # lineno, op, ops, word address

    # ---- operand helpers
    def reg(self, s: str, lo=0, hi=31) -> int:
        m = re.fullmatch(r"[rR](\d+)", s.strip())
        if not m:
            raise AsmError(f"expected a register, got {s!r}")
        n = int(m.group(1))
        if not lo <= n <= hi:
            raise AsmError(f"register {s} must be r{lo}..r{hi} for this instruction")
        return n

    def num(self, s: str, ctx=None) -> int:
        s = s.strip()
        m = re.fullmatch(r"(lo8|hi8|hlo8|hhi8)\((?:pm\()?([^()]+)\)?\)", s)
        if m:
            v = self.num(m.group(2))
            sh = {"lo8": 0, "hi8": 8, "hlo8": 16, "hhi8": 24}[m.group(1)]
            return (v >> sh) & 0xFF
        m = re.fullmatch(r"(.+?)\s*\*\s*(\d+)", s)
        if m:                                       # word address -> byte address:  tab*2
            return self.num(m.group(1), ctx) * int(m.group(2))
        if re.fullmatch(r"-?(0[xX][0-9a-fA-F]+|\d+|0[bB][01]+)", s):
            return int(s, 0)
        m = re.fullmatch(r"([A-Za-z_]\w*)\s*([+-]\s*\d+)?", s)
        if m and m.group(1) in self.syms:
            return self.syms[m.group(1)] + (int(m.group(2).replace(" ", "")) if m.group(2) else 0)
        if m and ctx == "first":
            return 0
        raise AsmError(f"unknown symbol or number {s!r}")

    @staticmethod
    def fit(v: int, lo: int, hi: int, what: str) -> int:
        if not lo <= v <= hi:
            raise AsmError(f"{what} {v} does not fit ({lo}..{hi})")
        return v

    # ---- two passes
    def assemble(self, text: str) -> Dict[int, int]:
        pc = 0
        stmts = []
        for n, raw in enumerate(text.split("\n"), 1):
            try:
                line = raw
                while True:
                    m = re.match(r"\s*([A-Za-z_]\w*):(.*)$", line)
                    if not m:
                        break
                    if m.group(1) in self.syms:
                        raise AsmError(f"label {m.group(1)} defined twice")
                    self.syms[m.group(1)] = pc
                    line = m.group(2)
                op, ops = _split(line)
                if op is None:
                    continue
                if op == ".org":
                    pc = self.num(ops[0])
                elif op == ".equ":
                    self.syms[ops[0]] = self.num(ops[1])
                elif op == ".word":
                    stmts.append((n, op, ops, pc))
                    pc += len(ops)
                elif op == ".byte":
                    stmts.append((n, op, ops, pc))
                    pc += (len(ops) + 1) // 2
                else:
                    stmts.append((n, op, ops, pc))
                    pc += 2 if op in TWO_WORD else 1
            except AsmError as e:
                raise AsmError(f"line {n}: {e}") from None
        for n, op, ops, at in stmts:
            try:
                for k, w in enumerate(self.encode(op, ops, at)):
                    if at + k in self.words:
                        raise AsmError(f"address 0x{(at + k) * 2:04x} is written twice (overlapping .org?)")
                    self.words[at + k] = w & 0xFFFF
            except IndexError:
                raise AsmError(f"line {n}: {op} {', '.join(ops)}: missing operand") from None
            except AsmError as e:
                raise AsmError(f"line {n}: {op} {', '.join(ops)}: {e}") from None
        return self.words

    def rel(self, target: str, at: int, bits: int) -> int:
        m = re.fullmatch(r"\.\s*(?:([+-])\s*(\d+))?", target.strip())
        if m:                                       # '.+N' / '.-N': byte offset from the NEXT instruction, as avr-as prints it
            off = int(m.group(2) or 0) * (-1 if m.group(1) == "-" else 1)
            if off % 2:
                raise AsmError("a branch target must be an even number of bytes away")
            k = off // 2
        else:
            k = self.num(target) - (at + 1)
        lim = 1 << (bits - 1)
        self.fit(k, -lim, lim - 1, "branch distance")
        return k & ((1 << bits) - 1)

    def encode(self, op: str, ops: List[str], at: int) -> List[int]:
        if op == ".word":
            return [self.num(o) & 0xFFFF for o in ops]
        if op == ".byte":
            b = [self.fit(self.num(o), -128, 255, ".byte value") & 0xFF for o in ops]
            if len(b) % 2:
                b.append(0)
            return [b[i] | (b[i + 1] << 8) for i in range(0, len(b), 2)]
        if op in ALIAS2:
            ops = [ops[0], ops[0]]
            op = ALIAS2[op]
        if op in NOARG:
            if ops:
                raise AsmError("takes no operands")
            return [NOARG[op]]
        if op in TWO_REG:
            d, r = self.reg(ops[0]), self.reg(ops[1])
            return [TWO_REG[op] | ((r & 0x10) << 5) | (d << 4) | (r & 0xF)]
        if op in IMM:
            d = self.reg(ops[0], 16, 31)
            k = self.num(ops[1])
            k = self.fit(k, -128, 255, "immediate") & 0xFF
            return [IMM[op] | ((k & 0xF0) << 4) | ((d - 16) << 4) | (k & 0xF)]
        if op in ONE_REG:
            return [ONE_REG[op] | (self.reg(ops[0]) << 4)]
        if op == "movw":
            d, r = self.reg(ops[0]), self.reg(ops[1])
            if d % 2 or r % 2:
                raise AsmError("movw needs even registers")
            return [0x0100 | ((d // 2) << 4) | (r // 2)]
        if op in ("adiw", "sbiw"):
            d = self.reg(ops[0], 24, 30)
            if d % 2:
                raise AsmError("adiw/sbiw need r24, r26, r28 or r30")
            k = self.fit(self.num(ops[1]), 0, 63, "immediate")
            return [(0x9600 if op == "adiw" else 0x9700) | ((k & 0x30) << 2) | (((d - 24) // 2) << 4) | (k & 0xF)]
        if op in ("rjmp", "rcall"):
            return [(0xC000 if op == "rjmp" else 0xD000) | self.rel(ops[0], at, 12)]
        if op in BRANCH:
            base, bit = BRANCH[op]
            return [base | (self.rel(ops[0], at, 7) << 3) | bit]
        if op in ("jmp", "call"):
            k = self.fit(self.num(ops[0]), 0, 0x3FFFFF, "address")
            return [(0x940C if op == "jmp" else 0x940E) | ((k >> 17) << 4) | ((k >> 16) & 1), k & 0xFFFF]
        if op in ("in", "out"):
            if op == "in":
                d, a = self.reg(ops[0]), self.fit(self.num(ops[1]), 0, 63, "I/O address")
            else:
                a, d = self.fit(self.num(ops[0]), 0, 63, "I/O address"), self.reg(ops[1])
            return [(0xB000 if op == "in" else 0xB800) | ((a & 0x30) << 5) | (d << 4) | (a & 0xF)]
        if op in ("sbi", "cbi", "sbic", "sbis"):
            a = self.fit(self.num(ops[0]), 0, 31, "I/O address (sbi/cbi reach 0..31)")
            b = self.fit(self.num(ops[1]), 0, 7, "bit")
            return [{"cbi": 0x9800, "sbic": 0x9900, "sbi": 0x9A00, "sbis": 0x9B00}[op] | (a << 3) | b]
        if op in ("sbrc", "sbrs"):
            b = self.fit(self.num(ops[1]), 0, 7, "bit")
            return [(0xFC00 if op == "sbrc" else 0xFE00) | (self.reg(ops[0]) << 4) | b]
        if op in ("lds", "sts"):
            if op == "lds":
                d, k = self.reg(ops[0]), self.fit(self.num(ops[1]), 0, 0xFFFF, "address")
            else:
                k, d = self.fit(self.num(ops[0]), 0, 0xFFFF, "address"), self.reg(ops[1])
            return [(0x9000 if op == "lds" else 0x9200) | (d << 4), k]
        if op in ("ld", "st"):
            st = op == "st"
            reg, ptr = (ops[1], ops[0]) if st else (ops[0], ops[1])
            d = self.reg(reg)
            p = ptr.replace(" ", "").upper()
            m = re.fullmatch(r"([YZ])\+(\d+)", p)
            if m:                                   # ldd / std with displacement
                q = self.fit(int(m.group(2)), 0, 63, "displacement")
                base = 0x8000 | (0x0008 if m.group(1) == "Y" else 0) | (0x0200 if st else 0)
                return [base | ((q & 0x20) << 8) | ((q & 0x18) << 7) | (q & 7) | (d << 4)]
            if p in ("Y", "Z"):
                base = 0x8000 | (0x0008 if p == "Y" else 0) | (0x0200 if st else 0)
                return [base | (d << 4)]
            if (int(st), p) not in LDST:
                raise AsmError(f"bad pointer form {ptr!r}")
            return [LDST[(int(st), p)] | (d << 4)]
        if op in ("ldd", "std"):
            return self.encode("ld" if op == "ldd" else "st", ops, at)
        if op == "lpm":
            d = self.reg(ops[0])
            p = ops[1].replace(" ", "").upper()
            if p not in ("Z", "Z+"):
                raise AsmError("lpm reads through Z or Z+")
            return [0x9004 | (0x0001 if p == "Z+" else 0) | (d << 4)]
        raise AsmError(f"unknown instruction {op!r}")


def assemble(text: str) -> Dict[int, int]:
    return Assembler().assemble(text)


def to_bytes(words: Dict[int, int]) -> bytes:
    n = (max(words) + 1) if words else 0
    out = bytearray(n * 2)
    for a, w in words.items():
        out[a * 2], out[a * 2 + 1] = w & 0xFF, w >> 8
    return bytes(out)


def to_hex(words: Dict[int, int]) -> str:
    """Intel HEX (what avrdude and Wokwi take)."""
    data = to_bytes(words)
    lines = []
    for off in range(0, len(data), 16):
        chunk = data[off:off + 16]
        rec = bytes([len(chunk), off >> 8, off & 0xFF, 0]) + chunk
        lines.append(":" + rec.hex().upper() + f"{(-sum(rec)) & 0xFF:02X}")
    lines.append(":00000001FF")
    return "\n".join(lines) + "\n"
