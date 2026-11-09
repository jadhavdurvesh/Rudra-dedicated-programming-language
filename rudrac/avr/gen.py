"""rudrac backend for the ATmega328P (Uno, Nano): Rudra IR -> AVR assembly -> machine code, no avr-gcc.

First slice (stage B5.3). A program is accepted only if every feature it uses is implemented here; anything else is
refused with a list of reasons, never compiled differently. Supported: serial commands (`on`), `start`, `every`,
`heartbeat`, `send`, digital outputs, `never` interlocks, `var`/`let`/arrays/structs/enums, `func`/`proc`, `if`,
`for`, `break`, 32-bit integer arithmetic with the same wrap/div rules as the simulator.
Not yet: inputs, `when`, states, paths, other output kinds (pwm, servo, motor, ...), blackbox, bus addressing.
"""
import re
from typing import Dict, List, Tuple

from .. import ir as I
from ..check import Checked
from ..link import protocol_hash
from ..syntax import IO, Var
from .runtime import HELPERS

RAM_START, RAM_END = 0x100, 0x8FF
RX_RING, RX_MAX = 64, 32
# PWM-capable pins on timers we can use (pins 5 and 6 sit on Timer0, which runs the 1 ms clock):
# pin -> (timer, OCR high address or None, OCR low address, TCCRnA address, COM bit mask)
PWM_PINS = {3: (2, None, 0xB4, 0xB0, 0x20), 11: (2, None, 0xB3, 0xB0, 0x80),
            9: (1, 0x89, 0x88, 0x80, 0x80), 10: (1, 0x8B, 0x8A, 0x80, 0x20)}
MAX_SERVOS = 4
CMP = {"==": ("breq", False), "!=": ("brne", False), "<": ("brlt", False), ">=": ("brge", False),
       ">": ("brlt", True), "<=": ("brge", True)}      # (branch if true, swap operands)


class AvrUnsupported(Exception):
    def __init__(self, reasons):
        self.reasons = list(reasons)
        super().__init__("; ".join(self.reasons))


def _pin(pin) -> int:
    c = str(pin.c)
    return 14 + int(c[1:]) if c[0] in "Aa" else int(c)


def _port(n: int) -> Tuple[int, int, int]:
    """(PORT io address, DDR io address, bit) of an Uno digital pin number."""
    if n <= 7:
        return 0x0B, 0x0A, n
    if n <= 13:
        return 0x05, 0x04, n - 8
    if n <= 19:
        return 0x08, 0x07, n - 14
    raise AvrUnsupported([f"pin {n} does not exist on this chip"])


def _walk(stmts, expr_fn, stmt_fn):
    for s in stmts:
        stmt_fn(s)
        for f in ("e", "idx", "cond", "call"):
            v = getattr(s, f, None)
            if v is not None:
                expr_fn(v)
        for f in ("args",):
            for v in getattr(s, f, None) or []:
                expr_fn(v)
        for f in ("then", "els", "body"):
            v = getattr(s, f, None)
            if isinstance(v, list):
                _walk(v, expr_fn, stmt_fn)


def unsupported(ck: Checked) -> List[str]:
    node, why = ck.node, []
    ios = [d for d in node.decls if isinstance(d, IO)]
    for d in ios:
        if d.direction == "input":
            if d.kind not in ("digital", "analog"):
                why.append(f"input '{d.name}' of kind {d.kind} (only digital and analog inputs are implemented)")
        elif d.kind == "pwm":
            if _pin(d.pin) not in PWM_PINS:
                why.append(f"pwm output '{d.name}' on pin {_pin(d.pin)} (only pins 3, 9, 10, 11 can do PWM here: "
                           "5 and 6 share Timer0 with the 1 ms clock)")
        elif d.kind not in ("digital", "servo"):
            why.append(f"output '{d.name}' of kind {d.kind} (only digital, pwm and servo outputs are implemented)")
    servos = [d for d in ios if d.direction == "output" and d.kind == "servo"]
    if len(servos) > MAX_SERVOS:
        why.append(f"{len(servos)} servos (at most {MAX_SERVOS})")
    if servos and any(d.direction == "output" and d.kind == "pwm" and _pin(d.pin) in (9, 10) for d in ios):
        why.append("servos together with pwm on pins 9/10 (both need Timer1)")
    if node.address is not None:
        why.append("bus addressing ('address')")
    if node.blackbox_pos is not None:
        why.append("blackbox")
    if node.paths:
        why.append("paths")
    for e in node.events:
        if e.kind not in ("every", "on", "heartbeat", "start", "when", "enter", "exit"):
            why.append(f"'{e.kind}' handlers")
    low = I.Lowerer(ck)
    bad_stmt = (I.FillS, I.PixelS, I.AnimS, I.RunS, I.StopS, I.CancelS)

    def ex(e):
        if isinstance(e, I.Running):
            why.append("path names as values")
        for f in ("e", "l", "r", "idx"):
            v = getattr(e, f, None)
            if v is not None and not isinstance(v, (int, str, bool)):
                ex(v)
        for v in getattr(e, "args", None) or []:
            ex(v)

    def st(s):
        if isinstance(s, bad_stmt):
            why.append(type(s).__name__.replace("S", "", 1).lower() + " statements")

    bodies = [low.body(e.body, e.params) for e in node.events if e.kind in ("every", "on", "heartbeat", "start", "when", "enter", "exit")]
    for e in node.events:
        if e.kind == "when":
            ex(low.expr(e.arg))
    bodies += [low.fn_body(f.name) for f in ck.fns]
    for b in bodies:
        _walk(b, ex, st)
    for r in ck.rules:
        ex(low.expr(r.expr))
    return sorted(set(why))


class Gen:
    def __init__(self, ck: Checked):
        reasons = unsupported(ck)
        if reasons:
            raise AvrUnsupported(reasons)
        self.ck, self.node = ck, ck.node
        self.low = I.Lowerer(ck)
        self.lines: List[str] = []
        self.eq: List[Tuple[str, int]] = []
        self.next_ram = RAM_START
        self.strings: Dict[str, str] = {}
        self.nlabel = 0
        self.loop_exits: List[str] = []
        self.owner = ""
        self.slots: Dict[str, str] = {}
        self.ios = {d.name: d for d in self.node.decls if isinstance(d, IO)}
        self.outs = [d for d in self.node.decls if isinstance(d, IO) and d.direction == "output"]
        self.servos = [d for d in self.node.decls if isinstance(d, IO) and d.direction == "output" and d.kind == "servo"]
        self.ins = [d for d in self.node.decls if isinstance(d, IO) and d.direction == "input"]
        self.dbs = [d for d in self.ins if d.kind == "digital" and d.cfg.get("debounce")]
        self.vars = [d for d in self.node.decls if isinstance(d, Var)]
        self.events = list(enumerate(self.node.events))
        self.hb = next(((i, e) for i, e in self.events if e.kind == "heartbeat" and not e.off), None)
        self.maxp = max([len(e.params) for _, e in self.events if e.kind == "on"] or [0])

    # ------------------------------------------------------------------ small helpers
    def w(self, s: str):
        self.lines.append(s if s.endswith(":") or s.startswith((";", ".")) else "        " + s)

    def lab(self, p="L") -> str:
        self.nlabel += 1
        return f"{p}{self.nlabel}"

    def alloc(self, name: str, n: int) -> str:
        self.eq.append((name, self.next_ram))
        self.next_ram += n
        return name

    def string(self, text: str) -> str:
        if text not in self.strings:
            self.strings[text] = f"STR{len(self.strings)}"
        return self.strings[text]

    def slot(self, name: str) -> str:
        key = f"{self.owner}.{name}"
        if key not in self.slots:
            self.slots[key] = self.alloc(f"S_{re.sub(r'[^A-Za-z0-9_]', '_', self.owner)}_{name}", 4)
        return self.slots[key]

    def load(self, label: str, base: int = 22, off: int = 0):
        for k in range(4):
            self.w(f"lds r{base + k}, {label}+{off + k}")

    def store(self, label: str, base: int = 22):
        for k in range(4):
            self.w(f"sts {label}+{k}, r{base + k}")

    def const(self, v, base: int = 22):
        v = int(v) & 0xFFFFFFFF
        for k in range(4):
            self.w(f"ldi r{base + k}, {(v >> (8 * k)) & 0xFF}")

    def test_a(self):
        self.w("mov r16, r22")
        self.w("or r16, r23")
        self.w("or r16, r24")
        self.w("or r16, r25")

    def br_far(self, cond: str, target: str):
        """Jump to target if the condition holds (cond is an AVR branch mnemonic), at any distance."""
        inv = {"breq": "brne", "brne": "breq", "brlo": "brsh", "brsh": "brlo", "brlt": "brge", "brge": "brlt"}[cond]
        skip = self.lab("Ls")
        self.w(f"{inv} {skip}")
        self.w(f"jmp {target}")
        self.w(f"{skip}:")

    # ------------------------------------------------------------------ expressions: result in A (r22..r25)
    def simple(self, e) -> bool:
        return isinstance(e, (I.K, I.Loc, I.Glob, I.Out))

    def addr_of(self, e) -> str:
        if isinstance(e, I.Loc):
            return self.slot(e.name)
        if isinstance(e, I.Glob):
            return self.v_label(e.slot)
        return f"O_{e.name}"

    def load_simple(self, e, base: int):
        if isinstance(e, I.K):
            self.const(e.v, base)
        else:
            self.load(self.addr_of(e), base)

    def ex(self, e):
        if isinstance(e, I.K):
            self.const(e.v)
        elif isinstance(e, (I.Loc, I.Glob, I.Out)):
            self.load(self.addr_of(e))
        elif isinstance(e, I.Neg):
            self.ex(e.e)
            self.w("call neg32")
        elif isinstance(e, I.Not):
            self.ex(e.e)
            self.w("call not32")
        elif isinstance(e, I.Op):
            self.op(e)
        elif isinstance(e, I.In):
            self.w(f"call i_{e.name}")
        elif isinstance(e, I.CallE):
            self.call(e)
        elif isinstance(e, I.Elem):
            self.elem_read(e)
        else:
            raise AvrUnsupported([f"expression {type(e).__name__}"])

    def op(self, e):
        if e.op in ("&&", "||"):
            end, rhs = self.lab("Le"), self.lab("Lr")
            self.ex(e.l)
            self.test_a()
            self.w(f"{'brne' if e.op == '&&' else 'breq'} {rhs}")
            self.w(f"jmp {end}")
            self.w(f"{rhs}:")
            self.ex(e.r)
            self.w(f"{end}:")
            return
        if self.simple(e.r):
            self.ex(e.l)
            self.load_simple(e.r, 18)
        else:
            self.ex(e.l)
            for r in (22, 23, 24, 25):
                self.w(f"push r{r}")
            self.ex(e.r)
            self.w("movw r18, r22")
            self.w("movw r20, r24")
            for r in (25, 24, 23, 22):
                self.w(f"pop r{r}")
        if e.op in CMP:
            br, swap = CMP[e.op]
            a, b = (18, 22) if swap else (22, 18)
            self.w(f"cp r{a}, r{b}")
            for k in (1, 2, 3):
                self.w(f"cpc r{a + k}, r{b + k}")
            t = self.lab("Lt")
            self.w("ldi r22, 1")
            self.w(f"{br} {t}")
            self.w("ldi r22, 0")
            self.w(f"{t}:")
            for r in (23, 24, 25):
                self.w(f"ldi r{r}, 0")
            return
        self.w({"+": "call add32", "-": "call sub32", "*": "call mul32", "/": "call divs32", "%": "call mods32"}[e.op])

    def call(self, e):
        f = next(f for f in self.ck.fns if f.name == e.fn)
        for a in e.args:
            self.ex(a)
            for r in (22, 23, 24, 25):
                self.w(f"push r{r}")
        for p in reversed(f.params):
            for r in (25, 24, 23, 22):
                self.w(f"pop r{r}")
            self.store(f"S_f_{e.fn}_{p[0]}")
        self.w(f"call f_{e.fn}")

    def elem_addr(self, slot: str, size: int, proven: bool, oob: str):
        """Index in r18:r21 (B); leaves Z = &slot[index]; jumps to oob when the index is out of range."""
        if not proven:
            self.w("or r20, r21")
            self.br_far("brne", oob)
            self.w(f"cpi r18, {size & 0xFF}")
            self.w(f"ldi r16, {size >> 8}")
            self.w("cpc r19, r16")
            self.br_far("brsh", oob)
        self.w("lsl r18")
        self.w("rol r19")
        self.w("lsl r18")
        self.w("rol r19")
        base = self.v_label(slot)
        self.w(f"ldi r30, lo8({base})")
        self.w(f"ldi r31, hi8({base})")
        self.w("add r30, r18")
        self.w("adc r31, r19")

    def elem_read(self, e):
        self.ex(e.idx)
        self.w("movw r18, r22")
        self.w("movw r20, r24")
        oob, end = self.lab("Lo"), self.lab("Le")
        self.elem_addr(e.slot, e.size, e.proven, oob)
        self.w("ld r22, Z+")
        self.w("ld r23, Z+")
        self.w("ld r24, Z+")
        self.w("ld r25, Z")
        self.w(f"jmp {end}")
        self.w(f"{oob}:")
        self.const(0)
        self.w(f"{end}:")

    def v_label(self, slot: str) -> str:
        return "V_" + re.sub(r"[^A-Za-z0-9_]", "_", slot)

    # ------------------------------------------------------------------ statements
    def block(self, stmts):
        for s in stmts:
            self.stmt(s)

    def inc_blocked(self):
        self.w("lds r16, BLOCKED")
        self.w("subi r16, 0xFF")
        self.w("sts BLOCKED, r16")
        for k in (1, 2, 3):
            self.w(f"lds r16, BLOCKED+{k}")
            self.w("sbci r16, 0xFF")
            self.w(f"sts BLOCKED+{k}, r16")

    def stmt(self, s):
        if isinstance(s, I.LetS):
            self.ex(s.e)
            self.store(self.slot(s.name))
        elif isinstance(s, I.SetL):
            self.ex(s.e)
            self.store(self.slot(s.name))
        elif isinstance(s, I.SetG):
            self.ex(s.e)
            self.store(self.v_label(s.slot))
        elif isinstance(s, I.SetE):
            self.ex(s.idx)
            for r in (22, 23, 24, 25):
                self.w(f"push r{r}")
            self.ex(s.e)
            for r in (21, 20, 19, 18):
                self.w(f"pop r{r}")
            oob, end = self.lab("Lo"), self.lab("Le")
            self.elem_addr(s.slot, s.size, s.proven, oob)
            for r in (22, 23, 24):
                self.w(f"st Z+, r{r}")
            self.w("st Z, r25")
            self.w(f"jmp {end}")
            self.w(f"{oob}:")
            self.inc_blocked()
            self.w(f"{end}:")
        elif isinstance(s, I.SetO):
            self.ex(s.e)
            self.w(f"call set_{s.name}")
        elif isinstance(s, I.Ret):
            if s.e is not None:
                self.ex(s.e)
            self.w("ret")
        elif isinstance(s, I.GotoS):
            self.w(f"ldi r16, {s.index}")
            self.w("call r_goto")
        elif isinstance(s, I.Brk):
            self.w(f"jmp {self.loop_exits[-1]}")
        elif isinstance(s, I.CallS):
            self.call(s.call)
        elif isinstance(s, I.IfS):
            els, end = self.lab("Lf"), self.lab("Le")
            self.ex(s.cond)
            self.test_a()
            self.br_far("breq", els)
            self.block(s.then)
            if s.els:
                self.w(f"jmp {end}")
            self.w(f"{els}:")
            if s.els:
                self.block(s.els)
                self.w(f"{end}:")
        elif isinstance(s, I.Loop):
            sl = self.slot(s.var)
            top, exit_ = self.lab("Lw"), self.lab("Lx")
            self.const(s.lo)
            self.store(sl)
            self.w(f"{top}:")
            self.load(sl)
            self.const(s.hi, 18)
            self.w("cp r22, r18")
            for k in (1, 2, 3):
                self.w(f"cpc r{22 + k}, r{18 + k}")
            self.br_far("brge", exit_)
            self.loop_exits.append(exit_)
            self.block(s.body)
            self.loop_exits.pop()
            self.load(sl)
            self.w("subi r22, 0xFF")
            for k in (1, 2, 3):
                self.w(f"sbci r{22 + k}, 0xFF")
            self.store(sl)
            self.w(f"jmp {top}")
            self.w(f"{exit_}:")
        elif isinstance(s, I.SendS):
            if s.e is None:
                self.w(f"ldi r30, lo8({self.string(s.msg)}*2)")
                self.w(f"ldi r31, hi8({self.string(s.msg)}*2)")
                self.w("call tx_pstr")
            else:
                t = self.string(s.msg + " ")
                self.w(f"ldi r30, lo8({t}*2)")
                self.w(f"ldi r31, hi8({t}*2)")
                self.w("call tx_pstr")
                self.ex(s.e)
                if s.is_bool:
                    self.w("call not32")
                    self.w("call not32")
                self.w("call tx_i32")
            self.w("call tx_crlf")
        else:
            raise AvrUnsupported([type(s).__name__])

    # ------------------------------------------------------------------ whole program
    def generate(self) -> str:
        node, ck = self.node, self.ck
        # runtime state
        for n, size in (("NOW", 4), ("NOWC", 4), ("HBLAST", 4), ("HBTRIP", 1), ("BLOCKED", 4), ("RXHEAD", 1),
                        ("ST", 1), ("RXTAIL", 1), ("RXLEN", 1), ("RXDROP", 1), ("RXNAME", 2), ("NA", 1)):
            self.alloc(n, size)
        self.alloc("RXRING", RX_RING)
        self.alloc("RXBUF", RX_MAX)
        self.alloc("ARGS", 4 * max(self.maxp, 1))
        inits: List[Tuple[str, int]] = []
        for d in self.vars:
            for slot, ty in d.slots:
                lab = self.v_label(slot)
                if d.shape in ("array", "structarray"):
                    self.alloc(lab, 4 * d.size)
                else:
                    self.alloc(lab, 4)
                    if d.shape == "scalar" and int(d.init):
                        inits.append((lab, int(d.init)))
        for d in self.outs:
            self.alloc(f"O_{d.name}", 4)
        for i, e in self.events:
            if e.kind == "every":
                self.alloc(f"T_{i}", 4)
            elif e.kind == "when":
                self.alloc(f"W_{i}", 1)
                self.alloc(f"WO_{i}", 1)
                self.alloc(f"WT_{i}", 4)
        if self.servos:
            self.alloc("SVI", 1)
            self.alloc("SVC", 1)
            for d in self.servos:
                self.alloc(f"SVW_{d.name}", 2)
        for d in self.dbs:
            self.alloc(f"DBV_{d.name}", 1)
            self.alloc(f"DBC_{d.name}", 1)
            self.alloc(f"DBT_{d.name}", 4)

        out = self.lines
        # --- vector table and reset
        w = self.w
        w("; Generated by rudrac (AVR backend, stage B5.3). Do not edit by hand.")
        w(f"; node: {node.name}   target: {node.target}")
        w(".org 0")
        w("jmp reset")
        if self.servos:
            w(".org 0x16")
            w("jmp isr_t1a")
            w(".org 0x18")
            w("jmp isr_t1b")
        w(".org 0x1C")
        w("jmp isr_compa")
        w(".org 0x24")
        w("jmp isr_rx")
        w(".org 0x34")
        w("reset:")
        w("clr r2")
        w("out 0x3f, r2")
        w("ldi r28, lo8(0x08FF)")
        w("ldi r29, hi8(0x08FF)")
        w("out 0x3e, r29")
        w("out 0x3d, r28")
        w("ldi r30, lo8(0x0100)")
        w("ldi r31, hi8(0x0100)")
        w("ldi r16, lo8(RAM_USED)")
        w("ldi r17, hi8(RAM_USED)")
        w("zero_ram:")
        w("st Z+, r2")
        w("cp r30, r16")
        w("cpc r31, r17")
        w("brne zero_ram")
        for lab, v in inits:
            self.const(v)
            self.store(lab)
        pwms = [d for d in self.outs if d.kind == "pwm"]
        if any(PWM_PINS[_pin(d.pin)][0] == 2 for d in pwms):
            w("ldi r16, 3")                      # Timer2: fast PWM, clock/64
            w("sts 0xB0, r16")
            w("ldi r16, 4")
            w("sts 0xB1, r16")
        if any(PWM_PINS[_pin(d.pin)][0] == 1 for d in pwms):
            w("ldi r16, 1")                      # Timer1: 8-bit fast PWM, clock/64
            w("sts 0x80, r16")
            w("ldi r16, 0x0B")
            w("sts 0x81, r16")
        w("call apply_safe")
        for d in self.outs:
            _, ddr, bit = _port(_pin(d.pin))
            w(f"sbi {ddr}, {bit}")
        if self.servos:
            slot = 40000 // len(self.servos) - 1       # 2 MHz timer ticks; the slots together make a 20 ms frame
            w("ldi r16, 0")
            w("sts 0x80, r16")
            w(f"ldi r16, {slot >> 8}")
            w("sts 0x89, r16")
            w(f"ldi r16, {slot & 255}")
            w("sts 0x88, r16")
            w("ldi r16, 0xFF")                   # compare B must not match before the first pulse is scheduled
            w("sts 0x8B, r16")
            w("sts 0x8A, r16")
            w("ldi r16, 0x06")
            w("sts 0x6F, r16")
            w("ldi r16, 0x0A")                   # CTC, clock/8
            w("sts 0x81, r16")
        for d in self.ins:
            if d.kind == "digital":
                port, ddr, bit = _port(_pin(d.pin))
                w(f"cbi {ddr}, {bit}")
                if d.pullup:
                    w(f"sbi {port}, {bit}")
        if any(d.kind == "analog" for d in self.ins):
            w("ldi r16, 0x87")
            w("sts 0x7A, r16")
        for d in self.dbs:
            w(f"call i_raw_{d.name}")
            w(f"sts DBV_{d.name}, r22")
            w(f"sts DBC_{d.name}, r22")
        baud = node.baud
        ubrr, u2x = (16_000_000 // 4 // baud - 1) // 2, 2
        if ubrr > 4095:
            ubrr, u2x = (16_000_000 // 8 // baud - 1) // 2, 0
        if not 0 <= ubrr <= 4095:
            raise AvrUnsupported([f"baud rate {baud} is out of range"])
        w(f"ldi r16, {u2x}")
        w("sts 0xC0, r16")
        w(f"ldi r16, {ubrr & 0xFF}")
        w("sts 0xC4, r16")
        w(f"ldi r16, {ubrr >> 8}")
        w("sts 0xC5, r16")
        w("ldi r16, 0x06")
        w("sts 0xC2, r16")
        w("ldi r16, 0x98")                       # RX interrupt, receiver, transmitter
        w("sts 0xC1, r16")
        w("ldi r16, 2")                          # Timer0: CTC, 1 ms tick
        w("out 0x24, r16")
        w("ldi r16, 249")
        w("out 0x27, r16")
        w("ldi r16, 3")
        w("out 0x25, r16")
        w("ldi r16, 2")
        w("sts 0x6E, r16")
        w("sei")
        for i, e in self.events:
            if e.kind == "start":
                w(f"call h_{i}")
        first = next(iter(ck.state_index), None)
        for i, e in self.events:
            if e.kind == "enter" and e.state == first:
                w(f"call h_{i}")
        w("jmp main_loop")

        # --- interrupts
        w("isr_compa:")
        w("push r24")
        w("in r24, 0x3f")
        w("push r24")
        w("lds r24, NOW")
        w("subi r24, 0xFF")
        w("sts NOW, r24")
        for k in (1, 2, 3):
            w(f"lds r24, NOW+{k}")
            w("sbci r24, 0xFF")
            w(f"sts NOW+{k}, r24")
        w("pop r24")
        w("out 0x3f, r24")
        w("pop r24")
        w("reti")
        w("isr_rx:")
        w("push r16")
        w("in r16, 0x3f")
        w("push r16")
        w("push r17")
        w("push r30")
        w("push r31")
        w("lds r16, 0xC6")
        w("lds r17, RXHEAD")
        w("mov r30, r17")
        w("inc r17")
        w(f"andi r17, {RX_RING - 1}")
        w("lds r31, RXTAIL")
        w("cp r17, r31")
        w("breq isr_rx_x")
        w("push r18")
        w("ldi r18, lo8(RXRING)")
        w("ldi r31, hi8(RXRING)")
        w("add r30, r18")
        w("adc r31, r2")
        w("pop r18")
        w("st Z, r16")
        w("sts RXHEAD, r17")
        w("isr_rx_x:")
        w("pop r31")
        w("pop r30")
        w("pop r17")
        w("pop r16")
        w("out 0x3f, r16")
        w("pop r16")
        w("reti")
        return self._rest()

    def _rest(self) -> str:
        node, ck = self.node, self.ck
        w = self.w
        # --- output routines
        rules = [self.low.expr(r.expr) for r in ck.rules]
        if rules:
            self.owner = "rules"
            w("rt_rule:")
            for k, r in enumerate(rules, 1):
                nx = self.lab("Ln")
                self.ex(r)
                self.test_a()
                w(f"breq {nx}")
                w(f"ldi r22, {k}")
                w("ret")
                w(f"{nx}:")
            w("clr r22")
            w("ret")
        for d in self.ins:
            n = d.name
            if d.kind == "digital":
                port, _, bit = _port(_pin(d.pin))
                w(f"i_raw_{n}:")
                w(f"in r16, {port - 2}")
                w(f"andi r16, {1 << bit}")
                w("ldi r22, 0")
                w(f"breq i_raw_{n}_0")
                w("ldi r22, 1")
                w(f"i_raw_{n}_0:")
                if d.invert:
                    w("ldi r16, 1")
                    w("eor r22, r16")
                w("clr r23")
                w("clr r24")
                w("clr r25")
                w("ret")
                if d.cfg.get("debounce"):
                    w(f"i_{n}:")
                    w(f"lds r22, DBV_{n}")
                    w("clr r23")
                    w("clr r24")
                    w("clr r25")
                    w("ret")
                    w(f"r_sample_{n}:")
                    w(f"call i_raw_{n}")
                    w(f"lds r16, DBV_{n}")
                    w("cp r22, r16")
                    w(f"brne smp_{n}_1")
                    w(f"sts DBC_{n}, r22")
                    w("ret")
                    w(f"smp_{n}_1:")
                    w(f"lds r16, DBC_{n}")
                    w("cp r22, r16")
                    w(f"breq smp_{n}_2")
                    w(f"sts DBC_{n}, r22")
                    self.load("NOWC")
                    self.store(f"DBT_{n}")
                    w("ret")
                    w(f"smp_{n}_2:")
                    self.load("NOWC")
                    self.load(f"DBT_{n}", 18)
                    w("call sub32")
                    self.const(d.cfg["debounce"], 18)
                    w("cp r22, r18")
                    for k in (1, 2, 3):
                        w(f"cpc r{22 + k}, r{18 + k}")
                    w(f"brlo smp_{n}_3")
                    w(f"lds r16, DBC_{n}")
                    w(f"sts DBV_{n}, r16")
                    w(f"smp_{n}_3:")
                    w("ret")
                else:
                    w(f"i_{n}:")
                    w(f"jmp i_raw_{n}")
            else:
                ch = _pin(d.pin) - 14
                w(f"i_{n}:")
                w(f"ldi r16, {0x40 | ch}")
                w("sts 0x7C, r16")
                w("ldi r16, 0xC7")
                w("sts 0x7A, r16")
                w(f"i_{n}_w:")
                w("lds r16, 0x7A")
                w("sbrc r16, 6")
                w(f"rjmp i_{n}_w")
                w("lds r22, 0x78")
                w("lds r23, 0x79")
                w("clr r24")
                w("clr r25")
                w("ret")
        for d in self.outs:
            n = d.name
            port, _, bit = _port(_pin(d.pin))
            hi, lo = ("cbi", "sbi") if d.invert else ("sbi", "cbi")
            if d.kind == "digital":
                w(f"hw_{n}:")
                w(f"lds r16, O_{n}")
                w("tst r16")
                w(f"breq hw_{n}_lo")
                w(f"{hi} {port}, {bit}")
                w("ret")
                w(f"hw_{n}_lo:")
                w(f"{lo} {port}, {bit}")
                w("ret")
            elif d.kind == "pwm":
                self.hw_pwm(d, port, bit)
            else:
                self.hw_servo(d)
            w(f"set_{n}:")
            if d.kind == "digital":
                w("mov r16, r22")
                w("or r16, r23")
                w("or r16, r24")
                w("or r16, r25")
                w("ldi r22, 1")
                w(f"brne set_{n}_v")
                w("ldi r22, 0")
                w(f"set_{n}_v:")
            else:
                self.clamp(0, 100 if d.kind == "pwm" else 180)
            if rules:
                for k in range(4):
                    w(f"lds r16, O_{n}+{k}")
                    w("push r16")
                self.store(f"O_{n}")
                w("call rt_rule")
                w("tst r22")
                w(f"breq set_{n}_ok")
                for k in (3, 2, 1, 0):
                    w(f"pop r16")
                    w(f"sts O_{n}+{k}, r16")
                self.inc_blocked()
                w("ret")
                w(f"set_{n}_ok:")
                for _ in range(4):
                    w("pop r16")
            else:
                self.store(f"O_{n}")
            w(f"jmp hw_{n}")
        w("apply_safe:")
        for d in self.outs:
            self.const(int(d.cfg["safe"]))
            self.store(f"O_{d.name}")
            w(f"call hw_{d.name}")
        w("ret")
        w("rt_heard:")
        if self.hb:
            self.load("NOWC")
            self.store("HBLAST")
            w("sts HBTRIP, r2")
        w("ret")

        if ck.states:
            w("r_goto:                       ; r16 = new state index")
            w("lds r17, ST")
            w("cp r17, r16")
            w("brne rg_go")
            w("ret")
            w("rg_go:")
            w("push r16")
            for i, e in self.events:
                if e.kind == "exit":
                    sk = self.lab("Lg")
                    w("lds r16, ST")
                    w(f"cpi r16, {ck.state_index[e.state]}")
                    w(f"brne {sk}")
                    w(f"call h_{i}")
                    w(f"{sk}:")
            w("pop r16")
            w("push r16")
            w("sts ST, r16")
            for name, k in ck.state_index.items():
                items = [(i, e) for i, e in self.events if e.state == name and e.kind in ("every", "when")]
                if not items:
                    continue
                sk = self.lab("Lg")
                w(f"cpi r16, {k}")
                w(f"brne {sk}")
                for i, e in items:
                    if e.kind == "every":
                        self.load("NOWC")
                        self.store(f"T_{i}")
                    else:
                        w(f"sts W_{i}, r2")
                        w(f"sts WO_{i}, r2")
                w(f"{sk}:")
                w("pop r16")
                w("push r16")
            for i, e in self.events:
                if e.kind == "enter":
                    sk = self.lab("Lg")
                    w("pop r16")
                    w("push r16")
                    w(f"cpi r16, {ck.state_index[e.state]}")
                    w(f"brne {sk}")
                    w(f"call h_{i}")
                    w(f"{sk}:")
            w("pop r16")
            w("ret")

        # --- functions and handlers
        for f in ck.fns:
            self.owner = f"f_{f.name}"
            w(f"f_{f.name}:")
            self.block(self.low.fn_body(f.name))
            w("ret")
        for i, e in self.events:
            if e.kind not in ("every", "on", "heartbeat", "start", "when", "enter", "exit"):
                continue
            self.owner = f"h{i}"
            w(f"h_{i}:   ; {e.kind} {e.name or ''}")
            for p in e.params:
                self.slot(p)
            self.block(self.low.body(e.body, e.params))
            w("ret")

        if self.servos:
            self.servo_isrs()
        self.owner = "rx"
        self.rx_code()
        self.main_loop()
        # --- constants in flash
        w("name_eq:                      ; Z -> text in flash; compares with the received command name; r16 = 1 if equal")
        w("lds r26, RXNAME")
        w("lds r27, RXNAME+1")
        w("name_eq_l:")
        w("lpm r17, Z+")
        w("ld r16, X+")
        w("cp r16, r17")
        w("brne name_eq_n")
        w("tst r17")
        w("brne name_eq_l")
        w("ldi r16, 1")
        w("ret")
        w("name_eq_n:")
        w("clr r16")
        w("ret")
        self.lines.extend(HELPERS.strip("\n").split("\n"))
        for text, lab in self.strings.items():
            bs = list(text.encode("latin-1")) + [0]
            self.lines.append(f"{lab}:")
            self.lines.append("        .byte " + ", ".join(str(b) for b in bs))
        if self.next_ram > RAM_END - 96:
            raise AvrUnsupported([f"the program needs {self.next_ram - RAM_START} bytes of RAM; an Uno has 2048 and the stack needs room"])
        head = [f".equ {n}, {a}" for n, a in self.eq] + [f".equ RAM_USED, {self.next_ram}"]
        return "\n".join(head + self.lines) + "\n"

    def main_loop(self):
        w = self.w
        w("main_loop:")
        w("cli")
        for k in range(4):
            w(f"lds r16, NOW+{k}")
            w(f"sts NOWC+{k}, r16")
        w("sei")
        w("call rx_poll")
        for d in self.dbs:
            w(f"call r_sample_{d.name}")
        if self.hb:
            i, e = self.hb
            hb_ok, hb_no, hb_go = self.lab("Lh"), self.lab("Lh"), self.lab("Lh")
            w("lds r16, HBTRIP")
            w("tst r16")
            w(f"brne {hb_no}")
            self.load("NOWC")
            self.load("HBLAST", 18)
            w("call sub32")
            self.const(e.ms, 18)
            w("cp r22, r18")
            for k in (1, 2, 3):
                w(f"cpc r{22 + k}, r{18 + k}")
            self.br_far("brlo", hb_no)
            w("ldi r16, 1")
            w("sts HBTRIP, r16")
            w("call apply_safe")
            w(f"call h_{i}")
            w(f"{hb_no}:")
            w("lds r16, HBTRIP")
            w("tst r16")
            w(f"breq {hb_go}")
            w("jmp main_loop")
            w(f"{hb_go}:")
        for i, e in self.events:
            if e.kind == "when":
                self.when(i, e)
                continue
            if e.kind != "every":
                continue
            skip, plus, run = self.lab("Lk"), self.lab("Lk"), self.lab("Lk")
            if e.state is not None:
                w("lds r16, ST")
                w(f"cpi r16, {self.ck.state_index[e.state]}")
                self.br_far("brne", skip)
            self.load("NOWC")
            self.load(f"T_{i}", 18)
            w("call sub32")
            self.const(e.ms, 18)
            w("cp r22, r18")
            for k in (1, 2, 3):
                w(f"cpc r{22 + k}, r{18 + k}")
            self.br_far("brlo", skip)
            self.const(2 * e.ms, 18)
            w("cp r22, r18")
            for k in (1, 2, 3):
                w(f"cpc r{22 + k}, r{18 + k}")
            self.br_far("brlo", plus)
            self.load("NOWC")
            self.store(f"T_{i}")
            w(f"jmp {run}")
            w(f"{plus}:")
            self.load(f"T_{i}")
            self.const(e.ms, 18)
            w("call add32")
            self.store(f"T_{i}")
            w(f"{run}:")
            w(f"call h_{i}")
            w(f"{skip}:")
        w("jmp main_loop")

    def clamp(self, lo, hi):
        """A = min(max(A, lo), hi), signed."""
        ok1, ok2 = self.lab("Lm"), self.lab("Lm")
        self.const(lo, 18)
        self.w("cp r22, r18")
        for k in (1, 2, 3):
            self.w(f"cpc r{22 + k}, r{18 + k}")
        self.w(f"brge {ok1}")
        self.const(lo)
        self.w(f"{ok1}:")
        self.const(hi, 18)
        self.w("cp r18, r22")
        for k in (1, 2, 3):
            self.w(f"cpc r{18 + k}, r{22 + k}")
        self.w(f"brge {ok2}")
        self.const(hi)
        self.w(f"{ok2}:")

    def hw_pwm(self, d, port, bit):
        n, w = d.name, self.w
        timer, ocrh, ocrl, tccr, com = PWM_PINS[_pin(d.pin)]
        w(f"hw_{n}:")
        self.load(f"O_{n}")
        self.const(255, 18)
        w("call mul32")
        self.const(100, 18)
        w("call divs32")
        if d.invert:
            w("ldi r16, 255")
            w("eor r22, r16")
        w("tst r22")
        w(f"breq hw_{n}_0")
        w("cpi r22, 255")
        w(f"breq hw_{n}_f")
        if ocrh:
            w(f"sts {ocrh}, r2")
        w(f"sts {ocrl}, r22")
        w(f"lds r16, {tccr}")
        w(f"ori r16, {com}")
        w(f"sts {tccr}, r16")
        w("ret")
        for lab, op in (("0", "cbi"), ("f", "sbi")):
            w(f"hw_{n}_{lab}:")
            w(f"{op} {port}, {bit}")
            w(f"lds r16, {tccr}")
            w(f"andi r16, {(~com) & 0xFF}")
            w(f"sts {tccr}, r16")
            w("ret")

    def hw_servo(self, d):
        n, w = d.name, self.w
        w(f"hw_{n}:")
        self.load(f"O_{n}")
        self.const(1856, 18)
        w("call mul32")
        self.const(180, 18)
        w("call divs32")
        self.const(544, 18)
        w("call add32")
        w("lsl r22")                      # microseconds -> 0.5 us timer ticks
        w("rol r23")
        w("in r16, 0x3f")
        w("cli")
        w(f"sts SVW_{n}, r22")
        w(f"sts SVW_{n}+1, r23")
        w("out 0x3f, r16")
        w("ret")

    def servo_isrs(self):
        w, sv = self.w, self.servos
        w("isr_t1a:                      ; start of a servo slot: raise this servo's pin, schedule its end")
        for r in ("r16", "r17", "r30"):
            w(f"push {r}")
        w("in r16, 0x3f")
        w("push r16")
        w("lds r16, SVI")
        w("sts SVC, r16")
        done = self.lab("Lv")
        for k, d in enumerate(sv):
            nx = self.lab("Lv")
            port, _, bit = _port(_pin(d.pin))
            w(f"cpi r16, {k}")
            w(f"brne {nx}")
            w(f"sbi {port}, {bit}")
            w(f"lds r17, SVW_{d.name}+1")
            w(f"lds r30, SVW_{d.name}")
            w("sts 0x8B, r17")
            w("sts 0x8A, r30")
            w(f"rjmp {done}")
            w(f"{nx}:")
        w(f"{done}:")
        w("lds r16, SVI")
        w("inc r16")
        w(f"cpi r16, {len(sv)}")
        w("brlo isr_t1a_s")
        w("clr r16")
        w("isr_t1a_s:")
        w("sts SVI, r16")
        w("pop r16")
        w("out 0x3f, r16")
        for r in ("r30", "r17", "r16"):
            w(f"pop {r}")
        w("reti")
        w("isr_t1b:                      ; end of the pulse: lower the pin")
        w("push r16")
        w("in r16, 0x3f")
        w("push r16")
        w("lds r16, SVC")
        done = self.lab("Lv")
        for k, d in enumerate(sv):
            nx = self.lab("Lv")
            port, _, bit = _port(_pin(d.pin))
            w(f"cpi r16, {k}")
            w(f"brne {nx}")
            w(f"cbi {port}, {bit}")
            w(f"rjmp {done}")
            w(f"{nx}:")
        w(f"{done}:")
        w("pop r16")
        w("out 0x3f, r16")
        w("pop r16")
        w("reti")

    def when(self, i, e):
        w = self.w
        self.owner = f"w{i}"
        no, done = self.lab("Lc"), self.lab("Ld")
        if e.state is not None:
            w("lds r16, ST")
            w(f"cpi r16, {self.ck.state_index[e.state]}")
            self.br_far("brne", done)
        self.ex(self.low.expr(e.arg))
        self.test_a()
        self.br_far("breq", no)
        if e.dur_ms:
            have, fire = self.lab("Lc"), self.lab("Lc")
            w(f"lds r16, WO_{i}")
            w("tst r16")
            w(f"brne {have}")
            w("ldi r16, 1")
            w(f"sts WO_{i}, r16")
            self.load("NOWC")
            self.store(f"WT_{i}")
            w(f"{have}:")
            w(f"lds r16, W_{i}")
            w("tst r16")
            self.br_far("brne", done)
            self.load("NOWC")
            self.load(f"WT_{i}", 18)
            w("call sub32")
            self.const(e.dur_ms, 18)
            w("cp r22, r18")
            for k in (1, 2, 3):
                w(f"cpc r{22 + k}, r{18 + k}")
            self.br_far("brlo", done)
            w("ldi r16, 1")
            w(f"sts W_{i}, r16")
            w(f"call h_{i}")
            w(f"jmp {done}")
            w(f"{no}:")
            w(f"sts W_{i}, r2")
            w(f"sts WO_{i}, r2")
        else:
            w(f"lds r16, W_{i}")
            w("tst r16")
            self.br_far("brne", done)
            w(f"call h_{i}")
            w("ldi r16, 1")
            w(f"sts W_{i}, r16")
            w(f"jmp {done}")
            w(f"{no}:")
            w(f"sts W_{i}, r2")
        w(f"{done}:")

    def rx_code(self):
        w = self.w
        events = self.events
        w("rx_poll:")
        w("rp0:")
        w("lds r16, RXHEAD")
        w("lds r17, RXTAIL")
        w("cp r16, r17")
        w("brne rp1")
        w("ret")
        w("rp1:")
        w("ldi r30, lo8(RXRING)")
        w("ldi r31, hi8(RXRING)")
        w("add r30, r17")
        w("adc r31, r2")
        w("ld r16, Z")
        w("inc r17")
        w(f"andi r17, {RX_RING - 1}")
        w("sts RXTAIL, r17")
        w("cpi r16, 13")
        w("breq rp0")
        w("cpi r16, 10")
        w("breq rp_nl")
        w("lds r17, RXLEN")
        w(f"cpi r17, {RX_MAX - 1}")
        w("brsh rp_full")
        w("ldi r30, lo8(RXBUF)")
        w("ldi r31, hi8(RXBUF)")
        w("add r30, r17")
        w("adc r31, r2")
        w("st Z, r16")
        w("inc r17")
        w("sts RXLEN, r17")
        w("rjmp rp0")
        w("rp_full:")
        w("ldi r16, 1")
        w("sts RXDROP, r16")
        w("rjmp rp0")
        w("rp_nl:")
        w("lds r16, RXLEN")
        w("lds r17, RXDROP")
        w("tst r17")
        w("brne rp_reset")
        w("tst r16")
        w("breq rp_reset")
        w("ldi r30, lo8(RXBUF)")
        w("ldi r31, hi8(RXBUF)")
        w("add r30, r16")
        w("adc r31, r2")
        w("st Z, r2")
        w("call rx_exec")
        w("rp_reset:")
        w("sts RXLEN, r2")
        w("sts RXDROP, r2")
        w("rjmp rp0")

        w("rx_exec:")
        w("sts NA, r2")
        w("ldi r26, lo8(RXBUF)")
        w("ldi r27, hi8(RXBUF)")
        w("rx1:")
        w("ld r16, X")
        w("cpi r16, 32")
        w("brne rx2")
        w("adiw r26, 1")
        w("rjmp rx1")
        w("rx2:")
        w("sts RXNAME, r26")
        w("sts RXNAME+1, r27")
        w("rx3:")
        w("ld r16, X")
        w("tst r16")
        w("breq rx4")
        w("cpi r16, 32")
        w("breq rx4")
        w("adiw r26, 1")
        w("rjmp rx3")
        w("rx4:")
        w("tst r16")
        w("breq rx5")
        w("st X+, r2")
        w("rx5:")
        w("lds r30, RXNAME")
        w("lds r31, RXNAME+1")
        w("ld r16, Z")
        w("tst r16")
        w("brne rx6")
        w("ret")
        w("rx6:")
        if self.maxp:
            w("rx7:")
            w("ld r16, X")
            w("cpi r16, 32")
            w("brne rx8")
            w("adiw r26, 1")
            w("rjmp rx7")
            w("rx8:")
            w("ld r16, X")
            w("tst r16")
            w("breq rx_args_done")
            w("lds r17, NA")
            w(f"cpi r17, {self.maxp}")
            w("brsh rx_args_done")
            w("call parse_i32")
            w("lds r16, NA")
            w("mov r30, r16")
            w("clr r31")
            w("lsl r30")
            w("rol r31")
            w("lsl r30")
            w("rol r31")
            w("ldi r18, lo8(ARGS)")
            w("ldi r19, hi8(ARGS)")
            w("add r30, r18")
            w("adc r31, r19")
            w("st Z+, r22")
            w("st Z+, r23")
            w("st Z+, r24")
            w("st Z, r25")
            w("inc r16")
            w("sts NA, r16")
            w("rx9:")
            w("ld r16, X")
            w("tst r16")
            w("breq rx10")
            w("cpi r16, 32")
            w("breq rx10")
            w("adiw r26, 1")
            w("rjmp rx9")
            w("rx10:")
            w("tst r16")
            w("breq rx7")
            w("st X+, r2")
            w("rjmp rx7")
            w("rx_args_done:")

        def cmd(name, action):
            nxt, yes = self.lab("Lc"), self.lab("Lc")
            t = self.string(name)
            w(f"ldi r30, lo8({t}*2)")
            w(f"ldi r31, hi8({t}*2)")
            w("call name_eq")
            w("tst r16")
            w(f"brne {yes}")
            w(f"jmp {nxt}")
            w(f"{yes}:")
            action()
            w("ret")
            w(f"{nxt}:")

        def reply(text):
            t = self.string(text)
            w(f"ldi r30, lo8({t}*2)")
            w(f"ldi r31, hi8({t}*2)")
            w("call tx_pstr")

        def ping():
            w("call rt_heard")
            reply("PONG")
            w("call tx_crlf")

        def who():
            w("call rt_heard")
            reply("WHO ")
            self.const(protocol_hash(self.ck))
            w("call tx_i32")
            w("call tx_crlf")

        def status():
            w("call rt_heard")
            reply("STATUS ")
            self.load("BLOCKED")
            w("andi r25, 0x7F")
            w("call tx_i32")
            w("call tx_crlf")

        cmd("PING", ping)
        cmd("WHO", who)
        cmd("STATUS", status)
        if self.ck.states:
            def state_cmd():
                w("call rt_heard")
                reply("STATE ")
                w("lds r22, ST")
                w("clr r23")
                w("clr r24")
                w("clr r25")
                w("call tx_i32")
                w("call tx_crlf")
            cmd("STATE", state_cmd)
        seen = set()
        for i0, e0 in events:
            if e0.kind != "on" or e0.name in seen:
                continue
            seen.add(e0.name)
            hs = [(i, e) for i, e in events if e.kind == "on" and e.name == e0.name]
            n = len(e0.params)

            def act(hs=hs, n=n, e0=e0):
                if n:
                    ok = self.lab("Lc")
                    w("lds r16, NA")
                    w(f"cpi r16, {n}")
                    w(f"brsh {ok}")
                    reply("ERR")
                    w("call tx_crlf")
                    w("ret")
                    w(f"{ok}:")

                def run(i, e):
                    w("call rt_heard")
                    for k, p in enumerate(e.params):
                        self.load("ARGS", 22, 4 * k)
                        self.store(f"S_h{i}_{p}")
                    w(f"call h_{i}")
                    w("ret")
                for i, e in hs:
                    if e.state is None:
                        continue
                    sk = self.lab("Lc")
                    w("lds r16, ST")
                    w(f"cpi r16, {self.ck.state_index[e.state]}")
                    w(f"brne {sk}")
                    run(i, e)
                    w(f"{sk}:")
                glob = next(((i, e) for i, e in hs if e.state is None), None)
                if glob:
                    run(*glob)
                else:
                    reply("ERR")
                    w("call tx_crlf")
                    w("ret")

            cmd(e0.name, act)
        reply("ERR")
        w("call tx_crlf")
        w("ret")


def generate_asm(ck: Checked) -> str:
    return Gen(ck).generate()


def compile_hex(ck: Checked):
    from .asm import assemble, to_hex
    text = generate_asm(ck)
    return text, to_hex(assemble(text))
