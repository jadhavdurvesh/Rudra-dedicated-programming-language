"""Bytecode for rudra nodes: assembler, image format, and a reference VM (Python).

A *VM node* keeps everything that touches hardware or the protocol in firmware (pins, safe states,
`never` rules, timers, `when` conditions, the command set, the message names). Only the **bodies**
of handlers and functions are bytecode, so they can be replaced over the serial link without
reflashing. The image carries a *shape hash* of everything that stayed in firmware; an image built
for a different shape is refused.

    Image  = "RB" ver(1) shape(4) n_handlers(1) n_fns(1) stack(1) frame(2) calls(1) | tables | code | fnv(4)
    tables = per handler: off(2) nparams(1) nlocals(1) ; per fn: off(2) nparams(1) nlocals(1)
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from . import ir as I
from .util import fnv31, rt_div, rt_mod, wrap32

MAGIC = b"RB"
VERSION = 1

# opcodes ---------------------------------------------------------------
(PUSH8, PUSH32, LOADL, STOREL, LOADG, STOREG, LOADE, STOREE, IN, OUTR, SETO,
 ADD, SUB, MUL, DIV, MOD, NEG, NOT, LT, LE, GT, GE, EQ, NE,
 JMP, JZ, JZK, JNZK, CALL, RET, RETV, POP, GOTO, SEND, SENDN, FILL, PIXEL, ANIM,
 RUNP, STOPO, CANCEL, PRUN) = range(1, 43)

NAMES = {v: k for k, v in list(globals().items()) if k.isupper() and isinstance(v, int) and 1 <= v < 43}
OPERANDS = {PUSH8: 1, PUSH32: 4, LOADL: 1, STOREL: 1, LOADG: 1, STOREG: 1, LOADE: 1, STOREE: 1, IN: 1, OUTR: 1,
            SETO: 1, JMP: 2, JZ: 2, JZK: 2, JNZK: 2, CALL: 2, GOTO: 1, SEND: 1, SENDN: 1, FILL: 1, PIXEL: 1, ANIM: 2, RUNP: 1, STOPO: 1, CANCEL: 1, PRUN: 1}
BINOPS = {"+": ADD, "-": SUB, "*": MUL, "/": DIV, "%": MOD, "<": LT, "<=": LE, ">": GT, ">=": GE, "==": EQ, "!=": NE}


class VMError(Exception):
    pass


# ---------------------------------------------------------------- tables shared with the firmware
@dataclass
class Tables:
    slots: List[Tuple[str, str, int]]      # (slot name, "int"|"bool", size; 0 = scalar)
    ins: List[str]
    outs: List[str]
    strips: List[str]
    messages: List[str]
    states: List[str]
    paths: List[str]

    def slot(self, name):
        return next(i for i, s in enumerate(self.slots) if s[0] == name)


def tables(ck) -> Tables:
    slots = []
    for d in ck.node.decls:
        if d.__class__.__name__ == "Var":
            for slot, ty in d.slots:
                slots.append((slot, "bool" if ty == "bool" else "int", d.size if d.shape in ("array", "structarray") else 0))
    ios = [d for d in ck.node.decls if d.__class__.__name__ == "IO"]
    return Tables(slots,
                  [d.name for d in ios if d.direction == "input"],
                  [d.name for d in ios if d.direction == "output" and d.kind != "neopixel"],
                  [d.name for d in ios if d.direction == "output" and d.kind == "neopixel"],
                  list(ck.messages), list(ck.states), [p.name for p in ck.node.paths])


def shape_hash(ck) -> int:
    """Everything that stays in firmware. Two builds with the same shape accept each other's images."""
    from .link import protocol_hash
    low = I.Lowerer(ck)
    t = tables(ck)
    parts = [str(protocol_hash(ck)), repr(t), f"anim={ck.uses_anim}"]
    for e in ck.node.events:
        parts.append(f"{e.kind}|{e.name}|{e.params}|{e.ms}|{getattr(e, 'state', None)}|{getattr(e, 'dur_ms', None)}")
        if e.kind == "when":
            parts.append(I.fmt(low.expr(e.arg)))
    parts += [f"{f.name}|{[p[0] for p in f.params]}|{f.ptypes}|{f.ret}|{f.kind}" for f in ck.fns]
    parts += [I.fmt(low.expr(r.expr)) for r in ck.rules]
    for p in ck.node.paths:
        parts.append(f"path {p.name}|{p.out}|{p.kind}|{[(q.v, q.sp, q.zn, q.wt) for q in p.points]}|{p.repeat}|{p.forever}")
    for d in ck.node.decls:
        if d.__class__.__name__ == "IO":
            parts.append(f"{d.name}|{d.kind}|{d.direction}|{d.pins if hasattr(d, 'pins') else ''}|{sorted(d.cfg.items(), key=str)}")
        if d.__class__.__name__ == "Var":
            parts.append(f"var {d.name}|{d.type}|{d.shape}|{d.size}|{d.init}")
    return fnv31("\n".join(parts))


# ---------------------------------------------------------------- assembler
class _Asm:
    def __init__(self, ck, tb: Tables, low: I.Lowerer):
        self.ck, self.tb, self.low = ck, tb, low
        self.fn_index = {f.name: i for i, f in enumerate(ck.fns)}
        self.strip_index = {n: i for i, n in enumerate(tb.strips)}

    def unit(self, body, params, base):
        self.base = base
        self.code = bytearray()
        self.locals = {p: i for i, p in enumerate(params)}
        self.depth = self.maxd = 0
        self.breaks: List[List[int]] = []
        self.block(body)
        self.emit(RETV)
        return bytes(self.code), len(self.locals), self.maxd

    # -- helpers
    def emit(self, op, *args):
        self.code.append(op)
        n = OPERANDS.get(op, 0)
        if op == ANIM:
            self.code += bytes([args[0] & 0xFF, args[1] & 0xFF])
        elif n == 1:
            self.code.append(args[0] & 0xFF)
        elif n == 2:
            self.code += (args[0] & 0xFFFF).to_bytes(2, "little")
        elif n == 4:
            self.code += (args[0] & 0xFFFFFFFF).to_bytes(4, "little")

    def push(self, d=1):
        self.depth += d
        self.maxd = max(self.maxd, self.depth)

    def pc(self):
        return self.base + len(self.code)

    def jump(self, op):
        self.emit(op, 0)
        return len(self.code) - 2

    def patch(self, at):
        self.code[at:at + 2] = self.pc().to_bytes(2, "little")

    def lslot(self, name):
        if name not in self.locals:
            self.locals[name] = len(self.locals)
        return self.locals[name]

    # -- expressions (leave exactly one value on the stack)
    def ex(self, x):
        if isinstance(x, I.K):
            v = int(x.v)
            if -128 <= v <= 127:
                self.emit(PUSH8, v)
            else:
                self.emit(PUSH32, v)
            self.push()
        elif isinstance(x, I.Loc):
            self.emit(LOADL, self.locals[x.name]); self.push()
        elif isinstance(x, I.Glob):
            self.emit(LOADG, self.tb.slot(x.slot)); self.push()
        elif isinstance(x, I.In):
            self.emit(IN, self.tb.ins.index(x.name)); self.push()
        elif isinstance(x, I.Out):
            self.emit(OUTR, self.tb.outs.index(x.name)); self.push()
        elif isinstance(x, I.Running):
            self.emit(PRUN, self.tb.paths.index(x.name)); self.push()
        elif isinstance(x, I.Elem):
            self.ex(x.idx)
            self.emit(LOADE, self.tb.slot(x.slot))
        elif isinstance(x, I.Neg):
            self.ex(x.e); self.emit(NEG)
        elif isinstance(x, I.Not):
            self.ex(x.e); self.emit(NOT)
        elif isinstance(x, I.CallE):
            for a in x.args:
                self.ex(a)
            self.emit(CALL, self.fn_index[x.fn])
            self.depth -= len(x.args)
            if x.ty is not None:
                self.push()
        elif isinstance(x, I.Op):
            if x.op in ("&&", "||"):
                self.ex(x.l)
                j = self.jump(JZK if x.op == "&&" else JNZK)
                self.depth -= 1
                self.ex(x.r)
                self.patch(j)
            else:
                self.ex(x.l); self.ex(x.r)
                self.emit(BINOPS[x.op])
                self.depth -= 1
        else:
            raise VMError(f"cannot assemble {x}")

    # -- statements
    def block(self, body):
        for s in body:
            if isinstance(s, I.LetS):
                self.ex(s.e); self.emit(STOREL, self.lslot(s.name)); self.depth -= 1
            elif isinstance(s, I.SetL):
                self.ex(s.e); self.emit(STOREL, self.locals[s.name]); self.depth -= 1
            elif isinstance(s, I.SetG):
                self.ex(s.e); self.emit(STOREG, self.tb.slot(s.slot)); self.depth -= 1
            elif isinstance(s, I.SetE):
                self.ex(s.idx); self.ex(s.e); self.emit(STOREE, self.tb.slot(s.slot)); self.depth -= 2
            elif isinstance(s, I.SetO):
                self.ex(s.e); self.emit(SETO, self.tb.outs.index(s.name)); self.depth -= 1
            elif isinstance(s, I.Ret):
                if s.e is None:
                    self.emit(RETV)
                else:
                    self.ex(s.e); self.emit(RET); self.depth -= 1
            elif isinstance(s, I.Brk):
                self.breaks[-1].append(self.jump(JMP))
            elif isinstance(s, I.CallS):
                self.ex(s.call)
                if s.call.ty is not None:
                    self.emit(POP); self.depth -= 1
            elif isinstance(s, I.Loop):
                v = self.lslot(s.var)
                self.emit(PUSH32, s.lo); self.push(); self.emit(STOREL, v); self.depth -= 1
                top = self.pc()
                self.emit(LOADL, v); self.push(); self.emit(PUSH32, s.hi); self.push(); self.emit(LT); self.depth -= 1
                exit_ = self.jump(JZ); self.depth -= 1
                self.breaks.append([])
                self.block(s.body)
                self.emit(LOADL, v); self.push(); self.emit(PUSH8, 1); self.push(); self.emit(ADD); self.depth -= 1
                self.emit(STOREL, v); self.depth -= 1
                self.emit(JMP, top)
                self.patch(exit_)
                for b in self.breaks.pop():
                    self.patch(b)
            elif isinstance(s, I.IfS):
                self.ex(s.cond)
                jz = self.jump(JZ); self.depth -= 1
                self.block(s.then)
                if s.els:
                    je = self.jump(JMP)
                    self.patch(jz)
                    self.block(s.els)
                    self.patch(je)
                else:
                    self.patch(jz)
            elif isinstance(s, I.RunS):
                self.emit(RUNP, self.tb.paths.index(s.name))
            elif isinstance(s, I.StopS):
                self.emit(STOPO, self.tb.outs.index(s.out))
            elif isinstance(s, I.CancelS):
                self.emit(CANCEL, self.tb.outs.index(s.out))
            elif isinstance(s, I.GotoS):
                self.emit(GOTO, s.index)
            elif isinstance(s, I.SendS):
                mi = self.tb.messages.index(s.msg)
                if s.e is None:
                    self.emit(SENDN, mi)
                else:
                    self.ex(s.e); self.emit(SEND, mi); self.depth -= 1
            elif isinstance(s, I.FillS):
                for a in s.args:
                    self.ex(a)
                self.emit(FILL, self.strip_index[s.strip]); self.depth -= len(s.args)
            elif isinstance(s, I.PixelS):
                self.ex(s.idx)
                for a in s.args:
                    self.ex(a)
                self.emit(PIXEL, self.strip_index[s.strip]); self.depth -= 1 + len(s.args)
            elif isinstance(s, I.AnimS):
                for a in s.args:
                    self.ex(a)
                from .util import ANIM_MODES
                self.emit(ANIM, self.strip_index[s.strip], ANIM_MODES[s.mode] if s.mode != "off" else 255)
                self.depth -= len(s.args)
            else:
                raise VMError(f"cannot assemble {s}")


@dataclass
class Image:
    shape: int
    handlers: List[Tuple[int, int, int]]    # (offset, nparams, nlocals) per event index
    fns: List[Tuple[int, int, int]]
    code: bytes
    stack: int
    frame: int
    calls: int

    def encode(self) -> bytes:
        b = bytearray(MAGIC + bytes([VERSION]) + self.shape.to_bytes(4, "little"))
        b += bytes([len(self.handlers), len(self.fns), self.stack]) + self.frame.to_bytes(2, "little") + bytes([self.calls])
        for off, np_, nl in self.handlers + self.fns:
            b += off.to_bytes(2, "little") + bytes([np_, nl])
        b += len(self.code).to_bytes(2, "little") + self.code
        b += fnv31(bytes(b)).to_bytes(4, "little")
        return bytes(b)


def decode(data: bytes) -> Image:
    if len(data) < 18 or data[:2] != MAGIC:
        raise VMError("not a rudra image")
    if data[2] != VERSION:
        raise VMError("unsupported image version")
    if fnv31(data[:-4]) != int.from_bytes(data[-4:], "little"):
        raise VMError("image checksum does not match")
    shape = int.from_bytes(data[3:7], "little")
    nh, nf, stack = data[7], data[8], data[9]
    frame, calls = int.from_bytes(data[10:12], "little"), data[12]
    p = 13
    tab = []
    for _ in range(nh + nf):
        tab.append((int.from_bytes(data[p:p + 2], "little"), data[p + 2], data[p + 3]))
        p += 4
    n = int.from_bytes(data[p:p + 2], "little")
    code = data[p + 2:p + 2 + n]
    if p + 2 + n + 4 != len(data):
        raise VMError("image length is inconsistent")
    return Image(shape, tab[:nh], tab[nh:], bytes(code), stack, frame, calls)


def accept(data: bytes, shape: int, nh: int, nf: int, caps: dict, check: int) -> bool:
    """Same rules as the firmware's vm_accept(): anything less and the node keeps refusing to run the image."""
    n = len(data)
    if n < 18 or n > caps["cap"]:
        return False
    if data[:2] != MAGIC or data[2] != VERSION:
        return False
    h = fnv31(data[:-4])
    if h != int.from_bytes(data[-4:], "little") or h != check:
        return False
    if int.from_bytes(data[3:7], "little") != shape or data[7] != nh or data[8] != nf:
        return False
    if data[9] > caps["stack"] or int.from_bytes(data[10:12], "little") > caps["frame"] or data[12] > caps["calls"]:
        return False
    p = 13 + 4 * (nh + nf)
    if p + 2 + 4 > n:
        return False
    cl = int.from_bytes(data[p:p + 2], "little")
    if p + 2 + cl + 4 != n:
        return False
    for k in range(nh + nf):
        if int.from_bytes(data[13 + 4 * k:15 + 4 * k], "little") >= cl:
            return False
    return True


def assemble(ck) -> Image:
    low = I.Lowerer(ck)
    tb = tables(ck)
    a = _Asm(ck, tb, low)
    code = bytearray()
    handlers, fns = [], []
    stack = 1
    unit_frames = {}
    # every event handler first, then functions
    for e in ck.node.events:
        c, nl, md = a.unit(low.body(e.body, e.params), list(e.params), len(code))
        handlers.append((len(code), len(e.params), nl)); code += c
        stack = max(stack, md); unit_frames[("h", len(handlers) - 1)] = nl
    for f in ck.fns:
        c, nl, md = a.unit(low.fn_body(f.name), [p[0] for p in f.params], len(code))
        fns.append((len(code), len(f.params), nl)); code += c
        stack = max(stack, md); unit_frames[("f", f.name)] = nl
    # worst case: frames along the deepest call chain (acyclic, checked earlier) and eval stack summed likewise
    fn_nl = {f.name: unit_frames[("f", f.name)] for f in ck.fns}
    memo: Dict[str, Tuple[int, int]] = {}

    def depth(name):
        if name not in memo:
            memo[name] = (0, 0)
            best_f, best_c = 0, 0
            for c in set(I.callees(low.fn_body(name))):
                f_, c_ = depth(c)
                best_f, best_c = max(best_f, f_), max(best_c, c_)
            memo[name] = (fn_nl[name] + best_f, 1 + best_c)
        return memo[name]

    frame, calls = 0, 1
    for k, e in enumerate(ck.node.events):
        bf, bc = 0, 0
        for c in set(I.callees(low.body(e.body, e.params))):
            f_, c_ = depth(c)
            bf, bc = max(bf, f_), max(bc, c_)
        frame = max(frame, unit_frames[("h", k)] + bf)
        calls = max(calls, 1 + bc)
    # eval stack: each active frame may hold up to `stack` values
    return Image(shape_hash(ck), handlers, fns, bytes(code), stack * calls, frame, calls)


def push_lines(data: bytes, chunk: int = 8) -> List[str]:
    """The serial commands that load an image: IMG <offset> <hex> ..., then IMGEND <length> <checksum>."""
    L = [f"IMG {k} {data[k:k + chunk].hex()}" for k in range(0, len(data), chunk)]
    L.append(f"IMGEND {len(data)} {int.from_bytes(data[-4:], 'little')}")
    return L


def disassemble(img: Image) -> str:
    L = []
    starts = {off: f"handler {k}" for k, (off, _, _) in enumerate(img.handlers)}
    starts.update({off: f"fn {k}" for k, (off, _, _) in enumerate(img.fns)})
    p = 0
    c = img.code
    while p < len(c):
        if p in starts:
            L.append(f"{starts[p]}:")
        op = c[p]
        n = OPERANDS.get(op, 0)
        arg = ""
        if op == ANIM:
            arg = f" {c[p + 1]} {c[p + 2]}"; n = 2
        elif n == 1:
            arg = f" {c[p + 1]}"
        elif n == 2:
            arg = f" {int.from_bytes(c[p + 1:p + 3], 'little')}"
        elif n == 4:
            v = int.from_bytes(c[p + 1:p + 5], "little")
            arg = f" {v - (1 << 32) if v >= 1 << 31 else v}"
        if op == PUSH8:
            arg = f" {c[p + 1] - 256 if c[p + 1] >= 128 else c[p + 1]}"
        L.append(f"  {p:4d}  {NAMES[op]}{arg}")
        p += 1 + n
    return "\n".join(L)


# ---------------------------------------------------------------- reference VM
class Host:
    """What the VM needs from the node; the simulator implements it, the C++ firmware has the same shape."""

    def gget(self, slot, idx): raise NotImplementedError
    def gset(self, slot, idx, v): raise NotImplementedError
    def inp(self, i): raise NotImplementedError
    def outr(self, i): raise NotImplementedError
    def seto(self, i, v): raise NotImplementedError
    def goto(self, s): raise NotImplementedError
    def send(self, m, v): raise NotImplementedError
    def fill(self, strip, args): raise NotImplementedError
    def pixel(self, strip, idx, args): raise NotImplementedError
    def anim(self, strip, mode, args): raise NotImplementedError
    def bad_index(self, i): raise NotImplementedError
    def runp(self, i): raise NotImplementedError
    def stopo(self, i): raise NotImplementedError
    def cancel(self, i): raise NotImplementedError
    def prun(self, i): raise NotImplementedError


class VM:
    def __init__(self, img: Image, tb: Tables, host: Host):
        self.img, self.tb, self.host = img, tb, host

    def run(self, handler: int, args: List[int]):
        return self._call(self.img.handlers[handler], args)

    def _call(self, unit, args):
        off, np_, nl = unit
        img, tb, host = self.img, self.tb, self.host
        loc = list(args[:np_]) + [0] * (nl - np_)
        code = img.code
        st: List[int] = []
        pc = off
        while True:
            op = code[pc]
            pc += 1
            if op == PUSH8:
                v = code[pc]; pc += 1
                st.append(v - 256 if v >= 128 else v)
            elif op == PUSH32:
                v = int.from_bytes(code[pc:pc + 4], "little"); pc += 4
                st.append(v - (1 << 32) if v >= 1 << 31 else v)
            elif op == LOADL:
                st.append(loc[code[pc]]); pc += 1
            elif op == STOREL:
                loc[code[pc]] = st.pop(); pc += 1
            elif op == LOADG:
                st.append(host.gget(code[pc], None)); pc += 1
            elif op == STOREG:
                host.gset(code[pc], None, st.pop()); pc += 1
            elif op == LOADE:
                i = st.pop(); s = code[pc]; pc += 1
                size = tb.slots[s][2]
                st.append(host.gget(s, i) if 0 <= i < size else 0)
            elif op == STOREE:
                v = st.pop(); i = st.pop(); s = code[pc]; pc += 1
                if 0 <= i < tb.slots[s][2]:
                    host.gset(s, i, v)
                else:
                    host.bad_index(i)
            elif op == IN:
                st.append(host.inp(code[pc])); pc += 1
            elif op == OUTR:
                st.append(host.outr(code[pc])); pc += 1
            elif op == SETO:
                host.seto(code[pc], st.pop()); pc += 1
            elif op in (ADD, SUB, MUL):
                b, a = st.pop(), st.pop()
                st.append(wrap32(a + b if op == ADD else a - b if op == SUB else a * b))
            elif op == DIV:
                b, a = st.pop(), st.pop(); st.append(rt_div(a, b))
            elif op == MOD:
                b, a = st.pop(), st.pop(); st.append(rt_mod(a, b))
            elif op == NEG:
                st.append(wrap32(-st.pop()))
            elif op == NOT:
                st.append(0 if st.pop() else 1)
            elif op in (LT, LE, GT, GE, EQ, NE):
                b, a = st.pop(), st.pop()
                st.append(int({LT: a < b, LE: a <= b, GT: a > b, GE: a >= b, EQ: a == b, NE: a != b}[op]))
            elif op == JMP:
                pc = int.from_bytes(code[pc:pc + 2], "little")
            elif op == JZ:
                t = int.from_bytes(code[pc:pc + 2], "little"); pc += 2
                if not st.pop():
                    pc = t
            elif op == JZK:
                t = int.from_bytes(code[pc:pc + 2], "little"); pc += 2
                if not st[-1]:
                    pc = t
                else:
                    st.pop()
            elif op == JNZK:
                t = int.from_bytes(code[pc:pc + 2], "little"); pc += 2
                if st[-1]:
                    pc = t
                else:
                    st.pop()
            elif op == CALL:
                k = int.from_bytes(code[pc:pc + 2], "little") & 0xFF; pc += 2
                unit2 = img.fns[k]
                n = unit2[1]
                a = st[len(st) - n:] if n else []
                del st[len(st) - n:]
                r = self._call(unit2, a)
                if r is not None:
                    st.append(r)
            elif op == RET:
                v = st.pop()
                if st:
                    raise VMError("unbalanced stack at return (assembler bug)")
                return v
            elif op == RETV:
                if st:
                    raise VMError("unbalanced stack at return (assembler bug)")
                return None
            elif op == POP:
                st.pop()
            elif op == GOTO:
                host.goto(code[pc]); pc += 1
            elif op == RUNP:
                host.runp(code[pc]); pc += 1
            elif op == STOPO:
                host.stopo(code[pc]); pc += 1
            elif op == CANCEL:
                host.cancel(code[pc]); pc += 1
            elif op == PRUN:
                st.append(1 if host.prun(code[pc]) else 0); pc += 1
            elif op == SEND:
                host.send(code[pc], st.pop()); pc += 1
            elif op == SENDN:
                host.send(code[pc], None); pc += 1
            elif op == FILL:
                a = st[-3:]; del st[-3:]; host.fill(code[pc], a); pc += 1
            elif op == PIXEL:
                a = st[-4:]; del st[-4:]; host.pixel(code[pc], a[0], a[1:]); pc += 1
            elif op == ANIM:
                strip, mode = code[pc], code[pc + 1]; pc += 2
                n = 1 if mode == ANIM_RAINBOW else (0 if mode == 255 else 4)
                a = st[len(st) - n:] if n else []
                if n:
                    del st[len(st) - n:]
                host.anim(strip, mode, a)
            else:
                raise VMError(f"bad opcode {op} at {pc - 1}")


from .util import ANIM_MODES as _AM  # noqa: E402
ANIM_RAINBOW = _AM["rainbow"]
