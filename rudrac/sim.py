"""PC simulator. Mirrors the generated C++ loop step for step (1 ms ticks)."""
from __future__ import annotations

import re
from typing import List, Tuple

from .check import Checked
from .link import protocol_hash
from . import ir as I
from .syntax import (IO, Never, Var, fmt_expr)
from .util import (US_PERIOD, ANIM_MAX_PERIOD, ANIM_MIN_PERIOD, ANIM_MODES, ANIM_STEP_MS, anim_frame,
                   i2c_decode, pack_rgb, rt_div, rt_mod, wrap32)

_STRTOL = re.compile(r"^[ \t]*([+-]?\d+)")
_ADDR = re.compile(r"@([+-]?[0-9]+)")


def parse_time(text: str) -> int:
    m = re.fullmatch(r"(\d+)(ms|s)", text.strip())
    if not m:
        raise ValueError(f"bad time '{text}' (use 500ms or 2s)")
    return int(m.group(1)) * (1000 if m.group(2) == "s" else 1)


class Sim:
    """phys entries mirror what the hardware would see, in this form:
       ("W", t, pin, value)            digitalWrite / analogWrite
       ("S", t, pin, angle)            servo.write
       ("I", t, addr, "hexbytes")      completed I2C write
       ("N", t, c0, c1, ...)           neopixel show() with packed 0xRRGGBB pixels
    """

    def __init__(self, ck: Checked):
        self.ck = ck
        self.node = ck.node
        self.syms = ck.syms
        self.rules: List[Never] = ck.rules
        self.now = 0
        self.vars, self.out, self.inp = {}, {}, {}
        self.pixels, self.dirty, self.an = {}, {}, {}
        self.log: List[Tuple[int, str]] = []
        self.phys: List[tuple] = []
        self.tx: List[str] = []
        self.tx_log: List[Tuple[int, str]] = []
        self.blocked = 0
        self.oob = 0                           # out-of-range array writes (counted in STATUS with blocked)
        self.blocked_log: List[tuple] = []     # (t, output, refused value, rule number starting at 1)
        self.ios = {}
        for d in self.node.decls:
            if isinstance(d, Var):
                for slot, ty in d.slots:
                    z = False if ty == "bool" else 0
                    self.vars[slot] = d.init if d.shape == "scalar" else (z if d.shape == "struct" else [z] * d.size)
            elif isinstance(d, IO):
                self.ios[d.name] = d
                if d.direction == "input":
                    self.inp[d.name] = False if d.kind == "digital" else 0
                elif d.kind == "neopixel":
                    self.pixels[d.name] = [0] * d.cfg["count"]
                    self.dirty[d.name] = False
                    self.an[d.name] = {"mode": 0, "c": (0, 0, 0), "per": 1000, "t0": 0}
                else:
                    self.out[d.name] = False if d.kind == "digital" else 0
        # v0.4 state
        self.cur = {n: 0 for n, d in self.ios.items() if d.kind == "motor"}      # applied motor speed
        self.m_last = {n: 0 for n in self.cur}
        self.sp = {n: 0 for n, d in self.ios.items() if d.kind == "stepper"}     # stepper position
        self.t_stp = {n: 0 for n in self.sp}
        self.db = {n: {"v": False, "c": False, "t": 0} for n, d in self.ios.items()
                   if d.direction == "input" and d.kind == "digital" and d.cfg.get("debounce")}
        self.us = {n: {"v": 0, "t": 0} for n, d in self.ios.items() if d.kind == "ultrasonic"}
        self.fn_by_name = {f.name: f for f in ck.fns}
        self.low = I.Lowerer(ck)
        self._lxc = {}
        self.state_names = list(ck.states)
        self.st = 0
        self.bb_on = self.node.blackbox_pos is not None
        self.bb: List[tuple] = []          # ring buffer of (t, code, arg), oldest first
        self.bb_cap = self.node.blackbox
        self.bb_total = 0
        self.quiet = False
        self.bus = ck.bus
        self.need_mstep = any(d.kind == "motor" and (d.cfg.get("ramp") or d.cfg.get("timeout")) for d in self.ios.values())
        self.t_mot = 0
        self.bus_ok = {n: True for n, d in self.ios.items() if d.kind == "i2c" and d.direction == "input"}
        self.last_good = {n: 0 for n in self.bus_ok}
        self.maxp = max([len(e.params) for e in self.node.events if e.kind == "on"] or [0])
        self.events = list(enumerate(self.node.events))
        self.hb = next((e for _, e in self.events if e.kind == "heartbeat" and not e.off), None)
        self.hb_last, self.hb_tripped = 0, False
        self.t_every = {i: 0 for i, e in self.events if e.kind == "every"}
        self.w_prev = {i: False for i, e in self.events if e.kind == "when"}
        self.w_on = {i: False for i, e in self.events if e.kind == "when"}
        self.w_t = {i: 0 for i, e in self.events if e.kind == "when"}
        self.t_anim = 0
        self.proto_hash = protocol_hash(ck)
        self._setup()

    # ---- setup / loop (same order as the generated C++)
    def _apply_safe(self):
        """Store every safe value first, then write the hardware, like the generated r_apply_safe()."""
        plain = [d for d in self.node.decls if isinstance(d, IO) and d.direction == "output" and d.kind != "neopixel"]
        for d in plain:
            sv = d.cfg["safe"]
            if self.out[d.name] != sv:
                self.log.append((self.now, f"{d.name} = {int(sv) if isinstance(sv, bool) else sv}"))
            self.out[d.name] = sv
        for d in plain:
            if d.kind == "motor":
                self.cur[d.name] = d.cfg["safe"]
        for d in plain:
            if d.kind != "stepper":
                self._hw(d.name)
        for d in self.node.decls:
            if isinstance(d, IO) and d.direction == "output" and d.kind == "neopixel":
                self._fill(d.name, *d.cfg["safe"])

    def _setup(self):
        if self.node.rs485_pin is not None:
            self.phys.append(("W", self.now, self.node.rs485_pin.key, 0))
        for n, st in self.db.items():
            st["v"] = st["c"] = bool(self.inp[n])
        self._apply_safe()
        self._bb(1, 0)
        for _, e in self.events:
            if e.kind == "start":
                self._run(e.body, {})
        for _, e in self.events:
            if e.kind == "enter" and e.state == (self.state_names[0] if self.state_names else None):
                self._run(e.body, {})
        self._flush()

    # ---- black box / states
    def _bb(self, code, arg):
        if not self.bb_on:
            return
        self.bb.append((self.now, code, arg))
        if len(self.bb) > self.bb_cap:
            self.bb.pop(0)
        self.bb_total += 1

    def _goto(self, name):
        k = self.ck.state_index[name]
        if k == self.st:
            return
        old = self.state_names[self.st]
        for _, e in self.events:
            if e.kind == "exit" and e.state == old:
                self._run(e.body, {})
        self.st = k
        for i, e in self.events:
            if e.state == name:
                if e.kind == "every":
                    self.t_every[i] = self.now
                elif e.kind == "when":
                    self.w_prev[i] = False
                    self.w_on[i] = False
        self._bb(3, k)
        for _, e in self.events:
            if e.kind == "enter" and e.state == name:
                self._run(e.body, {})

    def _sonar(self, now):
        for n, u in self.us.items():
            if now - u["t"] < US_PERIOD:
                continue
            u["t"] = now if now - u["t"] >= 2 * US_PERIOD else u["t"] + US_PERIOD
            pins = self.ios[n].cfg["pins"]
            for lv in (0, 1, 0):
                self.phys.append(("W", now, pins["trig"].key, lv))
            u["v"] = max(0, int(self.inp[n]))

    def _active(self, e):
        return e.state is None or self.state_names[self.st] == e.state

    def _sample(self, now):
        for n, d in self.db.items():
            raw = bool(self.inp[n])
            if raw == d["v"]:
                d["c"] = raw
            elif raw != d["c"]:
                d["c"], d["t"] = raw, now
            elif now - d["t"] >= self.ios[n].cfg["debounce"]:
                d["v"] = raw

    def tick(self, now: int, lines=()):
        self.now = now
        for line in lines:
            self._rx_line(line)
        self._sample(now)
        self._sonar(now)
        if self.hb is not None:
            if not self.hb_tripped and (now - self.hb_last) >= self.hb.ms:
                self.hb_tripped = True
                self._apply_safe()
                self._bb(2, 0)
                self._run(self.hb.body, {})
            if self.hb_tripped:
                self._steppers(now)
                self._flush()
                return
        for i, e in self.events:
            if e.kind not in ("every", "when") or not self._active(e):
                continue
            if e.kind == "every":
                el = now - self.t_every[i]
                if el >= e.ms:
                    self.t_every[i] = now if el >= 2 * e.ms else self.t_every[i] + e.ms
                    self._run(e.body, {})
            else:
                c = bool(self._ev(e.arg, {}))
                if e.dur_ms:
                    if not c:
                        self.w_prev[i] = False
                        self.w_on[i] = False
                    else:
                        if not self.w_on[i]:
                            self.w_on[i], self.w_t[i] = True, now
                        if not self.w_prev[i] and now - self.w_t[i] >= e.dur_ms:
                            self.w_prev[i] = True
                            self._run(e.body, {})
                else:
                    if c and not self.w_prev[i]:
                        self._run(e.body, {})
                    self.w_prev[i] = c
        if self.ck.uses_anim:
            el = now - self.t_anim
            if el >= ANIM_STEP_MS:
                self.t_anim = now if el >= 2 * ANIM_STEP_MS else self.t_anim + ANIM_STEP_MS
                self._anim_step(now)
        if self.need_mstep:
            el = now - self.t_mot
            if el >= ANIM_STEP_MS:
                self.t_mot = now if el >= 2 * ANIM_STEP_MS else self.t_mot + ANIM_STEP_MS
                self._motors(now)
        self._steppers(now)
        self._flush()

    def _motors(self, now):
        k = 0
        for name, d in self.ios.items():
            if d.kind != "motor":
                continue
            k += 1
            c = d.cfg
            if not (c.get("ramp") or c.get("timeout")):
                continue
            sv = c["safe"]
            if c.get("timeout") and (self.out[name] != sv or self.cur[name] != sv) \
                    and now - self.m_last[name] >= c["timeout"]:
                self.out[name] = self.cur[name] = sv
                self._hw(name)
                self._bb(5, k)
                self.log.append((now, f"{name} timed out -> {sv}"))
            elif c.get("ramp") and self.cur[name] != self.out[name]:
                cur, tgt, step = self.cur[name], self.out[name], c["ramp_step"]
                diff = tgt - cur
                nx = cur + step if diff > step else (cur - step if diff < -step else tgt)
                if (cur > 0 and nx < 0) or (cur < 0 and nx > 0):
                    nx = 0
                self.cur[name] = nx
                self._hw(name)

    def _steppers(self, now):
        for name, d in self.ios.items():
            if d.kind != "stepper":
                continue
            iv = d.cfg["interval"]
            if now - self.t_stp[name] >= iv:
                self.t_stp[name] = now if now - self.t_stp[name] >= 2 * iv else self.t_stp[name] + iv
                if self.sp[name] != self.out[name]:
                    fw = self.out[name] > self.sp[name]
                    pins = d.cfg["pins"]
                    self.phys.append(("W", now, pins["dir"].key, 1 if fw else 0))
                    self.phys.append(("W", now, pins["step"].key, 1))
                    self.phys.append(("W", now, pins["step"].key, 0))
                    self.sp[name] += 1 if fw else -1

    def run(self, end_ms: int, events=()):
        """events: (ms,'set',name,value) | (ms,'rx',line,None) | (ms,'fail',i2c_input,True/False)."""
        pending = sorted(events, key=lambda x: x[0])
        k = 0
        for t in range(end_ms + 1):
            lines = []
            while k < len(pending) and pending[k][0] <= t:
                _, kind, a, b = pending[k]
                if kind == "set":
                    self.set_input(a, b)
                elif kind == "fail":
                    self.set_bus(a, not b)
                else:
                    lines.append(a)
                k += 1
            self.tick(t, lines)

    def set_bus(self, name, ok):
        if name not in self.bus_ok:
            raise KeyError(f"'{name}' is not an i2c input of this node")
        self.bus_ok[name] = bool(ok)

    def set_input(self, name, value):
        if name not in self.inp:
            raise KeyError(f"'{name}' is not an input of this node")
        d = self.ios[name]
        if d.kind == "digital":
            self.inp[name] = bool(value)
        elif d.kind == "ultrasonic":
            self.inp[name] = max(0, wrap32(int(value)))
        elif d.kind == "i2c":
            self.inp[name] = i2c_decode(int(value), d.cfg["bytes"], d.cfg["signed"])
        else:
            self.inp[name] = wrap32(int(value))

    # ---- serial
    def _rx_line(self, line: str):
        if len(line) > 31 or not line:
            return
        if self.bus:
            tok0, _, rest = line.lstrip(" ").partition(" ")
            m = _ADDR.fullmatch(tok0)
            if not m:
                return
            n = int(m.group(1))
            if n != self.node.address and n != 0:
                return
            self.quiet = n == 0
            try:
                self._rx_exec(rest)
            finally:
                self.quiet = False
            return
        self._rx_exec(line)

    def _rx_exec(self, line: str):
        parts = line.lstrip(" ").split(" ")
        name = parts[0]
        if not name:
            return
        toks = [t for t in parts[1:] if t][: self.maxp]
        args = []
        for t in toks:
            m = _STRTOL.match(t)
            args.append(wrap32(int(m.group(1))) if m else 0)
        if name == "PING":
            self._heard()
            self._tx("PONG")
            return
        if name == "WHO":
            self._heard()
            self._tx(f"WHO {self.proto_hash}")
            return
        if name == "STATUS":
            self._heard()
            self._tx(f"STATUS {(self.blocked + self.oob) & 0x7FFFFFFF}")
            return
        if name == "STATE" and self.state_names:
            self._heard()
            self._tx(f"STATE {self.st}")
            return
        if name == "LOG" and self.bb_on:
            self._heard()
            for t, code, arg in self.bb:
                self._tx(f"LOG {t & 0x7FFFFFFF} {code} {arg}")
            self._tx(f"LOGEND {self.bb_total & 0x7FFFFFFF}")
            return
        hs = [e for _, e in self.events if e.kind == "on" and e.name == name]
        if hs:
            if len(args) < len(hs[0].params):
                self._tx("ERR")
                return
            pick = next((e for e in hs if e.state is not None and e.state == self.state_names[self.st]), None)
            if pick is None:
                pick = next((e for e in hs if e.state is None), None)
            if pick is None:
                self._tx("ERR")
                return
            self._heard()
            self._run(pick.body, dict(zip(pick.params, args)))
            return
        self._tx("ERR")

    def _heard(self):
        if self.hb is not None:
            self.hb_last, self.hb_tripped = self.now, False

    def _tx(self, text):
        if self.quiet:
            return
        de = self.node.rs485_pin
        if de is not None:
            self.phys.append(("W", self.now, de.key, 1))
        if self.bus:
            text = f"@{self.node.address} {text}"
        self.tx.append(text)
        self.tx_log.append((self.now, text))
        self.log.append((self.now, f"TX  {text}"))
        if de is not None:
            self.phys.append(("W", self.now, de.key, 0))

    # ---- outputs
    def violated(self):
        """Number (from 1) of the first interlock rule that is true right now, or None."""
        for k, r in enumerate(self.rules, 1):
            if self._ev(r.expr, {}):
                return k
        return None

    def _hw(self, name):
        d, v = self.ios[name], self.out[name]
        if d.kind == "stepper":
            return
        if d.kind == "digital":
            self.phys.append(("W", self.now, d.pin.key, 1 if (v != d.invert) else 0))
        elif d.kind == "pwm":
            duty = (v * 255) // 100
            self.phys.append(("W", self.now, d.pin.key, 255 - duty if d.invert else duty))
        elif d.kind == "servo":
            self.phys.append(("S", self.now, d.pin.key, v))
        elif d.kind == "tone":
            self.phys.append(("T", self.now, d.pin.key, v))
        elif d.kind == "motor":
            c = self.cur[name]
            duty = (abs(c) * 255) // 100
            pins = d.cfg["pins"]
            if d.cfg["style"] == "dir":
                self.phys.append(("W", self.now, pins["dir"].key, 0 if c < 0 else 1))
                self.phys.append(("W", self.now, pins["pwm"].key, duty))
            elif c >= 0:
                self.phys.append(("W", self.now, pins["rev"].key, 0))
                self.phys.append(("W", self.now, pins["fwd"].key, duty))
            else:
                self.phys.append(("W", self.now, pins["fwd"].key, 0))
                self.phys.append(("W", self.now, pins["rev"].key, duty))
        else:  # i2c
            n = d.cfg["bytes"]
            data = ([d.cfg["reg"]] if d.cfg["reg"] is not None else []) + \
                   [((v & 0xFFFFFFFF) >> (8 * k)) & 0xFF for k in reversed(range(n))]
            self.phys.append(("I", self.now, d.cfg["addr"], "".join(f"{b:02x}" for b in data)))

    def _set_out(self, name, v):
        """Checked setter: a write that would make any 'never' rule true is refused."""
        d = self.ios[name]
        if d.kind == "digital":
            v = bool(v)
        elif d.kind == "pwm":
            v = max(0, min(100, v))
        elif d.kind == "servo":
            v = max(0, min(180, v))
        elif d.kind == "tone":
            v = max(0, min(20000, v))
        elif d.kind == "motor":
            v = max(-d.cfg["limit"], min(d.cfg["limit"], v))
            self.m_last[name] = self.now
        old = self.out[name]
        self.out[name] = v
        rule = self.violated()
        if rule is not None:
            self.out[name] = old
            self.blocked += 1
            self._bb(4, rule)
            self.blocked_log.append((self.now, name, v, rule))
            self.log.append((self.now, f"BLOCKED {name} = {int(v) if isinstance(v, bool) else v}"
                                       f"   (never {fmt_expr(self.rules[rule - 1].expr)})"))
            return
        if d.kind == "stepper":
            if old != v:
                self.log.append((self.now, f"{name} -> position {v}"))
            return
        if d.kind == "motor":
            if not d.cfg.get("ramp"):
                self.cur[name] = v
                self._hw(name)
            if old != v:
                self.log.append((self.now, f"{name} = {v}"))
            return
        self._hw(name)
        if old != v:
            self.log.append((self.now, f"{name} = {int(v) if isinstance(v, bool) else v}"))

    # ---- neopixel
    def _cancel_anim(self, name):
        self.an[name]["mode"] = 0

    def _fill(self, name, r, g, b):
        self._cancel_anim(name)
        self.pixels[name] = [pack_rgb(r, g, b)] * len(self.pixels[name])
        self.dirty[name] = True

    def _pixel(self, name, i, r, g, b):
        self._cancel_anim(name)
        if 0 <= i < len(self.pixels[name]):
            self.pixels[name][i] = pack_rgb(r, g, b)
            self.dirty[name] = True

    def _animate(self, name, mode, r, g, b, per):
        per = max(ANIM_MIN_PERIOD, min(ANIM_MAX_PERIOD, per))
        c = tuple(max(0, min(255, x)) for x in (r, g, b))
        self.an[name].update(mode=mode, c=c, per=per, t0=self.now)
        self.pixels[name] = anim_frame(mode, len(self.pixels[name]), 0, per, c)
        self.dirty[name] = True

    def _anim_step(self, now):
        for name, a in self.an.items():
            if a["mode"]:
                self.pixels[name] = anim_frame(a["mode"], len(self.pixels[name]),
                                               (now - a["t0"]) & 0xFFFFFFFF, a["per"], a["c"])
                self.dirty[name] = True

    def _flush(self):
        for d in self.node.decls:
            if isinstance(d, IO) and d.kind == "neopixel" and d.direction == "output" and self.dirty[d.name]:
                self.dirty[d.name] = False
                px = list(self.pixels[d.name])
                self.phys.append(("N", self.now, *px))
                shown = " ".join(f"{c:06x}" for c in px[:8]) + (" ..." if len(px) > 8 else "")
                self.log.append((self.now, f"{d.name}: {shown}"))

    # ---- interpreter (runs the shared IR, see ir.py)
    def _lx(self, e):
        k = id(e)
        if k not in self._lxc:
            self._lxc[k] = (e, self.low.expr(e))
        return self._lxc[k][1]

    def _run(self, ast_body, args):
        return self._exec(self.low.body(ast_body, tuple(args)), args)

    def _ev(self, e, args):
        if not isinstance(e, I.EXPR):                  # still an AST expression (rules, 'when' conditions)
            e = self._lx(e)
        if isinstance(e, I.K):
            return e.v
        if isinstance(e, I.Loc):
            return args[e.name]
        if isinstance(e, I.Glob):
            return self.vars[e.slot]
        if isinstance(e, I.Elem):
            i = self._ev(e.idx, args)
            if e.proven or 0 <= i < e.size:
                return self.vars[e.slot][i]
            return False if e.ty == "bool" else 0
        if isinstance(e, I.In):
            n = e.name
            if n in self.db:
                return self.db[n]["v"]
            if n in self.us:
                return self.us[n]["v"]
            if n in self.bus_ok:                 # like the C++: a failed read returns the last good value
                if self.bus_ok[n]:
                    self.last_good[n] = self.inp[n]
                return self.last_good[n]
            return self.inp[n]
        if isinstance(e, I.Out):
            return self.out[e.name]
        if isinstance(e, I.CallE):
            return self._call(e, args)
        if isinstance(e, I.Neg):
            return wrap32(-self._ev(e.e, args))
        if isinstance(e, I.Not):
            return not self._ev(e.e, args)
        if isinstance(e, I.Op):
            op = e.op
            if op == "&&":
                return bool(self._ev(e.l, args)) and bool(self._ev(e.r, args))
            if op == "||":
                return bool(self._ev(e.l, args)) or bool(self._ev(e.r, args))
            a, b = self._ev(e.l, args), self._ev(e.r, args)
            if op == "+":
                return wrap32(a + b)
            if op == "-":
                return wrap32(a - b)
            if op == "*":
                return wrap32(a * b)
            if op == "/":
                return rt_div(a, b)
            if op == "%":
                return rt_mod(a, b)
            return {"<": a < b, "<=": a <= b, ">": a > b, ">=": a >= b,
                    "==": a == b, "!=": a != b}[op]
        raise AssertionError(e)

    def _call(self, call, args):
        fn = self.fn_by_name[call.fn]
        env = {p[0]: self._ev(a, args) for p, a in zip(fn.params, call.args)}
        r = self._exec(self.low.fn_body(call.fn), env)
        return r[1] if isinstance(r, tuple) else None

    def _exec(self, body, args):
        """Runs an IR block. Returns None, "B" (break) or ("R", value) (return)."""
        for s in body:
            if isinstance(s, I.LetS):
                args[s.name] = self._ev(s.e, args)
            elif isinstance(s, I.Ret):
                return ("R", self._ev(s.e, args) if s.e is not None else None)
            elif isinstance(s, I.Brk):
                return "B"
            elif isinstance(s, I.CallS):
                self._call(s.call, args)
            elif isinstance(s, I.Loop):
                for k in range(s.lo, s.hi):
                    args[s.var] = k
                    r = self._exec(s.body, args)
                    if r == "B":
                        break
                    if r is not None:
                        return r
            elif isinstance(s, I.SetE):
                i = self._ev(s.idx, args)
                v = self._ev(s.e, args)
                if 0 <= i < s.size:
                    self.vars[s.slot][i] = v
                else:
                    self.oob += 1
                    self._bb(6, i)
            elif isinstance(s, I.SetG):
                self.vars[s.slot] = self._ev(s.e, args)
            elif isinstance(s, I.SetL):
                args[s.name] = self._ev(s.e, args)
            elif isinstance(s, I.SetO):
                self._set_out(s.name, self._ev(s.e, args))
            elif isinstance(s, I.IfS):
                r = None
                if self._ev(s.cond, args):
                    r = self._exec(s.then, args)
                elif s.els:
                    r = self._exec(s.els, args)
                if r is not None:
                    return r
            elif isinstance(s, I.GotoS):
                self._goto(s.state)
            elif isinstance(s, I.FillS):
                self._fill(s.strip, *[self._ev(a, args) for a in s.args])
            elif isinstance(s, I.PixelS):
                i = self._ev(s.idx, args)
                self._pixel(s.strip, i, *[self._ev(a, args) for a in s.args])
            elif isinstance(s, I.AnimS):
                if s.mode == "off":
                    self._cancel_anim(s.strip)
                elif s.mode == "rainbow":
                    self._animate(s.strip, ANIM_MODES["rainbow"], 0, 0, 0, self._ev(s.args[0], args))
                else:
                    self._animate(s.strip, ANIM_MODES[s.mode], *[self._ev(a, args) for a in s.args])
            elif isinstance(s, I.SendS):
                if s.e is None:
                    self._tx(s.msg)
                else:
                    v = self._ev(s.e, args)
                    self._tx(f"{s.msg} {int(v) if isinstance(v, bool) else v}")
        return None

    # ---- reporting
    def summary(self) -> str:
        parts = []
        for k, v in self.out.items():
            parts.append(f"{k}={int(v) if isinstance(v, bool) else v}")
        for k, v in self.vars.items():
            if isinstance(v, list):
                parts.append(f"{k}=[" + ",".join(str(int(x)) for x in v) + "]")
            else:
                parts.append(f"{k}={int(v) if isinstance(v, bool) else v}")
        if self.state_names:
            parts.append(f"state={self.state_names[self.st]}")
        if self.blocked:
            parts.append(f"blocked={self.blocked}")
        if self.oob:
            parts.append(f"bad_index={self.oob}")
        return "  ".join(parts)
