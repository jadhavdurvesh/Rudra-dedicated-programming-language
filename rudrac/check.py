"""Semantic checker: names, types, pins, peripherals, safety rules."""
from __future__ import annotations

import difflib
import re
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from .syntax import (Animate, Assign, AssignTo, Bin, BoolLit, Break, Call, CallStmt, Const, Event, Fill, Fn, For, Goto,
                     If, IO, Index, Let, Member, Name, Never, Node, Num, Pixel, Return, Send, Unary, Var,
                     fmt_expr, has_call, has_place, walk_names, walk_stmts)
from .util import (ANIM_MAX_PERIOD, ANIM_MIN_PERIOD, ANIM_MODES, Diag, I2C_PINS, RudraError,
                   TARGETS, resolve_pin, rt_div, rt_mod, wrap32)

RESERVED_COMMANDS = {"PING", "WHO", "STATUS", "STATE", "LOG"}
RESERVED_MESSAGES = {"PONG", "ERR", "PING", "WHO", "STATUS", "STATE", "LOG", "LOGEND"}
WORD = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")

KIND_TYPES = {"digital": "bool", "pwm": "int", "analog": "int", "servo": "int",
              "i2c": "int", "neopixel": "strip", "motor": "int", "stepper": "int", "tone": "int", "ultrasonic": "int"}
VALID_KINDS = {"input": ("digital", "analog", "i2c", "ultrasonic"),
               "output": ("digital", "pwm", "servo", "neopixel", "i2c", "motor", "stepper", "tone")}
ALLOWED = {
    ("input", "ultrasonic"): {"trig", "echo"},
    ("output", "tone"): {"pin", "safe"},
    ("output", "motor"): {"pwm", "dir", "fwd", "rev", "ramp", "limit", "timeout", "safe"},
    ("output", "stepper"): {"step", "dir", "rate", "safe"},
    ("input", "digital"): {"pin", "pullup", "invert", "debounce"},
    ("input", "analog"): {"pin"},
    ("input", "i2c"): {"addr", "reg", "bytes", "signed"},
    ("output", "digital"): {"pin", "invert", "safe"},
    ("output", "pwm"): {"pin", "invert", "safe"},
    ("output", "servo"): {"pin", "safe"},
    ("output", "neopixel"): {"pin", "count", "rgb", "safe"},
    ("output", "i2c"): {"addr", "reg", "bytes", "safe"},
}
SAFE_HINT = {"digital": "safe false", "pwm": "safe 0", "servo": "safe 90", "motor": "safe 0", "stepper": "safe 0", "tone": "safe off",
             "neopixel": "safe off  (or  safe 255, 0, 0)", "i2c": "safe 0"}


@dataclass
class Sym:
    kind: str              # const | var | input | output
    type: str              # int | bool | strip
    node: Any = None
    value: Any = None
    shape: str = "scalar"   # scalar | array | struct | structarray (vars only)


class Checked:
    def __init__(self, node: Node, syms: Dict[str, Sym], diags: List[Diag]):
        self.node = node
        self.syms = syms
        self.diags = diags
        self.messages: Dict[str, bool] = {}      # message name -> carries a value
        self.commands: List[tuple] = []          # (name, [param names])
        self.rules: List[Never] = []             # interlock rules, in source order
        self.uses_anim = False
        self.states: List[str] = []              # state names; the first is the initial state
        self.fns: list = []                      # func / proc declarations
        self.state_index: Dict[str, int] = {}
        self.bus = False                         # node has an 'address' (shared-bus mode)

    @property
    def errors(self):
        return [d for d in self.diags if d.level == "error"]

    @property
    def warnings(self):
        return [d for d in self.diags if d.level == "warning"]

    @property
    def ok(self):
        return not self.errors


class _NotConst(Exception):
    pass


class Checker:
    def __init__(self, node: Node):
        self.node = node
        self.syms: Dict[str, Sym] = {}
        self.diags: List[Diag] = []
        self.reads: set = set()
        self.writes: set = set()
        self.pins: Dict[int, str] = {}
        self.reserved: set = set()
        self.messages: Dict[str, bool] = {}
        self.commands: List[tuple] = []
        self.rules: List[Never] = []
        self.uses_anim = False
        self.bus = False
        self.state_index: Dict[str, int] = {}
        self.cmd_params: Dict[str, tuple] = {}
        self.enums: Dict[str, list] = {}        # enum name -> member names
        self.structs: Dict[str, list] = {}      # struct name -> [(field, type)]
        self.ranges: Dict[str, tuple] = {}      # active loop counters -> (lowest, highest) value
        self.fns: Dict[str, Fn] = {}
        self.calls: Dict[str, set] = {}      # function -> functions it calls
        self.called: set = set()
        self.cur: Optional[str] = None       # name of the func/proc being checked
        self.cur_fn: Optional[Fn] = None
        self.loop = 0
        self.ro: Dict[str, str] = {}         # read-only local names -> what they are

    # -- diagnostics
    def err(self, msg, pos, hint=""):
        self.diags.append(Diag("error", msg, pos[0], pos[1], hint))

    def warn(self, msg, pos, hint=""):
        self.diags.append(Diag("warning", msg, pos[0], pos[1], hint))

    def suggest(self, name, scope=()):
        pool = list(self.syms) + list(scope)
        m = difflib.get_close_matches(name, pool, n=1)
        return f"did you mean '{m[0]}'?" if m else ""

    # -- constant evaluation (const, var initialisers, periods, options)
    def ceval(self, e):
        if isinstance(e, Num):
            return e.value, "int"
        if isinstance(e, BoolLit):
            return e.value, "bool"
        if isinstance(e, Name):
            s = self.syms.get(e.name)
            if s is None:
                self.err(f"unknown name '{e.name}'", e.pos, self.suggest(e.name))
                raise _NotConst()
            if s.kind != "const":
                self.err(f"'{e.name}' is not a constant, so it cannot be used here",
                         e.pos, "only literals and 'const' names are allowed")
                raise _NotConst()
            return s.value, s.type
        if isinstance(e, Unary):
            v, t = self.ceval(e.e)
            if e.op == "-" and t == "int":
                return wrap32(-v), "int"
            if e.op == "!" and t == "bool":
                return (not v), "bool"
            self.err(f"operator '{e.op}' cannot be applied to {t}", e.pos)
            raise _NotConst()
        if isinstance(e, Bin):
            a, ta = self.ceval(e.l)
            b, tb = self.ceval(e.r)
            return self._cbin(e, a, ta, b, tb)
        if isinstance(e, Call):
            self.err(f"a call to '{e.name}' cannot be used here", e.pos, "only literals and 'const' names are allowed")
        if isinstance(e, Member) and isinstance(e.obj, Name) and e.obj.name in self.enums:
            ty = self.place(e, {})
            if ty is None:
                raise _NotConst()
            return e.enum_val, ty
        if isinstance(e, (Index, Member)):
            self.err("an array element or struct field cannot be used here", e.pos,
                     "only literals, 'const' names and enum constants are allowed")
        raise _NotConst()

    def _cbin(self, e, a, ta, b, tb):
        op = e.op
        if op in ("+", "-", "*", "/", "%"):
            if ta != "int" or tb != "int":
                self.err(f"'{op}' needs numbers on both sides", e.pos)
                raise _NotConst()
            if op in ("/", "%") and b == 0:
                self.err("division by zero", e.pos)
                raise _NotConst()
            r = {"+": lambda: a + b, "-": lambda: a - b, "*": lambda: a * b,
                 "/": lambda: rt_div(a, b), "%": lambda: rt_mod(a, b)}[op]()
            return wrap32(r), "int"
        if op in ("<", "<=", ">", ">="):
            if ta != "int" or tb != "int":
                self.err(f"'{op}' needs numbers on both sides", e.pos)
                raise _NotConst()
            return {"<": a < b, "<=": a <= b, ">": a > b, ">=": a >= b}[op], "bool"
        if op in ("==", "!="):
            if ta != tb:
                self.err(f"cannot compare {ta} with {tb}", e.pos)
                raise _NotConst()
            return (a == b) if op == "==" else (a != b), "bool"
        if ta != "bool" or tb != "bool":
            self.err(f"'{op}' needs true/false on both sides", e.pos)
            raise _NotConst()
        return (a and b) if op == "&&" else (a or b), "bool"

    def const_value(self, e):
        try:
            return self.ceval(e)
        except _NotConst:
            return None, None

    def opt_int(self, d: IO, key, lo, hi, default=None):
        if key not in d.opts:
            return default
        exprs, pos = d.opts[key]
        v, t = self.const_value(exprs[0])
        if t is None:
            return default
        if t != "int":
            self.err(f"'{key}' needs a number", pos)
            return default
        if not (lo <= v <= hi):
            self.err(f"'{key}' must be between {lo} and {hi}", pos)
            return default
        return v

    # -- declarations
    def declare(self, name, sym, pos):
        if name in self.enums or name in self.structs:
            self.err(f"'{name}' is already declared as a type", pos)
            return False
        if name in self.syms:
            self.err(f"'{name}' is already declared", pos)
            return False
        self.syms[name] = sym
        return True

    def safe_value(self, d: IO):
        if "safe" not in d.opts:
            self.err(f"output '{d.name}' needs a safe state", d.pos,
                     f"add  {SAFE_HINT[d.kind]}  inside the parentheses; it is what the output does at boot "
                     "and when the master goes quiet")
            return None
        exprs, pos = d.opts["safe"]
        kind = d.kind
        if kind == "neopixel":
            if not exprs:
                return (0, 0, 0)
            vals = []
            for e in exprs:
                v, t = self.const_value(e)
                if t is None:
                    return None
                if t != "int" or not 0 <= v <= 255:
                    self.err("colour values must be numbers from 0 to 255", pos)
                    return None
                vals.append(v)
            return tuple(vals)
        if not exprs:
            return False if kind == "digital" else 0
        v, t = self.const_value(exprs[0])
        if t is None:
            return None
        if kind == "digital":
            if t != "bool":
                self.err("a digital safe state is  safe true  or  safe false", pos)
                return None
            return v
        if t != "int":
            self.err("this safe state must be a number", pos)
            return None
        if kind == "motor":
            m = d.cfg.get("limit") or 100
            if not -m <= v <= m:
                self.err(f"a motor safe state must be between -{m} and {m}", pos)
                return None
            return v
        lim = {"pwm": (0, 100), "servo": (0, 180), "tone": (0, 20000)}.get(kind)
        if lim and not lim[0] <= v <= lim[1]:
            self.err(f"a {kind} safe state must be between {lim[0]} and {lim[1]}", pos)
            return None
        return v

    def decl_io(self, d: IO):
        ty = KIND_TYPES[d.kind]
        if d.kind not in VALID_KINDS[d.direction]:
            self.err(f"{d.direction}s cannot be '{d.kind}'", d.pos,
                     f"valid {d.direction} kinds: " + ", ".join(VALID_KINDS[d.direction]))
            self.declare(d.name, Sym(d.direction, ty, d), d.pos)
            return
        given = set(d.opts) | set(d.pins)
        if d.pin_text is not None:
            given.add("pin")
        if d.pullup:
            given.add("pullup")
        if d.invert:
            given.add("invert")
        allowed = ALLOWED[(d.direction, d.kind)]
        for g in sorted(given - allowed):
            self.err(f"'{g}' does not apply to a {d.kind} {d.direction}", d.pos,
                     "allowed here: " + ", ".join(sorted(allowed)))

        cfg = d.cfg
        if d.kind in ("motor", "stepper", "ultrasonic"):
            self.decl_multi(d, ty)
            return
        if d.kind == "digital" and d.direction == "input":
            cfg["debounce"] = self.opt_int(d, "debounce", 1, 5000)
        if d.kind != "i2c" and d.pin_text is None:
            self.err(f"a {d.kind} {d.direction} needs a pin, e.g.  {d.kind}(pin 5)", d.pos)

        if d.kind == "neopixel":
            cfg["count"] = self.opt_int(d, "count", 1, 1024)
            if "count" not in d.opts:
                self.err("a neopixel output needs a pixel count, e.g.  neopixel(pin 6, count 8)", d.pos)
            cfg["rgb"] = "rgb" in d.opts
            if self.node.target in ("nano", "uno") and cfg["count"]:
                block_us = cfg["count"] * 30 + 80          # 800 kHz: 30 us per pixel + latch
                byte_us = 10_000_000 // self.node.baud     # one serial byte = 10 bit times
                if block_us > byte_us:
                    self.warn(f"on AVR, updating {cfg['count']} pixels blocks interrupts for about {block_us} us, "
                              f"but one serial byte at {self.node.baud} baud lasts only {byte_us} us: "
                              "a command that arrives during an update can be lost", d.opts["count"][1],
                              "use fewer pixels, a lower 'serial' speed, or an ESP32")
        if d.kind == "i2c":
            if "addr" not in d.opts:
                self.err("an i2c device needs an address, e.g.  i2c(addr 0x48)", d.pos)
            cfg["addr"] = self.opt_int(d, "addr", 8, 119)
            cfg["reg"] = self.opt_int(d, "reg", 0, 255)
            cfg["bytes"] = self.opt_int(d, "bytes", 1, 4, 1)
            cfg["signed"] = "signed" in d.opts
        if d.direction == "output":
            cfg["safe"] = self.safe_value(d)

        pin = None
        if d.pin_text is not None and d.kind != "i2c" and self.node.target in TARGETS:
            pin, why = resolve_pin(self.node.target, d.pin_text)
            if why:
                self.err(why, d.pin_pos)
        if pin is not None:
            d.pin = pin
            if pin.warn:
                self.warn(pin.warn, d.pin_pos)
            need = {
                ("input", "digital"): (pin.din, "cannot be used as a digital input"),
                ("input", "analog"): (pin.ain, "is not an analog-capable pin on this board"),
                ("output", "digital"): (pin.dout, "cannot be used as a digital output"),
                ("output", "pwm"): (pin.pwm, "does not support PWM on this board"),
                ("output", "servo"): (pin.dout, "cannot drive a servo (it is not an output pin)"),
                ("output", "neopixel"): (pin.dout, "cannot drive a neopixel strip (it is not an output pin)"),
                ("output", "tone"): (pin.dout, "cannot drive a buzzer (it is not an output pin)"),
            }.get((d.direction, d.kind))
            if need and not need[0]:
                self.err(f"pin {d.pin_text} {need[1]}", d.pin_pos)
            if d.pullup and not pin.pullup_ok:
                self.err(f"pin {d.pin_text} has no internal pull-up", d.pin_pos,
                         "add an external resistor and drop 'pullup'")
            if pin.key in self.pins:
                who = self.pins[pin.key]
                label = who if pin.key in self.reserved else f"'{who}'"
                self.err(f"pin {d.pin_text} is already used by {label}", d.pin_pos)
            else:
                self.pins[pin.key] = d.name
        self.declare(d.name, Sym(d.direction, ty, d), d.pos)

    def decl_types(self):
        for en in self.node.enums:
            if en.name in self.enums or en.name in self.structs or en.name in ("int", "bool"):
                self.err(f"'{en.name}' is already declared", en.pos)
                continue
            names = []
            for m, pos in en.members:
                if m in names:
                    self.err(f"enum {en.name} has '{m}' twice", pos)
                names.append(m)
            if not names:
                self.err(f"enum {en.name} needs at least one member", en.pos)
            self.enums[en.name] = names
        for st in self.node.structs:
            if st.name in self.enums or st.name in self.structs or st.name in ("int", "bool"):
                self.err(f"'{st.name}' is already declared", st.pos)
                continue
            fields, seen = [], set()
            for f, ty, pos in st.fields:
                if f in seen:
                    self.err(f"struct {st.name} has the field '{f}' twice", pos)
                seen.add(f)
                if ty not in ("int", "bool") and ty not in self.enums:
                    self.err(f"unknown type '{ty}' for field '{f}'", pos, "fields are int, bool or an enum")
                    ty = "int"
                fields.append((f, ty))
            if not fields:
                self.err(f"struct {st.name} needs at least one field", st.pos)
            self.structs[st.name] = fields

    def decl_var(self, d: Var):
        if d.tname is None:
            v, t = self.const_value(d.expr)
            if t is None:
                v, t = 0, "int"
            d.init, d.type, d.slots, d.shape = v, t, [(d.name, t)], "scalar"
            self.declare(d.name, Sym("var", t, d, v), d.pos)
            return
        tn = d.tname
        size = None
        if d.size_expr is not None:
            sv, st = self.const_value(d.size_expr)
            if st is None:
                size = 1
            elif st != "int" or not 1 <= sv <= 1024:
                self.err("an array needs a constant length between 1 and 1024", d.pos, "e.g.  var buf: int[8]")
                size = 1
            else:
                size = sv
            d.size = size
        if tn in ("int", "bool") or tn in self.enums:
            d.slots = [(d.name, tn)]
            if size is None:
                d.shape = "scalar"
                if d.expr is None:
                    d.init = False if tn == "bool" else 0
                else:
                    v, t = self.const_value(d.expr)
                    if t is not None and t != tn:
                        self.err(f"'{d.name}' is {tn}, but the initial value is {t}", d.pos)
                    d.init = v if t == tn else (False if tn == "bool" else 0)
            else:
                d.shape, d.init = "array", None
                if d.expr is not None:
                    self.err("an array starts with every element zero (false, or the first enum member)", d.pos,
                             "set elements in a  start { }  block")
        elif tn in self.structs:
            d.slots = [(f"{d.name}.{f}", ft) for f, ft in self.structs[tn]]
            d.shape = "struct" if size is None else "structarray"
            d.init = None
            if d.expr is not None:
                self.err("a struct starts with every field zero (false, or the first enum member)", d.pos,
                         "set fields in a  start { }  block")
        else:
            pool = ["int", "bool"] + list(self.enums) + list(self.structs)
            m = difflib.get_close_matches(tn, pool, n=1)
            self.err(f"unknown type '{tn}'", d.pos, f"did you mean '{m[0]}'?" if m else "types: " + ", ".join(pool))
            d.slots, d.shape, d.init, tn = [(d.name, "int")], "scalar", 0, "int"
        d.type = tn
        self.declare(d.name, Sym("var", tn, d, d.init, shape=d.shape), d.pos)

    def claim(self, d, pin, text, pos):
        """Register a pin for output/input d; errors on clashes."""
        if pin.warn:
            self.warn(pin.warn, pos)
        if pin.key in self.pins:
            who = self.pins[pin.key]
            label = who if pin.key in self.reserved else f"'{who}'"
            self.err(f"pin {text} is already used by {label}", pos)
        else:
            self.pins[pin.key] = d.name

    def decl_multi(self, d: IO, ty):
        """motor and stepper: several pins with roles."""
        cfg = d.cfg
        roles = {}
        if d.kind == "ultrasonic":
            for r in ("trig", "echo"):
                if r not in d.pins:
                    self.err(f"an ultrasonic sensor needs  trig <pin>  and  echo <pin>  (missing '{r}')", d.pos)
            roles = {"trig": "dout", "echo": "din"}
        elif d.kind == "motor":
            have = set(d.pins)
            if {"pwm", "dir"} <= have and not ({"fwd", "rev"} & have):
                cfg["style"] = "dir"
                roles = {"pwm": "pwm", "dir": "dout"}
            elif {"fwd", "rev"} <= have and not ({"pwm", "dir"} & have):
                cfg["style"] = "dual"
                roles = {"fwd": "pwm", "rev": "pwm"}
            else:
                cfg["style"] = None
                self.err("a motor needs  pwm 5, dir 4  (one PWM pin and a direction pin) or  fwd 5, rev 6  "
                         "(two PWM pins)", d.pos)
            cfg["limit"] = self.opt_int(d, "limit", 1, 100, 100)
            cfg["ramp"] = self.opt_int(d, "ramp", 1, 100000)
            cfg["timeout"] = self.opt_int(d, "timeout", 1, 3_600_000)
            if cfg["ramp"] is not None:
                cfg["ramp_step"] = max(1, cfg["ramp"] * 20 // 1000)     # percent per 20 ms
        else:
            if not {"step", "dir"} <= set(d.pins):
                self.err("a stepper needs a step pin and a direction pin, e.g.  stepper(step 8, dir 9, rate 400, safe 0)", d.pos)
            roles = {"step": "dout", "dir": "dout"}
            cfg["rate"] = self.opt_int(d, "rate", 1, 1000, 500)
            cfg["interval"] = max(1, 1000 // (cfg["rate"] or 500))      # ms between steps
        if d.direction == "output":
            cfg["safe"] = self.safe_value(d)
        cfg["pins"] = {}
        if self.node.target in TARGETS:
            for role, need in roles.items():
                if role not in d.pins:
                    continue
                text, pos = d.pins[role]
                pin, why = resolve_pin(self.node.target, text)
                if why:
                    self.err(why, pos)
                    continue
                if need == "pwm" and not pin.pwm:
                    self.err(f"pin {text} does not support PWM on this board", pos)
                if need == "din" and not pin.din:
                    self.err(f"pin {text} cannot be used as a digital input", pos)
                if need == "dout" and not pin.dout:
                    self.err(f"pin {text} cannot be used as a digital output", pos)
                self.claim(d, pin, text, pos)
                cfg["pins"][role] = pin
        self.declare(d.name, Sym(d.direction, ty, d), d.pos)

    # -- expression typing
    def tc(self, e, scope) -> Optional[str]:
        if isinstance(e, Num):
            return "int"
        if isinstance(e, BoolLit):
            return "bool"
        if isinstance(e, Name):
            if e.name in scope:
                return scope[e.name]
            s = self.syms.get(e.name)
            if s is None:
                self.err(f"unknown name '{e.name}'", e.pos, self.suggest(e.name, scope))
                return None
            if s.type == "strip":
                self.err(f"'{e.name}' is a neopixel strip and has no value", e.pos,
                         "set it with  fill / pixel")
                return None
            if s.shape == "array" or s.shape == "structarray":
                self.err(f"'{e.name}' is an array; use {e.name}[i]" + (".field" if s.shape == "structarray" else ""), e.pos)
                return None
            if s.shape == "struct":
                self.err(f"'{e.name}' is a struct; use {e.name}.field", e.pos)
                return None
            self.reads.add(e.name)
            return s.type
        if isinstance(e, (Index, Member)):
            return self.place(e, scope)
        if isinstance(e, Unary):
            t = self.tc(e.e, scope)
            if t is None:
                return None
            if e.op == "-" and t != "int":
                self.err("'-' needs a number", e.pos)
                return None
            if e.op == "!" and t != "bool":
                self.err("'!' / 'not' needs true/false", e.pos)
                return None
            return t
        if isinstance(e, Bin):
            if isinstance(e.r, Num) and e.op in ("/", "%") and e.r.value == 0:
                self.err("division by zero", e.pos)
            a, b = self.tc(e.l, scope), self.tc(e.r, scope)
            if a is None or b is None:
                return None
            op = e.op
            if op in ("+", "-", "*", "/", "%"):
                if a != "int" or b != "int":
                    self.err(f"'{op}' needs numbers on both sides", e.pos)
                    return None
                return "int"
            if op in ("<", "<=", ">", ">="):
                if a != "int" or b != "int":
                    self.err(f"'{op}' needs numbers on both sides", e.pos)
                    return None
                return "bool"
            if op in ("==", "!="):
                if a != b:
                    self.err(f"cannot compare {a} with {b}", e.pos)
                    return None
                return "bool"
            if a != "bool" or b != "bool":
                self.err(f"'{op}' needs true/false on both sides", e.pos,
                         "compare first, e.g.  (temp > 500) and armed")
                return None
            return "bool"
        if isinstance(e, Call):
            fn = self.check_call(e, scope)
            if fn is None:
                return None
            if fn.kind == "proc":
                self.err(f"'{e.name}' is a proc and has no value", e.pos, f"call it as a statement:  {e.name}(...)")
                return None
            e.type = fn.ret
            return fn.ret
        return None

    def place(self, e, scope, write=False):
        """Resolve an array element, struct field or enum constant. Fills the node and returns its type."""
        if isinstance(e, Member):
            o = e.obj
            if isinstance(o, Name) and o.name in self.enums and o.name not in scope:
                names = self.enums[o.name]
                if write:
                    self.err(f"'{o.name}.{e.field}' is a constant and cannot be assigned", e.pos)
                    return None
                if e.field not in names:
                    m = difflib.get_close_matches(e.field, names, n=1)
                    self.err(f"enum {o.name} has no member '{e.field}'", e.pos,
                             f"did you mean '{m[0]}'?" if m else "members: " + ", ".join(names))
                    return None
                e.enum_val, e.ty = names.index(e.field), o.name
                return o.name
            base = o.base if isinstance(o, Index) else (o.name if isinstance(o, Name) else None)
            sym = self.syms.get(base) if base and base not in scope else None
            if sym is None:
                self.err(f"unknown name '{base}'" if base not in scope else f"'{base}' is a local number and has no fields",
                         e.pos, self.suggest(base or "", scope) if base not in scope else "")
                return None
            if sym.kind != "var" or sym.shape not in ("struct", "structarray"):
                self.err(f"'{base}' has no fields", e.pos, "fields belong to variables declared like  var p: Point")
                return None
            fields = dict(self.structs[sym.type])
            if e.field not in fields:
                m = difflib.get_close_matches(e.field, list(fields), n=1)
                self.err(f"struct {sym.type} has no field '{e.field}'", e.pos,
                         f"did you mean '{m[0]}'?" if m else "fields: " + ", ".join(fields))
                return None
            if isinstance(o, Index):
                if sym.shape != "structarray":
                    self.err(f"'{base}' is not an array", e.pos)
                    return None
                self.index_info(e, o.idx, sym, scope)
            elif sym.shape == "structarray":
                self.err(f"'{base}' is an array of structs; write {base}[i].{e.field}", e.pos)
                return None
            e.slot, e.ty = f"{base}.{e.field}", fields[e.field]
            self.reads.add(base)
            return e.ty
        # Index
        sym = self.syms.get(e.base) if e.base not in scope else None
        if sym is None:
            self.err(f"unknown name '{e.base}'" if e.base not in scope else f"'{e.base}' is a local number, not an array",
                     e.pos, self.suggest(e.base, scope) if e.base not in scope else "")
            return None
        if sym.kind != "var" or sym.shape not in ("array", "structarray"):
            self.err(f"'{e.base}' is not an array", e.pos, "declare one with  var buf: int[8]")
            return None
        if sym.shape == "structarray":
            self.err(f"'{e.base}' holds structs; write {e.base}[i].field", e.pos)
            return None
        self.index_info(e, e.idx, sym, scope)
        e.slot, e.ty = e.base, sym.type
        self.reads.add(e.base)
        return sym.type

    def index_info(self, target, idx, sym, scope):
        size = sym.node.size
        target.size = size
        if target is not idx:
            target.idx = idx if isinstance(target, Member) else target.idx
        t = self.tc(idx, scope)
        if t is not None and t != "int":
            self.err("an index must be a number", idx.pos)
        n = len(self.diags)
        v, tt = self.const_value(idx)
        del self.diags[n:]
        target.proven = False
        if tt == "int":
            if not 0 <= v < size:
                self.err(f"index {v} is outside the array (valid: 0..{size - 1})", idx.pos)
            else:
                target.proven = True
        elif isinstance(idx, Name) and idx.name in scope and idx.name in self.ranges:
            lo, hi = self.ranges[idx.name]
            if 0 <= lo and hi < size:
                target.proven = True
            else:
                self.warn(f"this loop counter runs up to {hi} but the array has {size} elements: "
                          "writes beyond the end are refused at run time and counted (STATUS)", idx.pos)

    def check_call(self, e: Call, scope):
        """Name, argument count and argument types of a call. Returns the Fn or None."""
        fn = self.fns.get(e.name)
        for a in e.args:
            self.tc(a, scope)
        if fn is None:
            m = difflib.get_close_matches(e.name, list(self.fns), n=1)
            self.err(f"unknown function '{e.name}'", e.pos, f"did you mean '{m[0]}'?" if m else
                     "declare it with  func NAME(x: int) -> int { ... }  or  proc NAME() { ... }")
            return None
        self.called.add(e.name)
        if self.cur is not None:
            self.calls[self.cur].add(e.name)
        if len(e.args) != len(fn.params):
            self.err(f"'{e.name}' takes {len(fn.params)} argument{'s' if len(fn.params) != 1 else ''} "
                     f"but {len(e.args)} given", e.pos)
            return fn
        for a, want, (pn, _t, _p) in zip(e.args, fn.ptypes, fn.params):
            got = self.tc_quiet(a, scope)
            if got is not None and want is not None and got != want:
                self.err(f"argument '{pn}' of '{e.name}' must be {want}, not {got}", a.pos)
        return fn

    def tc_quiet(self, e, scope):
        """Type of an expression without reporting its errors again (they were reported once already)."""
        n = len(self.diags)
        t = self.tc(e, scope)
        del self.diags[n:]
        return t

    # -- functions and procedures
    def always_returns(self, body) -> bool:
        if not body:
            return False
        last = body[-1]
        if isinstance(last, Return):
            return True
        if isinstance(last, If) and last.els is not None:
            return self.always_returns(last.then) and self.always_returns(last.els)
        return False

    def check_fns(self):
        node = self.node
        for f in node.fns:
            if f.name in self.syms or f.name in self.fns:
                self.err(f"'{f.name}' is already declared", f.pos)
            else:
                self.fns[f.name] = f
            f.ptypes, seen = [], set()
            for pn, pt, pos in f.params:
                if pt not in ("int", "bool") and pt not in self.enums:
                    self.err(f"unknown type '{pt}'", pos, "types: int, bool" + "".join(", " + n for n in self.enums))
                    f.ptypes.append(None)
                else:
                    f.ptypes.append(pt)
                if pn in self.syms or pn in seen or pn in self.fns:
                    self.err(f"parameter '{pn}' hides an existing name", pos)
                seen.add(pn)
            if f.kind == "func" and f.ret is None:
                self.err(f"func '{f.name}' needs a result type", f.pos, f"write it like  func {f.name}(x: int) -> int {{ ... }}")
            elif f.kind == "proc" and f.ret is not None:
                self.err(f"proc '{f.name}' returns nothing", f.pos, "use a func to compute a value")
            elif f.ret not in (None, "int", "bool") and f.ret not in self.enums:
                self.err(f"unknown result type '{f.ret}'", f.pos, "types: int, bool" + "".join(", " + n for n in self.enums))
        for f in node.fns:
            if self.fns.get(f.name) is not f:
                continue
            self.cur, self.cur_fn = f.name, f
            self.calls[f.name] = set()
            scope = {pn: (f.ptypes[i] or "int") for i, (pn, _t, _p) in enumerate(f.params)}
            self.stmts(f.body, scope, f.kind)
            if f.kind == "func" and f.ret and not self.always_returns(f.body):
                self.err(f"not every path of func '{f.name}' returns a value", f.pos,
                         "end the func (and each branch of a final if/else) with  return ...")
        self.cur = self.cur_fn = None
        # recursion would make the stack depth unknowable
        state = {}

        def visit(n, path):
            if state.get(n) == 2:
                return
            if state.get(n) == 1:
                cyc = path[path.index(n):] + [n]
                self.err(f"'{n}' calls itself ({' -> '.join(cyc)}): recursion is not allowed", self.fns[n].pos,
                         "the compiler must know the deepest the call stack can get; use a for loop")
                return
            state[n] = 1
            for m in sorted(self.calls.get(n, ())):
                visit(m, path + [n])
            state[n] = 2

        for n in self.fns:
            visit(n, [])

    # -- statements
    def strip_sym(self, s):
        sym = self.syms.get(s.strip)
        if sym is None:
            self.err(f"unknown name '{s.strip}'", s.pos, self.suggest(s.strip))
            return None
        if not (sym.kind == "output" and sym.type == "strip"):
            self.err(f"'{s.strip}' is not a neopixel output", s.pos)
            return None
        return sym

    def stmts(self, body, scope, evkind="on"):
        for idx, s in enumerate(body):
            if isinstance(s, (Return, Break)) and idx != len(body) - 1:
                self.err("this statement can never run: "
                         f"'{'return' if isinstance(s, Return) else 'break'}' ends the block", body[idx + 1].pos)
            if evkind == "func" and isinstance(s, (Send, Fill, Pixel, Animate, Goto)):
                self.err("a func only computes a value, it cannot send, change a strip or change state", s.pos,
                         "use a proc for that")
                continue
            if isinstance(s, Goto) and idx != len(body) - 1:
                nxt = body[idx + 1]
                self.err("this statement can never run: 'goto' ends the handler", nxt.pos,
                         "move the goto to the end of the block")
            if isinstance(s, Goto):
                if s.state not in self.state_index:
                    pool = list(self.state_index)
                    m = difflib.get_close_matches(s.state, pool, n=1)
                    self.err(f"unknown state '{s.state}'", s.pos, f"did you mean '{m[0]}'?" if m else
                             ("declare it with  state NAME { ... }" if not pool else "states: " + ", ".join(pool)))
                if evkind in ("start", "enter", "exit", "proc"):
                    self.err(f"'goto' cannot be used in a '{evkind}' block", s.pos,
                             "a state change must come from every / when / on / heartbeat")
                if self.loop:
                    self.err("'goto' cannot be used inside a loop", s.pos)
            elif isinstance(s, Assign):
                sym = self.syms.get(s.name)
                t = self.tc(s.expr, scope)
                if s.name in scope:
                    if s.name in self.ro:
                        self.err(f"'{s.name}' is a {self.ro[s.name]} and cannot be assigned", s.pos)
                    elif t is not None and scope[s.name] is not None and t != scope[s.name]:
                        self.err(f"cannot assign {t} to '{s.name}' ({scope[s.name]})", s.pos)
                    continue
                if sym is None:
                    self.err(f"unknown name '{s.name}'", s.pos, self.suggest(s.name, scope))
                    continue
                if sym.kind == "const":
                    self.err(f"'{s.name}' is a constant and cannot be assigned", s.pos, "use 'var' if it must change")
                    continue
                if sym.kind == "input":
                    self.err(f"'{s.name}' is an input and cannot be assigned", s.pos)
                    continue
                if sym.type == "strip":
                    self.err(f"'{s.name}' is a neopixel strip; use  fill {s.name}, r, g, b  or  pixel {s.name}, i, r, g, b", s.pos)
                    continue
                if evkind == "func":
                    self.err(f"a func cannot change '{s.name}'", s.pos,
                             "a func only computes a value; use a proc to change outputs and variables")
                    continue
                self.writes.add(s.name)
                if t is not None and t != sym.type:
                    hint = ""
                    if sym.type == "bool":
                        hint = "use true/false, or a comparison like  level > 0"
                    elif t == "bool":
                        hint = "this is a number; true/false cannot be stored in it"
                    self.err(f"cannot assign {t} to '{s.name}' ({sym.type})", s.pos, hint)
            elif isinstance(s, If):
                t = self.tc(s.cond, scope)
                if t is not None and t != "bool":
                    self.err("an 'if' condition must be true/false", s.pos, "try a comparison, e.g.  temp > 500")
                self.stmts(s.then, dict(scope), evkind)
                if s.els:
                    self.stmts(s.els, dict(scope), evkind)
            elif isinstance(s, AssignTo):
                tgt = s.target
                if evkind == "func":
                    self.err(f"a func cannot change '{fmt_expr(tgt)}'", s.pos,
                             "a func only computes a value; use a proc to change variables")
                    self.tc(s.expr, scope)
                    continue
                ty = self.place(tgt, scope, write=True)
                t = self.tc(s.expr, scope)
                base = tgt.base if isinstance(tgt, Index) else (tgt.obj.base if isinstance(tgt.obj, Index) else getattr(tgt.obj, "name", None))
                if base:
                    self.writes.add(base)
                if ty is not None and t is not None and t != ty:
                    self.err(f"cannot assign {t} to '{fmt_expr(tgt)}' ({ty})", s.pos)
            elif isinstance(s, Let):
                t = self.tc(s.expr, scope)
                s.type = t
                if s.name in scope or s.name in self.syms or s.name in self.fns:
                    self.err(f"'{s.name}' hides an existing name", s.pos, "choose another name for this local")
                else:
                    scope[s.name] = t or "int"
            elif isinstance(s, Return):
                f = self.cur_fn
                if f is None:
                    self.err("'return' can only be used inside a func or proc", s.pos)
                elif f.kind == "func":
                    if s.expr is None:
                        self.err(f"func '{f.name}' must return a value", s.pos)
                    else:
                        t = self.tc(s.expr, scope)
                        if t is not None and f.ret and t != f.ret:
                            self.err(f"func '{f.name}' returns {f.ret}, not {t}", s.pos)
                elif s.expr is not None:
                    self.err(f"proc '{f.name}' returns nothing", s.pos, "use a func to compute a value")
            elif isinstance(s, Break):
                if not self.loop:
                    self.err("'break' can only be used inside a loop", s.pos)
            elif isinstance(s, CallStmt):
                fn = self.check_call(s.call, scope)
                if fn is not None and evkind == "func" and fn.kind == "proc":
                    self.err(f"a func cannot call the proc '{fn.name}'", s.pos, "funcs only compute values")
            elif isinstance(s, For):
                lo, tl = self.const_value(s.lo)
                hi, th = self.const_value(s.hi)
                if tl is not None and th is not None:
                    if tl != "int" or th != "int":
                        self.err("loop bounds must be numbers", s.pos)
                    elif hi < lo:
                        self.err("this loop never runs: it starts after it ends", s.pos)
                    elif hi - lo > 10000:
                        self.err(f"a loop may run at most 10000 times, this one runs {hi - lo}", s.pos,
                                 "the compiler has to bound how long a handler can take")
                    else:
                        s.lo_v, s.hi_v = lo, hi
                if s.var in scope or s.var in self.syms or s.var in self.fns:
                    self.err(f"loop variable '{s.var}' hides an existing name", s.pos)
                inner = dict(scope)
                inner[s.var] = "int"
                self.ro[s.var] = "loop counter"
                if s.lo_v is not None and s.hi_v > s.lo_v:
                    self.ranges[s.var] = (s.lo_v, s.hi_v - 1)
                self.loop += 1
                self.stmts(s.body, inner, evkind)
                self.loop -= 1
                self.ro.pop(s.var, None)
                self.ranges.pop(s.var, None)
            elif isinstance(s, Animate):
                sym = self.strip_sym(s)
                if sym is not None:
                    self.writes.add(s.strip)
                    self.uses_anim = True
                if s.mode not in ANIM_MODES:
                    self.err(f"unknown animation '{s.mode}'", s.pos, "modes: blink, pulse, chase, rainbow, off")
                for e in s.args:
                    t = self.tc(e, scope)
                    if t is not None and t != "int":
                        self.err("animation colours and periods must be numbers", e.pos)
                if s.args and isinstance(s.args[-1], Num):
                    per = s.args[-1]
                    if not ANIM_MIN_PERIOD <= per.value <= ANIM_MAX_PERIOD:
                        self.err(f"an animation period must be between {ANIM_MIN_PERIOD}ms and "
                                 f"{ANIM_MAX_PERIOD // 1000}s", per.pos,
                                 f"frames are drawn every 20 ms, so shorter than {ANIM_MIN_PERIOD}ms cannot look smooth")
            elif isinstance(s, (Fill, Pixel)):
                sym = self.strip_sym(s)
                if sym is not None:
                    self.writes.add(s.strip)
                exprs = ([s.idx] if isinstance(s, Pixel) else []) + list(s.args)
                for e in exprs:
                    t = self.tc(e, scope)
                    if t is not None and t != "int":
                        self.err("pixel numbers and colour values must be numbers", e.pos)
                if sym is not None and isinstance(s, Pixel) and isinstance(s.idx, Num):
                    count = sym.node.cfg.get("count")
                    if count and not 0 <= s.idx.value < count:
                        self.err(f"pixel {s.idx.value} is outside '{s.strip}' (0..{count - 1})", s.idx.pos)
            elif isinstance(s, Send):
                if self.bus and evkind != "on":
                    self.err("on a shared bus a node may only speak when it is asked", s.pos,
                             "keep the value in a var and answer in a command:  on GET { send \"TEMP\", temp }")
                if not WORD.match(s.msg):
                    self.err("message names must be a single word (letters, digits, _)", s.pos,
                             "the master splits lines on spaces")
                elif s.msg in RESERVED_MESSAGES:
                    self.err(f"'{s.msg}' is a built-in message name", s.pos)
                else:
                    has = s.expr is not None
                    if self.messages.setdefault(s.msg, has) != has:
                        self.err(f"message '{s.msg}' is sent both with and without a value", s.pos,
                                 "the master-side code needs one fixed format per message")
                if s.expr is not None:
                    self.tc(s.expr, scope)

    # -- interlock rules
    def feval(self, e, env):
        """Evaluate a rule expression for a given output state (types were already checked)."""
        if isinstance(e, (Num, BoolLit)):
            return e.value
        if isinstance(e, Member) and e.enum_val is not None:
            return e.enum_val
        if isinstance(e, Name):
            return env[e.name] if e.name in env else self.syms[e.name].value
        if isinstance(e, Unary):
            v = self.feval(e.e, env)
            return wrap32(-v) if e.op == "-" else (not v)
        a_op = e.op
        if a_op == "&&":
            return bool(self.feval(e.l, env)) and bool(self.feval(e.r, env))
        if a_op == "||":
            return bool(self.feval(e.l, env)) or bool(self.feval(e.r, env))
        a, b = self.feval(e.l, env), self.feval(e.r, env)
        if a_op == "+":
            return wrap32(a + b)
        if a_op == "-":
            return wrap32(a - b)
        if a_op == "*":
            return wrap32(a * b)
        if a_op == "/":
            return rt_div(a, b)
        if a_op == "%":
            return rt_mod(a, b)
        return {"<": a < b, "<=": a <= b, ">": a > b, ">=": a >= b, "==": a == b, "!=": a != b}[a_op]

    def check_rule(self, d: Never):
        if has_call(d.expr):
            self.err("a 'never' rule cannot call a function", d.pos,
                     "the compiler has to evaluate the rule against the safe state")
            return
        t = self.tc(d.expr, {})
        if t is None:
            return
        if t != "bool":
            self.err("a 'never' rule must be true/false", d.pos, "e.g.  never heater and not fan")
            return
        if has_place(d.expr):
            self.err("a 'never' rule cannot use array elements or struct fields", d.pos,
                     "the compiler has to evaluate the rule against the safe state")
            return
        bad = False
        for n in walk_names(d.expr):
            sym = self.syms.get(n.name)
            if sym is not None and sym.kind in ("var", "input"):
                self.err(f"'never' rules can only use outputs and constants, not the "
                         f"{'variable' if sym.kind == 'var' else sym.kind} '{n.name}'", n.pos,
                         "the compiler has to be able to prove the rule against the safe state")
                bad = True
        if bad:
            return
        outs = [x for x in self.node.decls if isinstance(x, IO) and x.direction == "output"]
        env = {}
        for o in outs:
            sv = o.cfg.get("safe")
            if o.kind != "neopixel" and sv is not None:
                env[o.name] = sv
        if not any(n.name in env for n in walk_names(d.expr)):
            self.err("this 'never' rule does not mention any output, so it can never be enforced", d.pos)
            return
        if all(o.cfg.get("safe") is not None for o in outs) and self.feval(d.expr, env):
            self.err(f"the safe state itself breaks this rule:  never {fmt_expr(d.expr)}", d.pos,
                     "change a 'safe' value, or the rule: the safe state must always be allowed")
        self.rules.append(d)

    # -- whole node
    def run(self) -> Checked:
        node = self.node
        if node.target not in TARGETS:
            self.err(f"unknown target '{node.target}'", node.target_pos,
                     "supported: " + ", ".join(sorted(TARGETS)))
        ios = [d for d in node.decls if isinstance(d, IO)]
        if node.target in I2C_PINS and any(d.kind == "i2c" for d in ios):
            sda, scl = I2C_PINS[node.target]
            self.pins[sda], self.pins[scl] = "the I2C data pin (SDA)", "the I2C clock pin (SCL)"
            self.reserved.update({sda, scl})

        # ---- bus
        self.bus = node.address is not None
        if node.address is not None and not 1 <= node.address <= 99:
            self.err("a bus address must be between 1 and 99", node.address_pos, "address 0 is the broadcast address")
        if node.rs485_text is not None:
            if node.address is None:
                self.err("'rs485' needs an 'address' (it only makes sense on a shared bus)", node.rs485_pos)
            if node.target in TARGETS:
                pin, why = resolve_pin(node.target, node.rs485_text)
                if why:
                    self.err(why, node.rs485_pos)
                elif not pin.dout:
                    self.err(f"pin {node.rs485_text} cannot be used as the RS485 direction pin (it is not an output)", node.rs485_pos)
                else:
                    if pin.warn:
                        self.warn(pin.warn, node.rs485_pos)
                    self.pins[pin.key] = "the RS485 direction pin"
                    self.reserved.add(pin.key)
                    node.rs485_pin = pin
        if node.blackbox_pos is not None and not 1 <= node.blackbox <= 64:
            self.err("'blackbox' must keep between 1 and 64 entries", node.blackbox_pos)

        # ---- states
        for k, (nm, pos) in enumerate(node.states):
            if nm in self.state_index:
                self.err(f"state '{nm}' is declared twice", pos)
            else:
                self.state_index[nm] = k

        self.decl_types()

        for d in node.decls:
            if isinstance(d, Const):
                v, t = self.const_value(d.expr)
                if t is not None and self.declare(d.name, Sym("const", t, d, v), d.pos):
                    d.value, d.type = v, t
                elif t is None:
                    self.declare(d.name, Sym("const", "int", d, 0), d.pos)
            elif isinstance(d, Var):
                self.decl_var(d)
            elif isinstance(d, Never):
                continue
            else:
                self.decl_io(d)

        self.check_fns()

        for d in node.decls:
            if isinstance(d, Never):
                self.check_rule(d)

        if node.baud_pos is not None and not (300 <= node.baud <= 2_000_000):
            self.err("serial speed must be between 300 and 2000000", node.baud_pos)

        seen_cmd = set()
        singles = {}
        edges: Dict[Optional[str], set] = {}
        for ev in node.events:
            scope: Dict[str, str] = {}
            if ev.kind in ("every", "heartbeat") and not ev.off:
                v, t = self.const_value(ev.arg)
                if t is not None:
                    if t != "int":
                        self.err(f"'{ev.kind}' needs a duration like 500ms or 2s", ev.pos)
                    elif not (1 <= v <= 1_000_000_000):
                        self.err("duration must be between 1ms and 1000000000ms", ev.pos)
                    else:
                        ev.ms = v
            elif ev.kind == "when":
                t = self.tc(ev.arg, scope)
                if t is not None and t != "bool":
                    self.err("'when' needs a true/false condition", ev.pos, "e.g.  when temp > 600 { ... }")
                if ev.dur is not None:
                    v, t2 = self.const_value(ev.dur)
                    if t2 is not None:
                        if t2 != "int":
                            self.err("'for' needs a duration like 500ms or 2s", ev.pos)
                        elif not 1 <= v <= 1_000_000_000:
                            self.err("duration must be between 1ms and 1000000000ms", ev.pos)
                        else:
                            ev.dur_ms = v
            elif ev.kind == "on":
                if ev.name in RESERVED_COMMANDS:
                    self.err(f"'{ev.name}' is built in", ev.pos)
                key = (ev.name, ev.state)
                if key in seen_cmd:
                    where = f" in state {ev.state}" if ev.state else ""
                    self.err(f"command '{ev.name}' is handled twice{where}", ev.pos)
                seen_cmd.add(key)
                prev = self.cmd_params.get(ev.name)
                if prev is not None and len(prev) != len(ev.params):
                    self.err(f"command '{ev.name}' takes {len(ev.params)} argument(s) here but {len(prev)} elsewhere", ev.pos,
                             "the master sends one fixed format per command")
                for p in ev.params:
                    if p in self.syms:
                        self.err(f"argument '{p}' hides an existing name", ev.pos)
                    elif p in scope:
                        self.err(f"argument '{p}' is listed twice", ev.pos)
                    scope[p] = "int"
                if ev.name not in self.cmd_params:
                    self.cmd_params[ev.name] = list(ev.params)
                    self.commands.append((ev.name, list(ev.params)))
            if ev.kind in ("heartbeat", "start"):
                if ev.kind in singles:
                    self.err(f"only one '{ev.kind}' block is allowed", ev.pos)
                singles[ev.kind] = ev
            if ev.kind in ("enter", "exit") and ev.state is not None:
                dup = [e for e in node.events if e.kind == ev.kind and e.state == ev.state]
                if len(dup) > 1 and dup[0] is not ev:
                    self.err(f"state {ev.state} has two '{ev.kind}' blocks", ev.pos)
            self.ro = {p: "command argument" for p in ev.params}
            self.stmts(ev.body, scope, ev.kind)
            self.ro = {}
            for st in walk_stmts(ev.body):
                if isinstance(st, Goto):
                    edges.setdefault(ev.state, set()).add(st.state)

        for n, f in self.fns.items():
            if n not in self.called:
                self.warn(f"{f.kind} '{n}' is never used", f.pos)

        # ---- state graph
        if node.states and all(n in self.state_index for _, (n, _p) in enumerate(node.states)):
            names = [n for n, _ in node.states]
            global_targets = {t for t in edges.get(None, set()) if t in self.state_index}
            reach, todo = {names[0]}, [names[0]]
            while todo:
                cur = todo.pop()
                for t in (edges.get(cur, set()) | global_targets):
                    if t in self.state_index and t not in reach:
                        reach.add(t)
                        todo.append(t)
            for n, pos in node.states:
                if n not in reach:
                    self.warn(f"state '{n}' is never entered: no goto leads to it", pos)
                elif len(names) > 1 and not ({t for t in edges.get(n, set()) if t in self.state_index} | global_targets):
                    self.warn(f"no goto leaves state '{n}', so the node stays there until it restarts", pos,
                              "add a goto in this state, or in a global handler that applies to every state")

        # ---- peripherals vs. board
        if node.target in ("nano", "uno") and any(d.kind == "tone" for d in ios):
            for d in ios:
                pins = [d.pin] if d.kind == "pwm" else (list(d.cfg.get("pins", {}).values()) if d.kind == "motor" else [])
                for pin in pins:
                    if pin is not None and pin.key in (3, 11):
                        self.err(f"pin {pin.c} cannot do PWM while a tone output is used", d.pos,
                                 "the AVR tone() function takes over Timer2 (pins 3 and 11)")
        if node.target in ("nano", "uno") and any(d.kind == "servo" for d in ios):
            for d in ios:
                if d.kind == "pwm" and d.pin is not None and d.pin.key in (9, 10):
                    self.err(f"pin {d.pin_text} cannot do PWM while a servo is used", d.pin_pos,
                             "the AVR Servo library takes over Timer1 (pins 9 and 10)")
                if d.kind == "motor":
                    for role in ("pwm", "fwd", "rev"):
                        pin = d.cfg.get("pins", {}).get(role)
                        if pin is not None and pin.key in (9, 10):
                            self.err(f"pin {d.pins[role][0]} cannot do PWM while a servo is used", d.pins[role][1],
                                     "the AVR Servo library takes over Timer1 (pins 9 and 10)")

        # ---- safety rules
        outs = [d for d in ios if d.direction == "output"]
        hb = singles.get("heartbeat")
        if outs and hb is None:
            self.err("this node has outputs but no heartbeat", node.pos,
                     "add  heartbeat 2s  (outputs go to their safe state if the master goes quiet), "
                     "or write  heartbeat off  to say you really mean it")
        elif outs and hb is not None and hb.off:
            self.warn("heartbeat is off: outputs keep their last state if the master stops talking", hb.pos)

        for d in ios:
            if d.direction == "input" and d.name not in self.reads:
                self.warn(f"input '{d.name}' is never read", d.pos)
            if d.direction == "output" and d.name not in self.writes:
                self.warn(f"output '{d.name}' is never set (it stays at its safe state)", d.pos)

        ck = Checked(node, self.syms, self.diags)
        ck.messages, ck.commands = self.messages, self.commands
        ck.rules, ck.uses_anim = self.rules, self.uses_anim
        ck.states, ck.state_index, ck.bus = [n for n, _ in node.states], dict(self.state_index), self.bus
        ck.fns = list(node.fns)
        if ck.ok and node.target in TARGETS:
            self.resources(ck)
        return ck

    def resources(self, ck):
        """Worst-case time and memory (stage A4): refuse a node that cannot meet its declared budget."""
        from .cost import analyze, fmt_us
        node = self.node
        rep = analyze(ck)
        if node.budget_ms is not None:
            if not 1 <= node.budget_ms <= 60000:
                self.err("'budget' must be between 1ms and 60s", node.budget_pos)
            elif rep.pass_worst > node.budget_ms * 1000:
                top = "; ".join(f"{lbl}: {fmt_us(us)}" for lbl, us in rep.parts[:3])
                self.err(f"the main loop can take up to {fmt_us(rep.pass_worst)} per pass, but the budget is "
                         f"{node.budget_ms} ms", node.budget_pos,
                         ("biggest contributors: " + top + ". " if top else "") +
                         "make the slow part cheaper, or raise the budget if the node can really be that deaf")
        hb = next((e for e in node.events if e.kind == "heartbeat" and not e.off), None)
        if hb is not None and hb.ms is not None and rep.pass_worst * 2 > hb.ms * 1000:
            self.warn(f"one loop pass can take {fmt_us(rep.pass_worst)} but the heartbeat is only {hb.ms} ms: "
                      "the node may trip its failsafe just from being busy", hb.pos)
        if rep.board_ram:
            pct = 100 * rep.ram_total // rep.board_ram
            if rep.ram_total > rep.board_ram:
                self.err(f"this node needs about {rep.ram_total} bytes of RAM but the board has {rep.board_ram}",
                         node.pos, "fewer pixels, a smaller blackbox, fewer variables (see  rudrac cost)")
            elif pct >= 75:
                self.warn(f"about {pct}% of the board's RAM would be used ({rep.ram_total} of {rep.board_ram} bytes)", node.pos)


def check(node: Node) -> Checked:
    return Checker(node).run()
