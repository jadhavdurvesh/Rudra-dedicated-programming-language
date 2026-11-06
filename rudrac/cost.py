"""Stage A4: worst-case time and memory, from a cost model.

Because the language has no recursion, only bounded loops and no heap, every handler has a finite worst case that
can be computed without running it. The numbers come from a *model* (MODELS below, microseconds per operation on a
typical board of that family), not from measuring your board: they are an estimate you can calibrate
(`rudrac cost file --model my_board.json`), and they ignore interrupts, the serial transmit buffer filling up and
library internals we did not model. Treat the result as a design aid, not a certified bound.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from . import ir as I
from .syntax import (IO, Var)
from .util import ANIM_STEP_MS, TARGETS, US_PERIOD

MODELS = {
    "avr": dict(vm_op=6, node=2, mul=8, div=48, var=1, dread=6, dwrite=6, awrite=12, aread=112, servo=15, tone=25,
                print_=22, i2c_byte=100, i2c_over=60, anim_px=25, fill_px=4, pixel=8, call=10, loop=3,
                rx_poll=8, rx_parse=70, rx_arg=40, show_px=30, show_base=80, sonar_wait=25000, step_pulse=12,
                sample=8, core_ram=200, stack_base=120, frame=10, board_ram=2048),
    "esp": dict(vm_op=1, node=1, mul=1, div=3, var=1, dread=2, dwrite=2, awrite=15, aread=40, servo=15, tone=25,
                print_=8, i2c_byte=100, i2c_over=40, anim_px=3, fill_px=1, pixel=2, call=4, loop=1,
                rx_poll=3, rx_parse=25, rx_arg=15, show_px=30, show_base=80, sonar_wait=25000, step_pulse=8,
                sample=3, core_ram=0, stack_base=400, frame=16, board_ram=0),
}


@dataclass
class CostReport:
    items: List[tuple] = field(default_factory=list)       # (label, microseconds, in_pass)
    fixed: int = 0                                          # work done every pass whatever happens
    timers: int = 0                                         # every/when handlers, all together
    command: int = 0                                        # the most expensive command handler
    pass_worst: int = 0                                     # longest the node can go without reading serial
    parts: List[tuple] = field(default_factory=list)        # biggest contributors to pass_worst
    stack: int = 0
    ram_items: List[tuple] = field(default_factory=list)    # (what, bytes)
    ram_static: int = 0
    board_ram: int = 0
    model: Dict = field(default_factory=dict)
    family: str = ""

    @property
    def ram_total(self) -> int:
        return self.ram_static + self.stack


def load_model(family: str, override_path: Optional[str] = None) -> dict:
    m = dict(MODELS[family])
    if override_path:
        with open(override_path, encoding="utf-8") as f:
            m.update(json.load(f))
    return m


class _Analyzer:
    def __init__(self, ck, m):
        self.ck, self.m = ck, m
        self.syms = ck.syms
        self.ios = {d.name: d for d in ck.node.decls if isinstance(d, IO)}
        self.fn_by_name = {f.name: f for f in ck.fns}
        self.low = I.Lowerer(ck)
        self._fn_cost: Dict[str, int] = {}
        self._fn_stack: Dict[str, int] = {}
        self.rule_cost = 0
        self.enter: Dict[str, int] = {}
        self.exit: Dict[str, int] = {}
        self.max_exit = 0
        self.rule_cost = sum(self.e(self.low.expr(r.expr)) for r in ck.rules)

    # ---- expressions
    def read_cost(self, d: IO) -> int:
        m = self.m
        if d.kind == "digital":
            return 1 if d.cfg.get("debounce") else m["dread"]
        if d.kind == "analog":
            return m["aread"]
        if d.kind == "ultrasonic":
            return 1
        return m["i2c_over"] + m["i2c_byte"] * (d.cfg["bytes"] + (2 if d.cfg["reg"] is not None else 1))

    def e(self, x) -> int:
        m = self.m
        if isinstance(x, (I.K,)):
            return 0
        if isinstance(x, (I.Loc, I.Glob, I.Out)):
            return m["var"]
        if isinstance(x, I.In):
            return self.read_cost(self.ios[x.name])
        if isinstance(x, (I.Neg, I.Not)):
            return m["node"] + self.e(x.e)
        if isinstance(x, I.CallE):
            return m["call"] + sum(self.e(a) for a in x.args) + self.fn_cost(x.fn)
        if isinstance(x, I.Op):
            op = m["mul"] if x.op == "*" else (m["div"] if x.op in ("/", "%") else m["node"])
            return op + self.e(x.l) + self.e(x.r)
        if isinstance(x, I.Elem):
            return m["var"] + m["node"] + self.e(x.idx) + (0 if x.proven else m["node"])
        return m["node"]

    # ---- functions
    def fn_cost(self, name) -> int:
        if name not in self._fn_cost:
            self._fn_cost[name] = 0          # (acyclic, guaranteed by the checker)
            self._fn_cost[name] = self.s(self.low.fn_body(name))
        return self._fn_cost[name]

    def stack_of(self, body, nparams) -> int:
        """Bytes of stack a handler or function can need, counting calls into other functions."""
        frame = self.m["frame"] + 4 * (nparams + I.count_locals(body))
        deepest = 0
        for c in set(I.callees(body)):
            if c not in self._fn_stack:
                self._fn_stack[c] = 0
                self._fn_stack[c] = self.stack_of(self.low.fn_body(c), len(self.fn_by_name[c].params))
            deepest = max(deepest, self._fn_stack[c])
        return frame + deepest

    # ---- output writes
    def write_cost(self, d: IO, with_rules=True) -> int:
        m = self.m
        k = d.kind
        if k == "digital":
            c = m["dwrite"]
        elif k == "pwm":
            c = m["awrite"]
        elif k == "servo":
            c = m["servo"]
        elif k == "tone":
            c = m["tone"]
        elif k == "i2c":
            c = m["i2c_over"] + m["i2c_byte"] * (d.cfg["bytes"] + (2 if d.cfg["reg"] is not None else 1))
        elif k == "motor":
            c = 2 if d.cfg.get("ramp") else (m["awrite"] + m["dwrite"] if d.cfg["style"] == "dir" else 2 * m["awrite"])
        else:                                   # stepper: only stores the target
            c = 2
        return c + (self.rule_cost if with_rules else 0)

    # ---- statements
    def s(self, body) -> int:
        m = self.m
        total = 0
        for st in body:
            if isinstance(st, I.LetS):
                total += self.e(st.e) + 1
            elif isinstance(st, I.SetL):
                total += self.e(st.e) + 1
            elif isinstance(st, I.SetG):
                total += self.e(st.e) + m["var"]
            elif isinstance(st, I.SetO):
                total += self.e(st.e) + self.write_cost(self.ios[st.name])
            elif isinstance(st, I.SetE):
                total += self.e(st.e) + m["var"] + self.e(st.idx) + m["node"] + (0 if st.proven else m["node"])
            elif isinstance(st, I.IfS):
                total += self.e(st.cond) + 1 + max(self.s(st.then), self.s(st.els) if st.els else 0)
            elif isinstance(st, I.Loop):
                total += 2 + (st.hi - st.lo) * (self.s(st.body) + m["loop"])
            elif isinstance(st, I.Ret):
                total += 1 + (self.e(st.e) if st.e is not None else 0)
            elif isinstance(st, I.Brk):
                total += 1
            elif isinstance(st, I.CallS):
                total += self.e(st.call)
            elif isinstance(st, I.SendS):
                total += m["print_"] * (3 if st.e is None else 5) + (self.e(st.e) if st.e is not None else 0)
            elif isinstance(st, I.FillS):
                total += m["fill_px"] * self.ios[st.strip].cfg["count"] + sum(self.e(a) for a in st.args)
            elif isinstance(st, I.PixelS):
                total += m["pixel"] + self.e(st.idx) + sum(self.e(a) for a in st.args)
            elif isinstance(st, I.AnimS):
                total += 2 if st.mode == "off" else m["anim_px"] * self.ios[st.strip].cfg["count"]
                total += sum(self.e(a) for a in st.args)
            elif isinstance(st, I.GotoS):
                total += self.enter.get(st.state, 0) + self.max_exit + 8
        return total


def analyze(ck, model_path: Optional[str] = None, vm: bool = False, vm_slack: int = 256) -> CostReport:
    node = ck.node
    family = TARGETS[node.target]["family"]
    m = load_model(family, model_path)
    if vm:        # every expression node / statement becomes bytecode ops dispatched by the interpreter
        op = m["vm_op"]
        m.update(node=op, var=op, mul=op + m["mul"], div=op + m["div"], call=6 * op, loop=8 * op)
    a = _Analyzer(ck, m)
    rep = CostReport(model=m, family=family)
    ios = [d for d in node.decls if isinstance(d, IO)]
    strips = [d for d in ios if d.kind == "neopixel" and d.direction == "output"]

    # enter / exit bodies first (they contain no goto), then everything else
    for e in node.events:
        if e.kind in ("enter", "exit"):
            c = a.s(a.low.body(e.body, e.params))
            (a.enter if e.kind == "enter" else a.exit)[e.state] = c
    a.max_exit = max(a.exit.values(), default=0)

    pass_parts: List[tuple] = []
    timers = 0
    cmd_best = (0, "")
    stack_worst = 0
    hb_cost = 0
    fixed = m["rx_poll"] + 3
    safe_all = sum(a.write_cost(d, with_rules=False) for d in ios if d.direction == "output" and d.kind != "neopixel") \
        + sum(m["fill_px"] * d.cfg["count"] for d in strips)

    for e in node.events:
        where = f" [{e.state}]" if e.state else ""
        ir_body = a.low.body(e.body, e.params)
        c = a.s(ir_body)
        stack_worst = max(stack_worst, a.stack_of(ir_body, len(e.params)))
        if e.kind == "every":
            rep.items.append((f"every {e.ms}ms{where}", c, True))
            timers += c
            fixed += 3
        elif e.kind == "when":
            rep.items.append((f"when {'... for ' + str(e.dur_ms) + 'ms' if e.dur_ms else '...'}{where}", c, True))
            timers += c
            fixed += a.e(a.low.expr(e.arg)) + 4
        elif e.kind == "on":
            tot = m["rx_parse"] + m["rx_arg"] * len(e.params) + c
            rep.items.append((f"on {e.name}{where}", tot, True))
            if tot > cmd_best[0]:
                cmd_best = (tot, f"on {e.name}{where}")
        elif e.kind == "heartbeat":
            if not e.off:
                hb_cost = safe_all + c
                rep.items.append(("heartbeat (apply safe states + body)", hb_cost, False))
        elif e.kind == "done":
            # runs inside a pass, from the path engine, like a timer body
            rep.items.append((f"done {e.name}", c, True))
            timers += c
        else:
            rep.items.append((f"{e.kind}{where}", c, False))
    for f in node.fns:
        rep.items.append((f"{f.kind} {f.name}", a.fn_cost(f.name), False))

    # fixed per-pass services
    for d in ios:
        if d.direction == "input" and d.kind == "digital" and d.cfg.get("debounce"):
            fixed += m["dread"] + m["sample"]
        if d.kind == "ultrasonic":
            sonar = m["sonar_wait"] + 60
            pass_parts.append((f"ultrasonic '{d.name}' waiting for an echo (every {US_PERIOD} ms)", sonar))
            fixed += sonar
        if d.kind == "motor" and (d.cfg.get("ramp") or d.cfg.get("timeout")):
            fixed += m["awrite"] + m["dwrite"] + 8
        if d.kind == "stepper":
            fixed += m["step_pulse"] + 3 * m["dwrite"] + 6
    for p in node.paths:        # every pass: look at the path; at most one waypoint is entered (one write) per pass
        fixed += 6 * m["node"] + 4
        timers += a.write_cost(a.ios[p.out])
    anim_cost = 0
    for d in strips:
        n = d.cfg["count"]
        if ck.uses_anim:
            anim_cost += m["anim_px"] * n
        pass_parts.append((f"NeoPixel show() for '{d.name}' ({n} pixels, interrupts off)", m["show_base"] + m["show_px"] * n))
        fixed += m["show_base"] + m["show_px"] * n
    fixed += anim_cost

    rep.fixed, rep.timers, rep.command = fixed, timers, cmd_best[0]
    normal = fixed + timers + cmd_best[0]
    hb_pass = m["rx_poll"] + 3 + hb_cost
    rep.pass_worst = max(normal, hb_pass)
    for label, us, in_pass in rep.items:
        if in_pass and (label.startswith("every") or label.startswith("when")):
            pass_parts.append((label, us))
    if cmd_best[0]:
        pass_parts.append((cmd_best[1] + " (slowest command)", cmd_best[0]))
    rep.parts = sorted(pass_parts, key=lambda p: -p[1])

    # ---- memory
    items = []
    if family == "avr":
        items.append(("Arduino core and serial buffers", m["core_ram"]))
    for d in node.decls:
        if isinstance(d, Var):
            for slot, ty in d.slots:
                items.append((f"var {slot}" + (f"[{d.size}]" if d.shape in ("array", "structarray") else ""),
                              (1 if ty == "bool" else 4) * (d.size or 1)))
    for d in ios:
        if d.direction == "output" and d.kind not in ("neopixel",):
            items.append((f"output {d.name}", 1 if d.kind == "digital" else 4))
        if d.kind == "motor":
            items.append((f"motor {d.name} state", 8))
        if d.kind == "stepper":
            items.append((f"stepper {d.name} state", 8))
        if d.kind == "ultrasonic":
            items.append((f"ultrasonic {d.name} state", 8))
        if d.direction == "input" and d.kind == "digital" and d.cfg.get("debounce"):
            items.append((f"debounce {d.name}", 7))
        if d.kind == "i2c" and d.direction == "input":
            items.append((f"i2c {d.name} last value", 4))
        if d.kind == "neopixel":
            items.append((f"NeoPixel {d.name} buffer", 3 * d.cfg["count"] + 30 + (20 if ck.uses_anim else 0)))
        if d.kind == "servo":
            items.append((f"Servo {d.name}", 30))
    if any(d.kind == "i2c" for d in ios):
        items.append(("Wire library buffers", 130 if family == "avr" else 0))
    for p in node.paths:
        items.append((f"path {p.name} state", 9))
    for e in node.events:
        if e.kind == "every":
            items.append(("timer", 4))
        elif e.kind == "when":
            items.append(("when state", 1 + (8 if e.dur_ms else 0)))
    items.append(("serial line buffer and bookkeeping", 40))
    if any(e.kind == "heartbeat" and not e.off for e in node.events):
        items.append(("heartbeat", 5))
    if node.blackbox_pos is not None:
        items.append(("black box", 9 * node.blackbox + 12))
    if ck.states:
        items.append(("state", 1))
    vm_stack_extra = 0
    if vm:
        from . import bytecode as B, genvm
        img = B.assemble(ck)
        cp = genvm.caps(img, vm_slack)
        items.append(("bytecode image buffer (RAM)", cp["cap"]))
        items.append(("VM value and local stacks", 4 * (cp["stack"] + cp["frame"])))
        vm_stack_extra = 6 * cp["calls"] + 40
    rep.ram_items = items
    rep.ram_static = sum(b for _, b in items)
    rep.stack = m["stack_base"] + stack_worst + vm_stack_extra
    rep.board_ram = m["board_ram"]
    return rep


def fmt_us(us: int) -> str:
    return f"{us / 1000:.1f} ms" if us >= 1000 else f"{us} us"


def format_report(rep: CostReport, name: str, budget_ms: Optional[int] = None) -> str:
    L = [f"rudrac cost {name}   (model: {rep.family}; estimates, not measurements)", "", "Worst case per handler:"]
    for label, us, _ in rep.items:
        L.append(f"  {label:44} {fmt_us(us):>10}")
    L += ["", "One pass of the main loop (the longest the node can go without reading its serial port):",
          f"  fixed work every pass (serial poll, timers, sensors, show) {fmt_us(rep.fixed):>10}",
          f"  all timer handlers firing in the same pass               {fmt_us(rep.timers):>10}",
          f"  the slowest command handler                              {fmt_us(rep.command):>10}",
          f"  worst case                                               {fmt_us(rep.pass_worst):>10}"
          + (f"   (budget {budget_ms} ms)" if budget_ms else "")]
    if rep.parts:
        L.append("  biggest contributors: " + "; ".join(f"{lbl} {fmt_us(us)}" for lbl, us in rep.parts[:3]))
    L += ["", "Memory:"]
    for what, b in sorted(rep.ram_items, key=lambda x: -x[1])[:6]:
        L.append(f"  {what:44} {b:>6} B")
    L.append(f"  static total {rep.ram_static} B, worst-case stack {rep.stack} B"
             + (f", board has {rep.board_ram} B" if rep.board_ram else " (not estimated for this board)"))
    if rep.board_ram:
        L.append(f"  estimated use: {100 * rep.ram_total // rep.board_ram}% of RAM")
    return "\n".join(L) + "\n"
