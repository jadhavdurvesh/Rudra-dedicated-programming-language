"""C++ (Arduino framework) code generator. Works for AVR and ESP boards."""
from __future__ import annotations

import os

from .check import Checked
from . import ir as I
from .link import protocol_hash
from .syntax import (IO, Var, fmt_expr)
from .util import ANIM_MODES, ANIM_STEP_MS, US_PERIOD, VERSION, path_interval


def _cn(slot: str) -> str:
    return "v_" + slot.replace(".", "__")


def _lit(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    return f"{v}L" if v >= 0 else f"({v}L)"


def generate(ck: Checked, source_name: str = "<input>", vm=None, vm_slack: int = 256) -> str:
    if vm is None:
        vm = os.environ.get("RUDRA_ENGINE") == "vm"
    node, syms = ck.node, ck.syms
    ios = [d for d in node.decls if isinstance(d, IO)]
    vars_ = [d for d in node.decls if isinstance(d, Var)]
    outs = [d for d in ios if d.direction == "output"]
    plain_outs = [d for d in outs if d.kind != "neopixel"]
    strips = [d for d in outs if d.kind == "neopixel"]
    motors = [d for d in outs if d.kind == "motor"]
    steppers = [d for d in outs if d.kind == "stepper"]
    ios_by = {d.name: d for d in node.decls if isinstance(d, IO)}
    sonars = [d for d in ios if d.kind == "ultrasonic"]
    debounced = [d for d in ios if d.direction == "input" and d.kind == "digital" and d.cfg.get("debounce")]
    need_mstep = any(d.cfg.get("ramp") or d.cfg.get("timeout") for d in motors)
    has_states = bool(ck.states)
    bus = ck.bus
    bb = node.blackbox_pos is not None
    de = node.rs485_pin
    events = list(enumerate(node.events))
    hb = next(((i, e) for i, e in events if e.kind == "heartbeat" and not e.off), None)
    has_servo = any(d.kind == "servo" for d in ios)
    has_i2c = any(d.kind == "i2c" for d in ios)
    rules = ck.rules
    anim = ck.uses_anim
    maxp = max([len(e.params) for _, e in events if e.kind == "on"] or [0])

    low = I.Lowerer(ck)

    def ge(e):
        if not isinstance(e, I.EXPR):                  # an AST expression (rules, 'when' conditions)
            e = low.expr(e)
        if isinstance(e, I.K):
            return _lit(e.v)
        if isinstance(e, I.Loc):
            return f"a_{e.name}"
        if isinstance(e, I.Glob):
            return _cn(e.slot)
        if isinstance(e, I.In):
            return f"i_{e.name}()"
        if isinstance(e, I.Out):
            return f"o_{e.name}"
        if isinstance(e, I.Running):
            return f"(pst_{e.name} != 0)"
        if isinstance(e, I.Elem):
            cn, ix = _cn(e.slot), ge(e.idx)
            if e.proven:
                return f"{cn}[{ix}]"
            return f"rt_get{'b' if e.ty == 'bool' else 'i'}({cn}, {e.size}, {ix})"
        if isinstance(e, I.Neg):
            return f"rt_neg({ge(e.e)})"
        if isinstance(e, I.Not):
            return f"(!{ge(e.e)})"
        if isinstance(e, I.CallE):
            return f"f_{e.fn}({', '.join(ge(a) for a in e.args)})"
        if isinstance(e, I.Op):
            l, r = ge(e.l), ge(e.r)
            fn = {"/": "rt_div", "%": "rt_mod", "+": "rt_add", "-": "rt_sub", "*": "rt_mul"}.get(e.op)
            if fn:
                return f"{fn}({l}, {r})"
            return f"({l} {e.op} {r})"
        raise AssertionError(e)

    def gs(body, ind):
        pad = "  " * ind
        out = []
        for s in body:
            if isinstance(s, I.SetL):
                out.append(f"{pad}a_{s.name} = {ge(s.e)};")
            elif isinstance(s, I.SetG):
                out.append(f"{pad}{_cn(s.slot)} = {ge(s.e)};")
            elif isinstance(s, I.SetO):
                out.append(f"{pad}r_set_{s.name}({ge(s.e)});")
            elif isinstance(s, I.RunS):
                out.append(f"{pad}path_run_{s.name}();")
            elif isinstance(s, I.StopS):
                out.append(f"{pad}path_stop_{s.out}();")
            elif isinstance(s, I.CancelS):
                out.append(f"{pad}path_cancel_{s.out}();")
            elif isinstance(s, I.SetE):
                cn, v = _cn(s.slot), ge(s.e)
                if s.proven:
                    out.append(f"{pad}{cn}[{ge(s.idx)}] = {v};")
                else:
                    ty = "bool" if s.ty == "bool" else "rt_int"
                    log = " bb_log(6, t_i);" if bb else ""
                    out.append(f"{pad}{{ rt_int t_i = {ge(s.idx)}; {ty} t_v = {v};")
                    out.append(f"{pad}  if (t_i >= 0 && t_i < {s.size}) {{ {cn}[t_i] = t_v; }} else {{ g_blocked++;{log} }} }}")
            elif isinstance(s, I.LetS):
                out.append(f"{pad}{'bool' if s.ty == 'bool' else 'rt_int'} a_{s.name} = {ge(s.e)};")
                out.append(f"{pad}(void)a_{s.name};")
            elif isinstance(s, I.Ret):
                out.append(f"{pad}return {ge(s.e)};" if s.e is not None else f"{pad}return;")
            elif isinstance(s, I.Brk):
                out.append(f"{pad}break;")
            elif isinstance(s, I.CallS):
                out.append(f"{pad}{ge(s.call)};")
            elif isinstance(s, I.Loop):
                out.append(f"{pad}for (rt_int a_{s.var} = {_lit(s.lo)}; a_{s.var} < {_lit(s.hi)}; a_{s.var}++) {{")
                out += gs(s.body, ind + 1)
                out.append(f"{pad}}}")
            elif isinstance(s, I.IfS):
                out.append(f"{pad}if ({ge(s.cond)}) {{")
                out += gs(s.then, ind + 1)
                if s.els:
                    out.append(f"{pad}}} else {{")
                    out += gs(s.els, ind + 1)
                out.append(f"{pad}}}")
            elif isinstance(s, I.FillS):
                out.append(f"{pad}r_fill_{s.strip}({', '.join(ge(a) for a in s.args)});")
            elif isinstance(s, I.PixelS):
                out.append(f"{pad}r_pixel_{s.strip}({ge(s.idx)}, {', '.join(ge(a) for a in s.args)});")
            elif isinstance(s, I.AnimS):
                if s.mode == "off":
                    out.append(f"{pad}r_anim_off_{s.strip}();")
                elif s.mode == "rainbow":
                    out.append(f"{pad}r_animate_{s.strip}({ANIM_MODES['rainbow']}, 0, 0, 0, {ge(s.args[0])});")
                else:
                    a = ", ".join(ge(x) for x in s.args)
                    out.append(f"{pad}r_animate_{s.strip}({ANIM_MODES[s.mode]}, {a});")
            elif isinstance(s, I.GotoS):
                out.append(f"{pad}r_goto({s.index});")
            elif isinstance(s, I.SendS):
                if s.e is None:
                    out.append(f'{pad}tx_begin(); TX_STR("{s.msg}"); tx_end();')
                else:
                    v = ge(s.e)
                    num = f"(long)({v} ? 1 : 0)" if s.is_bool else f"(long)({v})"
                    out.append(f'{pad}tx_begin(); TX_STR("{s.msg} "); TX_NUM({num}); tx_end();')
        return out

    L = []
    w = L.append
    w(f"// Generated by rudrac {VERSION} -- do not edit by hand.")
    w(f"// node: {node.name}   target: {node.target}   source: {source_name}")
    w("#include <Arduino.h>")
    w("#include <stdint.h>")
    w("#include <stdlib.h>")
    w("#include <string.h>")
    if has_i2c:
        w("#include <Wire.h>")
    if has_servo:
        w("#if defined(ARDUINO_ARCH_ESP32)")
        w("#include <ESP32Servo.h>      /* library: ESP32Servo */")
        w("#else")
        w("#include <Servo.h>")
        w("#endif")
    if strips:
        w("#include <Adafruit_NeoPixel.h>   /* library: Adafruit NeoPixel */")
    w("")
    w("typedef int32_t rt_int;")
    w("static uint32_t rt_now = 0;   /* one clock reading per loop pass: every timer in the sketch uses it */")
    w("/* Arithmetic wraps around at 32 bits (no undefined behaviour); division by zero gives 0. */")
    w("static inline rt_int rt_add(rt_int a, rt_int b) { return (rt_int)((uint32_t)a + (uint32_t)b); }")
    w("static inline rt_int rt_sub(rt_int a, rt_int b) { return (rt_int)((uint32_t)a - (uint32_t)b); }")
    w("static inline rt_int rt_mul(rt_int a, rt_int b) { return (rt_int)((uint32_t)a * (uint32_t)b); }")
    w("static inline rt_int rt_neg(rt_int a) { return (rt_int)(0U - (uint32_t)a); }")
    w("static inline rt_int rt_div(rt_int a, rt_int b) { return b == 0 ? 0 : (b == -1 ? rt_neg(a) : a / b); }")
    w("static inline rt_int rt_mod(rt_int a, rt_int b) { return (b == 0 || b == -1) ? 0 : a % b; }")
    if node.fns:
        w("#define RT_UNUSED __attribute__((unused))")
    if strips:
        w("static inline rt_int rt_c255(rt_int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }")
        w("static inline uint32_t rt_rgb(rt_int r, rt_int g, rt_int b) {")
        w("  return ((uint32_t)rt_c255(r) << 16) | ((uint32_t)rt_c255(g) << 8) | (uint32_t)rt_c255(b);")
        w("}")
    if anim:
        w("")
        w("/* Animation frames: integer-only, drawn from the elapsed time so they never drift. */")
        w("static inline rt_int rt_scale(rt_int c, uint32_t pct) { return (rt_int)(((uint32_t)c * pct) / 100UL); }")
        w("static void rt_anim_draw(Adafruit_NeoPixel &np, uint16_t n, uint8_t mode, uint32_t t, uint32_t per,")
        w("                         rt_int r, rt_int g, rt_int b) {")
        w("  t %= per;")
        w("  if (mode == 1) {                                   /* blink */")
        w("    uint32_t col = (t < per / 2) ? rt_rgb(r, g, b) : 0;")
        w("    for (uint16_t k = 0; k < n; k++) np.setPixelColor(k, col);")
        w("  } else if (mode == 2) {                            /* pulse */")
        w("    uint32_t h = per / 2;")
        w("    uint32_t x = (t < h) ? t : per - t;")
        w("    uint32_t lvl = (x * 100UL) / h;")
        w("    if (lvl > 100UL) lvl = 100UL;")
        w("    uint32_t col = rt_rgb(rt_scale(r, lvl), rt_scale(g, lvl), rt_scale(b, lvl));")
        w("    for (uint16_t k = 0; k < n; k++) np.setPixelColor(k, col);")
        w("  } else if (mode == 3) {                            /* chase */")
        w("    uint16_t head = (uint16_t)((t * (uint32_t)n) / per);")
        w("    uint16_t prev = (uint16_t)((head + n - 1) % n);")
        w("    for (uint16_t k = 0; k < n; k++) np.setPixelColor(k, 0);")
        w("    np.setPixelColor(prev, rt_rgb(rt_scale(r, 30UL), rt_scale(g, 30UL), rt_scale(b, 30UL)));")
        w("    np.setPixelColor(head, rt_rgb(r, g, b));")
        w("  } else if (mode == 4) {                            /* rainbow */")
        w("    uint32_t base = (t * 256UL) / per;")
        w("    for (uint16_t k = 0; k < n; k++) {")
        w("      uint32_t p = (base + ((uint32_t)k * 256UL) / n) % 256UL;")
        w("      rt_int rr, gg, bb;")
        w("      if (p < 85) { rr = (rt_int)(255 - p * 3); gg = 0; bb = (rt_int)(p * 3); }")
        w("      else if (p < 170) { p -= 85; rr = 0; gg = (rt_int)(p * 3); bb = (rt_int)(255 - p * 3); }")
        w("      else { p -= 170; rr = (rt_int)(p * 3); gg = (rt_int)(255 - p * 3); bb = 0; }")
        w("      np.setPixelColor(k, rt_rgb(rr, gg, bb));")
        w("    }")
        w("  }")
        w("}")
    w("")

    w("// ---- pins and devices ----")
    for d in ios:
        if d.kind == "i2c":
            w(f"static const uint8_t A_{d.name} = 0x{d.cfg['addr']:02X};")
        elif d.kind in ("motor", "stepper", "ultrasonic"):
            for role, pin in d.cfg["pins"].items():
                w(f"static const uint8_t P_{d.name}_{role} = {pin.c};")
        else:
            w(f"static const uint8_t P_{d.name} = {d.pin.c};")
    if de is not None:
        w(f"static const uint8_t P_rs485_de = {de.c};   /* RS485 driver-enable */")
    for d in outs:
        if d.kind == "servo":
            w(f"static Servo s_{d.name};")
        elif d.kind == "neopixel":
            order = "NEO_RGB" if d.cfg["rgb"] else "NEO_GRB"
            w(f"#define N_{d.name} {d.cfg['count']}")
            w(f"static Adafruit_NeoPixel np_{d.name}(N_{d.name}, P_{d.name}, {order} + NEO_KHZ800);")
            w(f"static bool np_dirty_{d.name} = false;")
    w("")

    w("// ---- state ----")
    w("static uint32_t g_blocked = 0;   /* writes refused by a 'never' rule (reported by STATUS) */")
    for d in vars_:
        for slot, ty in d.slots:
            ct = "bool" if ty == "bool" else "rt_int"
            if d.shape == "scalar":
                w(f"static {ct} {_cn(slot)} __attribute__((unused)) = {_lit(d.init)};")
            elif d.shape == "struct":
                w(f"static {ct} {_cn(slot)} __attribute__((unused)) = {'false' if ty == 'bool' else '0'};")
            else:
                w(f"static {ct} {_cn(slot)}[{d.size}] __attribute__((unused));   /* all zero at boot */")
    if any(d.shape in ("array", "structarray") for d in vars_):
        w("static inline rt_int rt_geti(const rt_int *a, rt_int n, rt_int i) { return (i >= 0 && i < n) ? a[i] : 0; }")
        w("static inline bool rt_getb(const bool *a, rt_int n, rt_int i) { return (i >= 0 && i < n) ? a[i] : false; }")
    for d in plain_outs:
        w(f"static {'bool' if d.kind == 'digital' else 'rt_int'} o_{d.name} = {'false' if d.kind == 'digital' else '0'};")
    for d in motors:
        w(f"static rt_int m_cur_{d.name} = 0;        /* speed actually applied (ramps towards o_{d.name}) */")
        w(f"static uint32_t m_last_{d.name} = 0;     /* when the program last wrote it */")
    for d in steppers:
        w(f"static rt_int sp_{d.name} = 0;           /* current position in steps (boot = 0) */")
        w(f"static uint32_t t_stp_{d.name} = 0;")
        w(f"static uint32_t st_iv_{d.name} = {d.cfg['interval']}UL;   /* step interval now: paths can change it */")
    for p in node.paths:
        n = p.name
        w(f"static uint8_t pst_{n} = 0;   /* path {n}: 0 idle, 1 moving, 2 waiting */")
        w(f"static uint16_t pidx_{n} = 0, prep_{n} = 0;")
        w(f"static uint32_t pt_{n} = 0;")
    for d in sonars:
        w(f"static rt_int us_v_{d.name} = 0;   /* last distance in cm; 0 = no echo */")
        w(f"static uint32_t us_t_{d.name} = 0;")
    for d in debounced:
        w(f"static bool db_v_{d.name} = false, db_c_{d.name} = false;   /* debounced value, candidate */")
        w(f"static uint32_t db_t_{d.name} = 0;")
    if has_states:
        w("static uint8_t st = 0;   /* current state; 0 is the first one declared */")
    for d in ios:
        if d.direction == "input" and d.kind == "i2c":
            w(f"static rt_int l_{d.name} = 0;   /* last good reading */")
    if anim:
        for d in strips:
            n = d.name
            w(f"static uint8_t an_mode_{n} = 0;")
            w(f"static rt_int an_r_{n} = 0, an_g_{n} = 0, an_b_{n} = 0;")
            w(f"static uint32_t an_per_{n} = 1000UL, an_t0_{n} = 0;")
    w("")

    # ---- serial output helpers (one place that knows about the bus prefix and the RS485 driver)
    if bus:
        w("static bool rt_quiet = false;   /* a broadcast (@0) is executed but never answered */")
        w("static void tx_begin(void) {")
        w("  if (rt_quiet) return;")
        if de is not None:
            w("  digitalWrite(P_rs485_de, HIGH);")
        w(f'  Serial.print(F("@{node.address} "));')
        w("}")
        w("static void tx_end(void) {")
        w("  if (rt_quiet) return;")
        w("  Serial.println();")
        if de is not None:
            w("  Serial.flush();")
            w("  digitalWrite(P_rs485_de, LOW);")
        w("}")
        w("#define TX_STR(s) do { if (!rt_quiet) Serial.print(F(s)); } while (0)")
        w("#define TX_NUM(v) do { if (!rt_quiet) Serial.print((long)(v)); } while (0)")
    else:
        w("#define tx_begin() ((void)0)")
        w("#define tx_end() Serial.println()")
        w("#define TX_STR(s) Serial.print(F(s))")
        w("#define TX_NUM(v) Serial.print((long)(v))")
    w("")
    if bb:
        w(f"/* Black box: the last {node.blackbox} events, kept in RAM (lost on reset). Read with the LOG command. */")
        w(f"#define BB_N {node.blackbox}")
        w("static uint32_t bb_t[BB_N];")
        w("static uint8_t bb_c[BB_N];")
        w("static rt_int bb_a[BB_N];")
        w("static uint8_t bb_head = 0, bb_cnt = 0;")
        w("static uint32_t bb_total = 0;")
        w("static void bb_log(uint8_t code, rt_int arg) {")
        w("  bb_t[bb_head] = rt_now; bb_c[bb_head] = code; bb_a[bb_head] = arg;")
        w("  bb_head = (uint8_t)((bb_head + 1) % BB_N);")
        w("  if (bb_cnt < BB_N) bb_cnt++;")
        w("  bb_total++;")
        w("}")
        w("static void bb_dump(void) {")
        w("  uint8_t first = (uint8_t)((bb_head + BB_N - bb_cnt) % BB_N);")
        w("  for (uint8_t k = 0; k < bb_cnt; k++) {")
        w("    uint8_t j = (uint8_t)((first + k) % BB_N);")
        w('    tx_begin(); TX_STR("LOG "); TX_NUM(bb_t[j] & 0x7FFFFFFFUL); TX_STR(" "); TX_NUM(bb_c[j]); TX_STR(" "); TX_NUM(bb_a[j]); tx_end();')
        w("  }")
        w('  tx_begin(); TX_STR("LOGEND "); TX_NUM(bb_total & 0x7FFFFFFFUL); tx_end();')
        w("}")
        w("")
        BB = lambda code, arg: f"bb_log({code}, {arg});"
    else:
        BB = lambda code, arg: ""

    if rules:
        w("/* Interlocks. Every output write is checked against these, so no code path can break them. */")
        w("static uint8_t rt_rule(void) {   /* number of the first rule that is broken, or 0 */")
        for k, r in enumerate(rules, 1):
            w(f"  if ({ge(r.expr)}) return {k};   /* never {fmt_expr(r.expr)} */")
        w("  return 0;")
        w("}")
        w("")

    w("// ---- inputs / outputs ----")
    for d in ios:
        n = d.name
        if d.direction == "input":
            if d.kind == "ultrasonic":
                w(f"static inline rt_int i_{n}(void) {{ return us_v_{n}; }}")
                w(f"/* HC-SR04 style sensor, measured every {US_PERIOD} ms (a measurement can block up to 25 ms) */")
                w(f"static void r_sample_{n}(uint32_t now) {{")
                w(f"  if ((uint32_t)(now - us_t_{n}) < {US_PERIOD}UL) return;")
                w(f"  us_t_{n} = ((uint32_t)(now - us_t_{n}) >= {2 * US_PERIOD}UL) ? now : us_t_{n} + {US_PERIOD}UL;")
                w(f"  digitalWrite(P_{n}_trig, LOW);")
                w("  delayMicroseconds(2);")
                w(f"  digitalWrite(P_{n}_trig, HIGH);")
                w("  delayMicroseconds(10);")
                w(f"  digitalWrite(P_{n}_trig, LOW);")
                w(f"  unsigned long us = pulseIn(P_{n}_echo, HIGH, 25000UL);")
                w(f"  us_v_{n} = (rt_int)(us / 58UL);")
                w("}")
                continue
            if d.kind == "digital":
                raw = f"digitalRead(P_{n}) == {'LOW' if d.invert else 'HIGH'}"
                if d.cfg.get("debounce"):
                    w(f"static inline bool i_raw_{n}(void) {{ return {raw}; }}")
                    w(f"static inline bool i_{n}(void) {{ return db_v_{n}; }}")
                    w(f"static void r_sample_{n}(uint32_t now) {{")
                    w(f"  bool raw = i_raw_{n}();")
                    w(f"  if (raw == db_v_{n}) {{ db_c_{n} = raw; }}")
                    w(f"  else if (raw != db_c_{n}) {{ db_c_{n} = raw; db_t_{n} = now; }}")
                    w(f"  else if ((uint32_t)(now - db_t_{n}) >= {d.cfg['debounce']}UL) {{ db_v_{n} = raw; }}")
                    w("}")
                else:
                    w(f"static inline bool i_{n}(void) {{ return {raw}; }}")
            elif d.kind == "analog":
                w(f"static inline rt_int i_{n}(void) {{")
                w(f"  rt_int r = (rt_int)analogRead(P_{n});")
                w("#if defined(ARDUINO_ARCH_ESP32)")
                w("  r >>= 2;   /* 12-bit ADC -> same 0..1023 scale as AVR */")
                w("#endif")
                w("  return r;")
                w("}")
            else:  # i2c register read, big-endian; a failed read returns the last good value
                c, nb = d.cfg, d.cfg["bytes"]
                w(f"static inline rt_int i_{n}(void) {{")
                if c["reg"] is not None:
                    w(f"  Wire.beginTransmission(A_{n});")
                    w(f"  Wire.write((uint8_t){c['reg']});")
                    w(f"  if (Wire.endTransmission(false) != 0) return l_{n};")
                w(f"  if ((int)Wire.requestFrom((uint8_t)A_{n}, (uint8_t){nb}) != {nb}) return l_{n};")
                w("  uint32_t u = 0;")
                for _ in range(nb):
                    w("  u = (u << 8) | (uint8_t)Wire.read();")
                w("  rt_int r = (rt_int)u;")
                if c["signed"] and nb < 4:
                    ext = (0xFFFFFFFF << (8 * nb)) & 0xFFFFFFFF
                    w(f"  if (u & 0x{1 << (8 * nb - 1):X}UL) r = (rt_int)(u | 0x{ext:X}UL);")
                w(f"  l_{n} = r;")
                w("  return r;")
                w("}")
            continue

        if d.kind == "neopixel":
            cancel = f"  an_mode_{n} = 0;   /* a fixed colour stops the animation */" if anim else None
            w(f"static void r_fill_{n}(rt_int r, rt_int g, rt_int b) {{")
            if cancel:
                w(cancel)
            w("  uint32_t c = rt_rgb(r, g, b);")
            w(f"  for (uint16_t k = 0; k < N_{n}; k++) np_{n}.setPixelColor(k, c);")
            w(f"  np_dirty_{n} = true;")
            w("}")
            w(f"static inline void r_pixel_{n}(rt_int i, rt_int r, rt_int g, rt_int b) {{")
            if cancel:
                w(cancel)
            w(f"  if (i < 0 || i >= N_{n}) return;")
            w(f"  np_{n}.setPixelColor((uint16_t)i, rt_rgb(r, g, b));")
            w(f"  np_dirty_{n} = true;")
            w("}")
            if anim:
                w(f"static void r_animate_{n}(uint8_t mode, rt_int r, rt_int g, rt_int b, rt_int per) {{")
                w("  if (per < 40) per = 40;")
                w("  if (per > 3600000) per = 3600000;")
                w(f"  an_mode_{n} = mode;")
                w(f"  an_r_{n} = rt_c255(r); an_g_{n} = rt_c255(g); an_b_{n} = rt_c255(b);")
                w(f"  an_per_{n} = (uint32_t)per;")
                w(f"  an_t0_{n} = rt_now;")
                w(f"  rt_anim_draw(np_{n}, N_{n}, mode, 0, an_per_{n}, an_r_{n}, an_g_{n}, an_b_{n});")
                w(f"  np_dirty_{n} = true;")
                w("}")
                w(f"static inline void r_anim_off_{n}(void) {{ an_mode_{n} = 0; }}")
            continue

        if d.kind == "motor":
            c = d.cfg
            L_ = c["limit"]
            w(f"static void r_hw_{n}(void) {{   /* writes m_cur_{n} to the driver */")
            w(f"  rt_int c = m_cur_{n};")
            w("  int duty = (int)(((c < 0 ? -c : c) * 255) / 100);")
            if c["style"] == "dir":
                w(f"  digitalWrite(P_{n}_dir, c < 0 ? LOW : HIGH);")
                w(f"  analogWrite(P_{n}_pwm, duty);")
            else:
                w(f"  if (c >= 0) {{ analogWrite(P_{n}_rev, 0); analogWrite(P_{n}_fwd, duty); }}")
                w(f"  else {{ analogWrite(P_{n}_fwd, 0); analogWrite(P_{n}_rev, duty); }}")
            w("}")
            w(f"static void r_set_{n}(rt_int v) {{")
            w(f"  if (v < -{L_}) v = -{L_};")
            w(f"  if (v > {L_}) v = {L_};")
            w(f"  m_last_{n} = rt_now;")
            if rules:
                w(f"  rt_int old = o_{n};")
                w(f"  o_{n} = v;")
                w("  uint8_t rr = rt_rule();")
                w(f"  if (rr) {{ o_{n} = old; g_blocked++; {BB(4, 'rr')} return; }}")
            else:
                w(f"  o_{n} = v;")
            if not c.get("ramp"):
                w(f"  m_cur_{n} = v;")
                w(f"  r_hw_{n}();")
            w("}")
            continue
        if d.kind == "stepper":
            w(f"static void r_set_{n}(rt_int v) {{   /* sets the target position; stepping happens in r_stepper_step */")
            if rules:
                w(f"  rt_int old = o_{n};")
                w(f"  o_{n} = v;")
                w("  uint8_t rr = rt_rule();")
                w(f"  if (rr) {{ o_{n} = old; g_blocked++; {BB(4, 'rr')} return; }}")
            else:
                w(f"  o_{n} = v;")
            w("}")
            continue

        # plain outputs: r_hw_<n> pushes the stored value to the pin, r_set_<n> is the checked setter
        if d.kind == "digital":
            on, off = ("LOW", "HIGH") if d.invert else ("HIGH", "LOW")
            w(f"static void r_hw_{n}(void) {{ digitalWrite(P_{n}, o_{n} ? {on} : {off}); }}")
            ty, clamp = "bool", []
        elif d.kind == "pwm":
            w(f"static void r_hw_{n}(void) {{")
            w(f"  int d = (int)((o_{n} * 255) / 100);")
            w(f"  analogWrite(P_{n}, {'255 - d' if d.invert else 'd'});")
            w("}")
            ty, clamp = "rt_int", ["  if (v < 0) v = 0;", "  if (v > 100) v = 100;"]
        elif d.kind == "tone":
            w(f"static void r_hw_{n}(void) {{ if (o_{n} > 0) tone(P_{n}, (unsigned int)o_{n}); else noTone(P_{n}); }}")
            ty, clamp = "rt_int", ["  if (v < 0) v = 0;", "  if (v > 20000) v = 20000;"]
        elif d.kind == "servo":
            w(f"static void r_hw_{n}(void) {{ s_{n}.write((int)o_{n}); }}")
            ty, clamp = "rt_int", ["  if (v < 0) v = 0;", "  if (v > 180) v = 180;"]
        else:  # i2c register write, big-endian
            c, nb = d.cfg, d.cfg["bytes"]
            w(f"static void r_hw_{n}(void) {{")
            w(f"  Wire.beginTransmission(A_{n});")
            if c["reg"] is not None:
                w(f"  Wire.write((uint8_t){c['reg']});")
            for k in reversed(range(nb)):
                w(f"  Wire.write((uint8_t)((uint32_t)o_{n} >> {8 * k}));" if k else f"  Wire.write((uint8_t)((uint32_t)o_{n}));")
            w("  Wire.endTransmission();")
            w("}")
            ty, clamp = "rt_int", []
        w(f"static void r_set_{n}({ty} v) {{")
        for line in clamp:
            w(line)
        if rules:
            w(f"  {ty} old = o_{n};")
            w(f"  o_{n} = v;")
            w("  uint8_t rr = rt_rule();")
            w(f"  if (rr) {{ o_{n} = old; g_blocked++; {BB(4, 'rr')} return; }}")
        else:
            w(f"  o_{n} = v;")
        w(f"  r_hw_{n}();")
        w("}")
    w("")
    w("/* Neopixel strips are sent once per loop (show() blocks interrupts, so never per statement). */")
    if strips:
        w("static void r_flush(void) {")
        for d in strips:
            w(f"  if (np_dirty_{d.name}) {{ np_dirty_{d.name} = false; np_{d.name}.show(); }}")
        w("}")
    else:
        w("#define r_flush() ((void)0)")
    w("")
    w("/* Every output's declared safe state: used at boot and when the master goes quiet.")
    w("   Stored first, written second, so the result never depends on the order of the outputs.")
    w("   The compiler has already proved that this state satisfies every 'never' rule. */")
    w("static void r_apply_safe(void) {")
    for p in node.paths:
        w(f"  pst_{p.name} = 0;   /* a safe state ends every path */")
    for d in steppers:
        w(f"  st_iv_{d.name} = {d.cfg['interval']}UL;")
    for d in plain_outs:
        w(f"  o_{d.name} = {_lit(d.cfg['safe'])};")
    for d in motors:
        w(f"  m_cur_{d.name} = {_lit(d.cfg['safe'])};")
    for d in plain_outs:
        if d.kind != "stepper":
            w(f"  r_hw_{d.name}();")
    for d in strips:
        sv = d.cfg["safe"]
        w(f"  r_fill_{d.name}({sv[0]}, {sv[1]}, {sv[2]});")
    if not outs:
        w("  /* no outputs */")
    w("}")
    w("")

    w("// ---- timers ----")
    for i, e in events:
        if e.kind == "every":
            w(f"static uint32_t t_{i} = 0;")
        elif e.kind == "when":
            w(f"static bool w_{i} = false;")
            if e.dur_ms:
                w(f"static bool wo_{i} = false;")
                w(f"static uint32_t wt_{i} = 0;")
    if anim:
        w("static uint32_t t_anim = 0;")
    if need_mstep:
        w("static uint32_t t_mot = 0;")
    if hb:
        w("static uint32_t hb_last = 0;")
        w("static bool hb_tripped = false;")
        w("#define RT_HEARD() do { hb_last = rt_now; hb_tripped = false; } while (0)")
    else:
        w("#define RT_HEARD() ((void)0)")
    w("")

    if need_mstep:
        w("/* Motors: ramp towards the target and enforce the write timeout, every 20 ms. */")
        w("static void r_motor_step(uint32_t now) {")
        w("  (void)now;")
        for k, d in enumerate(motors, 1):
            n, c = d.name, d.cfg
            if not (c.get("ramp") or c.get("timeout")):
                continue
            sv = _lit(c["safe"])
            w("  {")
            first = True
            if c.get("timeout"):
                w(f"    if ((o_{n} != {sv} || m_cur_{n} != {sv}) && (uint32_t)(now - m_last_{n}) >= {c['timeout']}UL) {{")
                w(f"      o_{n} = {sv}; m_cur_{n} = {sv}; r_hw_{n}();   /* the program stopped writing: go to the safe state */")
                w(f"      {BB(5, k)}")
                w("    }")
                first = False
            if c.get("ramp"):
                kw = "if" if first else "else if"
                w(f"    {kw} (m_cur_{n} != o_{n}) {{")
                w(f"      rt_int diff = o_{n} - m_cur_{n}, nx;")
                w(f"      if (diff > {c['ramp_step']}) nx = m_cur_{n} + {c['ramp_step']};")
                w(f"      else if (diff < -{c['ramp_step']}) nx = m_cur_{n} - {c['ramp_step']};")
                w(f"      else nx = o_{n};")
                w(f"      if ((m_cur_{n} > 0 && nx < 0) || (m_cur_{n} < 0 && nx > 0)) nx = 0;   /* always pass through stop */")
                w(f"      m_cur_{n} = nx; r_hw_{n}();")
                w("    }")
            w("  }")
        w("}")
        w("")
    if node.paths:
        w("// ---- paths: waypoint tables run by the main loop (a safe state ends them) ----")
        by_out = {}
        for p in node.paths:
            by_out.setdefault(p.out, []).append(p)
            n = p.name
            w(f"static const long pv_{n}[{len(p.points)}] = {{{', '.join(str(pt.v) for pt in p.points)}}};")
            if p.kind == "stepper":
                ivs = [path_interval(pt.sp, ios_by[p.out].cfg['interval']) if pt.sp is not None else 0 for pt in p.points]
                w(f"static const uint16_t piv_{n}[{len(p.points)}] = {{{', '.join(str(v) for v in ivs)}}};   /* ms per step; 0 = the declared rate */")
                w(f"static const uint16_t pz_{n}[{len(p.points)}] = {{{', '.join(str(pt.zn) for pt in p.points)}}};")
            w(f"static const uint32_t pw_{n}[{len(p.points)}] = {{{', '.join(str(pt.wt) + 'UL' for pt in p.points)}}};")
        for out, ps in by_out.items():
            w(f"static __attribute__((unused)) void path_cancel_{out}(void) {{")
            for q in ps:
                w(f"  pst_{q.name} = 0;")
            if ios_by[out].kind == "stepper":
                w(f"  st_iv_{out} = {ios_by[out].cfg['interval']}UL;")
            w("}")
        done_idx = {e.name: i for i, e in events if e.kind == "done"}
        for i in done_idx.values():
            w(f"static void h_{i}(void);")
        for p in node.paths:
            n, o = p.name, p.out
            w(f"static void path_enter_{n}(void) {{")
            if p.kind == "stepper":
                w(f"  st_iv_{o} = piv_{n}[pidx_{n}] ? piv_{n}[pidx_{n}] : {ios_by[o].cfg['interval']}UL;")
                w(f"  r_set_{o}((rt_int)pv_{n}[pidx_{n}]);")
                w(f"  pst_{n} = 1;")
            else:
                w(f"  r_set_{o}((rt_int)pv_{n}[pidx_{n}]);")
                w(f"  pst_{n} = 2; pt_{n} = rt_now;")
            w("}")
            w(f"static void path_next_{n}(void) {{")
            w(f"  pidx_{n}++;")
            w(f"  if (pidx_{n} < {len(p.points)}) {{ path_enter_{n}(); }}")
            if p.forever:
                w(f"  else {{ pidx_{n} = 0; path_enter_{n}(); }}")
            else:
                w(f"  else if (prep_{n} > 1) {{ prep_{n}--; pidx_{n} = 0; path_enter_{n}(); }}")
                w("  else {")
                w(f"    pst_{n} = 0;")
                if p.kind == "stepper":
                    w(f"    st_iv_{o} = {ios_by[o].cfg['interval']}UL;")
                if n in done_idx:
                    w(f"    h_{done_idx[n]}();")
                w("  }")
            w("}")
            w(f"static void path_run_{n}(void) {{")
            w(f"  path_cancel_{o}();")
            w(f"  pidx_{n} = 0; prep_{n} = {p.repeat};")
            w(f"  path_enter_{n}();")
            w("}")
        for out in by_out:
            w(f"static __attribute__((unused)) void path_stop_{out}(void) {{")
            w(f"  path_cancel_{out}();")
            if ios_by[out].kind == "stepper":
                w(f"  o_{out} = sp_{out};   /* hold: the target becomes where it is */")
            else:
                w(f"  r_set_{out}({_lit(ios_by[out].cfg['safe'])});")
            w("}")
        w("static void r_paths(uint32_t now) {")
        for p in node.paths:
            n, o = p.name, p.out
            w(f"  if (pst_{n}) {{")
            if p.kind == "stepper":
                w(f"    if (pst_{n} == 1) {{")
                w(f"      if (labs((long)sp_{o} - pv_{n}[pidx_{n}]) <= (long)pz_{n}[pidx_{n}]) {{")
                w(f"        if (pw_{n}[pidx_{n}] > 0) {{ pst_{n} = 2; pt_{n} = now; }} else {{ path_next_{n}(); }}")
                w("      }")
                w(f"    }} else if ((uint32_t)(now - pt_{n}) >= pw_{n}[pidx_{n}]) {{ path_next_{n}(); }}")
            else:
                w(f"    if ((uint32_t)(now - pt_{n}) >= pw_{n}[pidx_{n}]) {{ path_next_{n}(); }}")
            w("  }")
        w("}")
        w("")
    if steppers:
        w("/* Steppers: one step pulse per interval towards the target position (runs even after the link is lost,")
        w("   so a stepper can reach its safe position at a bounded rate). */")
        w("static void r_stepper_step(uint32_t now) {")
        for d in steppers:
            n = d.name
            w("  {")
            w(f"    if ((uint32_t)(now - t_stp_{n}) >= st_iv_{n}) {{")
            w(f"      t_stp_{n} = ((uint32_t)(now - t_stp_{n}) >= 2UL * st_iv_{n}) ? now : t_stp_{n} + st_iv_{n};")
            w(f"      if (sp_{n} != o_{n}) {{")
            w(f"        bool fw = o_{n} > sp_{n};")
            w(f"        digitalWrite(P_{n}_dir, fw ? HIGH : LOW);")
            w(f"        digitalWrite(P_{n}_step, HIGH);")
            w("        delayMicroseconds(4);")
            w(f"        digitalWrite(P_{n}_step, LOW);")
            w(f"        sp_{n} += fw ? 1 : -1;")
            w("      }")
            w("    }")
            w("  }")
        w("}")
        w("")

    if anim:
        w("static void r_anim_step(uint32_t now) {")
        for d in strips:
            n = d.name
            w(f"  if (an_mode_{n}) {{")
            w(f"    rt_anim_draw(np_{n}, N_{n}, an_mode_{n}, (uint32_t)(now - an_t0_{n}), an_per_{n}, an_r_{n}, an_g_{n}, an_b_{n});")
            w(f"    np_dirty_{n} = true;")
            w("  }")
        w("}")
        w("")

    if node.fns and not vm:
        def sig(f):
            ps = ", ".join(f"{'bool' if t == 'bool' else 'rt_int'} a_{p[0]}" for p, t in zip(f.params, f.ptypes)) or "void"
            rt = "void" if f.kind == "proc" else ("bool" if f.ret == "bool" else "rt_int")
            return f"static RT_UNUSED {rt} f_{f.name}({ps})"
        w("// ---- funcs and procs ----")
        for f in node.fns:
            w(sig(f) + ";")
        w("")
        for f in node.fns:
            w(f"// {f.kind} {f.name}")
            w(sig(f) + " {")
            for p, _t, _pos in f.params:
                w(f"  (void)a_{p};")
            L.extend(gs(low.fn_body(f.name), 1))
            if f.kind == "func":
                w("  return " + ("false" if f.ret == "bool" else "0") + ";   /* not reached: every path returns */")
            w("}")
            w("")
    live = [(i, e) for i, e in events if not (e.kind == "heartbeat" and e.off)]
    for i, e in live:
        w(f"static void h_{i}({', '.join(f'rt_int a_{p}' for p in e.params) or 'void'});")
    w("")
    if has_states:
        w("/* Change state: leave the old one, reset the timers that belong to the new one, enter it. */")
        w("static void r_goto(uint8_t k) {")
        w("  if (k == st) return;")
        for i, e in live:
            if e.kind == "exit":
                w(f"  if (st == {ck.state_index[e.state]}) h_{i}();   /* exit {e.state} */")
        w("  st = k;")
        for name, k in ck.state_index.items():
            lines = []
            for i, e in live:
                if e.state == name and e.kind == "every":
                    lines.append(f"t_{i} = rt_now;")
                elif e.state == name and e.kind == "when":
                    lines.append(f"w_{i} = false;" + (f" wo_{i} = false;" if e.dur_ms else ""))
            if lines:
                w(f"  if (k == {k}) {{ {' '.join(lines)} }}")
        w("  " + (BB(3, "k") or "/* no black box */"))
        for i, e in live:
            if e.kind == "enter":
                w(f"  if (k == {ck.state_index[e.state]}) h_{i}();   /* enter {e.state} */")
        w("}")
        w("")

    if vm:
        from . import bytecode as B, genvm
        for e in node.events:
            if e.kind == "when" and any(isinstance(x, I.CallE) for x in I.walk_expr(low.expr(e.arg))):
                raise ValueError("a 'when' condition that calls a function cannot be used with --vm yet "
                                 "(conditions stay in firmware, and functions live in the image)")
        vm_img = B.assemble(ck)
        genvm.emit(ck, w, vm_img, vm_slack, bb, has_states, [])
    w("// ---- handlers ----")
    for i, e in events:
        if e.kind == "heartbeat" and e.off:
            continue
        sig = ", ".join(f"rt_int a_{p}" for p in e.params) or "void"
        label = {"every": f"every {e.ms}ms", "when": "when", "on": f"on {e.name}",
                 "heartbeat": f"heartbeat {e.ms}ms (runs after the safe states are applied)",
                 "start": "start", "enter": f"enter {e.state}", "exit": f"exit {e.state}",
                 "done": f"done {e.name}"}[e.kind]
        if e.state and e.kind not in ("enter", "exit"):
            label += f"   [state {e.state}]"
        w(f"// {label}")
        w(f"static void h_{i}({sig}) {{")
        for p in e.params:
            w(f"  (void)a_{p};")
        if vm:
            if e.params:
                w(f"  rt_int vm_a[{len(e.params)}] = {{{', '.join('a_' + p for p in e.params)}}};")
                w(f"  vm_run({i}, vm_a, {len(e.params)});")
            else:
                w(f"  vm_run({i}, NULL, 0);")
        else:
            L.extend(gs(low.body(e.body, e.params), 1))
        w("}")
        w("")
    if vm:
        w("static void vm_start(void) {   /* runs the image's 'start' handlers: after boot, after an update, after a reset */")
        for i, e in events:
            if e.kind == "start":
                w(f"  h_{i}();")
        w("}")
        w("")

    w("// ---- serial link to the master (lines like  NAME  or  NAME 1 2 3) ----")
    w("#define RX_MAX 32")
    w("static char rx_buf[RX_MAX];")
    w("static uint8_t rx_len = 0;")
    w("static bool rx_drop = false;")
    w("")
    w("static void rx_exec(char *line) {")
    w("  char *p = line;")
    w("  while (*p == ' ') p++;")
    w("  char *name = p;")
    w("  while (*p && *p != ' ') p++;")
    w("  if (*p) *p++ = 0;")
    w("  if (!*name) return;")
    if vm:
        w('  if (!strcmp(name, "IMG")) { RT_HEARD(); vm_cmd_img(p); return; }')
        w('  if (!strcmp(name, "IMGEND")) { RT_HEARD(); vm_cmd_end(p); return; }')
        w('  if (!strcmp(name, "IMGRESET")) { RT_HEARD(); vm_cmd_reset(); return; }')
        w('  if (!strcmp(name, "IMGINFO")) { RT_HEARD(); vm_cmd_info(); return; }')
    if maxp:
        w(f"  rt_int args[{maxp}];")
        w("  uint8_t na = 0;")
        w("  while (*p == ' ') p++;")
        w(f"  while (*p && na < {maxp}) {{")
        w("    char *tok = p;")
        w("    while (*p && *p != ' ') p++;")
        w("    if (*p) *p++ = 0;")
        w("    args[na++] = (rt_int)strtol(tok, NULL, 10);")
        w("    while (*p == ' ') p++;")
        w("  }")
    w('  if (!strcmp(name, "PING")) { RT_HEARD(); tx_begin(); TX_STR("PONG"); tx_end(); return; }')
    w('  if (!strcmp(name, "WHO")) {')
    w(f'    RT_HEARD(); tx_begin(); TX_STR("WHO "); TX_NUM({protocol_hash(ck)}L); tx_end(); return;')
    w("  }")
    w('  if (!strcmp(name, "STATUS")) {')
    w('    RT_HEARD(); tx_begin(); TX_STR("STATUS "); TX_NUM(g_blocked & 0x7FFFFFFFUL); tx_end(); return;')
    w("  }")
    if has_states:
        w('  if (!strcmp(name, "STATE")) { RT_HEARD(); tx_begin(); TX_STR("STATE "); TX_NUM(st); tx_end(); return; }')
    if bb:
        w('  if (!strcmp(name, "LOG")) { RT_HEARD(); bb_dump(); return; }')
    done = []
    for i, e in events:
        if e.kind != "on" or e.name in done:
            continue
        done.append(e.name)
        hs = [(i2, e2) for i2, e2 in events if e2.kind == "on" and e2.name == e.name]
        n = len(e.params)
        call = ", ".join(f"args[{k}]" for k in range(n))
        w(f'  if (!strcmp(name, "{e.name}")) {{')
        if n:
            w(f'    if (na < {n}) {{ tx_begin(); TX_STR("ERR"); tx_end(); return; }}')
        for i2, e2 in hs:
            if e2.state is not None:
                w(f"    if (st == {ck.state_index[e2.state]}) {{ RT_HEARD(); h_{i2}({call}); return; }}")
        for i2, e2 in hs:
            if e2.state is None:
                w(f"    RT_HEARD(); h_{i2}({call}); return;")
                break
        else:
            w('    tx_begin(); TX_STR("ERR"); tx_end(); return;   /* known command, but not in this state */')
        w("  }")
    w('  tx_begin(); TX_STR("ERR"); tx_end();')
    w("}")
    w("")
    if bus:
        w("static void rx_dispatch(char *line) {   /* shared bus: only lines addressed to @" + str(node.address) + " or @0 are for us */")
        w("  char *p = line;")
        w("  while (*p == ' ') p++;")
        w("  char *tok0 = p;")
        w("  while (*p && *p != ' ') p++;")
        w("  if (*p) *p++ = 0;")
        w("  if (tok0[0] != '@') return;")
        w("  if (!(tok0[1] == '+' || tok0[1] == '-' || (tok0[1] >= '0' && tok0[1] <= '9'))) return;")
        w("  char *aend = NULL;")
        w("  long an = strtol(tok0 + 1, &aend, 10);")
        w("  if (aend == tok0 + 1 || *aend != 0) return;")
        w(f"  if (an != {node.address}L && an != 0L) return;")
        w("  rt_quiet = (an == 0L);")
        w("  rx_exec(p);")
        w("  rt_quiet = false;")
        w("}")
    else:
        w("#define rx_dispatch rx_exec")
    w("")
    w("static void rx_poll(void) {")
    w("  while (Serial.available() > 0) {")
    w("    int c = Serial.read();")
    w("    if (c < 0) break;")
    w("    if (c == '\\r') continue;")
    w("    if (c == '\\n') {")
    w("      if (!rx_drop && rx_len > 0) { rx_buf[rx_len] = 0; rx_dispatch(rx_buf); }")
    w("      rx_len = 0; rx_drop = false;")
    w("    } else if (rx_len < RX_MAX - 1) {")
    w("      rx_buf[rx_len++] = (char)c;")
    w("    } else {")
    w("      rx_drop = true;")
    w("    }")
    w("  }")
    w("}")
    w("")

    w("void setup() {")
    w(f"  Serial.begin({node.baud});")
    w("  rt_now = millis();")
    if has_i2c:
        w("  Wire.begin();")
    if de is not None:
        w("  pinMode(P_rs485_de, OUTPUT);")
        w("  digitalWrite(P_rs485_de, LOW);   /* listen */")
    for d in ios:
        if d.direction == "output":
            if d.kind in ("motor", "stepper"):
                for role in d.cfg["pins"]:
                    w(f"  pinMode(P_{d.name}_{role}, OUTPUT);")
            elif d.kind == "tone":
                w(f"  pinMode(P_{d.name}, OUTPUT);")
            elif d.kind in ("digital", "pwm"):
                w(f"  pinMode(P_{d.name}, OUTPUT);")
            elif d.kind == "servo":
                w(f"  s_{d.name}.attach(P_{d.name});")
            elif d.kind == "neopixel":
                w(f"  np_{d.name}.begin();")
        elif d.kind == "ultrasonic":
            w(f"  pinMode(P_{d.name}_trig, OUTPUT);")
            w(f"  pinMode(P_{d.name}_echo, INPUT);")
            w(f"  us_t_{d.name} = rt_now;")
        elif d.kind == "digital":
            w(f"  pinMode(P_{d.name}, {'INPUT_PULLUP' if d.pullup else 'INPUT'});")
            if d.cfg.get("debounce"):
                w(f"  db_v_{d.name} = db_c_{d.name} = i_raw_{d.name}();")
    w("  r_apply_safe();   /* safe state before anything else runs */")
    if vm:
        w("  VM_COPY(vm_img, vm_factory, sizeof(vm_factory)); vm_len = sizeof(vm_factory); vm_valid = true;")
    w("  uint32_t now = rt_now;")
    if bb:
        w("  " + BB(1, 0))
    for i, e in events:
        if e.kind == "every":
            w(f"  t_{i} = now;")
    if anim:
        w("  t_anim = now;")
    if need_mstep:
        w("  t_mot = now;")
    for d in motors:
        w(f"  m_last_{d.name} = now;")
    for d in steppers:
        w(f"  t_stp_{d.name} = now;")
    if hb:
        w("  hb_last = now;")
    for i, e in events:
        if e.kind == "start":
            w(f"  h_{i}();")
    if has_states:
        for i, e in events:
            if e.kind == "enter" and ck.state_index[e.state] == 0:
                w(f"  h_{i}();   /* enter {e.state} */")
    w("  r_flush();")
    w("  (void)now;")
    w("}")
    w("")

    w("void loop() {")
    w("  rt_now = millis();")
    w("  uint32_t now = rt_now;")
    w("  rx_poll();")
    for d in debounced + sonars:
        w(f"  r_sample_{d.name}(now);")
    if hb:
        i, e = hb
        w(f"  if (!hb_tripped && (uint32_t)(now - hb_last) >= {e.ms}UL) {{")
        w(f"    hb_tripped = true; r_apply_safe(); {BB(2, 0)} h_{i}();")
        w("  }")
        w("  if (hb_tripped) {   /* safe state holds until the master speaks again */")
        if steppers:
            w("    r_stepper_step(now);")
        w("    r_flush();")
        w("    return;")
        w("  }")
    for i, e in events:
        g = f"st == {ck.state_index[e.state]} && " if e.state is not None else ""
        if e.kind == "every":
            w(f"  if ({g}(uint32_t)(now - t_{i}) >= {e.ms}UL) {{")
            w(f"    t_{i} = ((uint32_t)(now - t_{i}) >= {2 * e.ms}UL) ? now : t_{i} + {e.ms}UL;")
            w(f"    h_{i}();")
            w("  }")
        elif e.kind == "when":
            w(f"  {'if (' + g[:-4] + ') ' if g else ''}{{")
            w(f"    bool c = {ge(e.arg)};")
            if e.dur_ms:
                w(f"    if (!c) {{ w_{i} = false; wo_{i} = false; }}")
                w("    else {")
                w(f"      if (!wo_{i}) {{ wo_{i} = true; wt_{i} = now; }}")
                w(f"      if (!w_{i} && (uint32_t)(now - wt_{i}) >= {e.dur_ms}UL) {{ w_{i} = true; h_{i}(); }}")
                w("    }")
            else:
                w(f"    if (c && !w_{i}) h_{i}();")
                w(f"    w_{i} = c;")
            w("  }")
    if node.paths:
        w("  r_paths(now);")
    if anim:
        w(f"  if ((uint32_t)(now - t_anim) >= {ANIM_STEP_MS}UL) {{")
        w(f"    t_anim = ((uint32_t)(now - t_anim) >= {2 * ANIM_STEP_MS}UL) ? now : t_anim + {ANIM_STEP_MS}UL;")
        w("    r_anim_step(now);")
        w("  }")
    if need_mstep:
        w(f"  if ((uint32_t)(now - t_mot) >= {ANIM_STEP_MS}UL) {{")
        w(f"    t_mot = ((uint32_t)(now - t_mot) >= {2 * ANIM_STEP_MS}UL) ? now : t_mot + {ANIM_STEP_MS}UL;")
        w("    r_motor_step(now);")
        w("  }")
    if steppers:
        w("  r_stepper_step(now);")
    w("  r_flush();")
    w("}")
    w("")
    return "\n".join(L)
