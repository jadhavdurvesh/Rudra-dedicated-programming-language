"""Stage B5.2/B5.3: the AVR runtime helpers and the code generator, checked on the emulator.

Helpers are compared with the Python reference arithmetic (rt_div/rt_mod/wrap32); generated programs are compared
with the simulator: same hardware writes in the same order, same serial text, times within a few milliseconds."""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rudrac.avr.asm import assemble
from rudrac.avr.emu import AVR
from rudrac.avr.gen import AvrUnsupported, compile_hex, generate_asm, unsupported
from rudrac.avr.runtime import HELPERS
from rudrac.check import check
from rudrac.sim import Sim
from rudrac.syntax import parse
from rudrac.util import rt_div, rt_mod, wrap32

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STUB = ".org 0\n        jmp entry\n.org 0x40\nentry:\n        rjmp .\ndone:\n        rjmp .\n"
EDGE = [0, 1, -1, 2, -2, 3, 7, 10, -10, 255, 256, 65535, 65536, 2**31 - 1, -2**31, -2**31 + 1, 123456789, -987654321]


# The emulated chip's 1 ms clock ticks a few microseconds after each nominal millisecond; an outside event is placed
# half a millisecond later so it is seen on the same tick as in the simulator (a real master is never that exact anyway).
PHASE = 0.5


def u32(x):
    return x & 0xFFFFFFFF


class Helpers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fns = ["add32", "sub32", "mul32", "divs32", "mods32", "neg32", "not32"]

    def call(self, name, a=0, b=0):
        from rudrac.avr.asm import Assembler
        asm = Assembler()
        words = asm.assemble(".org 0\n jmp 0x40\n.org 0x40\n" + HELPERS + "\n" + "".join(
            f"e_{f}:\n call {f}\n e_{f}_done: rjmp e_{f}_done\n" for f in self.fns))
        return words, asm

    def run_fn(self, f, a, b=0):
        m = AVR(self.__class__._words)
        m.set_sp(0x8FF)
        for k in range(4):
            m.r[22 + k] = (u32(a) >> 8 * k) & 255
            m.r[18 + k] = (u32(b) >> 8 * k) & 255
        m.pc = self.__class__._labels[f"e_{f}"]
        done = self.__class__._labels[f"e_{f}_done"]
        for _ in range(200000):
            if m.pc == done:
                break
            m.step()
        else:
            self.fail(f"{f}({a},{b}) did not finish")
        return m, sum(m.r[22 + k] << 8 * k for k in range(4))

    @classmethod
    def _build(cls):
        from rudrac.avr.asm import Assembler
        asm = Assembler()
        src = ".org 0\n jmp 0x40\n.org 0x40\n" + HELPERS + "\n" + "".join(
            f"e_{f}:\n call {f}\n e_{f}_done: rjmp e_{f}_done\n" for f in cls.fns)
        cls._words = asm.assemble(src)
        cls._labels = {k: v for k, v in asm.syms.items()}

    def pairs(self, n=150):
        r = random.Random(7)
        p = [(a, b) for a in EDGE for b in EDGE]
        p += [(r.randint(-2**31, 2**31 - 1), r.choice([r.randint(-2**31, 2**31 - 1), r.randint(-1000, 1000)])) for _ in range(n)]
        return p

    def test_arithmetic_matches_reference(self):
        if not hasattr(self.__class__, "_words"):
            self._build()
        ref = {"add32": lambda a, b: wrap32(a + b), "sub32": lambda a, b: wrap32(a - b),
               "mul32": lambda a, b: wrap32(a * b), "divs32": rt_div, "mods32": rt_mod}
        for f, fn in ref.items():
            for a, b in self.pairs():
                _, got = self.run_fn(f, a, b)
                self.assertEqual(got, u32(fn(a, b)), f"{f}({a}, {b})")
        for a in EDGE:
            self.assertEqual(self.run_fn("neg32", a)[1], u32(wrap32(-a)), f"neg {a}")
            self.assertEqual(self.run_fn("not32", a)[1], 1 if a == 0 else 0, f"not {a}")

    def test_tx_and_parse(self):
        from rudrac.avr.asm import Assembler
        asm = Assembler()
        src = (".org 0\n jmp 0x40\n.org 0x40\n" + HELPERS + "\ne_tx:\n call tx_i32\n e_tx_d: rjmp e_tx_d\n"
               "e_pa:\n ldi r26, 0x00\n ldi r27, 0x02\n call parse_i32\n e_pa_d: rjmp e_pa_d\n")
        words = asm.assemble(src)
        L = asm.syms
        for a in EDGE + [random.Random(3).randint(-2**31, 2**31 - 1) for _ in range(40)]:
            m = AVR(words); m.set_sp(0x8FF)
            for k in range(4):
                m.r[22 + k] = (u32(a) >> 8 * k) & 255
            m.pc = L["e_tx"]
            for _ in range(500000):
                if m.pc == L["e_tx_d"]:
                    break
                m.step()
            m.run_ms(5)
            self.assertEqual(m.tx_text(), str(a), a)
        cases = ["0", "5", "-5", "+7", "123", "2147483647", "-2147483648", "2147483648", "99999999999", "-99999999999", "12ab", "007"]
        import re
        for t in cases:
            m = AVR(words); m.set_sp(0x8FF)
            for i, ch in enumerate((t + "\0").encode()):
                m.mem[0x200 + i] = ch
            m.pc = L["e_pa"]
            for _ in range(500000):
                if m.pc == L["e_pa_d"]:
                    break
                m.step()
            got = sum(m.r[22 + k] << 8 * k for k in range(4))
            mt = re.match(r"[+-]?\d+", t)
            want = max(-2**31, min(2**31 - 1, int(mt.group()))) if mt else 0
            self.assertEqual(got, u32(want), t)


def build(src):
    ck = check(parse(src, "t", {}))
    return ck


def diff(src, events, ms, tol=4):
    ck = build(src)
    s = Sim(ck)
    s.run(ms, events)
    m = AVR(assemble(generate_asm(ck)))
    m.set_sp(0x8FF)
    ios = {d.name: d for d in ck.node.decls if hasattr(d, "direction")}
    ext = []

    def pinkey(d):
        c = str(d.pin.c)
        n = 14 + int(c[1:]) if c[0] in "Aa" else int(c)
        return ("D", n) if n <= 7 else (("B", n - 8) if n <= 13 else ("C", n - 14))

    for d in ios.values():
        if d.direction == "input" and d.kind == "digital":
            port, bit = pinkey(d)
            m.input_pins[port] |= (1 if d.invert else 0) << bit
    for t, k, a, b in events:
        if k == "set":
            d = ios[a]
            if d.kind == "digital":
                port, bit = pinkey(d)
                ext.append((t + PHASE, "pin", f"{port}{bit}", 1 if bool(b) != d.invert else 0))
            else:
                ext.append((t + PHASE, "adc", pinkey(d)[1], int(b)))
    m.run_ms(ms + 1, [(t + PHASE, (l + "\n").encode()) for t, k, l, _ in events if k == "rx"], ext_events=ext)
    # simulator writes -> changes only, per pin
    kinds = {}
    for d in ios.values():
        if d.direction != "output":
            continue
        if d.kind in ("motor", "stepper"):
            for role, pin in d.cfg["pins"].items():
                kinds[int(str(pin.c))] = "pwm" if role in ("pwm", "fwd", "rev") else "digital"
        elif d.kind in ("digital", "pwm", "servo"):
            kinds[int(str(d.pin.c))] = d.kind
    exp, last = [], {pin: 0 for pin, k in kinds.items() if k != "servo"}
    sexp, slast = [], {}
    for kind, t, pin, v in [e[:4] for e in s.phys if e[0] in ("W", "S")]:
        if kind == "S":
            if slast.get(pin) != v:
                sexp.append((t, pin, v))
            slast[pin] = v
        elif last.get(pin) != v:
            exp.append((t, pin, v))
            last[pin] = v
    got, glast = [], {pin: 0 for pin, k in kinds.items() if k != "servo"}
    def pin_of(port):
        return {"B": 8, "D": 0, "C": 14}[port[0]] + int(port[1:])
    raw = [(t, pin_of(port), v, "pin") for t, port, v in m.pin_log] + [(t, pin_of(port), v, "pwm") for t, port, v in m.pwm_log]
    raw.sort(key=lambda e: e[0])
    for t, pin, v, src_ in raw:
        k = kinds.get(pin)
        if k == "servo" or (k == "pwm") != (src_ == "pwm") or k is None:
            continue
        if glast.get(pin) != v:
            got.append((t, pin, v))
            glast[pin] = v
    # servo pulses -> angles
    sgot, rise, sl = [], {}, {}
    for t, port, v in m.pin_log:
        pin = pin_of(port)
        if kinds.get(pin) != "servo":
            continue
        if v:
            rise[pin] = t
        elif pin in rise:
            us = (t - rise.pop(pin)) * 1000.0
            ang = int(round((us - 544) * 180 / 1856))
            if sl.get(pin) != ang:
                sgot.append((t, pin, ang))
            sl[pin] = ang
    return ck, s, m, exp, got, sexp, sgot


class Generated(unittest.TestCase):
    def check_match(self, src, events, ms, tol=4):
        ck, s, m, exp, got, sexp, sgot = diff(src, events, ms)
        self.assertEqual([x[1:] for x in exp], [x[1:] for x in got], "hardware write order")
        for (t1, *_), (t2, *_) in zip(exp, got):
            self.assertLessEqual(abs(t1 - t2), tol, f"{t1} vs {t2}")
        # a servo angle is only visible one pulse later: within a frame (20 ms) plus the chosen slot
        self.assertEqual([x[1:] for x in sexp], [x[1:] for x in sgot], "servo angles")
        for (t1, *_), (t2, *_) in zip(sexp, sgot):
            self.assertTrue(0 <= t2 - t1 <= 45, f"servo change at {t1} seen at {t2}")
        self.assertEqual(m.tx_text(), "".join(x + "\r\n" for x in s.tx), "serial text")
        return m

    def test_blink_example(self):
        src = open(os.path.join(ROOT, "examples", "blink_uno.rudra")).read()
        ev = [(107, "rx", "PING", None), (307, "rx", "FAST", None), (1507, "rx", "WHO", None), (2507, "rx", "ON", None),
              (3007, "rx", "STATUS", None), (3207, "rx", "BOGUS", None), (3507, "rx", "BLINK", None), (4007, "rx", "SLOW", None)]
        self.check_match(src, ev, 7000)

    def test_conformance_programs(self):
        import json
        done = 0
        for name in ("p01-digital-out", "p03-builtin-commands", "p06-never-interlock", "p14-func-proc-let-for-break",
                     "p05-when-for-and-debounce", "p13-states", "p15-integer-arithmetic", "p16-arrays-structs-enums", "p22-multiple-heartbeat-recover"):
            d = os.path.join(ROOT, "conformance", name)
            sc = json.load(open(os.path.join(d, "scenario.json")))
            ev = [(e[0], e[1], e[2], None) for e in sc["events"]]
            with self.subTest(name):
                self.check_match(open(os.path.join(d, "prog.rudra")).read(), ev, min(sc["run_ms"], 6000))
                done += 1
        self.assertEqual(done, 9)

    def test_analog_and_when(self):
        src = """node t target uno {
  input pot: analog(pin A1)
  input btn: digital(pin 2, pullup, invert)
  output led: digital(pin 13, safe false)
  output buz: digital(pin 8, safe false)
  var hits = 0
  when pot > 600 { led = true  send "HI", pot }
  when pot <= 600 { led = false }
  when btn { hits = hits + 1  send "HIT", hits }
  when btn for 300ms { buz = true }
  when not btn { buz = false }
  every 250ms { send "P", pot }
  on READ { send "R", pot }
  heartbeat off
}"""
        ev = [(50, "set", "pot", 100), (400, "set", "pot", 700), (700, "set", "pot", 1023), (900, "set", "pot", 0),
              (950, "rx", "READ", None), (1010, "set", "btn", True), (1110, "set", "btn", False), (1210, "set", "btn", True),
              (1610, "set", "btn", False), (1650, "set", "pot", 601), (1800, "set", "pot", 600)]
        m = self.check_match(src, ev, 2500)
        self.assertIn("HI 700", m.tx_text())

    def test_states_full(self):
        src = """node t target uno {
  input btn: digital(pin 2, pullup, invert, debounce 20ms)
  output a: digital(pin 12, safe false)
  output b: digital(pin 13, safe false)
  var n = 0
  state idle {
    enter { a = false  send "IDLE" }
    exit  { send "LEAVE-IDLE" }
    on SET(x) { n = x  send "I", n }
    on GO { goto run }
    when btn { goto run }
  }
  state run {
    enter { b = true  n = 0  send "RUN" }
    exit  { b = false }
    every 100ms { n = n + 1  a = not a  if n >= 5 { goto done } }
    on SET(x) { send "BUSY" }
    on STOP { goto idle }
  }
  state done {
    enter { send "DONE", n  a = true }
    when true for 300ms { goto idle }
  }
  on SET(x) { send "G", x }
  on PING2 { send "P2" }
  heartbeat off
}"""
        ev = [(50, "rx", "SET 7", None), (100, "rx", "SET", None), (150, "rx", "STATE", None), (200, "rx", "GO", None),
              (250, "rx", "SET 9", None), (750, "rx", "STATE", None), (1400, "rx", "PING2", None), (1500, "rx", "SET -3", None),
              (1600, "set", "btn", True), (1800, "set", "btn", False), (2100, "rx", "STOP", None),
              (2200, "rx", "GO", None), (2350, "rx", "STOP", None), (2450, "rx", "STOP", None), (2500, "rx", "NOPE", None)]
        m = self.check_match(src, ev, 4000)
        self.assertIn("DONE 5", m.tx_text())
        self.assertIn("LEAVE-IDLE", m.tx_text())

    def test_state_entry_resets_timers(self):
        src = """node t target uno {
  input btn: digital(pin 2, pullup, invert)
  output a: digital(pin 12, safe false)
  state one {
    enter { send "ONE" }
    when btn { send "EDGE1"  goto two }
    when btn for 200ms { send "HELD1" }
    every 300ms { a = not a }
  }
  state two {
    enter { send "TWO" }
    on BACK { goto one }
  }
  heartbeat off
}"""
        ev = [(105, "set", "btn", True), (400, "rx", "BACK", None), (650, "rx", "BACK", None), (1000, "rx", "BACK", None),
              (1210, "set", "btn", False), (1450, "set", "btn", True), (1700, "rx", "BACK", None), (2100, "rx", "BACK", None)]
        m = self.check_match(src, ev, 3000)
        self.assertGreaterEqual(m.tx_text().count("EDGE1"), 3)

    def test_inputs_refused_kinds(self):
        src = "node n target uno { serial 9600\n input d: ultrasonic(trig 3, echo 4)\n output l: digital(pin 13, safe false)\n heartbeat off }"
        with self.assertRaises(AvrUnsupported):
            generate_asm(build(src))

    def test_pwm_and_servo(self):
        src = """node t target uno {
  output p: pwm(pin 11, safe 0)
  output q: pwm(pin 3, safe 20, invert)
  output r: pwm(pin 10, safe 100)
  output sv: servo(pin 7, safe 90)
  output sw: servo(pin 4, safe 10)
  var cnt = 0
  every 200ms { cnt = cnt + 1  r = cnt * 7 }
  on SET(a, b) { p = a  sv = b }
  on Q(a) { q = a  sw = a * 2 }
  heartbeat off
}"""
        ev = [(105, "rx", "SET 50 0", None), (305, "rx", "SET 100 180", None), (505, "rx", "SET 0 45", None),
              (705, "rx", "Q 33", None), (905, "rx", "Q 150", None), (1105, "rx", "Q -5", None), (1305, "rx", "SET 101 181", None),
              (1507, "rx", "SET 7 -9", None)]
        self.check_match(src.replace("output r: pwm(pin 10, safe 100)", "output r: digital(pin 12, safe false)").replace("r = cnt * 7", "r = cnt > 3"), ev, 2200)

    def test_motors_ramp_timeout_blackbox(self):
        src = """node t target uno {
  output a: motor(pwm 11, dir 4, ramp 400, timeout 600, limit 80, safe 0)
  output b: motor(fwd 9, rev 3, safe 0)
  output led: digital(pin 13, safe false)
  never a > 60 && led
  blackbox 6
  on A(v) { a = v }
  on B(v) { b = v }
  on L(v) { led = v > 0 }
  heartbeat off
}"""
        ev = [(105, "rx", "A 50", None), (205, "rx", "B -30", None), (405, "rx", "A -90", None), (905, "rx", "B 100", None),
              (1105, "rx", "L 1", None), (1205, "rx", "A 70", None), (1405, "rx", "L 0", None), (1505, "rx", "A 70", None),
              (1805, "rx", "A 95", None), (2005, "rx", "B -150", None), (2605, "rx", "LOG", None), (2705, "rx", "STATUS", None)]
        m = self.check_match(src, ev, 3300)
        self.assertIn("LOGEND", m.tx_text())

    def test_stepper(self):
        src = """node t target uno {
  output s: stepper(step 8, dir 12, rate 200, safe 0)
  output t: stepper(step 6, dir 7, rate 50, safe 5)
  on GO(p) { s = p }
  on GT(p) { t = p }
  heartbeat 400ms { s = 0 }
}"""
        ev = [(107, "rx", "GO 12", None), (307, "rx", "GT 4", None), (359, "rx", "PING", None), (507, "rx", "GO -3", None),
              (707, "rx", "PING", None), (1102, "rx", "GO 6", None)]
        self.check_match(src, ev, 2500)

    def test_blackbox_events(self):
        src = """node t target uno {
  output led: digital(pin 13, safe false)
  var arr: int[3]
  state a { enter { led = true } on GO { goto b } }
  state b { enter { led = false } on GO { goto a } }
  blackbox 4
  on W(i) { arr[i] = 1 }
  heartbeat 300ms { led = false }
}"""
        ev = [(105, "rx", "GO", None), (155, "rx", "W 7", None), (205, "rx", "GO", None), (255, "rx", "W -1", None),
              (305, "rx", "GO", None), (355, "rx", "LOG", None), (400, "rx", "PING", None), (1000, "rx", "LOG", None)]
        m = self.check_match(src, ev, 1500)
        self.assertIn("LOG ", m.tx_text())

    def test_pwm_timer1_and_refusals(self):
        src = "node t target uno { output a: pwm(pin 9, safe 0)\n output b: pwm(pin 10, safe 100)\n on X(v) { a = v  b = 100 - v }\n heartbeat off }"
        self.check_match(src, [(105, "rx", "X 30", None), (305, "rx", "X 99", None), (505, "rx", "X 100", None), (705, "rx", "X 0", None)], 1000)
        for bad, why in (("output a: pwm(pin 5, safe 0)", "pin 5"), ("output a: pwm(pin 9, safe 0)\n output b: servo(pin 7, safe 0)", "Timer1")):
            with self.assertRaises(AvrUnsupported) as c:
                generate_asm(build("node t target uno { " + bad + "\n heartbeat off }"))
            self.assertIn(why, " ".join(c.exception.reasons))

    def test_cli_cc(self):
        import subprocess, tempfile
        with tempfile.TemporaryDirectory() as t:
            r = subprocess.run([sys.executable, "-m", "rudrac", "cc", os.path.join(ROOT, "examples", "blink_uno.rudra"),
                                "-o", os.path.join(t, "b"), "--asm"], cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertTrue(open(os.path.join(t, "b.hex")).read().startswith(":"))
            r = subprocess.run([sys.executable, "-m", "rudrac", "cc", os.path.join(ROOT, "examples", "scanner_esp32.rudra")],
                               cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(r.returncode, 1)

    def test_checked_in_hex_is_current(self):
        src = open(os.path.join(ROOT, "examples", "blink_uno.rudra")).read()
        hexed = compile_hex(build(src))[1]
        self.assertEqual(hexed, open(os.path.join(ROOT, "examples", "avr", "blink_rudra.hex")).read(),
                         "run: rudrac cc examples/blink_uno.rudra -o examples/avr/blink_rudra --asm")

    def test_refuses_unsupported(self):
        src = "node n target uno { serial 9600\n output l: tone(pin 8, safe 0)\n heartbeat off }"
        with self.assertRaises(AvrUnsupported) as c:
            generate_asm(build(src))
        self.assertIn("tone", c.exception.reasons[0])


if __name__ == "__main__":
    unittest.main()
