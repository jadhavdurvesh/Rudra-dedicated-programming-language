"""Run with:  python3 -m unittest discover -s tests -v   (from the rudra/ folder)"""
import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from rudrac import RudraError, Sim, check, generate, parse  # noqa: E402
from rudrac.link import gen_cpp, gen_protocol, gen_python, info, protocol_hash  # noqa: E402
from rudrac.util import anim_frame  # noqa: E402
from rudrac.verify import format_report, verify  # noqa: E402

EX = os.path.join(ROOT, "examples")
EXAMPLES = ["fan_nano.rudra", "led_node.rudra", "scout_esp32.rudra", "heater_nano.rudra", "heater_buggy.rudra"]


def compile_src(src):
    return check(parse(src))


def messages(src):
    try:
        ck = compile_src(src)
    except RudraError as e:
        return [d.msg for d in e.diags]
    return [d.msg for d in ck.errors]


def wrap(body, target="nano", hb="heartbeat 3600s"):
    """A node around `body`; gets a heartbeat unless hb is ''."""
    return f"node t target {target} {{\n{body}\n{hb}\n}}"


def load_example(name):
    with open(os.path.join(EX, name)) as f:
        return compile_src(f.read())


class CheckerTests(unittest.TestCase):
    def assertErr(self, src, fragment):
        msgs = messages(src)
        self.assertTrue(any(fragment in m for m in msgs), f"wanted '{fragment}' in {msgs}")

    def test_examples_are_clean(self):
        for name in EXAMPLES:
            ck = load_example(name)
            self.assertTrue(ck.ok, [d.msg for d in ck.errors])
            self.assertEqual(ck.warnings, [], [d.msg for d in ck.warnings])

    def test_unknown_name_with_suggestion(self):
        src = wrap("output led: digital(pin 5, safe false)\nevery 1s { lde = true }")
        ck = compile_src(src)
        d = [d for d in ck.errors if "unknown name 'lde'" in d.msg][0]
        self.assertIn("led", d.hint)

    def test_type_mismatch(self):
        self.assertErr(wrap("output led: digital(pin 5, safe false)\nevery 1s { led = 5 }"), "cannot assign int")
        self.assertErr(wrap("var n = 0\nevery 1s { n = true }"), "cannot assign bool")
        self.assertErr(wrap("var n = 0\nevery 1s { if n { n = 1 } }"), "must be true/false")

    def test_assign_input_and_const(self):
        self.assertErr(wrap("input b: digital(pin 2)\nevery 1s { b = true }"), "input and cannot be assigned")
        self.assertErr(wrap("const K = 1\nevery 1s { K = 2 }"), "constant and cannot be assigned")

    def test_pin_rules_nano(self):
        self.assertErr(wrap("output f: pwm(pin 4, safe 0)"), "does not support PWM")
        self.assertErr(wrap("output a: digital(pin 0, safe false)"), "serial port")
        self.assertErr(wrap("output a: digital(pin 5, safe false)\noutput b: digital(pin 5, safe false)"), "already used")
        self.assertErr(wrap("input a: analog(pin 5)"), "not an analog-capable")
        self.assertErr(wrap("input a: digital(pin A6)"), "cannot be used as a digital input")
        self.assertEqual(messages(wrap("input a: analog(pin A6)\nevery 1s { }")), [])

    def test_pin_rules_esp32(self):
        self.assertErr(wrap("output a: digital(pin 6, safe false)", "esp32"), "flash")
        self.assertErr(wrap("output a: digital(pin 34, safe false)", "esp32"), "cannot be used as a digital output")
        self.assertErr(wrap("input a: digital(pin 34, pullup)", "esp32"), "no internal pull-up")
        self.assertErr(wrap("input a: analog(pin 4)", "esp32"), "not an analog-capable")
        self.assertErr(wrap("output a: digital(pin A0, safe false)", "esp32"), "GPIO numbers")

    def test_const_must_be_constant(self):
        self.assertErr(wrap("var n = 1\nconst K = n"), "not a constant")
        self.assertErr(wrap("const K = 1 / 0"), "division by zero")

    def test_duration_and_syntax_errors(self):
        self.assertErr(wrap("every 0ms { }"), "duration must be")
        self.assertErr(wrap("every 1s { x }"), "expected '='")
        self.assertErr("node t target nano { every 1s { } ", "missing '}'")
        self.assertErr(wrap("var pwm = 1"), "reserved word")

    def test_duplicate_blocks_and_commands(self):
        self.assertErr(wrap("heartbeat 1s", hb="heartbeat 2s"), "only one 'heartbeat'")
        self.assertErr(wrap("on A { }\non A { }"), "handled twice")
        self.assertErr(wrap("on PING { }"), "built in")

    def test_bad_send_name(self):
        self.assertErr(wrap('every 1s { send "TWO WORDS" }'), "single word")
        self.assertErr(wrap('every 1s { send "PONG" }'), "built-in message")
        self.assertErr(wrap('every 1s { send "X" }\nevery 2s { send "X", 1 }'), "both with and without")

    # ---- v0.2: the safety rules
    def test_output_needs_safe_state(self):
        self.assertErr(wrap("output a: digital(pin 5)"), "needs a safe state")
        self.assertErr(wrap("output a: pwm(pin 9)"), "needs a safe state")
        self.assertErr(wrap("output a: servo(pin 9)", "esp32"), "needs a safe state")

    def test_safe_state_range_and_type(self):
        self.assertErr(wrap("output a: pwm(pin 9, safe 101)"), "between 0 and 100")
        self.assertErr(wrap("output a: servo(pin 5, safe 181)"), "between 0 and 180")
        self.assertErr(wrap("output a: digital(pin 5, safe 1)"), "safe true")
        self.assertErr(wrap("output a: neopixel(pin 5, count 4, safe 1, 2, 300)"), "0 to 255")
        self.assertEqual(messages(wrap("output a: neopixel(pin 5, count 4, safe off)")), [])

    def test_heartbeat_required_with_outputs(self):
        self.assertErr(wrap("output a: digital(pin 5, safe false)", hb=""), "no heartbeat")
        self.assertEqual(messages(wrap("output a: digital(pin 5, safe false)", hb="heartbeat off")), [])
        ck = compile_src(wrap("output a: digital(pin 5, safe false)", hb="heartbeat off"))
        self.assertTrue(any("heartbeat is off" in d.msg for d in ck.warnings))
        # no outputs: a heartbeat is not required
        self.assertEqual(messages(wrap("input a: analog(pin A0)\nevery 1s { send \"X\", a }", hb="")), [])

    # ---- v0.2: peripherals
    def test_servo_rules(self):
        self.assertErr(wrap("output s: servo(pin 4)"), "needs a safe state")
        self.assertErr(wrap("output s: servo(pin 0, safe 90)"), "serial port")
        self.assertErr(wrap("output s: servo(pin 5, safe 90, invert)"), "'invert' does not apply")
        self.assertErr(wrap("output s: servo(pin 5, safe 90)\noutput p: pwm(pin 9, safe 0)"), "while a servo is used")
        self.assertEqual(messages(wrap("output s: servo(pin 5, safe 90)\nevery 1s { s = 10 }")), [])

    def test_neopixel_rules(self):
        self.assertErr(wrap("output n: neopixel(pin 5, safe off)"), "pixel count")
        self.assertErr(wrap("output n: neopixel(pin 5, count 0, safe off)"), "between 1 and 1024")
        self.assertErr(wrap("output n: neopixel(pin 5, count 4, safe off)\nevery 1s { n = 1 }"), "neopixel strip")
        self.assertErr(wrap("output n: neopixel(pin 5, count 4, safe off)\nevery 1s { pixel n, 4, 1, 2, 3 }"), "outside")
        self.assertErr(wrap("output d: digital(pin 5, safe false)\nevery 1s { fill d, 1, 2, 3 }"), "not a neopixel")
        self.assertErr(wrap("output n: neopixel(pin 5, count 4, safe off)\nevery 1s { send \"X\", n }"), "no value")
        self.assertEqual(messages(wrap("output n: neopixel(pin 5, count 4, rgb, safe 9, 8, 7)\n"
                                       "every 1s { fill n, 1, 2, 3  pixel n, 3, 4, 5, 6 }")), [])
        ck = compile_src(wrap("output n: neopixel(pin 5, count 100, safe off)\nevery 1s { fill n, 1, 2, 3 }"))
        self.assertTrue(any("blocks interrupts" in d.msg for d in ck.warnings))

    def test_i2c_rules(self):
        self.assertErr(wrap("input t: i2c(reg 0)"), "needs an address")
        self.assertErr(wrap("input t: i2c(addr 2)"), "between 8 and 119")
        self.assertErr(wrap("input t: i2c(addr 0x48, bytes 5)"), "between 1 and 4")
        self.assertErr(wrap("input t: i2c(addr 0x48, pin 5)"), "'pin' does not apply")
        self.assertErr(wrap("input t: i2c(addr 0x48)\noutput x: digital(pin A4, safe false)"), "I2C data pin")
        self.assertErr(wrap("input t: i2c(addr 0x48)\ninput x: digital(pin 21)", "esp32"), "I2C data pin")
        self.assertEqual(messages(wrap("input t: i2c(addr 0x48, reg 0, bytes 2, signed)\nevery 1s { send \"T\", t }")), [])
        self.assertEqual(messages(wrap("output d: i2c(addr 0x20, reg 9, safe 0)\nevery 1s { d = 255 }")), [])

    def test_multi_argument_commands(self):
        self.assertErr(wrap("on C(a, a) { }"), "listed twice")
        self.assertErr(wrap("var v = 0\non C(v) { }"), "hides an existing name")
        self.assertEqual(messages(wrap("var v = 0\non C(a, b, c) { v = a + b * c }")), [])

    def test_option_errors(self):
        self.assertErr(wrap("output a: digital(pin 5, safe false, colour 3)"), "unknown option")
        self.assertErr(wrap("output a: digital(pin 5, pin 6, safe false)"), "given twice")
        self.assertErr(wrap("output a: digital(safe false)"), "needs a pin")
        self.assertErr(wrap("input a: digital(pin 5, safe false)"), "'safe' does not apply")


class CheckerV03Tests(unittest.TestCase):
    def assertErr(self, src, fragment):
        msgs = messages(src)
        self.assertTrue(any(fragment in m for m in msgs), f"wanted '{fragment}' in {msgs}")

    OUTS = "output a: digital(pin 5, safe false)\noutput b: digital(pin 6, safe true)\n"

    def test_never_rules_valid(self):
        self.assertEqual(messages(wrap(self.OUTS + "never a and not b")), [])
        self.assertEqual(messages(wrap("const MAX = 80\noutput f: pwm(pin 9, safe 0)\noutput h: pwm(pin 10, safe 0)\n"
                                       "never f < 20 and h > MAX")), [])

    def test_never_rules_errors(self):
        self.assertErr(wrap("input t: analog(pin A0)\n" + self.OUTS + "never a and t > 5"), "not the input 't'")
        self.assertErr(wrap("var n = 0\n" + self.OUTS + "never a and n > 1"), "not the variable 'n'")
        self.assertErr(wrap(self.OUTS + "never 3 > 2"), "does not mention any output")
        self.assertErr(wrap(self.OUTS + "never a + 1"), "'+' needs numbers")
        self.assertErr(wrap("output f: pwm(pin 9, safe 0)\nnever f"), "must be true/false")
        self.assertErr(wrap("output n: neopixel(pin 5, count 2, safe off)\nnever n"), "no value")
        self.assertErr(wrap(self.OUTS + "never zzz"), "unknown name")

    def test_safe_state_must_satisfy_rules(self):
        self.assertErr(wrap("output a: digital(pin 5, safe true)\noutput b: digital(pin 6, safe false)\n"
                            "never a and not b"), "the safe state itself breaks this rule")
        self.assertErr(wrap("output f: pwm(pin 9, safe 10)\nnever f < 20"), "safe state itself")
        self.assertEqual(messages(wrap("output f: pwm(pin 9, safe 30)\nnever f < 20")), [])

    def test_animate_rules(self):
        strip = "output r: neopixel(pin 6, count 8, safe off)\n"
        ok = wrap(strip + "on A { animate r, pulse, 255, 0, 0, 800ms }\non B { animate r, rainbow, 2s }\non C { animate r, off }")
        self.assertEqual(messages(ok), [])
        self.assertErr(wrap(strip + "on A { animate r, sparkle, 1, 2, 3, 500ms }"), "unknown animation 'sparkle'")
        self.assertErr(wrap(strip + "on A { animate r, blink, 255, 0, 0, 10ms }"), "period must be between")
        self.assertErr(wrap(strip + "on A { animate r, blink, 255, 0 }"), "blink takes  r, g, b, period")
        self.assertErr(wrap(strip + "on A { animate r, rainbow }"), "rainbow takes one value")
        self.assertErr(wrap(strip + "on A { animate r, blink, true, 0, 0, 500ms }"), "must be numbers")
        self.assertErr(wrap("output d: digital(pin 5, safe false)\non A { animate d, off }"), "not a neopixel")

    def test_reserved_names(self):
        self.assertErr(wrap("on WHO { }"), "built in")
        self.assertErr(wrap("on STATUS { }"), "built in")
        self.assertErr(wrap('every 1s { send "STATUS", 1 }'), "built-in message")

    def test_neopixel_baud_aware_warning(self):
        def warns(target, count, baud):
            src = (f"node t target {target} {{\nserial {baud}\noutput n: neopixel(pin 6, count {count}, safe off)\n"
                   "every 1s { fill n, 1, 2, 3 }\nheartbeat 3s\n}")
            return [d.msg for d in compile_src(src).warnings if "blocks interrupts" in d.msg]
        self.assertEqual(warns("nano", 8, 9600), [])          # 320 us < one byte (1041 us)
        self.assertTrue(warns("nano", 100, 9600))              # 3080 us
        self.assertTrue(warns("nano", 1, 115200))              # 110 us > 86 us
        self.assertEqual(warns("nano", 8, 1200), [])           # slow link: a byte lasts 8333 us
        self.assertEqual(warns("esp32", 100, 115200), [])      # ESP32 does not block like AVR


class SimTests(unittest.TestCase):
    def sim(self, src, end, events=()):
        s = Sim(compile_src(src))
        s.run(end, list(events))
        return s

    def test_blink_timing(self):
        s = self.sim(wrap("output led: digital(pin 5, safe false)\nvar lit = false\n"
                          "every 500ms { lit = !lit  led = lit }"), 2000)
        self.assertEqual([t for t, m in s.log], [500, 1000, 1500, 2000])

    def test_heartbeat_applies_safe_state_and_recovers(self):
        src = wrap("output led: digital(pin 5, safe false)\nevery 100ms { led = true }\n"
                   "on GO { led = true }", hb="heartbeat 1s")
        s = self.sim(src, 3000, [(800, "rx", "PING", None), (2000, "rx", "GO", None)])
        self.assertEqual(s.tx, ["PONG"])
        off = [t for t, m in s.log if m == "led = 0"]
        self.assertEqual(off, [1800, 3000])       # PING at 800 delayed the first trip; silent again after 2000
        on = [t for t, m in s.log if m == "led = 1"]
        self.assertEqual(on, [100, 2000])

    def test_safe_state_applied_at_boot(self):
        s = self.sim(wrap("output f: pwm(pin 9, safe 40)\noutput d: digital(pin 5, safe true)"), 0)
        self.assertEqual(s.out, {"f": 40, "d": True})

    def test_heartbeat_body_runs_after_safe_state(self):
        src = wrap("output a: digital(pin 5, safe false)\noutput b: digital(pin 6, safe false)",
                   hb="heartbeat 1s { b = true }")
        s = self.sim(src, 1500)
        self.assertEqual(s.out, {"a": False, "b": True})

    def test_command_arguments_and_errors(self):
        src = wrap("output f: pwm(pin 9, safe 0)\nvar sum = 0\non FAN(v) { f = v }\non ADD(a, b, c) { sum = a + b * c }")
        s = self.sim(src, 20, [(1, "rx", "FAN 150", None), (2, "rx", "FAN", None), (3, "rx", "NOPE", None),
                               (4, "rx", "FAN 30", None), (5, "rx", "ADD 1 2", None), (6, "rx", "ADD 1 2 3 99", None)])
        self.assertEqual(s.out["f"], 30)
        self.assertEqual(s.vars["sum"], 7)
        self.assertEqual(s.tx, ["ERR", "ERR", "ERR"])

    def test_when_is_edge_triggered(self):
        src = wrap('input b: digital(pin 2)\nvar n = 0\nwhen b { n = n + 1  send "N", n }', hb="")
        s = self.sim(src, 100, [(10, "set", "b", True), (20, "set", "b", False), (30, "set", "b", True)])
        self.assertEqual(s.tx, ["N 1", "N 2"])

    def test_integer_math_matches_c(self):
        src = wrap('var r = 0\nstart { r = -7 / 2 }\nevery 1s { send "A", -7 % 3 }\nevery 1s { send "B", 5 / 0 }', hb="")
        s = self.sim(src, 1000)
        self.assertEqual(s.vars["r"], -3)
        self.assertEqual(s.tx, ["A -1", "B 0"])

    def test_servo_and_neopixel_effects(self):
        src = wrap("output s: servo(pin 5, safe 90)\noutput n: neopixel(pin 6, count 3, safe 1, 2, 3)\n"
                   "on GO(a) { s = a  fill n, 255, 0, 0  pixel n, 1, 0, 0, 300 }")
        s = self.sim(src, 10, [(5, "rx", "GO 200", None)])
        self.assertEqual(s.phys[0], ("S", 0, 5, 90))
        self.assertEqual(s.phys[1], ("N", 0, 0x010203, 0x010203, 0x010203))
        self.assertEqual(s.phys[2], ("S", 5, 5, 180))                       # clamped
        self.assertEqual(s.phys[3], ("N", 5, 0xFF0000, 0x0000FF, 0xFF0000))  # one show() per loop, clamped colour

    def test_i2c_read_write_and_failure(self):
        src = wrap('input t: i2c(addr 0x48, reg 0, bytes 2, signed)\noutput d: i2c(addr 0x20, reg 9, bytes 2, safe 7)\n'
                   'every 1s { send "T", t }\non SET(v) { d = v }')
        s = self.sim(src, 5000, [(500, "set", "t", 0xFF80), (1500, "fail", "t", True), (2500, "set", "t", 100),
                                 (3500, "fail", "t", False), (4000, "rx", "SET 258", None)])
        self.assertEqual(s.tx, ["T -128", "T -128", "T -128", "T 100", "T 100"])
        self.assertEqual(s.phys[0], ("I", 0, 0x20, "090007"))
        self.assertEqual(s.phys[-1], ("I", 4000, 0x20, "090102"))


class SimV03Tests(unittest.TestCase):
    def sim(self, src, end, events=()):
        s = Sim(compile_src(src))
        s.run(end, list(events))
        return s

    def test_interlock_blocks_and_status_counts(self):
        src = wrap("output a: digital(pin 5, safe false)\noutput b: digital(pin 6, safe true)\nnever a and not b\n"
                   "on A { a = true }\non B(v) { b = v > 0 }")
        s = self.sim(src, 100, [(10, "rx", "A", None), (20, "rx", "B 0", None), (30, "rx", "STATUS", None),
                                (40, "rx", "B 1", None), (50, "rx", "STATUS", None)])
        self.assertEqual(s.out, {"a": True, "b": True})
        self.assertEqual(s.blocked_log, [(20, "b", False, 1)])
        self.assertEqual(s.tx, ["STATUS 1", "STATUS 1"])
        self.assertEqual(s.phys[-1], ("W", 40, 6, 1))     # the refused write never reached the pin

    def test_safe_state_is_atomic_and_wins(self):
        # intermediate boot state (both false) would break the rule, but the safe state (true, false) is fine
        src = wrap("output a: digital(pin 5, safe true)\noutput b: digital(pin 6, safe false)\nnever not a and not b")
        s = self.sim(src, 0)
        self.assertEqual(s.out, {"a": True, "b": False})
        self.assertEqual(s.blocked, 0)
        self.assertEqual([p[:2] + p[2:] for p in s.phys], [("W", 0, 5, 1), ("W", 0, 6, 0)])

    def test_clamped_value_is_what_the_rule_sees(self):
        src = wrap("output f: pwm(pin 9, safe 50)\nnever f > 80\non SET(v) { f = v }")
        s = self.sim(src, 10, [(1, "rx", "SET 500", None), (2, "rx", "SET 70", None)])
        self.assertEqual(s.out["f"], 70)
        self.assertEqual(s.blocked, 1)                      # 500 is clamped to 100, which breaks the rule

    def test_who_reports_protocol_hash(self):
        ck = compile_src(wrap("on GO(a) { }\nevery 1s { send \"X\" }", hb=""))
        s = Sim(ck)
        s.run(10, [(5, "rx", "WHO", None)])
        self.assertEqual(s.tx, [f"WHO {protocol_hash(ck)}"])
        other = compile_src(wrap("on GO(a, b) { }\nevery 1s { send \"X\" }", hb=""))
        self.assertNotEqual(protocol_hash(ck), protocol_hash(other))   # a changed command changes the fingerprint

    def test_animation_frames_follow_the_math(self):
        src = wrap("output r: neopixel(pin 6, count 4, safe off)\non A(m) {\n"
                   "  if m == 1 { animate r, blink, 255, 0, 0, 1000ms }\n  if m == 2 { animate r, pulse, 200, 100, 0, 400ms }\n"
                   "  if m == 3 { animate r, chase, 255, 0, 0, 400ms }\n  if m == 4 { animate r, rainbow, 1000ms }\n}",
                   "esp32", hb="heartbeat 3600s")
        for mode, per, col in ((1, 1000, (255, 0, 0)), (2, 400, (200, 100, 0)), (3, 400, (255, 0, 0)), (4, 1000, (0, 0, 0))):
            s = self.sim(src, 1000, [(100, "rx", f"A {mode}", None)])
            frames = {p[1]: list(p[2:]) for p in s.phys if p[0] == "N"}
            self.assertEqual(frames[100], anim_frame(mode, 4, 0, per, col))              # first frame is immediate
            for t in range(120, 1001, 20):                                                # then every 20 ms
                self.assertEqual(frames[t], anim_frame(mode, 4, t - 100, per, col), (mode, t))
            self.assertNotIn(110, frames)

    def test_fill_and_heartbeat_stop_an_animation(self):
        src = wrap("output r: neopixel(pin 6, count 2, safe 9, 9, 9)\non A { animate r, blink, 255, 0, 0, 200ms }\n"
                   "on F { fill r, 1, 2, 3 }\non O { animate r, off }", hb="heartbeat 1s")
        s = self.sim(src, 5000, [(100, "rx", "A", None), (300, "rx", "F", None), (400, "rx", "A", None),
                                 (500, "rx", "O", None)])
        n_after_fill = [p for p in s.phys if p[0] == "N" and 300 <= p[1] < 400]
        self.assertEqual([p[1] for p in n_after_fill], [300])             # only the fill itself, no animation frames
        n_after_off = [p for p in s.phys if p[0] == "N" and 500 < p[1] < 1500]
        self.assertEqual(n_after_off, [])                                  # 'off' leaves the strip alone
        last = [p for p in s.phys if p[0] == "N"][-1]
        self.assertEqual(last[2:], (0x090909, 0x090909))                   # heartbeat: safe colour, animation gone


def have_gpp():
    try:
        subprocess.run(["g++", "--version"], capture_output=True, check=True)
        return True
    except (OSError, subprocess.CalledProcessError):
        return False


def sim_tx_lines(sim):
    return [f"{t} {m}" for t, m in sim.tx_log]


def sim_event_lines(sim):
    return [" ".join(str(x) for x in p) for p in sim.phys]


class ParityMixin:
    """Compile the generated C++ against a fake Arduino and compare it with the simulator."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="rudrac_test_")

    def gpp(self, args, exe):
        cmd = ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", os.path.join(HERE, "stub"), *args, "-o", exe]
        r = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    VM = None       # True: build the bytecode-VM firmware and drive the simulator through the VM

    def build_src(self, ck, tag, extra=()):
        ino = os.path.join(self.tmp, tag + ".ino")
        with open(ino, "w") as f:
            f.write(generate(ck, tag, vm=self.VM))
        exe = os.path.join(self.tmp, tag + ".exe")
        self.gpp([f'-DSKETCH="{ino}"', *extra, os.path.join(HERE, "harness.cpp")], exe)
        return exe

    def run_both(self, ck, tag, end, events, extra=()):
        exe = self.build_src(ck, tag, extra)
        ios = {d.name: d for d in ck.node.decls if hasattr(d, "direction")}
        stdin = [f"RUN {end}"]
        for d in ios.values():          # real pins idle at the inactive level; the simulator starts inactive too
            if d.direction == "input" and d.kind == "digital":
                stdin.append(f"0 PIN {d.pin.key} {int(d.invert)}")
        for ms, kind, a, b in events:
            if kind == "rx":
                stdin.append(f"{ms} RX {a}")
            elif kind == "fail":
                stdin.append(f"{ms} I2CFAIL {ios[a].cfg['addr']} {int(bool(b))}")
            else:
                d = ios[a]
                if d.kind == "digital":
                    stdin.append(f"{ms} PIN {d.pin.key} {int(bool(b) != d.invert)}")
                elif d.kind == "ultrasonic":
                    stdin.append(f"{ms} ECHO {d.cfg['pins']['echo'].key} {int(b) * 58}")
                elif d.kind == "i2c":
                    c = d.cfg
                    hexv = "".join(f"{((b & 0xFFFFFFFF) >> (8 * k)) & 0xFF:02x}" for k in reversed(range(c['bytes'])))
                    reg = c["reg"] if c["reg"] is not None else -1
                    stdin.append(f"{ms} I2C {c['addr']} {reg} {hexv}")
                else:
                    stdin.append(f"{ms} ANALOG {d.pin.key} {b}")
        out = subprocess.run([exe], input="\n".join(stdin) + "\n", capture_output=True, text=True).stdout
        head, _, tx = out.partition("---TX---\n")
        sim = Sim(ck, engine="vm" if self.VM else None)
        sim.run(end, events)
        return head.splitlines(), tx.splitlines(), sim


@unittest.skipUnless(have_gpp(), "g++ not available")
class GeneratedCodeTests(ParityMixin, unittest.TestCase):
    def test_fan_nano_matches_simulator(self):
        events = [(1500, "set", "temp", 700), (2000, "rx", "PING", None), (3500, "set", "temp", 400),
                  (4000, "set", "temp", 950), (6000, "rx", "FAN 50", None), (6500, "rx", "FAN", None),
                  (7000, "rx", "AUTO", None), (7500, "rx", "BOGUS", None), (9000, "rx", "PING", None)]
        ev, tx, sim = self.run_both(load_example("fan_nano.rudra"), "fan", 12000, events)
        self.assertEqual(ev, sim_event_lines(sim))
        self.assertEqual(tx, sim_tx_lines(sim))
        self.assertGreater(len(sim.phys), 5)
        self.assertIn("ERR", sim.tx)

    def test_led_node_matches_simulator(self):
        events = [(1000, "rx", "ARM", None), (1500, "rx", "SIREN 1", None), (2000, "set", "btn", True),
                  (2200, "set", "btn", False), (2400, "set", "btn", True), (3000, "rx", "DISARM", None),
                  (3500, "rx", "SIREN 0", None), (9000, "rx", "ARM", None)]
        ev, tx, sim = self.run_both(load_example("led_node.rudra"), "led", 12000, events)
        self.assertEqual(ev, sim_event_lines(sim))
        self.assertEqual(tx, sim_tx_lines(sim))
        self.assertEqual(sim.tx.count("BTN"), 2)

    def test_scout_matches_simulator(self):
        events = [(0, "set", "temp", 0x0190), (1000, "rx", "SWEEP", None), (1500, "rx", "COLOR 0 255 0", None),
                  (2100, "rx", "DOT 3 255 255 255", None), (2200, "rx", "DOT 9 1 1 1", None),
                  (2500, "rx", "PAN 20", None), (3000, "set", "temp", 0xFF80), (3500, "set", "btn", True),
                  (4000, "fail", "temp", True), (4500, "set", "temp", 3300), (5000, "fail", "temp", False),
                  (5100, "rx", "COLOR 1", None), (5200, "rx", "PING", None), (9000, "rx", "HOLD", None)]
        ev, tx, sim = self.run_both(load_example("scout_esp32.rudra"), "scout", 14000, events)
        self.assertEqual(ev, sim_event_lines(sim))
        self.assertEqual(tx, sim_tx_lines(sim))
        self.assertIn("HOT", sim.tx)
        self.assertIn("ERR", sim.tx)
        self.assertTrue(any(e.startswith("S ") for e in ev) and any(e.startswith("N ") for e in ev))

    def test_i2c_write_and_odd_sizes_match(self):
        src = wrap('output d: i2c(addr 0x20, reg 9, bytes 3, safe 5)\noutput e: i2c(addr 0x21, bytes 1, safe 0)\n'
                   'input a: i2c(addr 0x50, bytes 3)\ninput b: i2c(addr 0x51, reg 4, bytes 4, signed)\n'
                   'on SET(v) { d = v  e = v }\nevery 1s { send "A", a  send "B", b }', "esp32", hb="heartbeat 4s")
        ck = compile_src(src)
        self.assertTrue(ck.ok, [d.msg for d in ck.errors])
        events = [(0, "set", "a", 0x123456), (0, "set", "b", -2), (1500, "rx", "SET 66051", None),
                  (2500, "rx", "SET -1", None), (3000, "set", "b", 7)]
        ev, tx, sim = self.run_both(ck, "i2cmix", 8000, events)
        self.assertEqual(ev, sim_event_lines(sim))
        self.assertEqual(tx, sim_tx_lines(sim))
        self.assertIn("A 1193046", sim.tx)

    def test_neopixel_rgb_order_and_big_clamps(self):
        src = wrap('output n: neopixel(pin 5, count 2, rgb, safe off)\non C(r, g, b) { fill n, r, g, b }', "esp32",
                   hb="heartbeat 2s")
        ck = compile_src(src)
        ev, tx, sim = self.run_both(ck, "np", 3000, [(100, "rx", "C 999 -5 10", None)])
        self.assertEqual(ev, sim_event_lines(sim))
        self.assertIn(f"N 100 {0xFF000A} {0xFF000A}", ev)

    def test_heater_interlock_matches_simulator(self):
        events = [(0, "set", "temp", 100), (3000, "rx", "FAN 0", None), (3500, "rx", "STATUS", None),
                  (4000, "rx", "FAN 1", None), (6000, "rx", "FAN -1", None), (6500, "rx", "WHO", None),
                  (8000, "rx", "FAN 1", None), (9000, "set", "temp", 900), (10000, "rx", "STATUS", None)]
        for name in ("heater_nano.rudra", "heater_buggy.rudra"):
            ev, tx, sim = self.run_both(load_example(name), name, 16000, events)
            self.assertEqual(ev, sim_event_lines(sim), name)
            self.assertEqual(tx, sim_tx_lines(sim), name)
        self.assertGreater(sim.blocked, 0)                                   # the buggy one (last in the loop)
        self.assertTrue(any(m.startswith("STATUS ") and m != "STATUS 0" for m in sim.tx))

    def test_all_animation_modes_match_simulator(self):
        src = wrap("output r: neopixel(pin 6, count 5, safe 4, 5, 6)\non A(m, p) {\n"
                   "  if m == 1 { animate r, blink, 255, 128, 1, p }\n  if m == 2 { animate r, pulse, 255, 128, 1, p }\n"
                   "  if m == 3 { animate r, chase, 255, 128, 1, p }\n  if m == 4 { animate r, rainbow, p }\n}\n"
                   "on O { animate r, off }\non F { fill r, 7, 8, 9 }\non P { pixel r, 2, 300, -4, 5 }", hb="heartbeat 6s")
        for target in ("nano", "esp32"):
            ck = compile_src(src.replace("target nano", f"target {target}")
                             .replace("pin 6", "pin 6" if target == "nano" else "pin 18"))
            self.assertTrue(ck.ok, [d.msg for d in ck.errors])
            events = [(100, "rx", "A 1 500", None), (700, "rx", "A 2 333", None), (1500, "rx", "A 3 90", None),
                      (2500, "rx", "A 4 2000", None), (4000, "rx", "P", None), (4500, "rx", "A 3 41", None),
                      (5000, "rx", "O", None), (5400, "rx", "F", None), (5600, "rx", "A 2 7", None),   # 7 ms is clamped to 40
                      (5800, "rx", "A 1 5000000000", None)]
            ev, tx, sim = self.run_both(ck, "anim_" + target, 14000, events)
            self.assertEqual(ev, sim_event_lines(sim), target)
            self.assertEqual(tx, sim_tx_lines(sim), target)
            self.assertGreater(len([e for e in ev if e.startswith("N ")]), 100)

    def test_esp32_variant_compiles(self):
        for name in EXAMPLES:
            self.build_src(load_example(name), "esp_" + name, extra=("-DARDUINO_ARCH_ESP32",))

    def test_nano_examples_compile_with_avr_servo_header(self):
        src = wrap("output s: servo(pin 5, safe 90)\nevery 1s { s = 10 }")
        self.build_src(compile_src(src), "avrservo")


@unittest.skipUnless(have_gpp(), "g++ not available")
class LinkTests(unittest.TestCase):
    """The master-side code generated from the same source."""

    @classmethod
    def setUpClass(cls):
        cls.ck = load_example("scout_esp32.rudra")
        cls.tmp = tempfile.mkdtemp(prefix="rudrac_link_")

    def test_cpp_header_compiles_and_works(self):
        h = os.path.join(self.tmp, "scout_link.h")
        with open(h, "w") as f:
            f.write(gen_cpp(self.ck))
        exe = os.path.join(self.tmp, "link_test.exe")
        cmd = ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", os.path.join(HERE, "stub"),
               f'-DLINKH="{h}"', os.path.join(HERE, "link_test.cpp"), "-o", exe]
        r = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = subprocess.run([exe], capture_output=True, text=True).stdout
        self.assertEqual(out, "PING\nPAN 45\nCOLOR 1 2 3\nSWEEP\nWHO\nSTATUS\n"
                              "1 TEMP 1 -250\n1 BTN 0\n1 1\n0\n115200 666\n"
                              f"{protocol_hash(self.ck)}\n")

    def test_python_client_talks_to_the_simulator(self):
        path = os.path.join(self.tmp, "scout_link.py")
        with open(path, "w") as f:
            f.write(gen_python(self.ck))
        spec = importlib.util.spec_from_file_location("scout_link", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        self.assertEqual(mod.COMMANDS["DOT"], 4)
        self.assertEqual(mod.KEEPALIVE_MS, 666)

        sim = Sim(self.ck)

        class Port:
            """Fake serial port wired straight into the simulator."""
            def __init__(self):
                self.out, self.rd = [], 0

            def write(self, b):
                self.out += b.decode().splitlines()

            def readline(self):
                if self.rd < len(sim.tx):
                    self.rd += 1
                    return (sim.tx[self.rd - 1] + "\n").encode()
                return b""

        port = Port()
        link = mod.ScoutLink(port)
        t = 10

        def step(n=1):
            nonlocal t
            lines, port.out = port.out, []
            sim.tick(t, lines)
            t += n

        link.cmd_COLOR(0, 255.0, 0)
        link.cmd_PAN(30)
        link.ping()
        link.cmd_DOT(2, 9, 9, 9)
        step()
        self.assertEqual(sim.out["head"], 30)
        self.assertEqual(sim.pixels["ring"][0], 0x00FF00)
        self.assertEqual(sim.pixels["ring"][2], 0x090909)
        self.assertEqual(mod.ScoutLink.read_message(link), ("PONG", None))
        self.assertIsNone(link.read_message())
        sim.tick(2000, [])
        msg = link.read_message()
        self.assertEqual(msg, ("TEMP", 0))
        link.cmd_PAN(1)
        link._send("BOGUS")
        step()
        self.assertEqual(link.read_message(), ("ERR", None))

    def test_python_client_verify_handshake(self):
        path = os.path.join(self.tmp, "scout_link2.py")
        with open(path, "w") as f:
            f.write(gen_python(self.ck))
        spec = importlib.util.spec_from_file_location("scout_link2", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        sim = Sim(self.ck)

        class PumpPort:
            def __init__(self):
                self.out, self.rd, self.t = [], 0, 100

            def write(self, b):
                self.out += b.decode().splitlines()

            def readline(self):
                if self.out:
                    sim.tick(self.t, self.out)
                    self.out, self.t = [], self.t + 1
                if self.rd < len(sim.tx):
                    self.rd += 1
                    return (sim.tx[self.rd - 1] + "\n").encode()
                return b""

        link = mod.ScoutLink(PumpPort())
        self.assertTrue(link.verify())
        mod.PROTO_HASH = 12345                       # a master generated for a different firmware
        self.assertFalse(link.verify())
        link.status()
        self.assertEqual(link.read_message(), ("STATUS", 0))

    def test_protocol_doc_and_info(self):
        doc = gen_protocol(self.ck)
        for needle in ("115200 baud", "`COLOR 1 2 3`", "`DOT 1 2 3 4`", "`TEMP <int>`", "`HOT`", "666 ms", "`PING`",
                       "`WHO`", "`STATUS`", str(protocol_hash(self.ck))):
            self.assertIn(needle, doc)
        text = info(self.ck)
        for needle in ("servo", "neopixel", "I2C addr 0x48", "safe 255,0,0", "Adafruit NeoPixel", "ESP32Servo",
                       "WHO", "STATUS", "ALERT", "fingerprint"):
            self.assertIn(needle, text)
        heater = info(load_example("heater_nano.rudra"))
        self.assertIn("never heater and not fan", heater)
        nohb = compile_src(wrap("input a: analog(pin A0)\nevery 1s { send \"X\", a }", hb=""))
        self.assertIn("None.", gen_protocol(nohb))


class VerifyTests(unittest.TestCase):
    def test_correct_script_is_clean(self):
        rep = verify(load_example("heater_nano.rudra"), runs=60, seconds=10, seed=3)
        self.assertEqual(rep.findings, [])
        self.assertEqual(rep.internal_errors, [])
        self.assertIn("OK", format_report(rep, "heater_nano.rudra"))

    def test_buggy_script_is_caught_with_a_trace(self):
        rep = verify(load_example("heater_buggy.rudra"), runs=60, seconds=10, seed=3)
        self.assertEqual([(f.rule, f.output, f.value) for f in rep.findings], [(1, "fan", 0)])
        f = rep.findings[0]
        self.assertGreater(len(f.runs), 3)
        self.assertTrue(any("FAN" in line for line in f.trace))
        text = format_report(rep, "x")
        self.assertIn("never heater and not fan", text)
        self.assertIn("FOUND", text)

    def test_same_seed_same_report(self):
        a = verify(load_example("heater_buggy.rudra"), runs=20, seconds=5, seed=9)
        b = verify(load_example("heater_buggy.rudra"), runs=20, seconds=5, seed=9)
        self.assertEqual(format_report(a, "x"), format_report(b, "x"))

    def test_no_rules_message(self):
        rep = verify(load_example("fan_nano.rudra"), runs=2, seconds=1)
        self.assertIn("no 'never' rules", format_report(rep, "fan_nano.rudra"))


if __name__ == "__main__":
    unittest.main()
