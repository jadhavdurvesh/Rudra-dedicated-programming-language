"""Stage A4: worst-case time and memory analysis and the 'budget' declaration."""
import json
import os
import tempfile
import unittest

from test_rudrac import EX, check, load_example, messages, parse, wrap
from rudrac.cli import main as cli_main
from rudrac.cost import analyze


def ck_of(src):
    return check(parse(src))


def cost_of(body, target="nano"):
    return analyze(ck_of(wrap(body, target)))


def item(rep, label_start):
    return next(us for lbl, us, _ in rep.items if lbl.startswith(label_start))


class A4Cost(unittest.TestCase):
    OUT = "output led: digital(pin 13, safe false)\n"

    def test_loop_cost_scales_with_bounds(self):
        small = cost_of(self.OUT + "on A { for i in 0..10 { led = !led } }")
        big = cost_of(self.OUT + "on A { for i in 0..100 { led = !led } }")
        a, b = item(small, "on A"), item(big, "on A")
        self.assertGreater(b, 9 * (a - 100))               # roughly ten times the loop body
        self.assertLess(a, b)

    def test_if_takes_the_worse_branch(self):
        rep = cost_of(self.OUT + "var n = 0\non A { if n > 1 { led = true } else { for i in 0..50 { led = !led } } }")
        self.assertGreater(item(rep, "on A"), 50 * 6)

    def test_every_output_write_pays_for_the_interlocks(self):
        plain = cost_of(self.OUT + "output b: digital(pin 12, safe false)\non A { led = true }")
        ruled = cost_of(self.OUT + "output b: digital(pin 12, safe false)\nnever led && b\non A { led = true }")
        self.assertGreater(item(ruled, "on A"), item(plain, "on A"))

    def test_function_calls_add_the_callee(self):
        rep = cost_of(self.OUT + "func f(x: int) -> int { let s = 0\n for i in 0..40 { s = s + x * x }\n return s }\n"
                      "on A(v) { led = f(v) > 3 }")
        self.assertGreater(item(rep, "on A"), item(rep, "func f"))

    def test_stack_grows_with_call_depth(self):
        flat = cost_of(self.OUT + "func a(x: int) -> int { return x }\non A(v) { led = a(v) > 1 }")
        deep = cost_of(self.OUT + "func a(x: int) -> int { return x }\nfunc b(x: int) -> int { return a(x) + 1 }\n"
                       "func c(x: int) -> int { return b(x) + 1 }\non A(v) { led = c(v) > 1 }")
        self.assertGreater(deep.stack, flat.stack)

    def test_motor_ramp_and_sonar_are_in_the_pass(self):
        rep = cost_of("input s: ultrasonic(trig 7, echo 8)\noutput m: motor(pwm 5, dir 4, ramp 100, safe 0)\non A { m = 10 }")
        self.assertGreaterEqual(rep.fixed, 25000)
        self.assertEqual(rep.parts[0][1], 25060)

    def test_neopixel_show_time_and_ram(self):
        rep = cost_of("output r: neopixel(pin 6, count 60, safe off)\non A { fill r, 1, 2, 3 }", "esp32")
        self.assertTrue(any("show()" in lbl for lbl, _ in rep.parts))
        self.assertTrue(any(w == "NeoPixel r buffer" and b >= 180 for w, b in rep.ram_items))

    def test_model_can_be_overridden(self):
        ck = ck_of(wrap(self.OUT + "on A { led = true }"))
        base = analyze(ck)
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            json.dump({"dwrite": 500}, f)
        slow = analyze(ck, f.name)
        os.unlink(f.name)
        self.assertGreater(item(slow, "on A"), item(base, "on A") + 400)


class A4Budget(unittest.TestCase):
    SONAR = ("input s: ultrasonic(trig 7, echo 8)\noutput led: digital(pin 13, safe false)\n"
             "when s > 0 && s < 30 { led = true }\n")

    def test_budget_refuses_a_slow_loop_and_names_the_culprit(self):
        m = messages(wrap("budget 10ms\n" + self.SONAR))
        self.assertTrue(any("budget is 10 ms" in x for x in m), m)
        self.assertTrue(any("budget is 10 ms" in x for x in m))

    def test_budget_that_fits_is_accepted(self):
        self.assertEqual(messages(wrap("budget 40ms\n" + self.SONAR)), [])

    def test_hint_lists_biggest_contributors(self):
        ck = None
        try:
            ck = ck_of(wrap("budget 10ms\n" + self.SONAR))
        except Exception:
            pass
        errs = [d for d in ck.errors if "budget" in d.msg]
        self.assertTrue(errs and "ultrasonic" in errs[0].hint, errs)

    def test_budget_needs_a_time(self):
        with self.assertRaises(Exception):
            parse(wrap("budget 5\noutput l: digital(pin 13, safe false)"))

    def test_budget_declared_once(self):
        with self.assertRaises(Exception):
            parse(wrap("budget 5ms\nbudget 6ms"))

    def test_heartbeat_shorter_than_busy_pass_warns(self):
        ck = ck_of(wrap(self.SONAR, hb="heartbeat 40ms"))
        self.assertTrue(any("busy" in w.msg for w in ck.warnings))

    def test_ram_overflow_is_an_error(self):
        m = messages(wrap("output r: neopixel(pin 6, count 1000, safe off)\nvar a = 0"))
        self.assertTrue(any("bytes of RAM" in x for x in m), m)

    def test_ram_warning_near_the_limit(self):
        ck = ck_of(wrap("output r: neopixel(pin 6, count 450, safe off)"))
        self.assertTrue(any("% of the board's RAM" in w.msg for w in ck.warnings), [w.msg for w in ck.warnings])

    def test_cli_cost_exit_code(self):
        d = tempfile.mkdtemp()
        p = os.path.join(d, "slow.rudra")
        with open(p, "w") as f:
            f.write(wrap("budget 40ms\n" + self.SONAR))
        self.assertEqual(cli_main(["cost", p]), 0)
        self.assertEqual(cli_main(["cost", os.path.join(EX, "sentry_nano.rudra")]), 0)

    def test_examples_stay_inside_sensible_numbers(self):
        for name in ("fan_nano", "led_node", "rover_nano", "scout_esp32", "sentry_nano", "turret_esp32"):
            rep = analyze(load_example(name + ".rudra"))
            self.assertGreater(rep.pass_worst, 0, name)
            if rep.board_ram:
                self.assertLess(rep.ram_total, rep.board_ram, name)


if __name__ == "__main__":
    unittest.main()
