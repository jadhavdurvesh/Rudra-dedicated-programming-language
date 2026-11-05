"""Stage A1/A2: func, proc, let, for, break, return, and wrap-safe arithmetic.
Behaviour tests compile the generated C++ and require the simulator to agree event for event."""
import unittest

from test_rudrac import ParityMixin, Sim, check, have_gpp, messages, parse, sim_event_lines, sim_tx_lines, wrap


def ck_of(src):
    return check(parse(src))


PROGRAM = '''
  output led: digital(pin 13, safe false)
  output fan: pwm(pin 9, safe 0)
  var total = 0
  var hits = 0

  func clamp(v: int, lo: int, hi: int) -> int {
    if v < lo { return lo }
    if v > hi { return hi }
    return v
  }
  func sq(x: int) -> int { return x * x }
  func isqrt(n: int) -> int {                       // loop with an early return
    for i in 0..50 { if i * i > n { return i - 1 } }
    return 49
  }
  func even(n: int) -> bool { return n % 2 == 0 }
  func score(a: int, b: int) -> int { return clamp(sq(a) + sq(b), 0, 100) }

  proc bump() { hits = hits + 1 }
  proc blink(n: int) {
    for i in 0..6 {
      if i >= n { break }
      led = !led
      bump()
    }
  }

  on SUM(n) {
    let s = 0
    for k in 1..9 { s = s + sq(k) }
    total = clamp(s, 0, n)
    send "TOTAL", total
  }
  on ROOT(n)  { send "ROOT", isqrt(n) }
  on PARITY(n) { let e = even(n)  if e { send "EVEN" } else { send "ODD" } }
  on SCORE(a, b) { send "SCORE", score(a, b) }
  on BLINK(n) { blink(n)  send "HITS", hits }
  on GRID {
    let c = 0
    for r in 0..4 {
      for q in 0..4 {
        if q > r { break }
        c = c + 1
      }
    }
    fan = c * 10
    send "GRID", c
  }
  on MATH(a, b) {
    send "MUL", a * b
    send "ADD", a + b
    send "SUB", a - b
    send "DIV", a / b
    send "MOD", a % b
    send "NEG", -a
  }
'''


@unittest.skipUnless(have_gpp(), "g++ not available")
class A1Parity(ParityMixin, unittest.TestCase):
    def both(self, ck, tag, end, events):
        ev, tx, sim = self.run_both(ck, tag, end, events)
        self.assertEqual(ev, sim_event_lines(sim), "hardware events differ")
        self.assertEqual(tx, sim_tx_lines(sim), "serial output differs")
        return sim

    def test_funcs_procs_loops(self):
        ck = ck_of(wrap(PROGRAM))
        cmds = ["SUM 100", "SUM 50", "ROOT 0", "ROOT 99", "ROOT 100", "ROOT 10000", "PARITY 4", "PARITY 7", "PARITY -3",
                "SCORE 3 4", "SCORE 20 20", "BLINK 3", "BLINK 9", "BLINK 0", "GRID"]
        ev = [(10 * (k + 1), "rx", c, None) for k, c in enumerate(cmds)]
        sim = self.both(ck, "a1", 400, ev)
        self.assertIn("TOTAL 100", sim.tx)
        self.assertIn("TOTAL 50", sim.tx)
        self.assertIn("ROOT 9", sim.tx)
        self.assertIn("ROOT 10", sim.tx)
        self.assertIn("EVEN", sim.tx)
        self.assertIn("SCORE 25", sim.tx)
        self.assertIn("SCORE 100", sim.tx)
        self.assertIn("HITS 9", sim.tx)          # 3 + 6 + 0 blinks
        self.assertIn("GRID 10", sim.tx)

    def test_arithmetic_wraps_like_32_bit_hardware(self):
        ck = ck_of(wrap(PROGRAM))
        ev = [(10, "rx", "MATH 2000000000 3", None), (20, "rx", "MATH -2147483648 -1", None),
              (30, "rx", "MATH 7 0", None), (40, "rx", "MATH -7 2", None), (50, "rx", "MATH 2147483647 2147483647", None),
              (60, "rx", "MATH -2147483648 1", None)]
        sim = self.both(ck, "wrap", 200, ev)
        self.assertIn("DIV -2147483648", sim.tx)       # INT_MIN / -1 wraps instead of crashing
        self.assertIn("MOD 0", sim.tx)
        self.assertIn("MUL 1705032704", sim.tx)        # 6e9 mod 2**32, as signed
        self.assertIn("NEG -2147483648", sim.tx)

    def test_for_loop_runs_each_time_it_is_entered(self):
        ck = ck_of(wrap('''
          var n = 0
          every 100ms { for i in 0..3 { n = n + 1 } send "N", n }
        '''))
        sim = self.both(ck, "loopev", 350, [])
        self.assertEqual(sim.tx, ["N 3", "N 6", "N 9"])


class A1Checker(unittest.TestCase):
    def errs(self, body):
        return messages(wrap("output led: digital(pin 13, safe false)\n" + body))

    def test_recursion_rejected(self):
        m = self.errs("func f(x: int) -> int { return g(x) }\nfunc g(x: int) -> int { return f(x) }\non A { send \"X\", f(1) }")
        self.assertTrue(any("recursion" in x for x in m), m)

    def test_direct_recursion_rejected(self):
        m = self.errs("proc p(n: int) { p(n) }\non A { p(1) }")
        self.assertTrue(any("recursion" in x for x in m), m)

    def test_func_cannot_change_state(self):
        m = self.errs("var v = 0\nfunc f() -> int { v = 1\n return 1 }\non A { send \"X\", f() }")
        self.assertTrue(any("cannot change" in x for x in m), m)
        m = self.errs("func f() -> int { led = true\n return 1 }\non A { send \"X\", f() }")
        self.assertTrue(any("cannot change" in x for x in m), m)

    def test_func_cannot_send_or_call_proc(self):
        m = self.errs("func f() -> int { send \"X\"\n return 1 }\non A { send \"Y\", f() }")
        self.assertTrue(any("only computes a value" in x for x in m), m)
        m = self.errs("proc p() { }\nfunc f() -> int { p()\n return 1 }\non A { send \"Y\", f() }")
        self.assertTrue(any("cannot call the proc" in x for x in m), m)

    def test_proc_has_no_value(self):
        m = self.errs("proc p() { }\non A { send \"Y\", p() }")
        self.assertTrue(any("is a proc and has no value" in x for x in m), m)

    def test_every_path_must_return(self):
        m = self.errs("func f(x: int) -> int { if x > 1 { return 1 } }\non A { send \"Y\", f(2) }")
        self.assertTrue(any("not every path" in x for x in m), m)

    def test_types_and_arity(self):
        m = self.errs("func f(x: int) -> int { return x }\non A { send \"Y\", f(true) }")
        self.assertTrue(any("must be int, not bool" in x for x in m), m)
        m = self.errs("func f(x: int) -> int { return x }\non A { send \"Y\", f(1, 2) }")
        self.assertTrue(any("takes 1 argument" in x for x in m), m)
        m = self.errs("on A { send \"Y\", nope(1) }")
        self.assertTrue(any("unknown function" in x for x in m), m)
        m = self.errs("func f() -> bool { return 1 }\non A { send \"Y\", 1 }")
        self.assertTrue(any("returns bool, not int" in x for x in m), m)

    def test_loops_are_bounded(self):
        self.assertTrue(any("at most 10000" in x for x in self.errs("on A { for i in 0..10001 { } }")))
        self.assertTrue(any("is not a constant" in x for x in self.errs("var n = 3\non A { for i in 0..n { } }")))
        self.assertTrue(any("never runs" in x for x in self.errs("on A { for i in 5..2 { } }")))
        self.assertTrue(any("only be used inside a loop" in x for x in self.errs("on A { break }")))
        self.assertTrue(any("loop counter" in x for x in self.errs("on A { for i in 0..3 { i = 1 } }")))

    def test_locals_are_scoped(self):
        m = self.errs("on A { if true { let q = 1 }\n send \"X\", q }")
        self.assertTrue(any("unknown name 'q'" in x for x in m), m)
        m = self.errs("var v = 0\non A { let v = 1 }")
        self.assertTrue(any("hides" in x for x in m), m)
        m = self.errs("on A { let x = 1\n let x = 2 }")
        self.assertTrue(any("hides" in x for x in m), m)

    def test_goto_not_in_loops_or_procs(self):
        m = self.errs("state s { on A { for i in 0..2 { goto s } } }")
        self.assertTrue(any("inside a loop" in x for x in m), m)
        m = self.errs("proc p() { goto s }\nstate s { on A { p() } }")
        self.assertTrue(any("'proc' block" in x for x in m), m)

    def test_no_calls_in_rules_or_consts(self):
        m = self.errs("func f() -> bool { return true }\nnever f() && led")
        self.assertTrue(any("cannot call a function" in x for x in m), m)
        m = self.errs("func f() -> int { return 1 }\nconst K = f()")
        self.assertTrue(any("cannot be used here" in x for x in m), m)

    def test_unused_function_warns(self):
        ck = ck_of(wrap("func f() -> int { return 1 }"))
        self.assertTrue(any("never used" in w.msg for w in ck.warnings))


if __name__ == "__main__":
    unittest.main()
