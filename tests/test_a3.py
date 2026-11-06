"""Stage A3: arrays, structs and enums (generated C++ must behave exactly like the simulator)."""
import os
import unittest

from test_rudrac import (ParityMixin, compile_src, have_gpp, messages, sim_event_lines, sim_tx_lines, wrap)

OUT = "output led: digital(pin 13, safe false)\n"
BASE = """
enum Mode { idle, scan, fire }
struct Point { x: int, y: int, hot: bool }
const N = 4
var mode: Mode = Mode.idle
var buf: int[N]
var flags: bool[3]
var modes: Mode[2]
var pts: Point[3]
var home: Point
output led: digital(pin 13, safe false)
blackbox 8
on SET(i, v) { buf[i] = v }
on GET(i) { send "V", buf[i] }
on FILL { for k in 0..N { buf[k] = k * k } }
on SUM { let s = 0  for k in 0..N { s = s + buf[k] }  send "SUM", s }
on FLAG(i, v) { flags[i] = v > 0 }
on FL(i) { send "F", flags[i] }
on MD(i) { modes[i] = Mode.fire }
on MQ(i) { if modes[i] == Mode.fire { send "FIRE" } else { send "NOFIRE" } }
on P(i, a) { pts[i].x = a  pts[i].hot = a > 5  home.y = a  home.x = pts[i].x * 2 }
on PQ(i) { send "X", pts[i].x  send "H", pts[i].hot  send "HY", home.y  send "HX", home.x }
on GO { mode = Mode.scan }
on Q { if mode == Mode.scan { send "SCANNING" } send "M", mode }
on SHIFT { for k in 0..3 { buf[k] = buf[k + 1] } }
on SELF(i) { buf[buf[i]] = 7 }
on LED { led = true }
"""


class A3Parity(ParityMixin, unittest.TestCase):
    def go(self, tag, cmds, src=BASE, end=None):
        ck = compile_src(wrap(src, "nano", hb="heartbeat 100s"))
        self.assertTrue(ck.ok, [d.msg for d in ck.errors])
        events = [(100 + 50 * k, "rx", c, None) for k, c in enumerate(cmds)]
        ev, tx, sim = self.run_both(ck, tag, end or (200 + 50 * len(cmds)), events)
        self.assertEqual(ev, sim_event_lines(sim))
        self.assertEqual(tx, sim_tx_lines(sim))
        return sim

    def test_array_read_write_and_out_of_range(self):
        sim = self.go("a3arr", ["SET 2 11", "GET 2", "SET 9 5", "GET 9", "SET -1 5", "GET -1", "SET 4 1", "GET 3",
                                "FILL", "SUM", "GET 3", "SHIFT", "SUM", "STATUS", "LOG"])
        self.assertIn("V 11", sim.tx)
        self.assertIn("SUM 14", sim.tx)
        self.assertEqual(sim.oob, 3)                      # 9, -1 and 4 are outside 0..3
        self.assertIn("STATUS 3", sim.tx)
        self.assertEqual(sim.tx.count("V 0"), 3)          # reads outside return 0
        self.assertTrue(any(c == 6 for _, c, _ in sim.bb))

    def test_values_that_are_themselves_indexes(self):
        sim = self.go("a3self", ["SET 0 2", "SELF 0", "GET 2", "SET 1 99", "SELF 1", "STATUS"])
        self.assertIn("V 7", sim.tx)
        self.assertIn("STATUS 1", sim.tx)

    def test_bool_arrays_and_enum_arrays(self):
        sim = self.go("a3bool", ["FLAG 1 5", "FL 1", "FL 0", "FL 8", "FLAG 8 1", "MD 1", "MQ 1", "MQ 0", "MQ 7", "STATUS"])
        self.assertIn("F 1", sim.tx)
        self.assertIn("FIRE", sim.tx)
        self.assertEqual(sim.tx.count("NOFIRE"), 2)       # idx 0 never set, idx 7 reads as 'idle' (0)
        self.assertIn("STATUS 1", sim.tx)

    def test_structs_and_struct_arrays(self):
        sim = self.go("a3struct", ["P 1 9", "PQ 1", "P 0 2", "PQ 0", "PQ 1", "P 5 5", "PQ 5", "GO", "Q"])
        self.assertIn("X 9", sim.tx)
        self.assertIn("HX 18", sim.tx)
        self.assertIn("H 1", sim.tx)
        self.assertIn("SCANNING", sim.tx)
        self.assertIn("M 1", sim.tx)

    def test_enum_starts_at_first_member(self):
        sim = self.go("a3enum", ["Q"])
        self.assertIn("M 0", sim.tx)
        self.assertNotIn("SCANNING", sim.tx)

    def test_nodes_without_blackbox_still_count(self):
        src = BASE.replace("blackbox 8\n", "")
        sim = self.go("a3nobb", ["SET 7 1", "STATUS"], src)
        self.assertIn("STATUS 1", sim.tx)

    @unittest.skipIf(os.environ.get("RUDRA_ENGINE") == "vm", "inspects the native C++ text")
    def test_proven_loop_indexes_use_no_runtime_check(self):
        from rudrac.gen import generate
        ck = compile_src(wrap(BASE, "nano", hb="heartbeat 100s"))
        code = generate(ck)
        self.assertIn("v_buf[a_k] = rt_mul(a_k, a_k);", code)       # proven: plain indexing
        self.assertIn("rt_geti(v_buf, 4, a_i)", code)                # parameter: checked
        self.assertIn("v_buf[a_k]", code)

    def test_loop_reaching_past_the_end_is_checked_not_trusted(self):
        sim = self.go("a3past", ["SHIFT", "STATUS"])                  # buf[k + 1] reaches index 3 at most: fine
        self.assertIn("STATUS 0", sim.tx)
        src = BASE.replace("on SHIFT { for k in 0..3", "on SHIFT { for k in 0..6")
        sim = self.go("a3past2", ["FILL", "SHIFT", "STATUS", "GET 3"], src)
        self.assertIn("STATUS 2", sim.tx)                             # k = 4, 5 write beyond the end


class A3Checker(unittest.TestCase):
    def err(self, body, frag):
        msgs = messages(wrap(OUT + body))
        self.assertTrue(any(frag in m for m in msgs), f"wanted '{frag}' in {msgs}")

    def test_constant_index_outside(self):
        self.err("var a: int[3]\non A { a[3] = 1 }", "outside the array")
        self.err("var a: int[3]\non A { send \"X\", a[-1] }", "outside the array")

    def test_array_needs_index_struct_needs_field(self):
        self.err("var a: int[3]\non A { send \"X\", a }", "is an array")
        self.err("struct P { x: int }\nvar p: P\non A { send \"X\", p }", "is a struct")
        self.err("struct P { x: int }\nvar p: P[2]\non A { send \"X\", p.x }", "array of structs")
        self.err("struct P { x: int }\nvar p: P[2]\non A { send \"X\", p[0] }", "holds structs")

    def test_enum_rules(self):
        e = "enum M { a, b }\nenum K { c, d }\nvar m: M = M.a\n"
        self.err(e + "on A { m = M.z }", "no member")
        self.err(e + "on A { m = K.c }", "cannot assign")
        self.err(e + "on A { m = m + 1 }", "")
        self.err(e + "on A { m = 1 }", "cannot assign")
        self.err(e + "on A { if m < M.b { led = true } }", "")
        self.err(e + "on A { if m == K.c { led = true } }", "")

    def test_unknown_things(self):
        self.err("var a: Nope", "unknown type")
        self.err("struct P { x: int }\nvar p: P\non A { p.q = 1 }", "no field")
        self.err("var a: int[0]", "length")
        self.err("var a: int[2000]", "length")
        self.err("var a: int[2] = 3", "starts with every element zero")

    def test_no_nesting_no_2d_no_rule_use(self):
        self.err("struct Q { x: int }\nstruct P { q: Q }", "unknown type")
        self.err("var a: int[3]\non A { a[1][2] = 1 }", "")
        self.err("var a: int[3]\nnever a[0] > 1", "cannot use array")

    def test_func_cannot_write_arrays(self):
        self.err("var a: int[3]\nfunc f() -> int { a[0] = 1\n return 1 }", "cannot change")

    def test_index_must_be_number(self):
        self.err("var a: int[3]\non A { a[true] = 1 }", "index must be a number")

    def test_type_mismatch_on_assign(self):
        self.err("var a: bool[3]\non A { a[0] = 5 }", "cannot assign")
        self.err("struct P { x: int }\nvar p: P\non A { p.x = true }", "cannot assign")

    def test_good_program_has_no_errors(self):
        ck = compile_src(wrap(BASE, "nano", hb="heartbeat 100s"))
        self.assertTrue(ck.ok)


if __name__ == "__main__":
    unittest.main()
