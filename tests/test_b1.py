"""Stage B1: one typed IR shared by the simulator, the C++ generator and the cost prover."""
import glob
import os
import unittest

from test_rudrac import EX, compile_src, wrap
from rudrac import ir
from rudrac.cli import main as cli_main

SRC = """
enum Mode { idle, scan }
struct P { x: int, hot: bool }
const LIM = 3
var m: Mode = Mode.idle
var a: int[LIM]
var ps: P[2]
var q: P
output led: digital(pin 13, safe false)
func twice(x: int) -> int { return x * 2 }
proc flip() { led = !led }
on A(i, v) {
  let t = twice(v)
  a[i] = t
  a[1] = 5
  for k in 0..LIM { a[k] = k  if a[k] > 1 { break } }
  ps[i].x = t  ps[0].hot = t > 2  q.x = 1
  m = Mode.scan
  if m == Mode.scan and not led { flip() }
  send "R", ps[0].hot
  send "S", a[i]
}
"""


def lowered(src=SRC):
    ck = compile_src(wrap(src, "nano"))
    assert ck.ok, [d.msg for d in ck.errors]
    return ck, ir.Lowerer(ck)


def all_nodes(low, ck):
    for e in ck.node.events:
        for s in ir.walk_stmts(low.body(e.body, e.params)):
            yield s
            for x in ir.stmt_exprs(s):
                yield from ir.walk_expr(x)


class IRTests(unittest.TestCase):
    def test_everything_is_resolved(self):
        ck, low = lowered()
        for n in all_nodes(low, ck):
            self.assertEqual(type(n).__module__, "rudrac.ir", f"AST node leaked into the IR: {n}")

    def test_constants_and_enums_are_folded(self):
        ck, low = lowered()
        body = low.body(next(e for e in ck.node.events if e.name == "A").body, ["i", "v"])
        loop = next(s for s in body if isinstance(s, ir.Loop))
        self.assertEqual((loop.lo, loop.hi), (0, 3))                     # const LIM folded
        setm = next(s for s in ir.walk_stmts(body) if isinstance(s, ir.SetG) and s.slot == "m")
        self.assertEqual((setm.e.v, setm.e.ty), (1, "int"))              # Mode.scan -> 1

    def test_proven_flags_and_slots(self):
        ck, low = lowered()
        body = low.body(next(e for e in ck.node.events if e.name == "A").body, ["i", "v"])
        sets = [s for s in ir.walk_stmts(body) if isinstance(s, ir.SetE)]
        by = {(s.slot, type(s.idx).__name__, s.proven) for s in sets}
        self.assertIn(("a", "Loc", False), by)           # a[i]: i is a parameter, checked at run time
        self.assertIn(("a", "K", True), by)              # a[1]: constant inside the array
        self.assertIn(("a", "Loc", True), by)            # a[k]: loop counter inside the array
        self.assertIn(("ps.x", "Loc", False), by)        # struct array fields are their own slots
        self.assertTrue(any(isinstance(s, ir.SetG) and s.slot == "q.x" for s in ir.walk_stmts(body)))

    def test_bool_sends_are_typed(self):
        ck, low = lowered()
        body = low.body(next(e for e in ck.node.events if e.name == "A").body, ["i", "v"])
        sends = {s.msg: s.is_bool for s in ir.walk_stmts(body) if isinstance(s, ir.SendS)}
        self.assertEqual(sends, {"R": True, "S": False})

    def test_validate_passes_on_every_example(self):
        for path in sorted(glob.glob(os.path.join(EX, "*.rudra"))):
            with open(path) as f:
                ck = compile_src(f.read())
            self.assertGreater(ir.validate(ck), 0, path)

    def test_validate_rejects_a_wrong_proof(self):
        ck, low = lowered()
        body = low.body(next(e for e in ck.node.events if e.name == "A").body, ["i", "v"])
        s = next(s for s in ir.walk_stmts(body) if isinstance(s, ir.SetE) and isinstance(s.idx, ir.K))
        s.idx.v = 9                                      # corrupt: a "proven" index outside the array
        with self.assertRaises(ir.IRError):
            ir.validate(ck, low)

    def test_validate_rejects_bad_types(self):
        ck, low = lowered()
        ret = low.fn_body("twice")[0]
        self.assertEqual(ret.e.op, "*")
        ret.e.l = ir.K(True, "bool")                     # corrupt: true * 2
        with self.assertRaises(ir.IRError):
            ir.validate(ck, low)

    def test_lowering_is_cached_and_deterministic(self):
        ck, low = lowered()
        e = ck.node.events[0]
        self.assertIs(low.body(e.body, e.params), low.body(e.body, e.params))
        self.assertEqual(ir.dump(ck), ir.dump(ck))

    def test_dump_shows_proofs(self):
        ck, _ = lowered()
        text = ir.dump(ck)
        self.assertIn("(index proven)", text)
        self.assertIn("(index checked at run time)", text)
        self.assertIn("nodes checked", text)

    def test_cli_ir(self):
        path = os.path.join(EX, "fan_nano.rudra")
        self.assertEqual(cli_main(["ir", path]), 0)

    def test_a_proven_index_costs_less_than_a_checked_one(self):
        from rudrac.cost import analyze
        base = "var a: int[4]\noutput led: digital(pin 13, safe false)\non A(i) { %s }\non B { led = true }"
        def us(body):
            rep = analyze(compile_src(wrap(base % body, "nano")))
            return next(u for lbl, u, _ in rep.items if lbl.startswith("on A"))
        self.assertLess(us("a[1] = 5"), us("a[i] = 5"))
        self.assertLess(us("for k in 0..4 { a[k] = 5 }"), us("for k in 0..4 { a[i] = 5 }"))

    def test_golden_values_not_just_sim_equals_cpp(self):
        """Sim and C++ read the same IR, so a wrong lowering would fool a sim-vs-C++ comparison.
        These pin the expected numbers independently."""
        from rudrac.sim import Sim
        src = ("const K = 7\nenum E { a, b, c }\nvar e: E = E.c\nvar arr: int[3]\nvar f: bool\n"
               "output led: digital(pin 13, safe false)\n"
               "on T(z, j) { send \"K\", K + 1  send \"E\", e  send \"B\", E.b\n"
               "  let x = 2147483647  send \"W\", x + 1\n  send \"N\", -K  send \"D\", 7 / z  send \"M\", -7 % 3\n"
               "  f = K > 3  send \"F\", f  send \"G\", !f\n"
               "  for i in 0..3 { arr[i] = i * K }  send \"S\", arr[2]  send \"O\", arr[j]  led = true }")
        ck = compile_src(wrap(src, "nano"))
        self.assertTrue(ck.ok, [d.msg for d in ck.errors])
        sim = Sim(ck)
        sim.run(300, [(100, "rx", "T 0 3", None)])
        got = {m.split()[0]: m.split()[1] for m in sim.tx if " " in m}
        self.assertEqual(got, {"K": "8", "E": "2", "B": "1", "W": "-2147483648", "N": "-7", "D": "0",
                               "M": "-1", "F": "1", "G": "0", "S": "14", "O": "0"})

    def test_callees_and_locals(self):
        ck, low = lowered()
        body = low.body(next(e for e in ck.node.events if e.name == "A").body, ["i", "v"])
        self.assertEqual(set(ir.callees(body)), {"twice", "flip"})
        self.assertEqual(ir.count_locals(body), 2)       # let t, loop counter k


if __name__ == "__main__":
    unittest.main()
