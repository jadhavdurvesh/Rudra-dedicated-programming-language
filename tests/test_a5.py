"""Stage A5: import - libraries of shared definitions (consts, enums, structs, funcs, procs, never rules)."""
import os
import tempfile
import unittest

from test_rudrac import ParityMixin, have_gpp, sim_event_lines, sim_tx_lines
from rudrac import check, parse
from rudrac.util import LINE_STRIDE, RudraError, format_diag

COMMON = """// shared by every node on the robot
const HOT = 80
enum Mode { idle, run }
func clamp(x: int, lo: int, hi: int) -> int { if x < lo { return lo } if x > hi { return hi } return x }
func spare() -> int { return 1 }
never heater and not fan
"""
NODE = """import "common.rudra"
node %s target nano {
  output heater: digital(pin %d, safe false)
  output fan: digital(pin %d, safe true)
  var m: Mode = Mode.idle
  on SET(v) { send "V", clamp(v, 0, HOT) }
  on HEAT { heater = true }
  on FANOFF { fan = false }
  on RUN { m = Mode.run  send "M", m }
  heartbeat 100s
}
"""


class Dir:
    def __init__(self):
        self.d = tempfile.mkdtemp()

    def write(self, name, text):
        p = os.path.join(self.d, name)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w") as f:
            f.write(text)
        return p

    def load(self, name):
        path = os.path.join(self.d, name)
        with open(path) as f:
            src = f.read()
        sources = {}
        try:
            return check(parse(src, path, sources)), sources, src, path
        except RudraError as e:
            e.sources, e.src, e.path = sources, src, path
            raise


class ImportBasics(unittest.TestCase):
    def setUp(self):
        self.t = Dir()
        self.t.write("common.rudra", COMMON)

    def test_two_nodes_share_one_definition(self):
        self.t.write("a.rudra", NODE % ("a", 5, 6))
        self.t.write("b.rudra", NODE % ("b", 7, 8))
        for n in "ab":
            ck = self.t.load(f"{n}.rudra")[0]
            self.assertTrue(ck.ok, [d.msg for d in ck.errors])
            self.assertEqual(len(ck.rules), 1)                                  # the shared interlock
            self.assertEqual({f.name for f in ck.fns}, {"clamp", "spare"})

    def test_changing_the_library_changes_every_node(self):
        self.t.write("a.rudra", NODE % ("a", 5, 6))
        self.t.write("b.rudra", NODE % ("b", 7, 8))
        self.t.write("common.rudra", COMMON.replace("HOT = 80", "HOT = 10"))
        from rudrac import Sim
        for n in "ab":
            sim = Sim(self.t.load(f"{n}.rudra")[0])
            sim.run(300, [(100, "rx", "SET 50", None)])
            self.assertIn("V 10", sim.tx)

    def test_shared_pin_map(self):
        self.t.write("pins.rudra", "const FAN_PIN = 6\nconst HEATER_PIN = 5\n")
        self.t.write("c.rudra", 'import "pins.rudra"\nnode c target nano {\n output fan: digital(pin FAN_PIN, safe true)\n'
                     ' output heater: pwm(pin HEATER_PIN, safe 0)\n on X { heater = 5 }\n on Y { fan = false }\n heartbeat 3s\n}\n')
        ck = self.t.load("c.rudra")[0]
        self.assertTrue(ck.ok, [d.msg for d in ck.errors])
        from rudrac import generate
        code = generate(ck)
        self.assertIn("P_fan = 6", code)
        self.assertIn("P_heater = 5", code)

    def test_two_nodes_must_not_share_a_pin_even_through_the_map(self):
        self.t.write("pins.rudra", "const P = 6\n")
        self.t.write("c.rudra", 'import "pins.rudra"\nnode c target nano {\n output a: digital(pin P, safe false)\n'
                     ' output b: digital(pin 6, safe false)\n on X { a = true }\n heartbeat 3s\n}\n')
        ck = self.t.load("c.rudra")[0]
        self.assertFalse(ck.ok)

    def test_unused_library_function_is_not_a_warning_but_a_node_one_is(self):
        self.t.write("a.rudra", NODE.replace("  heartbeat", "  func mine() -> int { return 2 }\n  heartbeat") % ("a", 5, 6))
        ck = self.t.load("a.rudra")[0]
        msgs = [d.msg for d in ck.diags]
        self.assertTrue(any("'mine' is never used" in m for m in msgs), msgs)
        self.assertFalse(any("spare" in m for m in msgs), msgs)

    def test_diamond_imports_load_each_file_once(self):
        self.t.write("x.rudra", 'import "common.rudra"\nconst X = 1\n')
        self.t.write("y.rudra", 'import "common.rudra"\nconst Y = 2\n')
        self.t.write("a.rudra", 'import "x.rudra"\nimport "y.rudra"\n' + NODE.split("\n", 1)[1] % ("a", 5, 6))
        ck = self.t.load("a.rudra")[0]
        self.assertTrue(ck.ok, [d.msg for d in ck.errors])

    def test_imports_resolve_relative_to_the_importing_file(self):
        self.t.write("lib/inner.rudra", "const DEEP = 7\n")
        self.t.write("lib/outer.rudra", 'import "inner.rudra"\nconst OUT = DEEP + 1\n')
        self.t.write("n.rudra", 'import "lib/outer.rudra"\nnode n target nano {\n output o: digital(pin 5, safe false)\n'
                     ' on A { send "X", OUT }\n heartbeat 5s\n}\n')
        from rudrac import Sim
        sim = Sim(self.t.load("n.rudra")[0])
        sim.run(300, [(100, "rx", "A", None)])
        self.assertIn("X 8", sim.tx)


class ImportErrors(unittest.TestCase):
    def setUp(self):
        self.t = Dir()

    def fail(self, name):
        with self.assertRaises(RudraError) as cm:
            self.t.load(name)
        return cm.exception

    def test_missing_file(self):
        self.t.write("a.rudra", 'import "nope.rudra"\nnode a target nano { heartbeat 1s }\n')
        e = self.fail("a.rudra")
        self.assertIn("cannot import", e.diags[0].msg)

    def test_cycle(self):
        self.t.write("p.rudra", 'import "q.rudra"\nconst P = 1\n')
        self.t.write("q.rudra", 'import "p.rudra"\nconst Q = 1\n')
        self.t.write("a.rudra", 'import "p.rudra"\nnode a target nano { heartbeat 1s }\n')
        e = self.fail("a.rudra")
        self.assertIn("import cycle", e.diags[0].msg)

    def test_library_may_not_contain_hardware_or_handlers(self):
        for body in ("output o: digital(pin 5, safe false)", "var x = 1", "on A { }", "heartbeat 2s"):
            self.t.write("bad.rudra", body + "\n")
            self.t.write("a.rudra", 'import "bad.rudra"\nnode a target nano { heartbeat 1s }\n')
            e = self.fail("a.rudra")
            self.assertIn("library file may only contain", e.diags[0].msg, body)

    def test_error_in_a_library_points_at_the_library(self):
        self.t.write("lib.rudra", "const A = 1\nfunc f() -> int { return nope }\n")
        self.t.write("a.rudra", 'import "lib.rudra"\nnode a target nano {\n on X { send "V", f() }\n heartbeat 1s\n}\n')
        ck, sources, src, path = self.t.load("a.rudra")
        self.assertFalse(ck.ok)
        d = ck.errors[0]
        text = format_diag(d, src, path, sources)
        self.assertEqual(d.line // LINE_STRIDE, 1)
        self.assertIn("lib.rudra:2:", text)
        self.assertIn("return nope", text)

    def test_a_rule_that_names_a_missing_output_is_reported_in_the_library(self):
        self.t.write("lib.rudra", "never heater and not fan\n")
        self.t.write("a.rudra", 'import "lib.rudra"\nnode a target nano {\n output other: digital(pin 5, safe false)\n heartbeat 1s\n}\n')
        try:
            self.t.load("a.rudra")
        except RudraError as e:
            self.assertEqual(e.diags[0].line // LINE_STRIDE, 1)
            return
        ck = self.t.load("a.rudra")[0]
        self.assertFalse(ck.ok)
        self.assertTrue(any(d.line // LINE_STRIDE == 1 for d in ck.errors))

    def test_name_clash_between_library_and_node(self):
        self.t.write("lib.rudra", "const HOT = 1\n")
        self.t.write("a.rudra", 'import "lib.rudra"\nnode a target nano {\n const HOT = 2\n heartbeat 1s\n}\n')
        ck = self.t.load("a.rudra")[0]
        self.assertFalse(ck.ok)


@unittest.skipUnless(have_gpp(), "g++ not available")
class ImportedInterlockParity(ParityMixin, unittest.TestCase):
    """The shared 'never' rule must be enforced identically by the generated C++ and the simulator."""

    def test_shared_rule_in_generated_code(self):
        t = Dir()
        t.write("common.rudra", COMMON)
        t.write("a.rudra", NODE % ("a", 5, 6))
        ck = t.load("a.rudra")[0]
        events = [(100, "rx", "HEAT", None), (150, "rx", "FANOFF", None), (200, "rx", "HEAT", None),
                  (250, "rx", "SET 500", None), (300, "rx", "RUN", None), (350, "rx", "STATUS", None)]
        ev, tx, sim = self.run_both(ck, "a5", 600, events)
        self.assertEqual(ev, sim_event_lines(sim))
        self.assertEqual(tx, sim_tx_lines(sim))
        self.assertIn("V 80", sim.tx)
        self.assertIn("M 1", sim.tx)
        self.assertGreater(sim.blocked, 0)                       # fan off while heater on was refused


if __name__ == "__main__":
    unittest.main()
