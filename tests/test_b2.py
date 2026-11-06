"""Stage B2: bytecode, the VM, and replacing a node's program over the serial link."""
import unittest

from test_rudrac import ParityMixin, compile_src, have_gpp, sim_event_lines, sim_tx_lines, wrap
from rudrac import bytecode as B
from rudrac.sim import Sim

BASE = """
output led: digital(pin 13, safe false)
var n = 0
var buf: int[3]
enum M { a, b }
func twice(x: int) -> int { return x * 2 }
proc bump() { n = n + 1 }
start { n = 0 }
on GO { send "R", __GO__ }
on ON { led = true }
on CNT(v) { bump()  buf[v] = twice(v)  send "C", n + buf[1] }
"""


def prog(go="1", extra="", start="n = 0"):
    return wrap(BASE.replace("__GO__", go).replace("n = 0 }", start + " }", 1) + extra, "nano", hb="heartbeat 100s")


def build(src):
    ck = compile_src(src)
    assert ck.ok, [d.msg for d in ck.errors]
    return ck


class ImageFormat(unittest.TestCase):
    def test_round_trip_and_disassembly(self):
        ck = build(prog())
        img = B.assemble(ck)
        again = B.decode(img.encode())
        self.assertEqual((again.shape, again.handlers, again.fns, again.code), (img.shape, img.handlers, img.fns, img.code))
        text = B.disassemble(img)
        self.assertIn("handler 0:", text)
        self.assertIn("SEND", text)
        self.assertIn("CALL", text)

    def test_damage_is_detected(self):
        data = bytearray(B.assemble(build(prog())).encode())
        for k in (0, 2, 5, 20, len(data) - 1):
            bad = bytearray(data)
            bad[k] ^= 0x40
            with self.assertRaises(B.VMError):
                B.decode(bytes(bad))
        with self.assertRaises(B.VMError):
            B.decode(bytes(data[:-3]))

    def test_shape_ignores_bodies_but_not_structure(self):
        a = B.assemble(build(prog("1"))).shape
        self.assertEqual(a, B.assemble(build(prog("2"))).shape)                       # only a body changed
        self.assertNotEqual(a, B.assemble(build(prog("1", "var extra = 0"))).shape)   # a new variable
        self.assertNotEqual(a, B.assemble(build(prog("1", "on NEW { led = false }"))).shape)

    def test_stack_and_frame_needs_are_reported(self):
        img = B.assemble(build(prog()))
        self.assertGreater(img.stack, 1)
        self.assertGreaterEqual(img.calls, 2)         # CNT -> twice / bump
        self.assertGreater(img.frame, 0)


class VMvsNative(unittest.TestCase):
    """The simulator on bytecode must behave exactly like the simulator on the IR."""

    SCRIPT = [(100, "rx", "GO", None), (150, "rx", "CNT 1", None), (200, "rx", "CNT 9", None), (250, "rx", "ON", None),
              (300, "rx", "CNT 2", None), (350, "rx", "STATUS", None)]

    def test_same_behaviour(self):
        ck = build(prog())
        a, b = Sim(ck, engine=None), Sim(ck, engine="vm")
        a.run(500, self.SCRIPT)
        b.run(500, self.SCRIPT)
        self.assertEqual(sim_tx_lines(a), sim_tx_lines(b))
        self.assertEqual(sim_event_lines(a), sim_event_lines(b))
        self.assertEqual(a.summary(), b.summary())
        self.assertIn("C 3", a.tx)                    # n=1, buf[1]=2


@unittest.skipUnless(have_gpp(), "g++ not available")
class UpdateOverSerial(ParityMixin, unittest.TestCase):
    VM = True

    def go(self, tag, ck, cmds, end=None):
        events = [(100 + 40 * k, "rx", c, None) for k, c in enumerate(cmds)]
        ev, tx, sim = self.run_both(ck, tag, end or 300 + 40 * len(cmds), events)
        self.assertEqual(ev, sim_event_lines(sim), "hardware events differ")
        self.assertEqual(tx, sim_tx_lines(sim), "serial output differs")
        return sim

    def test_update_changes_behaviour_and_reset_restores_it(self):
        ck = build(prog("1"))
        new = B.assemble(build(prog("2"))).encode()
        sim = self.go("upd", ck, ["GO", *B.push_lines(new), "GO", "IMGINFO", "IMGRESET", "GO"])
        self.assertEqual([m for m in sim.tx if m.startswith("R ")], ["R 1", "R 2", "R 1"])
        self.assertIn("IMGOK", sim.tx)
        self.assertTrue(any(m.startswith("IMGINFO ") and m.endswith(" 1") for m in sim.tx))

    def test_node_is_safe_while_it_is_being_reprogrammed(self):
        ck = build(prog("1"))
        new = B.assemble(build(prog("2"))).encode()
        lines = B.push_lines(new)
        sim = self.go("safe", ck, ["ON", lines[0], "GO", "ON", *lines[1:], "GO"])
        # led goes on, is forced off by the first IMG line, ignores ON/GO while invalid, then works again
        led = [e.split()[-1] for e in sim_event_lines(sim) if e.startswith("W ") and e.split()[2] == "13"]
        self.assertEqual(led, ["0", "1", "0"])           # boot, ON, forced safe by the first IMG line; later ON ignored
        self.assertEqual([m for m in sim.tx if m.startswith("R ")], ["R 2"])

    def test_wrong_shape_is_refused_and_the_node_stays_quiet(self):
        ck = build(prog("1"))
        other = B.assemble(build(prog("2", "var extra = 0"))).encode()
        sim = self.go("shape", ck, [*B.push_lines(other), "GO", "IMGINFO", "IMGRESET", "GO"])
        self.assertEqual(sim.tx.count("ERR"), 1)                         # IMGEND refused
        self.assertEqual([m for m in sim.tx if m.startswith("R ")], ["R 1"])   # only after IMGRESET
        self.assertTrue(any(m.endswith(" 0") for m in sim.tx if m.startswith("IMGINFO")))

    def test_bad_lines_are_refused_identically(self):
        ck = build(prog("1"))
        good = B.assemble(build(prog("2"))).encode()
        bad_hash = list(B.push_lines(good))
        bad_hash[-1] = f"IMGEND {len(good)} 12345"
        short = list(B.push_lines(good))
        short[-1] = f"IMGEND {len(good) - 1} {int.from_bytes(good[-4:], 'little')}"
        cmds = ["IMG", "IMG x", "IMG 0 zz", "IMG 0 abc", "IMG 0 ", "IMG -4 aa", "IMG 99999 aa", "IMG 0 " + "aa" * 17,
                "IMGEND", "IMGEND 5", "IMGEND 99999 1", *bad_hash, "GO", *short, "GO", "IMGRESET", "GO"]
        sim = self.go("badlines", ck, cmds, end=1500 + 40 * len(cmds))
        self.assertEqual([m for m in sim.tx if m.startswith("R ")], ["R 1"])

    def test_start_handler_runs_again_after_an_update(self):
        ck = build(prog("1", start="led = true"))
        new = B.assemble(build(prog("2", start="led = true"))).encode()
        sim = self.go("start", ck, [*B.push_lines(new)])
        led = [e.split()[-1] for e in sim_event_lines(sim) if e.startswith("W ") and e.split()[2] == "13"]
        self.assertEqual(led, ["0", "1", "0", "1"])     # boot safe, start, forced safe by IMG, start again

    def test_update_counts_as_hearing_the_master(self):
        ck = compile_src(prog("1").replace("heartbeat 100s", "heartbeat 1s"))
        new = B.assemble(ck).encode()
        sim = self.go("hb", ck, ["ON", *B.push_lines(new)], end=1400)
        self.assertEqual(sim.tx.count("IMGOK"), 1)


class MasterSide(unittest.TestCase):
    """rudrac.push drives a simulated node through the same lines a real master would send."""

    def node(self, src):
        sim = Sim(build(src), engine="vm")
        t = [0]

        def ask(line):
            t[0] += 1
            n = len(sim.tx)
            sim.tick(t[0], [line])
            for _ in range(5):
                t[0] += 1
                sim.tick(t[0])
            return sim.tx[n] if len(sim.tx) > n else ""
        return sim, ask

    def test_push_info_reset(self):
        from rudrac import push as P
        sim, ask = self.node(prog("1"))
        before = P.info(ask)
        self.assertTrue(before["valid"])
        P.push(ask, B.assemble(build(prog("2"))).encode())
        self.assertEqual(ask("GO"), "R 2")
        P.reset(ask)
        self.assertEqual(ask("GO"), "R 1")
        self.assertEqual(P.info(ask)["capacity"], before["capacity"])

    def test_push_reports_a_refusal(self):
        from rudrac import push as P
        sim, ask = self.node(prog("1"))
        wrong = B.assemble(build(prog("2", "var extra = 0"))).encode()
        with self.assertRaises(P.PushError) as cm:
            P.push(ask, wrong)
        self.assertIn("IMGRESET", str(cm.exception))
        self.assertFalse(P.info(ask)["valid"])
        P.reset(ask)
        self.assertTrue(P.info(ask)["valid"])

    def test_image_too_big_for_the_node(self):
        from rudrac import push as P
        sim, ask = self.node(prog("1"))
        cap = P.info(ask)["capacity"]
        huge = prog("1", "\n".join(f"on X{k} {{ led = true }}" for k in range(60)))
        data = B.assemble(build(huge)).encode()
        self.assertGreater(len(data), cap)

    def test_vm_cost_is_higher_and_counts_image_ram(self):
        from rudrac.cost import analyze
        ck = build(prog())
        a, b = analyze(ck), analyze(ck, vm=True)
        self.assertGreater(b.ram_total, a.ram_total)
        self.assertTrue(any("bytecode image" in w for w, _ in b.ram_items))
        wa = next(u for l, u, _ in a.items if l.startswith("on CNT"))
        wb = next(u for l, u, _ in b.items if l.startswith("on CNT"))
        self.assertGreater(wb, wa)

    def test_cli_image_and_build_vm(self):
        import os, tempfile
        from rudrac.cli import main
        d = tempfile.mkdtemp()
        src = os.path.join(d, "n.rudra")
        with open(src, "w") as f:
            f.write(prog("1"))
        self.assertEqual(main(["image", src, "--disasm"]), 0)
        self.assertTrue(os.path.getsize(os.path.join(d, "n.rimg")) > 20)
        self.assertEqual(main(["build", src, "--vm", "-o", d]), 0)
        with open(os.path.join(d, "t", "t.ino")) as f:
            self.assertIn("vm_run", f.read())
        os.environ.pop("RUDRA_ENGINE", None)


@unittest.skipUnless(have_gpp(), "g++ not available")
class Sanitized(ParityMixin, unittest.TestCase):
    """Out-of-range indexes must never touch memory outside the arrays, natively or in the VM."""

    SRC = ("var a: int[2]\nvar b: int[2]\nvar t: bool[2]\noutput led: digital(pin 13, safe false)\n"
           "on W(i, v) { a[i] = v  t[i] = v > 0 }\non R(i) { send \"A\", a[i]  send \"T\", t[i] }\n"
           "on B(i, v) { b[i] = v }\non L { led = true }")
    CMDS = ["B 0 77", "B 1 88", "W 0 5", "W 1 6", "W 2 9", "W -1 9", "R 2", "R -1", "R 0", "R 1", "R 100000",
            "W 2147483647 1", "R -2147483648", "STATUS"]

    def run_mode(self, vm):
        self.VM = vm
        ck = build(wrap(self.SRC, "nano", hb="heartbeat 100s"))
        exe = self.build_src(ck, "san" + str(vm), extra=["-fsanitize=address,undefined", "-fno-sanitize-recover=all"])
        import subprocess
        events = [f"{100 + 30 * k} RX {c}" for k, c in enumerate(self.CMDS)]
        out = subprocess.run([exe], input="\n".join(["RUN 1000", *events]) + "\n", capture_output=True, text=True)
        self.assertEqual(out.returncode, 0, out.stderr[-1500:])
        tx = out.stdout.partition("---TX---\n")[2].splitlines()
        sim = Sim(ck, engine="vm" if vm else None)
        sim.run(1000, [(100 + 30 * k, "rx", c, None) for k, c in enumerate(self.CMDS)])
        self.assertEqual(tx, sim_tx_lines(sim))
        self.assertIn("STATUS 6", sim.tx)       # 3 bad W commands x 2 refused stores

    def test_native(self):
        self.run_mode(False)

    def test_vm(self):
        self.run_mode(True)


if __name__ == "__main__":
    unittest.main()
