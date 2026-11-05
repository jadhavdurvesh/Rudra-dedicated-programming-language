"""v0.4 master side: addressed clients, bus master, black box readout, all driven against the simulator."""
import importlib.util
import os
import subprocess
import tempfile
import unittest

from test_rudrac import HERE, Sim, check, have_gpp, load_example, parse, wrap
from rudrac.bus import check_bus, gen_bus_python
from rudrac.link import gen_cpp, gen_python, gen_protocol, info


def ck_of(src):
    return check(parse(src))


def load_module(text, name):
    d = tempfile.mkdtemp(prefix="rudrac_link_")
    path = os.path.join(d, name + ".py")
    with open(path, "w") as f:
        f.write(text)
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


class WirePort:
    """One shared wire: every line written reaches every simulated node; replies queue up for the master."""

    def __init__(self, sims):
        self.sims, self.now, self.q, self.seen = sims, 0, [], [0] * len(sims)

    def _step(self, lines=()):
        self.now += 1
        for k, s in enumerate(self.sims):
            s.tick(self.now, lines)
            for t in s.tx[self.seen[k]:]:
                self.q.append(t)
            self.seen[k] = len(s.tx)

    def write(self, data):
        self._step([data.decode().strip()])

    def readline(self):
        for _ in range(60):
            if self.q:
                return (self.q.pop(0) + "\n").encode()
            self._step()
        return b""


NODE_A = '''node alpha target nano {
  address 3
  blackbox 8
  output o: digital(pin 5, safe false)
  state idle { on ON { o = true  goto lit } }
  state lit  { on OFF { o = false  goto idle } }
  on READ { send "O", o }
  heartbeat 5s
}'''
NODE_B = '''node bravo target nano {
  address 4
  output p: pwm(pin 6, safe 0)
  on SET(v) { p = v }
  on READ { send "P", p }
  heartbeat 5s
}'''


class V04Link(unittest.TestCase):
    def test_bus_master_talks_to_two_nodes_and_broadcast_is_silent(self):
        a, b = ck_of(NODE_A), ck_of(NODE_B)
        self.assertEqual(check_bus([a, b]), [])
        mod = load_module(gen_bus_python([a, b]), "bus_master")
        sims = [Sim(a), Sim(b)]
        bus = mod.Bus(WirePort(sims))
        self.assertEqual(bus.verify_all(), {"alpha": True, "bravo": True})
        bus.send("bravo", "SET", 70)
        self.assertEqual(bus.ask("bravo", "READ")[1:], ("P", [70]))
        self.assertEqual(bus.ask("alpha", "READ")[1:], ("O", [0]))
        bus.send("alpha", "ON")
        self.assertEqual(bus.ask("alpha", "STATE")[1:], ("STATE", [1]))
        before = sum(len(s.tx) for s in sims)
        bus.broadcast("PING")
        for _ in range(5):
            bus.port._step()
        self.assertEqual(sum(len(s.tx) for s in sims), before)        # broadcast: no replies
        with self.assertRaises(KeyError):
            bus.send("alpha", "NOPE")
        with self.assertRaises(TypeError):
            bus.send("bravo", "SET")

    def test_black_box_readout(self):
        a = ck_of(NODE_A)
        mod = load_module(gen_bus_python([a]), "bus_master_a")
        sim = Sim(a)
        bus = mod.Bus(WirePort([sim]))
        bus.send("alpha", "ON")
        bus.send("alpha", "OFF")
        log = bus.black_box("alpha")
        self.assertEqual([x[1] for x in log], ["boot", "state", "state"])
        self.assertEqual([x[2] for x in log], [0, 1, 0])

    def test_per_node_client_ignores_other_addresses(self):
        a, b = ck_of(NODE_A), ck_of(NODE_B)
        client = load_module(gen_python(a), "alpha_link")
        port = WirePort([Sim(a), Sim(b)])
        node = client.AlphaLink(port)
        self.assertTrue(node.verify())
        node.cmd_ON()
        node.state()
        got = [node.read_message() for _ in range(3)]
        self.assertIn(("STATE", 1), got)
        self.assertEqual(client.STATES, {"idle": 0, "lit": 1})
        self.assertEqual(client.ADDRESS, 3)
        log = node.log()
        self.assertEqual(log[-1][1], "state")

    def test_bus_checks(self):
        a = ck_of(NODE_A)
        dup = ck_of(NODE_B.replace("address 4", "address 3"))
        self.assertTrue(any("both use address 3" in e for e in check_bus([a, dup])))
        noaddr = ck_of(wrap("output o: digital(pin 5, safe false)"))
        self.assertTrue(any("no 'address'" in e for e in check_bus([noaddr])))
        fast = ck_of(NODE_B.replace("address 4", "address 4\n serial 115200"))
        self.assertTrue(any("baud" in e for e in check_bus([a, fast])))

    def test_fingerprint_changes_with_states_and_address(self):
        base = ck_of(NODE_A)
        from rudrac.link import protocol_hash
        other_addr = ck_of(NODE_A.replace("address 3", "address 5"))
        more_state = ck_of(NODE_A.replace("state lit  {", "state extra { on X { goto idle } }\n  state lit  {"))
        self.assertNotEqual(protocol_hash(base), protocol_hash(other_addr))
        self.assertNotEqual(protocol_hash(base), protocol_hash(more_state))

    def test_docs_mention_bus_states_and_log(self):
        doc = gen_protocol(ck_of(NODE_A))
        for word in ("address 3", "broadcast", "`STATE`", "`LOG`", "Only in state"):
            self.assertIn(word, doc)
        txt = info(load_example("rover_nano.rudra"))
        self.assertIn("States:", txt)

    @unittest.skipUnless(have_gpp(), "g++ not available")
    def test_cpp_header_for_addressed_node_compiles(self):
        ck = load_example("turret_esp32.rudra")
        d = tempfile.mkdtemp(prefix="rudrac_cpp_")
        hdr = os.path.join(d, "turret_link.h")
        with open(hdr, "w") as f:
            f.write(gen_cpp(ck))
        exe = os.path.join(d, "t.exe")
        r = subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", os.path.join(HERE, "stub"),
                            f'-DLINKH="{hdr}"', os.path.join(HERE, "link_bus_test.cpp"), "-o", exe],
                           capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = subprocess.run([exe], capture_output=True, text=True).stdout.splitlines()
        self.assertEqual(out[:3], ["@3 PING", "@3 AIM 120", "@3 WHERE"])
        self.assertEqual(out[3], "1 3 AT 1 120")
        self.assertEqual(out[4], "-1 PONG")
        self.assertEqual(out[5], "3")


if __name__ == "__main__":
    unittest.main()
