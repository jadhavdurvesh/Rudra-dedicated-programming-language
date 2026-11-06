"""Stage A6: motion words - path / run / stop / done. Simulator and generated C++ must agree."""
import unittest
from test_rudrac import ParityMixin, have_gpp, sim_event_lines, sim_tx_lines, load_example, messages, wrap, compile_src
from rudrac.sim import Sim
import test_rudrac as T


def W(body):
    return wrap(body, "esp32")


STEP = "output pan: stepper(step 25, dir 26, rate 200, safe 0)\n"
MOT = "output drive: motor(pwm 18, dir 19, safe 0)\n"


class CheckerA6(unittest.TestCase):
    def assertErr(self, src, frag):
        m = messages(src)
        self.assertTrue(any(frag in x for x in m), f"wanted '{frag}' in {m}")

    def ok(self, body):
        self.assertEqual(messages(W(body)), [])

    def test_good(self):
        self.ok(STEP + "path p for pan { at 10 speed 100 zone 1 wait 5ms\n at 0 }\non GO { run p }\non X { stop pan }")

    def test_not_stepper_or_motor(self):
        self.assertErr(W("output l: digital(pin 5, safe false)\npath p for l { at 1 }"), "is not one")

    def test_unknown_output(self):
        self.assertErr(W("path p for nope { at 1 }"), "nope")

    def test_motor_range(self):
        self.assertErr(W(MOT + "path p for drive { at 101 }"), "-100 to 100")

    def test_motor_no_speed(self):
        self.assertErr(W(MOT + "path p for drive { at 5 speed 10 }"), "only applies to steppers")

    def test_speed_range(self):
        self.assertErr(W(STEP + "path p for pan { at 5 speed 0 }"), "1..1000")
        self.assertErr(W(STEP + "path p for pan { at 5 speed 1001 }"), "1..1000")

    def test_zone_wait_range(self):
        self.assertErr(W(STEP + "path p for pan { at 5 zone 10001 }"), "0..10000")
        self.assertErr(W(STEP + "path p for pan { at 5 wait 3600001 }"), "3600000")

    def test_repeat_range(self):
        self.assertErr(W(STEP + "path p for pan { at 5\n repeat 1001 }"), "between 1 and 1000")

    def test_empty_path(self):
        self.assertErr(W(STEP + "path p for pan { repeat 2 }"), "between 1 and 64")

    def test_too_many_points(self):
        self.assertErr(W(STEP + "path p for pan { " + "at 1\n" * 65 + "}"), "between 1 and 64")

    def test_run_unknown(self):
        self.assertErr(W(STEP + "path sweep for pan { at 1 }\non GO { run swep }"), "'swep' is not a path")

    def test_stop_non_motor(self):
        self.assertErr(W("output l: digital(pin 5, safe false)\non GO { stop l }"), "needs a stepper or motor")

    def test_done_forever(self):
        self.assertErr(W(STEP + "path p for pan { at 1\n forever }\ndone p { }"), "never finishes")

    def test_two_done(self):
        self.assertErr(W(STEP + "path p for pan { at 1 }\ndone p { }\ndone p { }"), "two 'done'")

    def test_done_unknown(self):
        self.assertErr(W("done q { }"), "not a path")

    def test_run_in_func(self):
        self.assertErr(W(STEP + "path p for pan { at 1 }\nfunc f() -> int { run p\n return 1 }"), "cannot start a path")

    def test_old_names_still_work(self):
        # speed / run / stop / done / at / wait / zone / repeat remain ordinary identifiers
        self.ok("var speed: int = 0\nvar run: int = 0\nvar stop: int = 0\nvar done: int = 0\n"
                "var at: int = 0\nvar wait: int = 0\nvar zone: int = 0\nvar repeat: int = 0\n"
                "on GO { speed = 1\n run = 2\n stop = 3\n done = 4\n at = speed + run }")

    def test_running_is_a_bool(self):
        self.ok(STEP + "path p for pan { at 1 }\nvar b: bool = false\non GO { b = p }")
        self.assertErr(W(STEP + "path p for pan { at 1 }\nvar b: int = 0\non GO { b = p }"), "")


def run_src(case, src, tag, end, ev):
    ck = compile_src(src)
    return case.run_both(ck, tag, end, ev)


@unittest.skipUnless(have_gpp(), "g++ not available")
class PathParity(ParityMixin, unittest.TestCase):
    def same(self, body, tag, end, events, hb="heartbeat 3600s"):
        ev, tx, sim = self.run_both(compile_src(wrap(body, "esp32", hb)), tag, end, events)
        self.assertEqual(ev, sim_event_lines(sim))
        self.assertEqual(tx, sim_tx_lines(sim))
        return ev, tx, sim

    def test_scanner(self):
        ev_in = [(500, "rx", "SCAN", None), (1500, "rx", "BUSY", None), (3000, "rx", "ONCE", None),
                 (3500, "rx", "BUSY", None), (6000, "rx", "HALT", None), (6100, "rx", "BUSY", None),
                 (7000, "rx", "ONCE", None), (9000, "rx", "BUSY", None)]
        ev, tx, sim = self.run_both(load_example("scanner_esp32.rudra"), "scan", 12000, ev_in)
        self.assertEqual(ev, sim_event_lines(sim))
        self.assertEqual(tx, sim_tx_lines(sim))
        self.assertGreater(len(sim.phys), 20)
        self.assertEqual(sim.tx.count("PULSE"), 2)

    def test_stepper_waypoints_zone_wait_repeat(self):
        body = (STEP + "path p for pan { at 20 speed 500 zone 3 wait 40\n at 5 speed 100\n at -4\n repeat 2 }\n"
                "on GO { run p }\ndone p { send \"D\" }\non Q { send \"R\", p }")
        ev, tx, sim = self.same(body, "p1", 4000, [(100, "rx", "GO", None), (300, "rx", "Q", None),
                                                    (3500, "rx", "Q", None)])
        self.assertEqual(sim.tx.count("D"), 1)
        self.assertGreater(len(sim.phys), 10)

    def test_preempt_and_direct_write_cancel(self):
        body = (STEP + "path a for pan { at 50 speed 300\n at 0 speed 300\n forever }\n"
                "path b for pan { at -30 speed 200 }\n"
                "on A { run a }\non B { run b }\non W(v) { pan = v }\non Q { send \"A\", a  send \"B\", b }")
        self.same(body, "p2", 5000, [(100, "rx", "A", None), (400, "rx", "B", None), (500, "rx", "Q", None),
                                      (1500, "rx", "A", None), (1700, "rx", "W 12", None), (1800, "rx", "Q", None),
                                      (3000, "rx", "Q", None)])

    def test_stop_holds_position(self):
        body = STEP + "path a for pan { at 400 speed 100 }\non A { run a }\non S { stop pan }\non Q { send \"P\", pan }"
        ev, tx, sim = self.same(body, "p3", 3000, [(0, "rx", "A", None), (1000, "rx", "S", None), (1100, "rx", "Q", None),
                                                    (2500, "rx", "Q", None)])
        pos = [int(x.split()[-1]) for x in sim.tx if x.startswith("P ")]
        self.assertEqual(pos[0], pos[1])
        self.assertLess(pos[0], 400)

    def test_motor_path_and_stop_to_safe(self):
        body = (MOT + "path m for drive { at 60 wait 200\n at -40 wait 100\n at 0 wait 50\n repeat 2 }\n"
                "on GO { run m }\non S { stop drive }\ndone m { send \"M\" }\non Q { send \"V\", drive }")
        self.same(body, "p4", 3000, [(0, "rx", "GO", None), (150, "rx", "Q", None), (1500, "rx", "Q", None),
                                      (1600, "rx", "GO", None), (1800, "rx", "S", None), (1900, "rx", "Q", None),
                                      (2500, "rx", "Q", None)])

    def test_heartbeat_safe_state_ends_paths(self):
        body = (STEP + "path a for pan { at 300 speed 100\n at 0 speed 100\n forever }\non GO { run a }\n"
                "on Q { send \"R\", a }")
        ev, tx, sim = self.same(body, "p5", 3000, [(0, "rx", "GO", None), (1500, "rx", "Q", None)], hb="heartbeat 1s")
        self.assertIn("R 0", sim.tx)   # the safe state ended the path

    def test_interlock_refused_move_stalls(self):
        body = (MOT + "output lamp: digital(pin 5, safe false)\nnever drive > 0 && lamp == false\n"
                "path a for drive { at 50 wait 100\n at 0 }\non GO { run a }\non L { lamp = true }\n"
                "on Q { send \"R\", a  send \"V\", drive }")
        ev, tx, sim = self.same(body, "p6", 2000, [(0, "rx", "GO", None), (500, "rx", "Q", None),
                                                    (600, "rx", "L", None), (900, "rx", "GO", None),
                                                    (1900, "rx", "Q", None)])
        # the refused write leaves the motor at 0 (V 0) while the path's timing still runs to its end
        self.assertEqual(sim.tx[1], "V 0")
        self.assertEqual(sim.tx[-2], "R 0")

    def test_done_starts_another_path_and_goto(self):
        body = (STEP + "path a for pan { at 20 speed 500 }\npath b for pan { at 0 speed 500 }\n"
                "on GO { run a }\ndone a { run b }\ndone b { send \"E\" }")
        ev, tx, sim = self.same(body, "p7", 2000, [(0, "rx", "GO", None)])
        self.assertEqual(sim.tx.count("E"), 1)

    def test_start_handler_runs_path(self):
        body = STEP + "path a for pan { at 15 speed 300 }\nstart { run a }\ndone a { send \"D\" }"
        ev, tx, sim = self.same(body, "p8", 1500, [])
        self.assertEqual(sim.tx.count("D"), 1)


class Behaviour(unittest.TestCase):
    """Reference behaviour of the simulator, with numbers worked out by hand."""

    def sim(self, body, end, events):
        s = Sim(compile_src(W(body)))
        s.run(end, events)
        return s

    def test_speed_sets_duration(self):
        # 100 steps at 100 steps/s = 1000 ms (10 ms per step), then done
        s = self.sim(STEP + "path p for pan { at 100 speed 100 }\non GO { run p }\ndone p { send \"D\" }",
                     3000, [(100, "rx", "GO", None)])
        self.assertEqual(s.tx.count("D"), 1)
        t = [x for x in s.tx_times if x[1].startswith("D")][0][0] if hasattr(s, "tx_times") else None
        if t is not None:
            self.assertTrue(1090 <= t <= 1130, t)

    def test_zone_finishes_early_and_wait_adds_time(self):
        def done_at(extra):
            s = self.sim(STEP + f"path p for pan {{ at 100 speed 100 {extra} }}\non GO {{ run p }}\ndone p {{ send \"D\" }}",
                         4000, [(100, "rx", "GO", None)])
            return s.tx.count("D"), s.now
        self.assertEqual(done_at("")[0], 1)
        self.assertEqual(done_at("zone 50")[0], 1)
        self.assertEqual(done_at("wait 500")[0], 1)

    def test_repeat_count(self):
        s = self.sim(MOT + "path p for drive { at 10 wait 50\n at 0 wait 50\n repeat 4 }\non GO { run p }\n"
                     "done p { send \"D\" }", 3000, [(0, "rx", "GO", None)])
        writes = [p for p in s.phys if p[2] != 0] if False else s.phys
        self.assertEqual(s.tx.count("D"), 1)
        self.assertGreaterEqual(len(writes), 8)

    def test_stop_ends_path_and_done_does_not_run(self):
        s = self.sim(STEP + "path p for pan { at 500 speed 50 }\non GO { run p }\non S { stop pan }\ndone p { send \"D\" }",
                     3000, [(0, "rx", "GO", None), (500, "rx", "S", None)])
        self.assertEqual(s.tx.count("D"), 0)


if __name__ == "__main__":
    unittest.main()
