"""v0.4 features: debounce, when..for, states, motors, steppers, black box, shared bus.
Every behaviour test compiles the generated C++ and requires the simulator to agree event for event."""
import unittest

from test_rudrac import (ParityMixin, RudraError, Sim, check, have_gpp, load_example, messages,
                         parse, sim_event_lines, sim_tx_lines, wrap)


def ck_of(src):
    return check(parse(src))


@unittest.skipUnless(have_gpp(), "g++ not available")
class V04Parity(ParityMixin, unittest.TestCase):
    def both(self, ck, tag, end, events):
        ev, tx, sim = self.run_both(ck, tag, end, events)
        self.assertEqual(ev, sim_event_lines(sim), "hardware events differ")
        self.assertEqual(tx, sim_tx_lines(sim), "serial output differs")
        return sim

    # ---- debounce and when..for
    def test_debounce_filters_bounce(self):
        ck = ck_of(wrap('''
          input b: digital(pin 2, pullup, invert, debounce 20ms)
          output led: digital(pin 13, safe false)
          var n = 0
          when b { n = n + 1  send "PRESS", n }
          every 50ms { led = b }
        '''))
        bounce = [(100, "set", "b", True), (103, "set", "b", False), (106, "set", "b", True),
                  (109, "set", "b", False), (112, "set", "b", True),          # settles pressed at 112
                  (300, "set", "b", False), (305, "set", "b", True), (310, "set", "b", False),
                  (500, "rx", "PING", None)]
        sim = self.both(ck, "deb", 800, bounce)
        self.assertEqual(sim.tx.count("PRESS 1"), 1)
        self.assertNotIn("PRESS 2", sim.tx)

    def test_when_for_needs_a_steady_condition(self):
        ck = ck_of(wrap('''
          input b: digital(pin 2, pullup, invert)
          var n = 0
          when b for 200ms { n = n + 1  send "HELD", n }
        '''))
        ev = [(100, "set", "b", True), (250, "set", "b", False),      # too short
              (400, "set", "b", True), (700, "set", "b", False),      # long enough -> one HELD
              (800, "set", "b", True), (1100, "set", "b", False)]     # again -> second HELD
        sim = self.both(ck, "whenfor", 1500, ev)
        self.assertEqual([m for m in sim.tx if m.startswith("HELD")], ["HELD 1", "HELD 2"])

    # ---- state machines
    def test_states_scoped_handlers(self):
        ck = ck_of(wrap('''
          output a: digital(pin 5, safe false)
          output b: digital(pin 6, safe false)
          state one {
            enter { a = true }
            exit  { a = false }
            on NEXT { goto two }
            on WHO2 { send "IN", 1 }
            every 100ms { send "T1" }
          }
          state two {
            enter { b = true }
            exit  { b = false }
            on NEXT { goto one }
            on WHO2 { send "IN", 2 }
            when true for 350ms { send "LATE"  goto one }
          }
          on ONLYGLOBAL { send "G" }
        '''))
        ev = [(250, "rx", "WHO2", None), (300, "rx", "NEXT", None), (320, "rx", "WHO2", None),
              (330, "rx", "ONLYGLOBAL", None), (900, "rx", "STATE", None), (1000, "rx", "NEXT", None),
              (1010, "rx", "NEXT", None), (1020, "rx", "NEXT", None), (1500, "rx", "STATE", None)]
        sim = self.both(ck, "states", 2000, ev)
        self.assertIn("IN 1", sim.tx)
        self.assertIn("IN 2", sim.tx)
        self.assertIn("LATE", sim.tx)
        self.assertIn("STATE 0", sim.tx)

    def test_command_unknown_in_this_state_is_refused(self):
        ck = ck_of(wrap('''
          state s1 { on HI { goto s2 } }
          state s2 { on BYE { goto s1 } }
        '''))
        sim = self.both(ck, "refuse", 300, [(10, "rx", "BYE", None), (20, "rx", "HI", None), (30, "rx", "HI", None),
                                           (40, "rx", "BYE", None)])
        self.assertEqual(sim.tx.count("ERR"), 2)

    # ---- black box
    def test_black_box_ring_and_dump(self):
        ck = ck_of(wrap('''
          blackbox 3
          output o: digital(pin 5, safe false)
          output p: digital(pin 6, safe false)
          never o && p
          state a { on GO { goto b } }
          state b { on GO { goto a } }
          on BOTH { o = true  p = true }
        ''', hb="heartbeat 2s"))
        ev = [(100, "rx", "GO", None), (200, "rx", "GO", None), (300, "rx", "BOTH", None),
              (400, "rx", "GO", None), (500, "rx", "LOG", None)]
        sim = self.both(ck, "bb", 5000, ev)
        # boot(1) state(3) state(3) blocked(4) state(3) ... ring of 3 keeps the newest; heartbeat loss (2) at 2400 ms
        self.assertEqual(len(sim.bb), 3)
        self.assertGreater(sim.bb_total, 3)
        self.assertTrue(any(m.startswith("LOGEND") for m in sim.tx))

    def test_black_box_records_heartbeat_loss(self):
        ck = ck_of(wrap('''
          blackbox 4
          output o: digital(pin 5, safe false)
          on ON { o = true }
        ''', hb="heartbeat 1s"))
        sim = self.both(ck, "bbhb", 3000, [(100, "rx", "ON", None), (2500, "rx", "LOG", None)])
        self.assertIn("LOG 1100 2 0", sim.tx)

    # ---- motors
    def test_motor_ramp_limit_and_timeout(self):
        ck = ck_of(wrap('''
          output m: motor(pwm 5, dir 4, ramp 100, limit 60, timeout 600ms, safe 0)
          on SPEED(v) { m = v }
        '''))
        ev = [(100, "rx", "SPEED 100", None), (700, "rx", "SPEED -50", None), (800, "rx", "SPEED -50", None),
              (1000, "rx", "SPEED -50", None), (1300, "rx", "SPEED 20", None), (2500, "rx", "PING", None)]
        sim = self.both(ck, "motor", 4000, ev)
        self.assertEqual(sim.out["m"], 0)                 # timed out and stopped
        self.assertTrue(all(abs(p[3]) <= 255 for p in sim.phys if p[0] == "W"))

    def test_motor_ramp_always_passes_through_stop(self):
        ck = ck_of(wrap('''
          output m: motor(pwm 5, dir 4, ramp 150, safe 0)
          on SPEED(v) { m = v }
        '''))
        sim = self.both(ck, "motorzero", 2500, [(10, "rx", "SPEED 50", None), (900, "rx", "SPEED -50", None)])
        pwm = ck.node.decls[0].cfg["pins"]["pwm"].key
        duties = [p[3] for p in sim.phys if p[0] == "W" and p[2] == pwm]
        k = max(i for i, d in enumerate(duties) if d == 0 and i > 3)
        self.assertEqual(duties[k], 0)                     # reached 0 before the reverse direction ramped up
        self.assertEqual(sim.cur["m"], -50)

    def test_goto_current_state_does_nothing(self):
        ck = ck_of(wrap('''
          state a {
            enter { send "ENTER" }
            on SAME { goto a }
            on OTHER { goto b }
          }
          state b { on BACK { goto a } }
        '''))
        sim = self.both(ck, "same", 300, [(10, "rx", "SAME", None), (20, "rx", "OTHER", None), (30, "rx", "BACK", None)])
        self.assertEqual(sim.tx.count("ENTER"), 2)         # at boot and after BACK, never for SAME

    def test_motor_dual_pwm_style(self):
        ck = ck_of(wrap('''
          output m: motor(fwd 5, rev 6, safe 0)
          on SPEED(v) { m = v }
        '''))
        sim = self.both(ck, "motor2", 500, [(50, "rx", "SPEED 70", None), (100, "rx", "SPEED -30", None),
                                           (150, "rx", "SPEED 0", None)])
        self.assertEqual(sim.cur["m"], 0)

    def test_motor_never_rule_blocks_write(self):
        ck = ck_of(wrap('''
          output lamp: digital(pin 13, safe false)
          output m: motor(pwm 5, dir 4, safe 0)
          never m > 0 && lamp == false
          on RUN(v) { m = v }
          on LAMP { lamp = true }
        '''))
        sim = self.both(ck, "motorrule", 400, [(10, "rx", "RUN 50", None), (20, "rx", "LAMP", None),
                                              (30, "rx", "RUN 50", None), (40, "rx", "STATUS", None)])
        self.assertEqual(sim.blocked, 1)

    # ---- stepper
    def test_stepper_moves_at_its_rate_and_returns_home_on_link_loss(self):
        ck = ck_of(wrap('''
          output s: stepper(step 8, dir 9, rate 250, safe 0)
          on GOTO(p) { s = p }
        ''', hb="heartbeat 500ms"))
        sim = self.both(ck, "stepper", 1500, [(10, "rx", "GOTO 40", None), (200, "rx", "GOTO -10", None)])
        self.assertEqual(sim.sp["s"], 0)                  # link lost at ~700 ms: back to the safe position

    # ---- bus
    def test_bus_addressing_broadcast_and_rs485(self):
        ck = load_example("turret_esp32.rudra")
        ev = [(10, "rx", "@3 PING", None), (20, "rx", "@4 PING", None), (30, "rx", "PING", None),
              (40, "rx", "@0 AIM 25", None), (50, "rx", "@3 WHERE", None), (60, "rx", "@3 AIM 3", None),
              (70, "rx", "@0 PING", None), (80, "rx", "@3 NOPE", None), (90, "rx", "@3x PING", None),
              (100, "rx", "@ 3 PING", None), (400, "rx", "@3 WHERE", None), (500, "set", "home_sw", True),
              (560, "rx", "@3 SWITCH", None)]
        sim = self.both(ck, "turret", 800, ev)
        self.assertEqual([m for m in sim.tx if "PONG" in m], ["@3 PONG"])     # broadcast and strangers: silent
        self.assertIn("@3 AT 25", sim.tx)
        self.assertEqual(sim.sp["pan"], 3)            # the later @3 AIM 3 won over the broadcast
        de = ck.node.rs485_pin.key
        self.assertTrue(any(p[0] == "W" and p[2] == de and p[3] == 1 for p in sim.phys))

    def test_rover_example(self):
        ck = load_example("rover_nano.rudra")
        ev = [(100, "rx", "GO 60", None), (700, "set", "bump", True), (720, "set", "bump", False),   # bounce: ignored
              (900, "set", "bump", True), (1500, "set", "bump", False), (1700, "rx", "STATE", None),
              (4000, "rx", "LOG", None), (4100, "rx", "GO 70", None), (4600, "rx", "STOP", None),
              (4700, "rx", "STATUS", None), (9000, "rx", "LOG", None)]
        sim = self.both(ck, "rover", 10000, ev)
        self.assertIn("BUMP", sim.tx)
        self.assertEqual(sim.tx.count("BUMP"), 1)


class V04Checker(unittest.TestCase):
    def test_unknown_state_suggestion(self):
        m = messages(wrap("state idle { on A { goto idel } }"))
        self.assertTrue(any("unknown state 'idel'" in x for x in m), m)

    def test_goto_must_be_last(self):
        m = messages(wrap("output o: digital(pin 5, safe false)\nstate s { on A { goto s  o = true } }"))
        self.assertTrue(any("can never run" in x for x in m), m)

    def test_goto_not_in_enter(self):
        m = messages(wrap("state s { enter { goto s } }"))
        self.assertTrue(any("'enter'" in x for x in m), m)

    def test_bus_send_only_in_commands(self):
        m = messages(wrap("address 2\nevery 1s { send \"X\" }"))
        self.assertTrue(any("only speak when it is asked" in x for x in m), m)

    def test_address_range(self):
        self.assertTrue(any("between 1 and 99" in x for x in messages(wrap("address 100"))))

    def test_rs485_needs_address(self):
        self.assertTrue(any("needs an 'address'" in x for x in messages(wrap("rs485 pin 8"))))

    def test_motor_needs_pins(self):
        m = messages(wrap("output m: motor(pwm 5, safe 0)"))
        self.assertTrue(any("motor needs" in x for x in m), m)

    def test_motor_pwm_pin_must_support_pwm(self):
        m = messages(wrap("output m: motor(pwm 4, dir 7, safe 0)"))
        self.assertTrue(any("PWM" in x for x in m), m)

    def test_motor_safe_in_range(self):
        m = messages(wrap("output m: motor(pwm 5, dir 4, limit 50, safe 60)"))
        self.assertTrue(any("safe state must be between" in x for x in m), m)

    def test_pin_clash_between_motor_and_output(self):
        m = messages(wrap("output m: motor(pwm 5, dir 4, safe 0)\noutput o: digital(pin 4, safe false)"))
        self.assertTrue(any("already used" in x for x in m), m)

    def test_duplicate_state_handlers(self):
        m = messages(wrap("state s { on A { } on A { } }"))
        self.assertTrue(any("handled twice" in x for x in m), m)

    def test_command_argument_count_must_match(self):
        m = messages(wrap("state a { on X(v) { } }\nstate b { on X { } }"))
        self.assertTrue(any("argument" in x for x in m), m)

    def test_unreachable_state_warns(self):
        ck = ck_of(wrap("state a { on X { } }\nstate b { on Y { goto a } }"))
        self.assertTrue(any("never entered" in w.msg for w in ck.warnings))

    def test_reserved_commands(self):
        for name in ("STATE", "LOG"):
            self.assertTrue(any("built in" in x for x in messages(wrap(f"on {name} {{ }}"))))

    def test_when_for_needs_duration(self):
        self.assertTrue(any("duration" in x or "for" in x for x in messages(wrap(
            "input b: digital(pin 2)\nwhen b for 0ms { }"))))

    def test_stepper_needs_pins(self):
        self.assertTrue(any("stepper needs" in x for x in messages(wrap("output s: stepper(step 8, safe 0)"))))


if __name__ == "__main__":
    unittest.main()


@unittest.skipUnless(have_gpp(), "g++ not available")
class SensorParity(ParityMixin, unittest.TestCase):
    def both(self, ck, tag, end, events):
        ev, tx, sim = self.run_both(ck, tag, end, events)
        self.assertEqual(ev, sim_event_lines(sim), "hardware events differ")
        self.assertEqual(tx, sim_tx_lines(sim), "serial output differs")
        return sim

    def test_sentry_example(self):
        ck = load_example("sentry_nano.rudra")
        ev = [(100, "set", "sonar", 120), (300, "rx", "GET", None),
              (500, "set", "sonar", 20), (600, "set", "sonar", 45),      # too short a visit: ignored
              (800, "set", "sonar", 20), (1200, "rx", "GET", None),       # steady: horn on after 200 ms
              (1500, "set", "sonar", 0), (1700, "rx", "BEEP 440", None), (1800, "rx", "BEEP 99999", None),
              (1900, "rx", "BEEP -5", None), (2200, "rx", "GET", None)]
        sim = self.both(ck, "sentry", 4000, ev)
        tones = [p for p in sim.phys if p[0] == "T"]
        first = next(p for p in tones if p[3] == 880)
        self.assertEqual(first[1], 1040)       # seen at the 840 ms measurement, steady for 200 ms
        self.assertTrue(any(p[3] == 880 for p in tones))
        self.assertTrue(any(p[3] == 20000 for p in tones))                # clamped
        self.assertEqual(sim.out["horn"], 0)                              # heartbeat lost long ago: safe = silent
        self.assertIn("DIST 20", sim.tx)

    def test_tone_clamp_and_safe_state(self):
        ck = ck_of(wrap('''
          output b: tone(pin 6, safe 0)
          on F(v) { b = v }
        ''', hb="heartbeat 500ms"))
        sim = self.both(ck, "tone", 1200, [(10, "rx", "F 1000", None), (20, "rx", "F -3", None), (30, "rx", "F 30000", None)])
        self.assertEqual(sim.out["b"], 0)


class SensorChecker(unittest.TestCase):
    def test_ultrasonic_needs_both_pins(self):
        m = messages(wrap("input s: ultrasonic(trig 7)"))
        self.assertTrue(any("'echo'" in x for x in m), m)

    def test_ultrasonic_echo_pin_clash(self):
        m = messages(wrap("input s: ultrasonic(trig 7, echo 8)\noutput o: digital(pin 8, safe false)"))
        self.assertTrue(any("already used" in x for x in m), m)

    def test_tone_needs_safe(self):
        self.assertTrue(any("safe state" in x for x in messages(wrap("output b: tone(pin 6)"))))

    def test_tone_safe_range(self):
        self.assertTrue(any("between 0 and 20000" in x for x in messages(wrap("output b: tone(pin 6, safe 30000)"))))

    def test_tone_conflicts_with_pwm_on_timer2_pins(self):
        m = messages(wrap("output b: tone(pin 6, safe off)\noutput f: pwm(pin 3, safe 0)"))
        self.assertTrue(any("Timer2" in x or "tone" in x for x in m), m)

    def test_ultrasonic_is_input_only(self):
        self.assertTrue(any("cannot be" in x for x in messages(wrap("output s: ultrasonic(trig 7, echo 8)"))))
