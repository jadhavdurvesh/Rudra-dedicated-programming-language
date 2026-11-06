"""Stage B4: the conformance suite runs, and it can fail."""
import json
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rudrac import conform

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SUITE = os.path.join(ROOT, "conformance")


class Suite(unittest.TestCase):
    def test_every_case_passes_everywhere(self):
        cases = conform.case_dirs(SUITE)
        self.assertGreaterEqual(len(cases), 30)
        modes = ("sim", "vm", "cpp", "cpp-vm") if conform.have_gpp() else ("sim", "vm")
        for d in cases:
            for mode, ok, detail in conform.run_case(d, modes):
                self.assertTrue(ok, f"{os.path.basename(d)} [{mode}]: {detail}")

    def test_cases_cover_the_language(self):
        names = " ".join(os.path.basename(d) for d in conform.case_dirs(SUITE))
        for w in ("digital", "safe", "interlock", "motor", "stepper", "servo", "i2c", "ultrasonic", "states",
                  "arithmetic", "arrays", "path", "imports", "blackbox", "recursion", "syntax"):
            self.assertIn(w, names)

    def test_expectations_are_not_empty_by_accident(self):
        for d in conform.case_dirs(SUITE):
            ex = json.load(open(os.path.join(d, "expect.json")))
            if "errors" in ex:
                self.assertTrue(ex["errors"], d)
            else:
                self.assertTrue(ex["phys"] or ex["tx"], d)

    def _copy(self, name):
        tmp = tempfile.mkdtemp()
        dst = os.path.join(tmp, name)
        shutil.copytree(os.path.join(SUITE, name), dst)
        return dst

    def test_a_wrong_expectation_fails(self):
        d = self._copy("p15-integer-arithmetic")
        p = os.path.join(d, "expect.json")
        ex = json.load(open(p))
        ex["tx"][3] = "10 DIV 4"
        json.dump(ex, open(p, "w"))
        res = conform.run_case(d, ("sim", "vm"))
        self.assertTrue(res and all(not ok for _, ok, _ in res))

    def test_a_missing_line_fails(self):
        d = self._copy("p06-never-interlock")
        p = os.path.join(d, "expect.json")
        ex = json.load(open(p))
        ex["phys"].pop()
        json.dump(ex, open(p, "w"))
        self.assertTrue(any(not ok for _, ok, _ in conform.run_case(d, ("sim",))))

    def test_accepting_a_bad_program_fails(self):
        d = self._copy("n01-unknown-name")
        src = open(os.path.join(d, "prog.rudra")).read().replace("lamp", "led")
        open(os.path.join(d, "prog.rudra"), "w").write(src)
        self.assertTrue(any(not ok for _, ok, _ in conform.run_case(d)))

    def test_wrong_error_line_fails(self):
        d = self._copy("n04-const-index-out-of-range")
        p = os.path.join(d, "expect.json")
        ex = json.load(open(p))
        ex["errors"][0]["line"] += 1
        json.dump(ex, open(p, "w"))
        self.assertTrue(any(not ok for _, ok, _ in conform.run_case(d)))

    def test_extra_diagnostic_fails(self):
        d = self._copy("n01-unknown-name")
        p = os.path.join(d, "expect.json")
        ex = json.load(open(p))
        ex["errors"].pop()           # the warning is no longer expected
        json.dump(ex, open(p, "w"))
        self.assertTrue(any(not ok for _, ok, _ in conform.run_case(d)))

    def test_cli(self):
        import subprocess
        r = subprocess.run([sys.executable, "-m", "rudrac", "conform"], capture_output=True, text=True,
                           cwd=ROOT, timeout=300)
        self.assertEqual(r.returncode, 0, r.stdout[-800:] + r.stderr)
        self.assertIn("cases pass", r.stdout)


if __name__ == "__main__":
    unittest.main()
