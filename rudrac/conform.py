"""Conformance suite runner (stage B4).

A case is a directory:

    prog.rudra       the program (plus any files it imports)
    scenario.json    {"run_ms": 5000, "events": [[ms, "rx", "PING"], [ms, "set", "temp", 700], [ms, "fail", "temp", true]]}
    expect.json      {"phys": ["t pin value", ...], "tx": ["t MESSAGE", ...]}      a run that must produce exactly this
               or    {"errors": [{"line": 3, "level": "error", "contains": "unknown name"}, ...]}   a program that must be rejected
                     (the diagnostics must match one to one; "level" defaults to "error")

`phys` lists every hardware write in order as "ms pin value" (pin = the Arduino pin; analog/PWM values 0..255 or
0..4095 as the board does it; servo, stepper and neopixel lines are in SPEC.md section 9). `tx` lists every line the
node prints, with the millisecond it was printed.

Any implementation that reads prog.rudra and produces the same phys/tx for the scenario conforms. This runner checks
the reference implementation three ways: the simulator, the simulator driven by the bytecode VM, and (when g++ is
available) the generated C++ built against a fake Arduino core, native and --vm.
"""
import json
import os
import subprocess
import sys
import tempfile
from typing import List, Optional, Tuple

from .check import check
from .gen import generate
from .sim import Sim
from .syntax import parse
from .util import LINE_STRIDE, RudraError

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, "tests")


def _events(sc: dict) -> list:
    out = []
    for e in sc.get("events", []):
        ms, kind = e[0], e[1]
        if kind == "rx":
            out.append((ms, "rx", e[2], None))
        else:                                   # set / fail: name, value
            out.append((ms, kind, e[2], e[3]))
    return out


def load_case(d: str):
    with open(os.path.join(d, "scenario.json"), encoding="utf-8") as f:
        sc = json.load(f)
    with open(os.path.join(d, "expect.json"), encoding="utf-8") as f:
        ex = json.load(f)
    return os.path.join(d, "prog.rudra"), sc, ex


def compile_prog(path: str):
    with open(path, encoding="utf-8") as f:
        src = f.read()
    sources: dict = {}
    try:
        ck = check(parse(src, path, sources))
        return ck, ck.diags
    except RudraError as e:
        return None, e.diags


def diag_list(diags) -> List[Tuple[str, int, str]]:
    return [(d.level, d.line if d.line < LINE_STRIDE else 0, d.msg) for d in diags]


def match_errors(diags, expected: list) -> Optional[str]:
    """None when the diagnostics match the expectation one to one, else a description of the difference."""
    have = diag_list(diags)
    left = list(have)
    for x in expected:
        lvl = x.get("level", "error")
        hit = next((h for h in left if h[0] == lvl and h[1] == x["line"] and x["contains"] in h[2]), None)
        if hit is None:
            return f"expected {lvl} at line {x['line']} containing {x['contains']!r}; got {have}"
        left.remove(hit)
    if left:
        return f"unexpected diagnostics: {left}"
    return None


def sim_run(ck, sc, vm: bool):
    sim = Sim(ck, engine="vm" if vm else "native")
    sim.run(sc["run_ms"], _events(sc))
    return [" ".join(str(x) for x in p) for p in sim.phys], [f"{t} {m}" for t, m in sim.tx_log]


def have_gpp() -> bool:
    try:
        subprocess.run(["g++", "--version"], capture_output=True, check=True)
        return os.path.exists(os.path.join(TESTS, "harness.cpp"))
    except (OSError, subprocess.CalledProcessError):
        return False


def cpp_run(ck, sc, vm: bool, tmp: str, tag: str):
    ino = os.path.join(tmp, tag + ".ino")
    with open(ino, "w") as f:
        f.write(generate(ck, tag, vm=vm))
    exe = os.path.join(tmp, tag + ".exe")
    r = subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", os.path.join(TESTS, "stub"),
                        f'-DSKETCH="{ino}"', os.path.join(TESTS, "harness.cpp"), "-o", exe],
                       capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError("generated C++ does not compile:\n" + r.stderr[:2000])
    ios = {d.name: d for d in ck.node.decls if hasattr(d, "direction")}
    lines = [f"RUN {sc['run_ms']}"]
    for d in ios.values():
        if d.direction == "input" and d.kind == "digital":
            lines.append(f"0 PIN {d.pin.key} {int(d.invert)}")
    for ms, kind, a, b in _events(sc):
        if kind == "rx":
            lines.append(f"{ms} RX {a}")
        elif kind == "fail":
            lines.append(f"{ms} I2CFAIL {ios[a].cfg['addr']} {int(bool(b))}")
        else:
            d = ios[a]
            if d.kind == "digital":
                lines.append(f"{ms} PIN {d.pin.key} {int(bool(b) != d.invert)}")
            elif d.kind == "ultrasonic":
                lines.append(f"{ms} ECHO {d.cfg['pins']['echo'].key} {int(b) * 58}")
            elif d.kind == "i2c":
                c = d.cfg
                hexv = "".join(f"{((int(b) & 0xFFFFFFFF) >> (8 * k)) & 0xFF:02x}" for k in reversed(range(c['bytes'])))
                lines.append(f"{ms} I2C {c['addr']} {c['reg'] if c['reg'] is not None else -1} {hexv}")
            else:
                lines.append(f"{ms} ANALOG {d.pin.key} {int(b)}")
    out = subprocess.run([exe], input="\n".join(lines) + "\n", capture_output=True, text=True).stdout
    head, _, tx = out.partition("---TX---\n")
    return head.splitlines(), tx.splitlines()


def run_case(d: str, modes=("sim", "vm", "cpp", "cpp-vm")) -> List[Tuple[str, bool, str]]:
    """Returns [(mode, passed, detail)]."""
    path, sc, ex = load_case(d)
    ck, diags = compile_prog(path)
    res: List[Tuple[str, bool, str]] = []
    if "errors" in ex:
        bad = match_errors(diags, ex["errors"])
        if bad is None and ck is not None and ck.ok and any(x.get("level", "error") == "error" for x in ex["errors"]):
            bad = "program was accepted but must be rejected"
        res.append(("check", bad is None, bad or ""))
        return res
    if ck is None or not ck.ok:
        res.append(("check", False, "program must be accepted: " + "; ".join(m for _, _, m in diag_list(diags))))
        return res
    exp_phys, exp_tx = ex["phys"], ex["tx"]
    cpp_ok = have_gpp()
    with tempfile.TemporaryDirectory(prefix="rudra_conf_") as tmp:
        for mode in modes:
            try:
                if mode in ("sim", "vm"):
                    phys, tx = sim_run(ck, sc, mode == "vm")
                elif not cpp_ok:
                    continue
                else:
                    phys, tx = cpp_run(ck, sc, mode == "cpp-vm", tmp, os.path.basename(d) + mode.replace("-", ""))
            except Exception as e:      # a crash is a failure, not a skipped case
                res.append((mode, False, f"{type(e).__name__}: {e}"))
                continue
            if phys == exp_phys and tx == exp_tx:
                res.append((mode, True, ""))
            else:
                res.append((mode, False, _diff(exp_phys, phys, "phys") or _diff(exp_tx, tx, "tx")))
    return res


def _diff(want, got, what) -> str:
    if want == got:
        return ""
    for i, (a, b) in enumerate(zip(want, got)):
        if a != b:
            return f"{what} line {i}: expected {a!r}, got {b!r}"
    return f"{what}: expected {len(want)} lines, got {len(got)}"


def regen(d: str):
    """Write expect.json from the reference simulator. Review the result by hand before committing it."""
    path, sc, ex = load_case(d)
    if "errors" in ex:
        return
    ck, diags = compile_prog(path)
    if ck is None or not ck.ok:
        raise SystemExit(f"{d}: does not compile: {diag_list(diags)}")
    phys, tx = sim_run(ck, sc, False)
    with open(os.path.join(d, "expect.json"), "w", encoding="utf-8") as f:
        json.dump({"phys": phys, "tx": tx}, f, indent=1)
        f.write("\n")


def case_dirs(root: str) -> List[str]:
    return sorted(os.path.join(root, n) for n in os.listdir(root)
                  if os.path.exists(os.path.join(root, n, "prog.rudra")))


def main(root: str, do_regen=False, modes=("sim", "vm", "cpp", "cpp-vm")) -> int:
    cases = case_dirs(root)
    if not cases:
        print(f"no cases under {root}", file=sys.stderr)
        return 1
    failed = 0
    for d in cases:
        name = os.path.basename(d)
        if do_regen:
            regen(d)
            print(f"{name}: written")
            continue
        res = run_case(d, modes)
        ok = all(p for _, p, _ in res)
        print(f"{'ok  ' if ok else 'FAIL'} {name}  [{', '.join(m for m, _, _ in res)}]")
        for m, p, detail in res:
            if not p:
                print(f"       {m}: {detail}")
        failed += 0 if ok else 1
    print(f"\n{len(cases) - failed}/{len(cases)} cases pass")
    return 1 if failed else 0
