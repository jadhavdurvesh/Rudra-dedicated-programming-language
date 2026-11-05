"""Command line: rudrac check | build | sim | flash | targets"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys

from .check import check
from .gen import generate
from .bus import check_bus, gen_bus_doc, gen_bus_python
from .link import gen_cpp, gen_protocol, gen_python, info as node_info
from .sim import Sim, parse_time
from .syntax import parse
from .verify import format_report, verify as run_verify
from .util import TARGETS, VERSION, RudraError, format_diag


def load(path):
    """Parse + check a .rudra file. Prints diagnostics. Returns Checked or None."""
    try:
        with open(path, encoding="utf-8") as f:
            src = f.read()
    except OSError as e:
        print(f"rudrac: cannot read {path}: {e}", file=sys.stderr)
        return None
    try:
        ck = check(parse(src))
    except RudraError as e:
        for d in e.diags:
            print(format_diag(d, src, path), file=sys.stderr)
        return None
    for d in ck.diags:
        print(format_diag(d, src, path), file=sys.stderr)
    if not ck.ok:
        n = len(ck.errors)
        print(f"\n{path}: {n} error{'s' if n != 1 else ''}", file=sys.stderr)
        return None
    return ck


def write_sketch(ck, path, outdir):
    d = os.path.join(outdir, ck.node.name)
    os.makedirs(d, exist_ok=True)
    out = os.path.join(d, ck.node.name + ".ino")
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(generate(ck, os.path.basename(path)))
    return d, out


def cmd_check(a):
    ck = load(a.file)
    if ck is None:
        return 1
    print(f"{a.file}: ok ({len(ck.warnings)} warning{'s' if len(ck.warnings) != 1 else ''})")
    return 0


def cmd_build(a):
    ck = load(a.file)
    if ck is None:
        return 1
    if a.stdout:
        print(generate(ck, os.path.basename(a.file)))
        return 0
    _, out = write_sketch(ck, a.file, a.out)
    print(f"wrote {out}")
    return 0


def _fqbn(ck, old_bootloader):
    f = TARGETS[ck.node.target]["fqbn"]
    return f + ":cpu=atmega328old" if old_bootloader and ck.node.target == "nano" else f


def cmd_link(a):
    ck = load(a.file)
    if ck is None:
        return 1
    os.makedirs(a.out, exist_ok=True)
    name = ck.node.name
    files = {f"{name}_link.h": gen_cpp(ck), f"{name}_link.py": gen_python(ck), f"{name.upper()}_PROTOCOL.md": gen_protocol(ck)}
    for fn, text in files.items():
        path = os.path.join(a.out, fn)
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        print(f"wrote {path}")
    return 0


def cmd_bus(a):
    cks = [load(f) for f in a.files]
    if any(c is None for c in cks):
        return 1
    errs = check_bus(cks)
    for e in errs:
        print(f"rudrac bus: {e}", file=sys.stderr)
    if errs:
        return 1
    os.makedirs(a.out, exist_ok=True)
    for fn, text in {"bus_master.py": gen_bus_python(cks), "BUS.md": gen_bus_doc(cks)}.items():
        path = os.path.join(a.out, fn)
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        print(f"wrote {path}")
    return 0


def cmd_ir(a):
    from . import ir
    ck = load(a.file)
    if ck is None:
        return 1
    print(ir.dump(ck))
    return 0


def cmd_cost(a):
    from .cost import analyze, format_report as fmt_cost
    ck = load(a.file)
    if ck is None:
        return 1
    rep = analyze(ck, a.model)
    print(fmt_cost(rep, a.file, ck.node.budget_ms), end="")
    if ck.node.budget_ms is not None and rep.pass_worst > ck.node.budget_ms * 1000:
        return 1
    return 0


def cmd_verify(a):
    ck = load(a.file)
    if ck is None:
        return 1
    rep = run_verify(ck, runs=a.runs, seconds=a.seconds, seed=a.seed)
    print(format_report(rep, a.file))
    return 1 if (rep.findings or rep.internal_errors) else 0


def cmd_info(a):
    ck = load(a.file)
    if ck is None:
        return 1
    print(node_info(ck), end="")
    return 0


def cmd_flash(a):
    ck = load(a.file)
    if ck is None:
        return 1
    d, out = write_sketch(ck, a.file, a.out)
    cli = shutil.which("arduino-cli")
    fqbn = _fqbn(ck, a.old_bootloader)
    if cli is None:
        print(f"wrote {out}\narduino-cli not found. Install it, then run:\n"
              f"  arduino-cli compile --fqbn {fqbn} {d}\n"
              f"  arduino-cli upload -p {a.port} --fqbn {fqbn} {d}")
        return 1
    if subprocess.call([cli, "compile", "--fqbn", fqbn, d]) != 0:
        return 1
    return subprocess.call([cli, "upload", "-p", a.port, "--fqbn", fqbn, d])


def cmd_sim(a):
    ck = load(a.file)
    if ck is None:
        return 1
    try:
        end = parse_time(a.run)
        events = []
        for item in a.set or []:
            t, rest = item.split(":", 1)
            name, val = rest.split("=", 1)
            v = {"true": True, "false": False}.get(val.lower())
            events.append((parse_time(t), "set", name, int(val) if v is None else v))
        for item in a.fail or []:
            t, name = item.split(":", 1)
            events.append((parse_time(t), "fail", name, True))
        for item in a.heal or []:
            t, name = item.split(":", 1)
            events.append((parse_time(t), "fail", name, False))
        for item in a.send or []:
            t, line = item.split(":", 1)
            events.append((parse_time(t), "rx", line, None))
        sim = Sim(ck)
        sim.run(end, events)
    except (ValueError, KeyError) as e:
        print(f"rudrac sim: {e}", file=sys.stderr)
        return 1
    print(f"rudrac sim   node={ck.node.name}  target={ck.node.target}  run={end}ms")
    for t, text in sim.log:
        print(f"  t={t:>7}ms  {text}")
    print(f"final: {sim.summary()}")
    return 0


def cmd_targets(_a):
    for k, v in TARGETS.items():
        print(f"{k:9} {v['fqbn']}")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(prog="rudrac", description=f"Rudra compiler {VERSION}")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("check", help="check a .rudra file")
    s.add_argument("file")
    s.set_defaults(fn=cmd_check)

    s = sub.add_parser("build", help="generate the Arduino sketch")
    s.add_argument("file")
    s.add_argument("-o", "--out", default="build")
    s.add_argument("--stdout", action="store_true", help="print the C++ instead of writing a file")
    s.set_defaults(fn=cmd_build)

    s = sub.add_parser("link", help="generate master-side C++ header, Python client and protocol doc")
    s.add_argument("file")
    s.add_argument("-o", "--out", default="build")
    s.set_defaults(fn=cmd_link)

    s = sub.add_parser("bus", help="generate one master (Python + doc) for several nodes on a shared bus")
    s.add_argument("files", nargs="+")
    s.add_argument("-o", "--out", default="build")
    s.set_defaults(fn=cmd_bus)

    s = sub.add_parser("ir", help="print the typed intermediate representation that sim, C++ and cost all share")
    s.add_argument("file")
    s.set_defaults(fn=cmd_ir)

    s = sub.add_parser("cost", help="worst-case time per handler, main-loop pass, stack and RAM (model based)")
    s.add_argument("file")
    s.add_argument("--model", help="JSON file that overrides the cost model, e.g. {\"dwrite\": 4}")
    s.set_defaults(fn=cmd_cost)

    s = sub.add_parser("verify", help="random-test the script against its 'never' rules")
    s.add_argument("file")
    s.add_argument("--runs", type=int, default=100)
    s.add_argument("--seconds", type=int, default=10, help="virtual seconds per run")
    s.add_argument("--seed", type=int, default=1)
    s.set_defaults(fn=cmd_verify)

    s = sub.add_parser("info", help="wiring, commands, messages and RAM estimate")
    s.add_argument("file")
    s.set_defaults(fn=cmd_info)

    s = sub.add_parser("flash", help="build, compile and upload with arduino-cli")
    s.add_argument("file")
    s.add_argument("-p", "--port", required=True)
    s.add_argument("-o", "--out", default="build")
    s.add_argument("--old-bootloader", action="store_true", help="Nano clones with the old bootloader")
    s.set_defaults(fn=cmd_flash)

    s = sub.add_parser("sim", help="run the node on the PC with virtual time")
    s.add_argument("file")
    s.add_argument("--run", default="5s", help="how long to simulate, e.g. 10s")
    s.add_argument("--set", action="append", metavar="TIME:INPUT=VALUE", help="e.g. 3s:temp=700")
    s.add_argument("--send", action="append", metavar='TIME:LINE', help='e.g. "4s:FAN 50"')
    s.add_argument("--fail", action="append", metavar="TIME:INPUT", help="i2c input stops answering, e.g. 2s:temp")
    s.add_argument("--heal", action="append", metavar="TIME:INPUT", help="i2c input answers again")
    s.set_defaults(fn=cmd_sim)

    s = sub.add_parser("targets", help="list supported boards")
    s.set_defaults(fn=cmd_targets)

    a = p.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    raise SystemExit(main())
