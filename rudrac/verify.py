"""rudrac verify: random testing of a node script against its 'never' rules.

The generated code already refuses any write that would break a rule, so the hardware is protected either way.
What this finds is script paths that *try*: those are bugs, because the master asked for something that silently
did not happen. It is random testing, not a proof: it can show a problem, never that there is none.
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import List

from .check import Checked
from .sim import Sim
from .syntax import IO, fmt_expr

ARG_POOL = [0, 1, 2, 5, 10, 45, 50, 90, 100, 180, 255, 1000, -1]


@dataclass
class Finding:
    rule: int
    rule_text: str
    output: str
    value: object
    runs: set = field(default_factory=set)
    first_run: int = 0
    first_t: int = 0
    trace: List[str] = field(default_factory=list)


@dataclass
class Report:
    node: str
    runs: int
    seconds: int
    seed: int
    rules: int
    commands: int
    inputs: int
    findings: List[Finding]
    internal_errors: List[str]
    blocked_total: int


def _gen_events(ck: Checked, rng: random.Random, end_ms: int):
    ios = [d for d in ck.node.decls if isinstance(d, IO) and d.direction == "input"]
    cmds = list(ck.commands)
    pre = f"@{ck.node.address} " if ck.bus else ""     # on a shared bus every line is addressed
    evs, t = [], rng.randint(1, 300)
    while t < end_ms:
        r = rng.random()
        if r < 0.05:
            t += rng.randint(1500, 6000)           # master goes quiet: lets the heartbeat trip and recover
            continue
        if r < 0.10:
            evs.append((t, "rx", pre + rng.choice(["PING", "WHO", "STATUS"]), None, "master keep-alive"))
        elif cmds and (r < 0.62 or not ios):
            name, params = rng.choice(cmds)
            args = [rng.choice(ARG_POOL) if rng.random() < 0.8 else rng.randint(-5, 300) for _ in params]
            line = pre + " ".join([name] + [str(a) for a in args])
            evs.append((t, "rx", line, None, f"master sends  {line}"))
        elif ios:
            d = rng.choice(ios)
            if d.kind == "digital":
                v = rng.random() < 0.5
            elif d.kind == "ultrasonic":
                v = rng.choice([0, 3, 15, 40, 100, 400])
            elif d.kind == "analog":
                v = rng.choice([0, 100, 300, 500, 700, 900, 1023]) if rng.random() < 0.7 else rng.randint(0, 1023)
            else:
                v = rng.randint(0, (1 << (8 * d.cfg["bytes"])) - 1)
            evs.append((t, "set", d.name, v, f"input {d.name} becomes {int(v) if isinstance(v, bool) else v}"))
        t += rng.randint(1, 800)
    return evs


def verify(ck: Checked, runs: int = 100, seconds: int = 10, seed: int = 1) -> Report:
    found = {}
    internal = []
    blocked_total = 0
    end_ms = seconds * 1000
    n_inputs = sum(1 for d in ck.node.decls if isinstance(d, IO) and d.direction == "input")
    for run in range(runs):
        rng = random.Random(seed * 1_000_003 + run)
        evs = _gen_events(ck, rng, end_ms)
        sim = Sim(ck)
        k, seen, history = 0, 0, []
        for t in range(end_ms + 1):
            lines = []
            while k < len(evs) and evs[k][0] <= t:
                _, kind, a, b, text = evs[k]
                history.append(f"t={evs[k][0]}ms  {text}")
                if kind == "set":
                    sim.set_input(a, b)
                else:
                    lines.append(a)
                k += 1
            sim.tick(t, lines)
            if len(sim.blocked_log) > seen:
                for bt, out, val, rule in sim.blocked_log[seen:]:
                    key = (rule, out)
                    f = found.get(key)
                    if f is None:
                        f = found[key] = Finding(rule, fmt_expr(ck.rules[rule - 1].expr), out,
                                                 int(val) if isinstance(val, bool) else val,
                                                 first_run=run, first_t=bt, trace=history[-4:])
                    f.runs.add(run)
                seen = len(sim.blocked_log)
            if sim.rules and sim.violated() is not None:   # the runtime must make this impossible
                internal.append(f"run {run}, t={t}ms: a rule is true although every write is checked")
        blocked_total += sim.blocked
        if len(internal) > 5:
            break
    return Report(ck.node.name, runs, seconds, seed, len(ck.rules), len(ck.commands), n_inputs,
                  sorted(found.values(), key=lambda f: (f.rule, f.output)), internal, blocked_total)


def format_report(r: Report, filename: str) -> str:
    L = [f"rudrac verify {filename}: {r.runs} random runs of {r.seconds}s each (seed {r.seed}), "
         f"{r.commands} command{'s' if r.commands != 1 else ''}, "
         f"{r.inputs} input{'s' if r.inputs != 1 else ''}, {r.rules} 'never' rule{'s' if r.rules != 1 else ''}"]
    if r.rules == 0:
        L.append("  nothing to verify: this node has no 'never' rules. Add some, e.g.  never heater and not fan")
        return "\n".join(L)
    for e in r.internal_errors:
        L.append(f"  INTERNAL ERROR: {e}  (this is a compiler bug, please report it)")
    for f in r.findings:
        L.append(f"  FOUND: the script tries to break rule {f.rule}:  never {f.rule_text}")
        L.append(f"     the write '{f.output} = {f.value}' was refused in {len(f.runs)} of {r.runs} runs "
                 f"(first: run {f.first_run}, t={f.first_t}ms)")
        L.append("     what led up to it:")
        L += [f"        {x}" for x in f.trace]
        L.append("     The interlock held, so the hardware was never in the forbidden state, but the script "
                 "asked for it and nothing happened. Change the script so it first moves the other output.")
    if not r.findings and not r.internal_errors:
        L.append(f"  OK: no script path tried to break a rule in {r.runs} runs.")
        L.append("  This is random testing, not a proof: it can show a problem but never that there is none.")
    return "\n".join(L)
