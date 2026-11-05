"""The typed intermediate representation (IR) of rudrac.

The checker proves a program is well formed; `Lowerer` then turns each body into this small,
fully resolved form, ONCE. The simulator, the C++ generator and the worst-case prover all read
the same IR, so they can no longer disagree about what a name or an index means:

  * every name is already resolved (local / global slot / input / output / constant folded away)
  * every expression carries its type ("int" or "bool"; enums are ints)
  * every array access says whether the compiler PROVED the index in range
  * structs are already flattened to slots, enum constants are already numbers

`validate()` re-checks those invariants, and `dump()` prints the IR (`rudrac ir file.rudra`).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from . import syntax as S


# ---------------------------------------------------------------- expressions
@dataclass
class K:                    # constant
    v: Any
    ty: str


@dataclass
class Loc:                  # parameter, let or loop counter
    name: str
    ty: str


@dataclass
class Glob:                 # scalar global var slot (struct fields are slots like "p.x")
    slot: str
    ty: str


@dataclass
class In:                   # input read
    name: str
    ty: str


@dataclass
class Out:                  # output read-back
    name: str
    ty: str


@dataclass
class Neg:
    e: Any
    ty: str = "int"


@dataclass
class Not:
    e: Any
    ty: str = "bool"


@dataclass
class Op:                   # binary operator
    op: str
    l: Any
    r: Any
    ty: str


@dataclass
class CallE:
    fn: str
    args: List[Any]
    ty: Optional[str]       # None for a proc


@dataclass
class Elem:                 # array element (also a field of an array of structs)
    slot: str
    idx: Any
    size: int
    proven: bool
    ty: str


# ---------------------------------------------------------------- statements
@dataclass
class LetS:
    name: str
    e: Any
    ty: str


@dataclass
class SetL:
    name: str
    e: Any


@dataclass
class SetG:
    slot: str
    e: Any


@dataclass
class SetE:
    slot: str
    idx: Any
    size: int
    proven: bool
    e: Any
    ty: str


@dataclass
class SetO:
    name: str
    e: Any


@dataclass
class Ret:
    e: Any


@dataclass
class Brk:
    pass


@dataclass
class CallS:
    call: CallE


@dataclass
class Loop:
    var: str
    lo: int
    hi: int
    body: List[Any]


@dataclass
class IfS:
    cond: Any
    then: List[Any]
    els: List[Any]


@dataclass
class FillS:
    strip: str
    args: List[Any]


@dataclass
class PixelS:
    strip: str
    idx: Any
    args: List[Any]


@dataclass
class AnimS:
    strip: str
    mode: str
    args: List[Any]


@dataclass
class GotoS:
    state: str
    index: int


@dataclass
class SendS:
    msg: str
    e: Any                  # None for a bare message
    is_bool: bool = False


EXPR = (K, Loc, Glob, In, Out, Neg, Not, Op, CallE, Elem)
CMP = ("<", "<=", ">", ">=", "==", "!=", "&&", "||")


def ty_of(t) -> str:
    return "bool" if t == "bool" else "int"


# ---------------------------------------------------------------- lowering
class Lowerer:
    def __init__(self, ck):
        self.ck = ck
        self.syms = ck.syms
        self.fns = {f.name: f for f in ck.fns}
        self._bodies: Dict[tuple, list] = {}
        self._fnir: Dict[str, list] = {}

    # -- public
    def fn_body(self, name: str) -> list:
        if name not in self._fnir:
            f = self.fns[name]
            scope = {p[0]: ty_of(t) for p, t in zip(f.params, f.ptypes or [p[1] for p in f.params])}
            self._fnir[name] = self.block(f.body, scope)
        return self._fnir[name]

    def body(self, ast_body, params=()) -> list:
        key = (id(ast_body), tuple(params))
        if key not in self._bodies:
            self._bodies[key] = self.block(ast_body, {p: "int" for p in params})
        return self._bodies[key]

    def expr(self, e, scope=None):
        return self.e(e, scope or {})

    # -- expressions
    def e(self, e, sc):
        if isinstance(e, S.Num):
            return K(e.value, "int")
        if isinstance(e, S.BoolLit):
            return K(e.value, "bool")
        if isinstance(e, S.Name):
            if e.name in sc:
                return Loc(e.name, sc[e.name])
            s = self.syms[e.name]
            ty = ty_of(s.type)
            if s.kind == "const":
                return K(s.value, ty)
            if s.kind == "var":
                return Glob(e.name, ty)
            if s.kind == "input":
                return In(e.name, ty)
            return Out(e.name, ty)
        if isinstance(e, S.Unary):
            x = self.e(e.e, sc)
            return Neg(x) if e.op == "-" else Not(x)
        if isinstance(e, S.Call):
            return CallE(e.name, [self.e(a, sc) for a in e.args], None if e.type is None else ty_of(e.type))
        if isinstance(e, S.Bin):
            return Op(e.op, self.e(e.l, sc), self.e(e.r, sc), "bool" if e.op in CMP else "int")
        if isinstance(e, S.Member) and e.enum_val is not None:
            return K(e.enum_val, "int")
        if isinstance(e, (S.Index, S.Member)):
            ty = ty_of(e.ty)
            if e.idx is None:
                return Glob(e.slot, ty)
            return Elem(e.slot, self.e(e.idx, sc), e.size, bool(e.proven), ty)
        raise AssertionError(e)

    # -- statements
    def block(self, body, sc) -> list:
        sc = dict(sc)
        out = []
        for s in body:
            if isinstance(s, S.Assign):
                v = self.e(s.expr, sc)
                if s.name in sc:
                    out.append(SetL(s.name, v))
                elif self.syms[s.name].kind == "var":
                    out.append(SetG(s.name, v))
                else:
                    out.append(SetO(s.name, v))
            elif isinstance(s, S.AssignTo):
                t = s.target
                v = self.e(s.expr, sc)
                if t.idx is None:
                    out.append(SetG(t.slot, v))
                else:
                    out.append(SetE(t.slot, self.e(t.idx, sc), t.size, bool(t.proven), v, ty_of(t.ty)))
            elif isinstance(s, S.Let):
                ty = ty_of(s.type)
                out.append(LetS(s.name, self.e(s.expr, sc), ty))
                sc[s.name] = ty
            elif isinstance(s, S.Return):
                out.append(Ret(None if s.expr is None else self.e(s.expr, sc)))
            elif isinstance(s, S.Break):
                out.append(Brk())
            elif isinstance(s, S.CallStmt):
                out.append(CallS(self.e(s.call, sc)))
            elif isinstance(s, S.For):
                inner = dict(sc)
                inner[s.var] = "int"
                out.append(Loop(s.var, s.lo_v, s.hi_v, self.block(s.body, inner)))
            elif isinstance(s, S.If):
                out.append(IfS(self.e(s.cond, sc), self.block(s.then, sc), self.block(s.els, sc) if s.els else []))
            elif isinstance(s, S.Fill):
                out.append(FillS(s.strip, [self.e(a, sc) for a in s.args]))
            elif isinstance(s, S.Pixel):
                out.append(PixelS(s.strip, self.e(s.idx, sc), [self.e(a, sc) for a in s.args]))
            elif isinstance(s, S.Animate):
                out.append(AnimS(s.strip, s.mode, [self.e(a, sc) for a in s.args]))
            elif isinstance(s, S.Goto):
                out.append(GotoS(s.state, self.ck.state_index[s.state]))
            elif isinstance(s, S.Send):
                if s.expr is None:
                    out.append(SendS(s.msg, None))
                else:
                    x = self.e(s.expr, sc)
                    out.append(SendS(s.msg, x, x.ty == "bool"))
            else:
                raise AssertionError(s)
        return out


# ---------------------------------------------------------------- walking
def sub_exprs(x):
    """Direct sub-expressions of an expression."""
    if isinstance(x, (Neg, Not)):
        return [x.e]
    if isinstance(x, Op):
        return [x.l, x.r]
    if isinstance(x, CallE):
        return list(x.args)
    if isinstance(x, Elem):
        return [x.idx]
    return []


def walk_expr(x):
    yield x
    for c in sub_exprs(x):
        yield from walk_expr(c)


def stmt_exprs(s):
    """The expressions a statement evaluates itself (not those of nested blocks)."""
    if isinstance(s, (LetS, SetL, SetG, SetO)):
        return [s.e]
    if isinstance(s, SetE):
        return [s.idx, s.e]
    if isinstance(s, Ret):
        return [] if s.e is None else [s.e]
    if isinstance(s, CallS):
        return [s.call]
    if isinstance(s, IfS):
        return [s.cond]
    if isinstance(s, FillS):
        return list(s.args)
    if isinstance(s, PixelS):
        return [s.idx, *s.args]
    if isinstance(s, AnimS):
        return list(s.args)
    if isinstance(s, SendS):
        return [] if s.e is None else [s.e]
    return []


def walk_stmts(body):
    for s in body:
        yield s
        if isinstance(s, IfS):
            yield from walk_stmts(s.then)
            yield from walk_stmts(s.els)
        elif isinstance(s, Loop):
            yield from walk_stmts(s.body)


def callees(body):
    """Names of every function called anywhere in a block."""
    for s in walk_stmts(body):
        for x in stmt_exprs(s):
            for y in walk_expr(x):
                if isinstance(y, CallE):
                    yield y.fn


def count_locals(body) -> int:
    return sum(1 for s in walk_stmts(body) if isinstance(s, (LetS, Loop)))


# ---------------------------------------------------------------- checking and printing
class IRError(Exception):
    pass


def validate(ck, low: Optional[Lowerer] = None) -> int:
    """Re-checks the invariants every consumer relies on. Returns how many nodes were examined."""
    low = low or Lowerer(ck)
    n = 0
    bodies = [low.body(e.body, e.params) for e in ck.node.events] + [low.fn_body(f.name) for f in ck.fns]
    for b in bodies:
        for s in walk_stmts(b):
            for x in stmt_exprs(s):
                for y in walk_expr(x):
                    n += 1
                    if isinstance(y, (K, Loc, Glob, In, Out, Op, Elem)) and y.ty not in ("int", "bool"):
                        raise IRError(f"untyped node {y}")
                    if isinstance(y, Elem) and y.proven and isinstance(y.idx, K) and not 0 <= y.idx.v < y.size:
                        raise IRError(f"index {y.idx.v} marked proven but outside 0..{y.size - 1}")
                    if isinstance(y, Op) and y.op in ("+", "-", "*", "/", "%") and (y.l.ty != "int" or y.r.ty != "int"):
                        raise IRError(f"arithmetic on a non-number: {y}")
                    if isinstance(y, Op) and y.op in ("&&", "||") and (y.l.ty != "bool" or y.r.ty != "bool"):
                        raise IRError(f"logic on a non-bool: {y}")
            if isinstance(s, SetE) and s.proven and isinstance(s.idx, K) and not 0 <= s.idx.v < s.size:
                raise IRError(f"index {s.idx.v} marked proven but outside 0..{s.size - 1}")
            if isinstance(s, Loop) and not (isinstance(s.lo, int) and isinstance(s.hi, int)):
                raise IRError("loop with a non-constant bound")
    return n


def fmt(x) -> str:
    if isinstance(x, K):
        return "true" if x.v is True else "false" if x.v is False else str(x.v)
    if isinstance(x, Loc):
        return x.name
    if isinstance(x, Glob):
        return f"@{x.slot}"
    if isinstance(x, In):
        return f"in:{x.name}"
    if isinstance(x, Out):
        return f"out:{x.name}"
    if isinstance(x, Neg):
        return f"(-{fmt(x.e)})"
    if isinstance(x, Not):
        return f"(!{fmt(x.e)})"
    if isinstance(x, Op):
        return f"({fmt(x.l)} {x.op} {fmt(x.r)})"
    if isinstance(x, CallE):
        return f"{x.fn}({', '.join(fmt(a) for a in x.args)})"
    if isinstance(x, Elem):
        return f"@{x.slot}[{fmt(x.idx)}]{'' if x.proven else '?'}"
    return repr(x)


def dump_body(body, ind=1) -> List[str]:
    pad = "  " * ind
    out = []
    for s in body:
        if isinstance(s, LetS):
            out.append(f"{pad}let {s.name}:{s.ty} = {fmt(s.e)}")
        elif isinstance(s, SetL):
            out.append(f"{pad}{s.name} = {fmt(s.e)}")
        elif isinstance(s, SetG):
            out.append(f"{pad}@{s.slot} = {fmt(s.e)}")
        elif isinstance(s, SetE):
            out.append(f"{pad}@{s.slot}[{fmt(s.idx)}] = {fmt(s.e)}   {'(index proven)' if s.proven else '(index checked at run time)'}")
        elif isinstance(s, SetO):
            out.append(f"{pad}out:{s.name} = {fmt(s.e)}")
        elif isinstance(s, Ret):
            out.append(f"{pad}return" + ("" if s.e is None else f" {fmt(s.e)}"))
        elif isinstance(s, Brk):
            out.append(f"{pad}break")
        elif isinstance(s, CallS):
            out.append(f"{pad}call {fmt(s.call)}")
        elif isinstance(s, Loop):
            out.append(f"{pad}for {s.var} in {s.lo}..{s.hi}")
            out += dump_body(s.body, ind + 1)
        elif isinstance(s, IfS):
            out.append(f"{pad}if {fmt(s.cond)}")
            out += dump_body(s.then, ind + 1)
            if s.els:
                out.append(f"{pad}else")
                out += dump_body(s.els, ind + 1)
        elif isinstance(s, GotoS):
            out.append(f"{pad}goto {s.state} (#{s.index})")
        elif isinstance(s, SendS):
            out.append(f"{pad}send {s.msg!r}" + ("" if s.e is None else f" {fmt(s.e)}{' (bool)' if s.is_bool else ''}"))
        else:
            args = ", ".join(fmt(a) for a in getattr(s, "args", []))
            out.append(f"{pad}{type(s).__name__} {getattr(s, 'strip', '')} {args}")
    return out


def dump(ck) -> str:
    low = Lowerer(ck)
    L = [f"IR of node {ck.node.name}"]
    for f in ck.fns:
        L.append(f"\n{f.kind} {f.name}({', '.join(p[0] for p in f.params)}) -> {f.ret or '-'}")
        L += dump_body(low.fn_body(f.name))
    for e in ck.node.events:
        head = e.kind + (f" {e.name}" if e.name else "") + (f" {e.state}" if getattr(e, "state", None) else "")
        L.append(f"\n{head}({', '.join(e.params)})")
        L += dump_body(low.body(e.body, e.params))
    L.append(f"\n{validate(ck, low)} expression nodes checked")
    return "\n".join(L)
