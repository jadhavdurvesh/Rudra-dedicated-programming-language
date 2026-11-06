"""Lexer, AST and parser for the Rudra language."""
from __future__ import annotations

import os

from dataclasses import dataclass, field
from typing import Any, List, Optional

from .util import LINE_STRIDE, Diag, RudraError

KEYWORDS = {
    "node", "target", "const", "input", "output", "var", "every", "when", "on",
    "heartbeat", "start", "if", "else", "send", "serial", "true", "false",
    "pin", "pullup", "invert", "digital", "pwm", "analog", "never",
    "state", "enter", "exit", "goto", "for", "address", "rs485", "blackbox",
    "func", "proc", "let", "return", "break", "in", "budget", "enum", "struct", "import",
}
WORD_OPS = {"and": "&&", "or": "||", "not": "!"}


def _err(msg, line, col, hint=""):
    raise RudraError([Diag("error", msg, line, col, hint)])


# --------------------------------------------------------------------------
# Lexer
# --------------------------------------------------------------------------
@dataclass
class Tok:
    kind: str       # num | dur | pct | ident | kw | str | sym | eof
    text: str
    value: Any
    line: int
    col: int


def lex(src: str, line0: int = 0) -> List[Tok]:
    toks: List[Tok] = []
    i, n, line, col = 0, len(src), 1 + line0, 1
    two = {"==", "!=", "<=", ">=", "&&", "||", "..", "->"}
    one = set("{}(),:=<>+-*/%![].")
    while i < n:
        ch = src[i]
        if ch == "\n":
            i, line, col = i + 1, line + 1, 1
            continue
        if ch in " \t\r":
            i, col = i + 1, col + 1
            continue
        if ch == "#" or src.startswith("//", i):
            while i < n and src[i] != "\n":
                i += 1
            continue
        if ch.isdigit():
            j = i
            kind = "num"
            if src.startswith(("0x", "0X"), i):
                j = i + 2
                while j < n and src[j] in "0123456789abcdefABCDEF":
                    j += 1
                if j == i + 2:
                    _err("bad hex number", line, col)
                val = int(src[i:j], 16)
            else:
                while j < n and src[j].isdigit():
                    j += 1
                val = int(src[i:j])
                if src.startswith("ms", j):
                    kind, j = "dur", j + 2
                elif j < n and src[j] == "s":
                    kind, val, j = "dur", val * 1000, j + 1
                elif j < n and src[j] == "%":
                    kind, j = "pct", j + 1
            if j < n and (src[j].isalnum() or src[j] == "_"):
                _err("bad number", line, col, "durations are written like 500ms or 2s, percentages like 50%")
            if val > 2147483647:
                _err("number too large (limit is 2147483647)", line, col)
            toks.append(Tok(kind, src[i:j], val, line, col))
            col += j - i
            i = j
            continue
        if ch.isalpha() or ch == "_":
            j = i
            while j < n and (src[j].isalnum() or src[j] == "_"):
                j += 1
            word = src[i:j]
            if word in WORD_OPS:
                toks.append(Tok("sym", WORD_OPS[word], None, line, col))
            elif word in KEYWORDS:
                toks.append(Tok("kw", word, None, line, col))
            else:
                toks.append(Tok("ident", word, None, line, col))
            col += j - i
            i = j
            continue
        if ch == '"':
            j = i + 1
            while j < n and src[j] not in '"\n':
                j += 1
            if j >= n or src[j] != '"':
                _err("unterminated string", line, col)
            toks.append(Tok("str", src[i + 1:j], src[i + 1:j], line, col))
            col += j + 1 - i
            i = j + 1
            continue
        if src[i:i + 2] in two:
            toks.append(Tok("sym", src[i:i + 2], None, line, col))
            i, col = i + 2, col + 2
            continue
        if ch in one:
            toks.append(Tok("sym", ch, None, line, col))
            i, col = i + 1, col + 1
            continue
        _err(f"unexpected character '{ch}'", line, col)
    toks.append(Tok("eof", "", None, line, col))
    return toks


# --------------------------------------------------------------------------
# AST
# --------------------------------------------------------------------------
Pos = tuple  # (line, col)


@dataclass
class Num:
    value: int
    pos: Pos


@dataclass
class BoolLit:
    value: bool
    pos: Pos


@dataclass
class Name:
    name: str
    pos: Pos


@dataclass
class Unary:
    op: str
    e: Any
    pos: Pos


@dataclass
class Bin:
    op: str
    l: Any
    r: Any
    pos: Pos


@dataclass
class Assign:
    name: str
    expr: Any
    pos: Pos


@dataclass
class If:
    cond: Any
    then: list
    els: Optional[list]
    pos: Pos


@dataclass
class Send:
    msg: str
    expr: Optional[Any]
    pos: Pos


@dataclass
class Fill:
    strip: str
    args: list          # [r, g, b]
    pos: Pos


@dataclass
class Pixel:
    strip: str
    idx: Any
    args: list          # [r, g, b]
    pos: Pos


@dataclass
class Animate:
    strip: str
    mode: str           # off | blink | pulse | chase | rainbow
    args: list          # blink/pulse/chase: [r, g, b, period]; rainbow: [period]; off: []
    pos: Pos


@dataclass
class Never:
    expr: Any
    pos: Pos


@dataclass
class Call:
    name: str
    args: list
    pos: Pos
    type: Optional[str] = None      # result type, filled in by the checker (None for a proc)


@dataclass
class Index:
    """name[idx]: element of a fixed-size array. Filled in by the checker: slot, idx, size, proven, ty."""
    base: str
    idx: Any
    pos: Pos
    slot: Optional[str] = None
    size: Optional[int] = None
    proven: bool = False          # the compiler proved the index is in range: no run-time check
    ty: Optional[str] = None
    enum_val: Optional[int] = None


@dataclass
class Member:
    """obj.field: a struct field, or an enum constant (Mode.scan). obj is a Name or an Index."""
    obj: Any
    field: str
    pos: Pos
    slot: Optional[str] = None
    idx: Any = None               # index expression when obj is an Index
    size: Optional[int] = None
    proven: bool = False
    ty: Optional[str] = None
    enum_val: Optional[int] = None


@dataclass
class AssignTo:
    target: Any                   # Index or Member
    expr: Any
    pos: Pos


@dataclass
class EnumDecl:
    name: str
    members: list                 # [(name, pos)]
    pos: Pos


@dataclass
class StructDecl:
    name: str
    fields: list                  # [(name, type text, pos)]
    pos: Pos


@dataclass
class CallStmt:
    call: Call
    pos: Pos


@dataclass
class Let:
    name: str
    expr: Any
    pos: Pos
    type: Optional[str] = None


@dataclass
class Return:
    expr: Optional[Any]
    pos: Pos


@dataclass
class For:
    var: str
    lo: Any
    hi: Any
    body: list
    pos: Pos
    lo_v: Optional[int] = None
    hi_v: Optional[int] = None


@dataclass
class Break:
    pos: Pos


@dataclass
class Fn:
    kind: str                 # func (pure, returns a value) | proc (may change outputs, returns nothing)
    name: str
    params: list              # [(name, type text, pos)]
    ret: Optional[str]        # 'int' | 'bool' | None
    body: list
    pos: Pos
    ptypes: list = field(default_factory=list)   # resolved parameter types


@dataclass
class Goto:
    state: str
    pos: Pos


def walk_stmts(body):
    """Every statement in a block, including those nested in if/else."""
    for s in body:
        yield s
        if isinstance(s, If):
            yield from walk_stmts(s.then)
            if s.els:
                yield from walk_stmts(s.els)
        elif isinstance(s, For):
            yield from walk_stmts(s.body)


def walk_names(e):
    """Every Name node inside an expression."""
    if isinstance(e, Name):
        yield e
    elif isinstance(e, Unary):
        yield from walk_names(e.e)
    elif isinstance(e, Bin):
        yield from walk_names(e.l)
        yield from walk_names(e.r)
    elif isinstance(e, Call):
        for a in e.args:
            yield from walk_names(a)
    elif isinstance(e, Index):
        yield from walk_names(e.idx)
    elif isinstance(e, Member) and isinstance(e.obj, Index):
        yield from walk_names(e.obj.idx)


def has_place(e) -> bool:
    """True if the expression reads an array element or a struct field (enum constants do not count)."""
    if isinstance(e, Index):
        return True
    if isinstance(e, Member):
        return e.enum_val is None
    if isinstance(e, Unary):
        return has_place(e.e)
    if isinstance(e, Bin):
        return has_place(e.l) or has_place(e.r)
    if isinstance(e, Call):
        return any(has_place(a) for a in e.args)
    return False


def has_call(e) -> bool:
    if isinstance(e, Call):
        return True
    if isinstance(e, Unary):
        return has_call(e.e)
    if isinstance(e, Bin):
        return has_call(e.l) or has_call(e.r)
    return False


def fmt_expr(e, top=True) -> str:
    """Readable text of an expression, for messages and reports."""
    if isinstance(e, Num):
        return str(e.value)
    if isinstance(e, BoolLit):
        return "true" if e.value else "false"
    if isinstance(e, Name):
        return e.name
    if isinstance(e, Unary):
        return ("not " if e.op == "!" else "-") + fmt_expr(e.e, False)
    if isinstance(e, Bin):
        op = {"&&": "and", "||": "or"}.get(e.op, e.op)
        s = f"{fmt_expr(e.l, False)} {op} {fmt_expr(e.r, False)}"
        return s if top else f"({s})"
    if isinstance(e, Call):
        return f"{e.name}(" + ", ".join(fmt_expr(a) for a in e.args) + ")"
    if isinstance(e, Index):
        return f"{e.base}[{fmt_expr(e.idx)}]"
    if isinstance(e, Member):
        return f"{fmt_expr(e.obj)}.{e.field}"
    return "?"


@dataclass
class Const:
    name: str
    expr: Any
    pos: Pos
    value: Any = None
    type: Optional[str] = None


@dataclass
class Var:
    name: str
    expr: Any
    pos: Pos
    init: Any = None
    type: Optional[str] = None
    tname: Optional[str] = None       # 'var x: Mode', 'var buf: int[8]': declared type
    size_expr: Any = None             # array length expression
    size: Optional[int] = None
    shape: str = "scalar"             # scalar | array | struct | structarray
    slots: list = field(default_factory=list)   # [(storage key, type)]: one per scalar, one per struct field


@dataclass
class IO:
    direction: str      # input | output
    name: str
    kind: str           # digital | pwm | analog | servo | neopixel | i2c
    pin_text: Optional[str]
    pin_pos: Optional[Pos]
    pullup: bool
    invert: bool
    pos: Pos
    pin: Any = None     # util.Pin, filled in by the checker
    opts: dict = field(default_factory=dict)   # count/addr/reg/bytes/signed/rgb/safe/... -> (exprs, pos)
    cfg: dict = field(default_factory=dict)    # resolved values, filled in by the checker
    pins: dict = field(default_factory=dict)   # role (pwm/dir/fwd/rev/step) -> (pin text, pos)


@dataclass
class PathPt:
    value: Any
    speed: Optional[Any]
    zone: Optional[Any]
    wait: Optional[Any]     # expression: duration in ms
    pos: Pos
    v: int = 0              # the checker fills these with constants
    sp: Optional[int] = None
    zn: int = 0
    wt: int = 0


@dataclass
class PathDecl:
    name: str
    out: str
    points: list
    repeat: int
    forever: bool
    pos: Pos
    out_pos: Pos
    rep_pos: Optional[Pos] = None


@dataclass
class RunPath:
    name: str
    pos: Pos


@dataclass
class StopOut:
    out: str
    pos: Pos


@dataclass
class Event:
    kind: str           # every | when | on | heartbeat | start | enter | exit
    arg: Optional[Any]  # period / condition expression
    name: Optional[str]  # command name for 'on'
    params: list        # command argument names for 'on'
    body: list
    pos: Pos
    ms: Optional[int] = None
    off: bool = False   # 'heartbeat off'
    state: Optional[str] = None   # name of the state this handler belongs to (None = global)
    dur: Optional[Any] = None     # 'when cond for <dur>'
    dur_ms: Optional[int] = None


@dataclass
class Node:
    name: str
    target: str
    target_pos: Pos
    pos: Pos
    decls: list = field(default_factory=list)    # Const | IO | Var in source order
    events: list = field(default_factory=list)
    baud: int = 9600
    baud_pos: Optional[Pos] = None
    states: list = field(default_factory=list)   # [(name, pos)] in declaration order; the first is initial
    address: Optional[int] = None
    address_pos: Optional[Pos] = None
    rs485_text: Optional[str] = None
    rs485_pos: Optional[Pos] = None
    rs485_pin: Any = None
    blackbox: int = 8
    blackbox_pos: Optional[Pos] = None
    fns: list = field(default_factory=list)      # Fn declarations in source order
    enums: list = field(default_factory=list)
    structs: list = field(default_factory=list)
    budget_ms: Optional[int] = None              # 'budget 5ms': longest allowed main-loop pass
    budget_pos: Optional[Pos] = None
    paths: list = field(default_factory=list)    # PathDecl in source order


# --------------------------------------------------------------------------
# Parser
# --------------------------------------------------------------------------
LIBRARY_ITEMS = ("const", "enum", "struct", "func", "proc", "never")


class ImportCtx:
    """Reads imported library files. Every file gets a number; its tokens carry  number * LINE_STRIDE + line."""

    def __init__(self, path, sources):
        self.sources = sources if sources is not None else {}
        self.sources.setdefault(0, (path, ""))
        self.loaded = {os.path.abspath(path)} if path and path != "<input>" else set()
        self.stack = [os.path.abspath(path)] if path and path != "<input>" else []

    def include(self, rel: str, tok: Tok, node, from_path: str):
        base = os.path.dirname(os.path.abspath(from_path)) if from_path and from_path != "<input>" else os.getcwd()
        full = os.path.normpath(os.path.join(base, rel))
        if full in self.stack:
            _err(f"import cycle: {' -> '.join(os.path.basename(p) for p in self.stack + [full])}", tok.line, tok.col)
        if full in self.loaded:
            return                                  # already pulled in by another import
        try:
            with open(full, encoding="utf-8") as f:
                text = f.read()
        except OSError as e:
            _err(f"cannot import '{rel}': {e.strerror or e}", tok.line, tok.col,
                 "the path is relative to the file that contains the import")
        self.loaded.add(full)
        fid = len(self.sources)
        self.sources[fid] = (full, text)
        self.stack.append(full)
        Parser(lex(text, fid * LINE_STRIDE), self, full).library(node)
        self.stack.pop()


class Parser:
    def __init__(self, toks, ctx=None, path="<input>"):
        self.t = toks
        self.i = 0
        self.ctx = ctx
        self.path = path
        self.pending = []           # imports seen before 'node'

    # -- token helpers
    def peek(self, k=0) -> Tok:
        return self.t[min(self.i + k, len(self.t) - 1)]

    def next(self) -> Tok:
        tok = self.t[self.i]
        if tok.kind != "eof":
            self.i += 1
        return tok

    def at_sym(self, s) -> bool:
        t = self.peek()
        return t.kind == "sym" and t.text == s

    def at_kw(self, s) -> bool:
        t = self.peek()
        return t.kind == "kw" and t.text == s

    def accept_sym(self, s) -> bool:
        if self.at_sym(s):
            self.next()
            return True
        return False

    def accept_kw(self, s) -> bool:
        if self.at_kw(s):
            self.next()
            return True
        return False

    def fail(self, msg, tok=None, hint=""):
        tok = tok or self.peek()
        _err(msg, tok.line, tok.col, hint)

    def expect_sym(self, s, ctx=""):
        t = self.peek()
        if t.kind == "sym" and t.text == s:
            return self.next()
        found = "end of file" if t.kind == "eof" else f"'{t.text}'"
        self.fail(f"expected '{s}' but found {found}{ctx}", t)

    def expect_kw(self, s, hint=""):
        t = self.peek()
        if t.kind == "kw" and t.text == s:
            return self.next()
        found = "end of file" if t.kind == "eof" else f"'{t.text}'"
        self.fail(f"expected '{s}' but found {found}{hint}", t)

    def expect_name(self, what="a name") -> Tok:
        t = self.peek()
        if t.kind == "ident":
            return self.next()
        if t.kind == "kw":
            self.fail(f"'{t.text}' is a reserved word, choose another name", t)
        found = "end of file" if t.kind == "eof" else f"'{t.text}'"
        self.fail(f"expected {what} but found {found}", t)

    # -- structure
    def imports(self):
        while self.peek().kind == "kw" and self.peek().text == "import":
            it = self.next()
            st = self.peek()
            if st.kind != "str":
                self.fail('expected a file name in quotes after import, e.g.  import "common.rudra"', st)
            self.next()
            self.pending.append((st.value, it))

    def flush_imports(self, node):
        for rel, tok in self.pending:
            if self.ctx is None:
                self.ctx = ImportCtx(self.path, None)
            self.ctx.include(rel, tok, node, self.path)
        self.pending = []

    def library(self, node):
        """A library file: shared consts, enums, structs, funcs, procs and 'never' rules; no node, no hardware."""
        self.imports()
        self.flush_imports(node)
        while self.peek().kind != "eof":
            t = self.peek()
            if t.kind != "kw" or t.text not in LIBRARY_ITEMS:
                self.fail("a library file may only contain const, enum, struct, func, proc and never rules", t,
                          "pins, variables, handlers and the node itself belong in the node's own file")
            self.item(node)

    def program(self) -> Node:
        self.imports()
        start = self.expect_kw("node")
        name = self.expect_name("a node name")
        self.expect_kw("target")
        tt = self.peek()
        if tt.kind != "ident":
            self.fail("expected a board name after 'target' (nano, uno, esp32, esp32s3)", tt)
        self.next()
        node = Node(name.text, tt.text, (tt.line, tt.col), (start.line, start.col))
        self.flush_imports(node)
        self.expect_sym("{")
        while not self.at_sym("}"):
            if self.peek().kind == "eof":
                self.fail("missing '}' at the end of the node")
            self.item(node)
        self.expect_sym("}")
        if self.peek().kind != "eof":
            self.fail("unexpected text after the node's closing '}' (one node per file)")
        return node

    def path_decl(self, node: Node):
        t = self.next()                                  # 'path'
        pos = (t.line, t.col)
        nm = self.expect_name("a path name")
        self.expect_kw("for", " (a path looks like  path sweep for pan { at 400  at 0 })")
        ot = self.expect_name("a stepper or motor output name")
        self.expect_sym("{")
        pts, repeat, forever, rep_pos = [], 1, False, None
        while not self.at_sym("}"):
            k = self.peek()
            if k.kind == "eof":
                self.fail("missing '}' at the end of the path")
            if k.kind == "ident" and k.text == "at":
                self.next()
                value = self.expr()
                speed = zone = wait = None
                seen = set()
                while self.peek().kind == "ident" and self.peek().text in ("speed", "zone", "wait"):
                    o = self.next()
                    if o.text in seen:
                        self.fail(f"option '{o.text}' is given twice", o)
                    seen.add(o.text)
                    if o.text == "speed":
                        speed = self.expr()
                    elif o.text == "zone":
                        zone = self.expr()
                    else:
                        wait = self.expr()
                pts.append(PathPt(value, speed, zone, wait, (k.line, k.col)))
            elif k.kind == "ident" and k.text == "repeat":
                self.next()
                nt = self.peek()
                if nt.kind != "num":
                    self.fail("expected a count after 'repeat', e.g.  repeat 3", nt)
                self.next()
                repeat, rep_pos = nt.value, (k.line, k.col)
            elif k.kind == "ident" and k.text == "forever":
                self.next()
                forever, rep_pos = True, (k.line, k.col)
            else:
                self.fail("a path contains  at <value> [speed S] [zone Z] [wait T]  lines, then optionally  repeat N  or  forever", k)
        self.expect_sym("}")
        node.paths.append(PathDecl(nm.text, ot.text, pts, repeat, forever, pos, (ot.line, ot.col), rep_pos))

    def item(self, node: Node):
        t = self.peek()
        if t.kind == "ident" and t.text == "path" and self.peek(1).kind == "ident":
            return self.path_decl(node)
        if (t.kind == "ident" and t.text == "done" and self.peek(1).kind == "ident"
                and self.peek(2).kind == "sym" and self.peek(2).text == "{"):
            self.next()
            nm = self.next()
            node.events.append(Event("done", None, nm.text, [], self.block(), (t.line, t.col)))
            return
        if t.kind != "kw":
            self.fail("expected a declaration: const, input, output, var, enum, struct, func, proc, every, when, on, heartbeat, start or serial", t)
        pos = (t.line, t.col)
        k = t.text
        if k == "const":
            self.next()
            nm = self.expect_name()
            self.expect_sym("=")
            node.decls.append(Const(nm.text, self.expr(), pos))
        elif k == "var":
            self.next()
            nm = self.expect_name()
            tname = size_expr = None
            if self.accept_sym(":"):
                tt = self.peek()
                if tt.kind not in ("ident", "kw"):
                    self.fail("expected a type: int, bool, an enum or a struct", tt)
                self.next()
                tname = tt.text
                if self.accept_sym("["):
                    size_expr = self.expr()
                    self.expect_sym("]")
            init = None
            if tname is None or self.at_sym("="):
                self.expect_sym("=", " (a variable looks like  var n = 0  or  var n: int = 0  or  var buf: int[8])")
                init = self.expr()
            node.decls.append(Var(nm.text, init, pos, tname=tname, size_expr=size_expr))
        elif k == "enum":
            self.next()
            nm = self.expect_name("an enum name")
            self.expect_sym("{")
            members = []
            while not self.at_sym("}"):
                mt = self.expect_name("an enum member name")
                members.append((mt.text, (mt.line, mt.col)))
                if not self.accept_sym(","):
                    break
            self.expect_sym("}")
            node.enums.append(EnumDecl(nm.text, members, pos))
        elif k == "struct":
            self.next()
            nm = self.expect_name("a struct name")
            self.expect_sym("{")
            fields = []
            while not self.at_sym("}"):
                ft = self.expect_name("a field name")
                self.expect_sym(":", " (a field looks like  x: int)")
                ty = self.peek()
                if ty.kind not in ("ident", "kw"):
                    self.fail("expected a type: int, bool or an enum", ty)
                self.next()
                fields.append((ft.text, ty.text, (ft.line, ft.col)))
                if not self.accept_sym(","):
                    break
            self.expect_sym("}")
            node.structs.append(StructDecl(nm.text, fields, pos))
        elif k == "never":
            self.next()
            node.decls.append(Never(self.expr(), pos))
        elif k in ("func", "proc"):
            node.fns.append(self.fn_decl())
        elif k in ("input", "output"):
            node.decls.append(self.io())
        elif k == "serial":
            self.next()
            v = self.peek()
            if v.kind != "num":
                self.fail("expected a baud rate, e.g. serial 115200", v)
            self.next()
            if node.baud_pos is not None:
                _err("'serial' is declared twice", pos[0], pos[1])
            node.baud, node.baud_pos = v.value, pos
        elif k == "address":
            self.next()
            v = self.peek()
            if v.kind != "num":
                self.fail("expected a bus address, e.g.  address 3", v)
            self.next()
            if node.address_pos is not None:
                _err("'address' is declared twice", pos[0], pos[1])
            node.address, node.address_pos = v.value, pos
        elif k == "rs485":
            self.next()
            self.expect_kw("pin")
            pt = self.peek()
            if pt.kind not in ("num", "ident"):
                self.fail("expected the direction pin, e.g.  rs485 pin 8", pt)
            self.next()
            if node.rs485_pos is not None:
                _err("'rs485' is declared twice", pos[0], pos[1])
            node.rs485_text, node.rs485_pos = pt.text, (pt.line, pt.col)
        elif k == "budget":
            self.next()
            v = self.peek()
            if v.kind != "dur":
                self.fail("expected a time, e.g.  budget 5ms", v)
            self.next()
            if node.budget_pos is not None:
                _err("'budget' is declared twice", pos[0], pos[1])
            node.budget_ms, node.budget_pos = v.value, pos
        elif k == "blackbox":
            self.next()
            v = self.peek()
            if v.kind != "num":
                self.fail("expected the number of log entries, e.g.  blackbox 16", v)
            self.next()
            if node.blackbox_pos is not None:
                _err("'blackbox' is declared twice", pos[0], pos[1])
            node.blackbox, node.blackbox_pos = v.value, pos
        elif k == "state":
            self.next()
            nm = self.expect_name("a state name")
            self.expect_sym("{")
            node.states.append((nm.text, (nm.line, nm.col)))
            while not self.at_sym("}"):
                if self.peek().kind == "eof":
                    self.fail(f"missing '}}' at the end of state {nm.text}")
                it = self.peek()
                if it.kind == "kw" and it.text in ("enter", "exit", "every", "when", "on"):
                    self.event(node, it.text, (it.line, it.col), nm.text)
                else:
                    self.fail("inside a state you can use: enter, exit, every, when, on", it,
                              "const, var, input, output, never, start and heartbeat go outside the states")
            self.expect_sym("}")
        elif k in ("every", "heartbeat", "when", "on", "start"):
            self.event(node, k, pos, None)
        elif k in ("enter", "exit"):
            self.fail(f"'{k}' only makes sense inside a state", t, "e.g.  state IDLE { enter { ... } }")
        else:
            self.fail(f"'{k}' cannot be used here", t)

    def event(self, node: Node, k: str, pos, state):
        self.next()
        if k == "every":
            arg = self.expr()
            node.events.append(Event(k, arg, None, [], self.block(), pos, state=state))
        elif k == "heartbeat":
            if state:
                self.fail("'heartbeat' belongs outside the states", self.t[self.i - 1])
            nt = self.peek()
            if nt.kind == "ident" and nt.text == "off":
                self.next()
                node.events.append(Event(k, None, None, [], [], pos, off=True))
            else:
                arg = self.expr()
                body = self.block() if self.at_sym("{") else []
                node.events.append(Event(k, arg, None, [], body, pos))
        elif k == "when":
            arg = self.expr()
            dur = self.expr() if self.accept_kw("for") else None
            node.events.append(Event(k, arg, None, [], self.block(), pos, state=state, dur=dur))
        elif k == "on":
            nm = self.expect_name("a command name")
            params = []
            if self.accept_sym("("):
                params.append(self.expect_name("an argument name").text)
                while self.accept_sym(","):
                    params.append(self.expect_name("an argument name").text)
                self.expect_sym(")")
            node.events.append(Event("on", None, nm.text, params, self.block(), pos, state=state))
        elif k == "start":
            if state:
                self.fail("'start' belongs outside the states (use 'enter' inside a state)", self.t[self.i - 1])
            node.events.append(Event("start", None, None, [], self.block(), pos))
        else:   # enter | exit
            node.events.append(Event(k, None, None, [], self.block(), pos, state=state))

    def io(self) -> IO:
        t = self.next()
        pos = (t.line, t.col)
        nm = self.expect_name()
        self.expect_sym(":")
        kt = self.peek()
        kinds = ("digital", "pwm", "analog", "servo", "neopixel", "i2c", "motor", "stepper", "tone", "ultrasonic")
        if not (kt.kind in ("kw", "ident") and kt.text in kinds):
            self.fail("expected one of: " + ", ".join(kinds), kt)
        self.next()
        self.expect_sym("(")
        pin_text = pin_pos = None
        pull = inv = False
        opts = {}
        pins = {}
        seen = set()
        while True:
            ot = self.peek()
            if ot.kind not in ("kw", "ident"):
                self.fail("expected an option such as  pin 5", ot)
            key, opos = ot.text, (ot.line, ot.col)
            self.next()
            if key in seen:
                self.fail(f"option '{key}' is given twice", ot)
            seen.add(key)
            if key == "pin":
                pt = self.peek()
                if pt.kind not in ("num", "ident"):
                    self.fail("expected a pin number like 5 or A0", pt)
                self.next()
                pin_text, pin_pos = pt.text, (pt.line, pt.col)
            elif key == "pullup":
                pull = True
            elif key == "invert":
                inv = True
            elif key in ("signed", "rgb"):
                opts[key] = ([], opos)
            elif key in ("count", "addr", "reg", "bytes", "ramp", "limit", "timeout", "rate", "debounce"):
                opts[key] = ([self.expr()], opos)
            elif key in ("pwm", "dir", "fwd", "rev", "step", "trig", "echo"):
                pt = self.peek()
                if pt.kind not in ("num", "ident"):
                    self.fail(f"expected a pin number after '{key}', e.g.  {key} 5", pt)
                self.next()
                pins[key] = (pt.text, (pt.line, pt.col))
            elif key == "safe":
                nt = self.peek()
                if nt.kind == "ident" and nt.text == "off":
                    self.next()
                    vals = []
                else:
                    vals = [self.expr()]
                    if kt.text == "neopixel":
                        for _ in range(2):
                            self.expect_sym(",", " (a neopixel safe colour is  safe 255, 0, 0  or  safe off)")
                            vals.append(self.expr())
                opts[key] = (vals, opos)
            else:
                self.fail(f"unknown option '{key}'", ot,
                          "options: pin, pullup, invert, debounce, count, rgb, addr, reg, bytes, signed, "
                          "pwm, dir, fwd, rev, ramp, limit, timeout, step, rate, trig, echo, safe")
            if not self.accept_sym(","):
                break
        self.expect_sym(")")
        return IO(t.text, nm.text, kt.text, pin_text, pin_pos, pull, inv, pos, opts=opts, pins=pins)

    def block(self) -> list:
        self.expect_sym("{")
        out = []
        while not self.at_sym("}"):
            if self.peek().kind == "eof":
                self.fail("missing '}' to close this block")
            out.append(self.stmt())
        self.expect_sym("}")
        return out

    def stmt(self):
        t = self.peek()
        pos = (t.line, t.col)
        if t.kind == "kw" and t.text == "if":
            return self.if_stmt()
        if t.kind == "ident" and t.text in ("run", "stop") and self.peek(1).kind == "ident":
            self.next()
            nm = self.next()
            return RunPath(nm.text, pos) if t.text == "run" else StopOut(nm.text, pos)
        if t.kind == "kw" and t.text == "goto":
            self.next()
            nm = self.expect_name("a state name")
            return Goto(nm.text, pos)
        if t.kind == "kw" and t.text == "let":
            self.next()
            nm = self.expect_name("a local variable name")
            self.expect_sym("=", " (a local looks like  let x = 5)")
            return Let(nm.text, self.expr(), pos)
        if t.kind == "kw" and t.text == "return":
            self.next()
            if self.at_sym("}"):
                return Return(None, pos)
            return Return(self.expr(), pos)
        if t.kind == "kw" and t.text == "break":
            self.next()
            return Break(pos)
        if t.kind == "kw" and t.text == "for":
            self.next()
            nm = self.expect_name("a loop variable name")
            self.expect_kw("in")
            lo = self.expr()
            self.expect_sym("..", " (a loop looks like  for i in 0..8 { }  and stops before the last number)")
            hi = self.expr()
            return For(nm.text, lo, hi, self.block(), pos)
        if t.kind == "ident" and self.peek(1).kind == "sym" and self.peek(1).text == "(":
            self.next()
            return CallStmt(Call(t.text, self.call_args(), pos), pos)
        if t.kind == "kw" and t.text == "send":
            self.next()
            s = self.peek()
            if s.kind != "str":
                self.fail('expected a quoted message name, e.g. send "TEMP", temp', s)
            self.next()
            e = None
            if self.accept_sym(","):
                e = self.expr()
            return Send(s.value, e, pos)
        if (t.kind == "ident" and t.text == "animate" and not (
                self.peek(1).kind == "sym" and self.peek(1).text == "=")):
            self.next()
            strip = self.expect_name("a neopixel output name")
            self.expect_sym(",", " (animate takes a strip name, then a mode: blink, pulse, chase, rainbow or off)")
            mt = self.peek()
            if mt.kind not in ("ident", "kw"):
                self.fail("expected an animation mode: blink, pulse, chase, rainbow or off", mt)
            self.next()
            need = {"blink": 4, "pulse": 4, "chase": 4, "rainbow": 1, "off": 0}.get(mt.text)
            if need is None:
                self.fail(f"unknown animation '{mt.text}'", mt, "modes: blink, pulse, chase, rainbow, off")
            exprs = []
            for _ in range(need):
                self.expect_sym(",", {
                    "rainbow": " (rainbow takes one value, the period:  animate ring, rainbow, 2s)",
                    "off": "",
                }.get(mt.text, f" ({mt.text} takes  r, g, b, period:  animate ring, {mt.text}, 255, 0, 0, 800ms)"))
                exprs.append(self.expr())
            return Animate(strip.text, mt.text, exprs, pos)
        if t.kind == "ident" and t.text in ("fill", "pixel") and not (
                self.peek(1).kind == "sym" and self.peek(1).text == "="):
            self.next()
            strip = self.expect_name("a neopixel output name")
            exprs = []
            for _ in range(3 if t.text == "fill" else 4):
                self.expect_sym(",", f" ({t.text} takes a strip name and then " +
                                ("r, g, b" if t.text == "fill" else "index, r, g, b") + ")")
                exprs.append(self.expr())
            if t.text == "fill":
                return Fill(strip.text, exprs, pos)
            return Pixel(strip.text, exprs[0], exprs[1:], pos)
        if t.kind == "ident":
            self.next()
            if self.at_sym("[") or self.at_sym("."):
                target = self.postfix(Name(t.text, pos))
                self.expect_sym("=", " (statements look like  a[i] = value  or  p.x = value)")
                return AssignTo(target, self.expr(), pos)
            self.expect_sym("=", " (statements look like  name = value)")
            return Assign(t.text, self.expr(), pos)
        self.fail("expected a statement (name = value, if, or send)", t)

    def postfix(self, e):
        """name[idx], name.field, name[idx].field"""
        while True:
            t = self.peek()
            if t.kind == "sym" and t.text == "[":
                if not isinstance(e, Name):
                    self.fail("only one index is allowed (arrays are one-dimensional)", t)
                self.next()
                idx = self.expr()
                self.expect_sym("]")
                e = Index(e.name, idx, (t.line, t.col))
            elif t.kind == "sym" and t.text == ".":
                if isinstance(e, Member):
                    self.fail("structs cannot be nested: use  a.field  or  a[i].field", t)
                self.next()
                f = self.expect_name("a field or enum member name")
                e = Member(e, f.text, (t.line, t.col))
            else:
                return e

    def call_args(self):
        self.expect_sym("(")
        args = []
        if not self.at_sym(")"):
            args.append(self.expr())
            while self.accept_sym(","):
                args.append(self.expr())
        self.expect_sym(")")
        return args

    def fn_decl(self) -> Fn:
        kt = self.next()
        nm = self.expect_name(f"a {kt.text} name")
        self.expect_sym("(")
        params = []
        if not self.at_sym(")"):
            while True:
                pn = self.expect_name("a parameter name")
                self.expect_sym(":", " (parameters are written  name: int  or  name: bool)")
                ty = self.peek()
                if ty.kind not in ("ident", "kw"):
                    self.fail("expected a type: int or bool", ty)
                self.next()
                params.append((pn.text, ty.text, (pn.line, pn.col)))
                if not self.accept_sym(","):
                    break
        self.expect_sym(")")
        ret = None
        if self.accept_sym("->"):
            rt = self.peek()
            if rt.kind not in ("ident", "kw"):
                self.fail("expected a result type: int or bool", rt)
            self.next()
            ret = rt.text
        return Fn(kt.text, nm.text, params, ret, self.block(), (kt.line, kt.col))

    def if_stmt(self):
        t = self.next()
        cond = self.expr()
        then = self.block()
        els = None
        if self.accept_kw("else"):
            if self.at_kw("if"):
                els = [self.if_stmt()]
            else:
                els = self.block()
        return If(cond, then, els, (t.line, t.col))

    # -- expressions (lowest to highest precedence)
    def expr(self):
        return self._or()

    def _binary(self, sub, ops):
        left = sub()
        while self.peek().kind == "sym" and self.peek().text in ops:
            op = self.next()
            left = Bin(op.text, left, sub(), (op.line, op.col))
        return left

    def _or(self):
        return self._binary(self._and, ("||",))

    def _and(self):
        return self._binary(self._eq, ("&&",))

    def _eq(self):
        return self._binary(self._rel, ("==", "!="))

    def _rel(self):
        return self._binary(self._add, ("<", "<=", ">", ">="))

    def _add(self):
        return self._binary(self._mul, ("+", "-"))

    def _mul(self):
        return self._binary(self._unary, ("*", "/", "%"))

    def _unary(self):
        t = self.peek()
        if t.kind == "sym" and t.text in ("-", "!"):
            self.next()
            return Unary(t.text, self._unary(), (t.line, t.col))
        return self._primary()

    def _primary(self):
        t = self.peek()
        pos = (t.line, t.col)
        if t.kind in ("num", "dur", "pct"):
            self.next()
            return Num(t.value, pos)
        if t.kind == "kw" and t.text in ("true", "false"):
            self.next()
            return BoolLit(t.text == "true", pos)
        if t.kind == "ident":
            self.next()
            if self.at_sym("("):
                return Call(t.text, self.call_args(), pos)
            return self.postfix(Name(t.text, pos))
        if t.kind == "sym" and t.text == "(":
            self.next()
            e = self.expr()
            self.expect_sym(")")
            return e
        found = "end of file" if t.kind == "eof" else f"'{t.text}'"
        self.fail(f"expected a value but found {found}", t)


def parse(src: str, path: str = "<input>", sources=None) -> Node:
    """`sources` (a dict) is filled with {file number: (path, text)} so errors in imported files can be shown."""
    ctx = ImportCtx(path, sources) if (sources is not None or "import" in src) else None
    if ctx is not None:
        ctx.sources[0] = (path, src)
    return Parser(lex(src), ctx, path).program()
