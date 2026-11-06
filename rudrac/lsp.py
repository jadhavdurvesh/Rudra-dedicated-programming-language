"""rudrac lsp: a small language server (stdio, JSON-RPC) so an editor shows compiler errors as you type.

Supported: diagnostics (errors and warnings, from the same parser and checker the compiler uses; imports are
followed), document symbols, hover on declared names, whole-document formatting (rudrac fmt).
No other dependencies than the standard library. Everything the server answers comes from the functions
below, so tests call them directly.
"""
import json
import os
import sys
from typing import Dict, List, Optional

from .check import check
from .fmt import format_checked
from .syntax import (IO, Const, EnumDecl, Fn, StructDecl, Var, lex, parse)
from .util import LINE_STRIDE, RudraError, VERSION

ERROR, WARNING = 1, 2
KIND = {"module": 2, "enum": 10, "struct": 23, "function": 12, "field": 8, "const": 14, "variable": 13,
        "event": 24, "class": 5, "key": 20}


def _range(line: int, col: int, length: int = 1) -> dict:
    line = max(line, 1) - 1
    col = max(col, 1) - 1
    return {"start": {"line": line, "character": col}, "end": {"line": line, "character": col + max(length, 1)}}


def _word_len(lines: List[str], line: int, col: int) -> int:
    if not 1 <= line <= len(lines):
        return 1
    s, i, n = lines[line - 1], col - 1, 0
    while i + n < len(s) and (s[i + n].isalnum() or s[i + n] == "_"):
        n += 1
    return n or 1


def diagnostics(text: str, path: str = "<buffer>") -> List[dict]:
    """LSP diagnostics for one document. Problems inside imported files are shown on the import line's file only
    as a note on line 1 (the editor has no way to show another file's squiggle here)."""
    lines = text.split("\n")
    out: List[dict] = []
    sources: dict = {}
    try:
        ck = check(parse(text, path, sources))
        diags = ck.diags
    except RudraError as e:
        diags = e.diags
    for d in diags:
        here = d.line < LINE_STRIDE
        msg = d.msg + (f"  ({d.hint})" if d.hint else "")
        if not here:
            msg = "in an imported file: " + msg
        line = d.line if here else 1
        col = d.col if here else 1
        out.append({"range": _range(line, col, _word_len(lines, line, col) if here else 1),
                    "severity": ERROR if d.level == "error" else WARNING, "source": "rudrac", "message": msg})
    return out


def symbols(text: str) -> List[dict]:
    """A flat outline: node, declarations, states, events, functions."""
    try:
        node = parse(text, "<buffer>", {})
    except RudraError:
        return []
    lines = text.split("\n")

    def sym(name, kind, pos):
        r = _range(pos[0], pos[1], _word_len(lines, pos[0], pos[1]))
        return {"name": name, "kind": KIND[kind], "range": r, "selectionRange": r}

    out = [sym(f"node {node.name}", "module", node.pos)]
    for d in node.decls:
        if d.pos[0] >= LINE_STRIDE:
            continue
        if isinstance(d, IO):
            out.append(sym(f"{d.direction} {d.name}", "field", d.pos))
        elif isinstance(d, Const):
            out.append(sym(f"const {d.name}", "const", d.pos))
        elif isinstance(d, Var):
            out.append(sym(f"var {d.name}", "variable", d.pos))
    for e in node.enums:
        if e.pos[0] < LINE_STRIDE:
            out.append(sym(f"enum {e.name}", "enum", e.pos))
    for s in node.structs:
        if s.pos[0] < LINE_STRIDE:
            out.append(sym(f"struct {s.name}", "struct", s.pos))
    for name, pos in node.states:
        out.append(sym(f"state {name}", "class", pos))
    for f in node.fns:
        if f.pos[0] < LINE_STRIDE:
            out.append(sym(f"{f.kind} {f.name}", "function", f.pos))
    for p in node.paths:
        out.append(sym(f"path {p.name}", "key", p.pos))
    for e in node.events:
        label = {"on": f"on {e.name}", "done": f"done {e.name}"}.get(e.kind, e.kind)
        if e.pos[0] < LINE_STRIDE:
            out.append(sym(label, "event", e.pos))
    return out


def _word_at(text: str, line: int, col: int) -> Optional[str]:
    lines = text.split("\n")
    if not 0 <= line < len(lines):
        return None
    s, i = lines[line], col
    if i > len(s):
        return None
    a = i
    while a > 0 and (s[a - 1].isalnum() or s[a - 1] == "_"):
        a -= 1
    b = i
    while b < len(s) and (s[b].isalnum() or s[b] == "_"):
        b += 1
    return s[a:b] or None


def hover(text: str, line: int, col: int) -> Optional[str]:
    """What the compiler knows about the name under the cursor (0-based line/character, as in LSP)."""
    w = _word_at(text, line, col)
    if not w:
        return None
    try:
        node = parse(text, "<buffer>", {})
    except RudraError:
        return None
    for d in node.decls:
        if getattr(d, "name", None) != w:
            continue
        if isinstance(d, IO):
            return f"{d.direction} {d.name}: {d.kind}" + (f", pin {d.pin.key}" if getattr(d, "pin", None) is not None else "")
        if isinstance(d, Const):
            return f"const {d.name}"
        if isinstance(d, Var):
            return f"var {d.name}: {d.tname or d.type}"
    for f in node.fns:
        if f.name == w:
            params = ", ".join(f"{p[0]}: {p[1]}" for p in f.params)
            ret = f" -> {f.ret}" if f.kind == "func" and f.ret else ""
            return f"{f.kind} {f.name}({params}){ret}"
    for e in node.enums:
        if e.name == w:
            return f"enum {e.name} {{ {', '.join(m[0] for m in e.members)} }}"
    for s in node.structs:
        if s.name == w:
            return f"struct {s.name} {{ {', '.join(f'{f[0]}: {f[1]}' for f in s.fields)} }}"
    for p in node.paths:
        if p.name == w:
            return f"path {p.name} for {p.out}: {len(p.points)} waypoint{'s' if len(p.points) != 1 else ''}, " + \
                   ("forever" if p.forever else f"repeat {p.repeat}")
    for name, _ in node.states:
        if name == w:
            return f"state {name}"
    return None


def format_edits(text: str) -> List[dict]:
    try:
        new = format_checked(text)
    except RudraError:
        return []
    if new == text:
        return []
    n = len(text.split("\n"))
    return [{"range": {"start": {"line": 0, "character": 0}, "end": {"line": n, "character": 0}}, "newText": new}]


# ---------------------------------------------------------------------------- transport
def _read(inp) -> Optional[dict]:
    length = None
    while True:
        line = inp.readline()
        if not line:
            return None
        line = line.strip()
        if not line:
            break
        if line.lower().startswith(b"content-length:"):
            length = int(line.split(b":", 1)[1])
    if length is None:
        return None
    return json.loads(inp.read(length).decode("utf-8"))


def _write(out, msg: dict):
    body = json.dumps(msg).encode("utf-8")
    out.write(b"Content-Length: " + str(len(body)).encode() + b"\r\n\r\n" + body)
    out.flush()


def _path(uri: str) -> str:
    return uri[7:] if uri.startswith("file://") else uri


class Server:
    def __init__(self, inp, out):
        self.inp, self.out = inp, out
        self.docs: Dict[str, str] = {}
        self.down = False

    def reply(self, id_, result=None, error=None):
        m = {"jsonrpc": "2.0", "id": id_}
        if error:
            m["error"] = error
        else:
            m["result"] = result
        _write(self.out, m)

    def notify(self, method, params):
        _write(self.out, {"jsonrpc": "2.0", "method": method, "params": params})

    def publish(self, uri):
        self.notify("textDocument/publishDiagnostics",
                    {"uri": uri, "diagnostics": diagnostics(self.docs[uri], _path(uri))})

    def handle(self, m: dict) -> bool:
        """Returns False when the server should exit."""
        method, p, id_ = m.get("method"), m.get("params") or {}, m.get("id")
        if method == "initialize":
            self.reply(id_, {"capabilities": {
                "textDocumentSync": 1, "documentSymbolProvider": True, "hoverProvider": True,
                "documentFormattingProvider": True}, "serverInfo": {"name": "rudrac", "version": VERSION}})
        elif method == "shutdown":
            self.down = True
            self.reply(id_, None)
        elif method == "exit":
            return False
        elif method == "textDocument/didOpen":
            td = p["textDocument"]
            self.docs[td["uri"]] = td["text"]
            self.publish(td["uri"])
        elif method == "textDocument/didChange":
            uri = p["textDocument"]["uri"]
            self.docs[uri] = p["contentChanges"][-1]["text"]
            self.publish(uri)
        elif method == "textDocument/didClose":
            uri = p["textDocument"]["uri"]
            self.docs.pop(uri, None)
            self.notify("textDocument/publishDiagnostics", {"uri": uri, "diagnostics": []})
        elif method == "textDocument/documentSymbol":
            self.reply(id_, symbols(self.docs.get(p["textDocument"]["uri"], "")))
        elif method == "textDocument/hover":
            pos = p["position"]
            h = hover(self.docs.get(p["textDocument"]["uri"], ""), pos["line"], pos["character"])
            self.reply(id_, {"contents": {"kind": "plaintext", "value": h}} if h else None)
        elif method == "textDocument/formatting":
            self.reply(id_, format_edits(self.docs.get(p["textDocument"]["uri"], "")))
        elif id_ is not None:
            self.reply(id_, error={"code": -32601, "message": f"method not supported: {method}"})
        return True

    def run(self) -> int:
        while True:
            m = _read(self.inp)
            if m is None:
                return 0 if self.down else 1
            if not self.handle(m):
                return 0 if self.down else 1


def serve() -> int:
    return Server(sys.stdin.buffer, sys.stdout.buffer).run()
