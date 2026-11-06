"""rudrac fmt: a layout-only formatter.

It never reflows: your line breaks stay where they are. It fixes indentation (two spaces per brace level, one more
inside open parentheses), missing or stray spaces (extra spaces you typed between tokens stay, so column alignment survives), trailing blanks, runs of blank lines (at most one) and the end
of the file. Comments are kept exactly as written.

It refuses to return text unless the token stream (what the compiler sees) is identical before and after, so a
formatter bug can never change what a program means.
"""
from dataclasses import dataclass
from typing import List, Optional

from .syntax import lex
from .util import RudraError

INDENT = "  "
WORDS = {"and", "or", "not"}
TWO = {"==", "!=", "<=", ">=", "&&", "||", "..", "->"}
ONE = set("{}(),:=<>+-*/%![].")


@dataclass
class T:
    kind: str       # word | num | str | sym | comment
    text: str
    line: int       # 1-based line of the token's first character
    gap: int = 0            # blanks before it on its line
    blank_before: int = 0   # blank lines directly before the line this token starts (only meaningful for the first token on a line)


def scan(src: str) -> List[T]:
    out: List[T] = []
    i, n, line = 0, len(src), 1
    gap = 0

    def add(t):
        nonlocal gap
        t.gap, gap = gap, 0
        out.append(t)

    while i < n:
        ch = src[i]
        if ch == "\n":
            line += 1
            i += 1
            gap = 0
        elif ch in " \t\r":
            i += 1
            gap += 1
        elif ch == "#" or src.startswith("//", i):
            j = i
            while j < n and src[j] != "\n":
                j += 1
            add(T("comment", src[i:j].rstrip(), line))
            i = j
        elif ch == '"':
            j = i + 1
            while j < n and src[j] not in '"\n':
                j += 1
            if j >= n or src[j] != '"':
                raise RudraError([])        # the real lexer reports it
            add(T("str", src[i:j + 1], line))
            i = j + 1
        elif ch.isdigit():
            j = i
            if src.startswith(("0x", "0X"), i):
                j = i + 2
                while j < n and src[j] in "0123456789abcdefABCDEF":
                    j += 1
            else:
                while j < n and src[j].isdigit():
                    j += 1
                if src.startswith("ms", j):
                    j += 2
                elif j < n and src[j] in "s%":
                    j += 1
            add(T("num", src[i:j], line))
            i = j
        elif ch.isalpha() or ch == "_":
            j = i
            while j < n and (src[j].isalnum() or src[j] == "_"):
                j += 1
            add(T("word", src[i:j], line))
            i = j
        elif src[i:i + 2] in TWO:
            add(T("sym", src[i:i + 2], line))
            i += 2
        elif ch in ONE:
            add(T("sym", ch, line))
            i += 1
        else:
            raise RudraError([])
    return out


OPEN, CLOSE = {"(", "["}, {")", "]"}
BINARY = {"==", "!=", "<=", ">=", "&&", "||", "<", ">", "+", "-", "*", "/", "%", "="}


def _is_value_end(t: Optional[T]) -> bool:
    """Does this token end an operand (so that a following '-' is binary)?"""
    if t is None:
        return False
    if t.kind in ("num", "str"):
        return True
    if t.kind == "word":
        return t.text not in WORDS and t.text not in {"return", "if", "else", "while", "in", "send", "let"}
    return t.text in (")", "]")


def _space_between(a: T, b: T, prev: Optional[T]) -> bool:
    """Is there a space between consecutive tokens a, b on one line?"""
    if a.kind == "comment":
        return True
    if b.text in (",", ")", "]", ".", ".."):
        return False
    if a.text in ("(", "[", ".", "..", "!") and a.kind == "sym":
        return False
    if b.text in ("(", "[") and b.kind == "sym":
        # call / index: no space after a name or a closer; keep a space after keywords and operators
        if a.kind == "word":
            return a.text in WORDS or a.text in {"if", "while", "return", "in", "else", "let", "send", "for"}
        return a.text not in (")", "]") and a.kind == "sym" and a.text not in (",",) and a.text not in ("(", "[") and a.text != "!"
    if a.text == "," and a.kind == "sym":
        return True
    if b.text == ":" and b.kind == "sym":
        return False
    if a.text == ":" and a.kind == "sym":
        return True
    if b.text in ("{", "}") and b.kind == "sym":
        return True
    if a.text in ("{", "}") and a.kind == "sym":
        return True
    if a.text == "-" and a.kind == "sym" and not _is_value_end(prev):
        return False            # unary minus
    if a.kind == "sym" and a.text in BINARY | {"->"}:
        return True
    if b.kind == "sym" and (b.text in BINARY or b.text == "->"):
        if b.text == "-" and not _is_value_end(a):
            return a.kind != "sym" or a.text in ("{", "}")
        return True
    if b.text == "!" and b.kind == "sym":
        return True
    return True


def format_source(src: str) -> str:
    toks = scan(src)
    lines: List[List[T]] = []
    for t in toks:
        if not lines or lines[-1][0].line != t.line:
            lines.append([t])
        else:
            lines[-1].append(t)
    # blank lines between consecutive source lines
    for k, ln in enumerate(lines):
        prev_line = lines[k - 1][-1].line if k else 0
        ln[0].blank_before = ln[0].line - prev_line - 1 if k else 0
    out: List[str] = []
    depth = 0           # brace level
    paren = 0           # open parens carried over lines
    # an own-line comment takes the indent of the next code line
    code_indent: List[Optional[int]] = []
    d, p = 0, 0
    for ln in lines:
        first = ln[0]
        if all(t.kind == "comment" for t in ln):
            code_indent.append(None)
            continue
        lead = d - (1 if first.kind == "sym" and first.text == "}" else 0)
        code_indent.append(max(lead, 0) + p)
        for t in ln:
            if t.kind == "sym":
                if t.text == "{":
                    d += 1
                elif t.text == "}":
                    d -= 1
                elif t.text in OPEN:
                    p += 1
                elif t.text in CLOSE:
                    p -= 1
    nxt: Optional[int] = None
    nxt_line = 0
    for k in range(len(lines) - 1, -1, -1):
        if code_indent[k] is None:
            closes = k + 1 < len(lines) and nxt is not None and lines[nxt_line][0].text == "}" and lines[nxt_line][0].kind == "sym"
            code_indent[k] = (nxt + 1 if closes else nxt) if nxt is not None else 0
        else:
            nxt, nxt_line = code_indent[k], k
    for k, ln in enumerate(lines):
        if k and ln[0].blank_before > 0:
            out.append("")
        parts: List[str] = []
        prev2: Optional[T] = None
        prev: Optional[T] = None
        for t in ln:
            if prev is not None:
                if t.kind == "comment":
                    parts.append(" " * max(t.gap, 2))
                elif _space_between(prev, t, prev2):
                    parts.append(" " * max(t.gap, 1))      # keep deliberate alignment, add a space where one is missing
            parts.append(t.text)
            prev2, prev = prev, t
        text = "".join(parts)
        out.append((INDENT * (code_indent[k] or 0) + text).rstrip())
    while out and out[-1] == "":
        out.pop()
    return "\n".join(out) + "\n" if out else ""


def _stream(src: str):
    return [(t.kind, t.text, t.value) for t in lex(src)]


def format_checked(src: str) -> str:
    """Format, and prove the compiler sees the same tokens. Raises RudraError on source that does not lex."""
    before = _stream(src)          # raises RudraError with a proper message if the source cannot be lexed
    try:
        new = format_source(src)
    except RudraError:
        raise
    after = _stream(new)
    if [(k, t) for k, t, _ in before] != [(k, t) for k, t, _ in after] or [v for *_, v in before] != [v for *_, v in after]:
        raise AssertionError("rudrac fmt would change the program's tokens; refusing (this is a bug in fmt)")
    # comments must survive too
    c1 = [t.text for t in scan(src) if t.kind == "comment"]
    c2 = [t.text for t in scan(new) if t.kind == "comment"]
    if c1 != c2:
        raise AssertionError("rudrac fmt would change a comment; refusing (this is a bug in fmt)")
    return new
