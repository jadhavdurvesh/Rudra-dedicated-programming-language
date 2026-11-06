"""Master side of an over-serial program update (see bytecode.push_lines for the wire format)."""
from __future__ import annotations

from typing import Callable, List

from . import bytecode as B


class PushError(Exception):
    pass


def push(ask: Callable[[str], str], data: bytes, chunk: int = 8, progress: Callable[[int, int], None] = None) -> None:
    """Sends an image. `ask(line)` writes one line and returns the node's one-line reply.
    The node goes to its safe state at the first line and starts running the new program at the last."""
    lines = B.push_lines(data, chunk)
    for k, line in enumerate(lines):
        want = "IMGOK" if line.startswith("IMGEND") else "OK"
        got = ask(line)
        if got != want:
            raise PushError(f"node answered {got!r} to {line.split()[0]} (line {k + 1} of {len(lines)}); "
                            "it stays in its safe state; send IMGRESET to restore the built-in program")
        if progress:
            progress(k + 1, len(lines))


def info(ask: Callable[[str], str]) -> dict:
    parts = ask("IMGINFO").split()
    if len(parts) != 4 or parts[0] != "IMGINFO":
        raise PushError(f"unexpected answer to IMGINFO: {' '.join(parts)!r}")
    return {"capacity": int(parts[1]), "length": int(parts[2]), "valid": parts[3] == "1"}


def reset(ask: Callable[[str], str]) -> None:
    if ask("IMGRESET") != "IMGOK":
        raise PushError("IMGRESET was not acknowledged")
