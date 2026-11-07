"""Shared helpers: diagnostics, C-compatible integer math, board/pin tables."""
from __future__ import annotations

from dataclasses import dataclass

VERSION = "0.14.0"
US_PERIOD = 60   # ms between ultrasonic measurements


# --------------------------------------------------------------------------
# Diagnostics
# --------------------------------------------------------------------------
@dataclass
class Diag:
    level: str          # "error" | "warning"
    msg: str
    line: int
    col: int
    hint: str = ""


class RudraError(Exception):
    def __init__(self, diags):
        super().__init__("; ".join(d.msg for d in diags))
        self.diags = list(diags)


LINE_STRIDE = 100000     # a position in an imported file is  file_number * LINE_STRIDE + line


def format_diag(d: Diag, src: str, filename: str, sources=None) -> str:
    fid, line = divmod(d.line, LINE_STRIDE)
    if fid and sources and fid in sources:
        filename, src = sources[fid]
    out = f"{filename}:{line}:{d.col}: {d.level}: {d.msg}"
    lines = src.splitlines()
    if 1 <= line <= len(lines):
        out += f"\n    {lines[line - 1]}\n    {' ' * max(d.col - 1, 0)}^"
    if d.hint:
        out += f"\n    hint: {d.hint}"
    return out


# --------------------------------------------------------------------------
# C-compatible 32-bit integer arithmetic (matches the generated C++)
# --------------------------------------------------------------------------
def wrap32(x: int) -> int:
    x &= 0xFFFFFFFF
    return x - (1 << 32) if x & 0x80000000 else x


def cdiv(a: int, b: int) -> int:
    q = abs(a) // abs(b)
    return q if (a < 0) == (b < 0) else -q


def rt_div(a: int, b: int) -> int:
    return 0 if b == 0 else wrap32(cdiv(a, b))


def rt_mod(a: int, b: int) -> int:
    return 0 if b == 0 else a - cdiv(a, b) * b


# --------------------------------------------------------------------------
# Boards and pins
# --------------------------------------------------------------------------
@dataclass
class Pin:
    key: int              # normalised number, used to detect duplicates
    c: str                # how the pin is written in generated C++
    din: bool             # usable as digital input
    dout: bool            # usable as digital output
    pwm: bool             # usable as PWM output
    ain: bool             # usable as analog input
    pullup_ok: bool = True
    warn: str = ""


TARGETS = {
    "nano":    {"fqbn": "arduino:avr:nano",     "family": "avr"},
    "uno":     {"fqbn": "arduino:avr:uno",      "family": "avr"},
    "esp32":   {"fqbn": "esp32:esp32:esp32",    "family": "esp"},
    "esp32s3": {"fqbn": "esp32:esp32:esp32s3",  "family": "esp"},
}


def _avr_pin(target: str, text: str):
    t = text.upper()
    if t.startswith("A") and t[1:].isdigit():
        n = int(t[1:])
        limit = 8 if target == "nano" else 6
        if n >= limit:
            return None, f"{text} does not exist on {target}"
        if n >= 6:   # A6 / A7: analog input only
            return Pin(14 + n, t, False, False, False, True, False), None
        return Pin(14 + n, t, True, True, False, True, True), None
    if not text.isdigit():
        return None, f"'{text}' is not a pin (use a number like 5, or A0..A5)"
    n = int(text)
    if n in (0, 1):
        return None, "pins 0 and 1 are the serial port that talks to the master"
    if 2 <= n <= 13:
        return Pin(n, str(n), True, True, n in (3, 5, 6, 9, 10, 11), False, True), None
    if 14 <= n <= 19:
        return Pin(n, f"A{n - 14}", True, True, False, True, True), None
    return None, f"pin {n} does not exist on {target}"


_ESP32_IO = {0, 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33}
_ESP32_IN_ONLY = {34, 35, 36, 39}
_ESP32_ADC1 = {32, 33, 34, 35, 36, 39}
_ESP32_STRAP = {
    0: "GPIO0 is a boot strapping pin (must be high at reset)",
    2: "GPIO2 is a boot strapping pin (must be low/floating at reset)",
    5: "GPIO5 is a boot strapping pin",
    12: "GPIO12 is a boot strapping pin (must be low at reset)",
    15: "GPIO15 is a boot strapping pin",
}


def _esp32_pin(text: str):
    if not text.isdigit():
        return None, "ESP32 pins are GPIO numbers, e.g. pin 18"
    n = int(text)
    if n in (1, 3):
        return None, f"GPIO{n} is UART0, which talks to the master"
    if 6 <= n <= 11:
        return None, f"GPIO{n} is connected to the flash chip"
    if n in _ESP32_IO:
        return Pin(n, str(n), True, True, True, n in _ESP32_ADC1, True, _ESP32_STRAP.get(n, "")), None
    if n in _ESP32_IN_ONLY:
        return Pin(n, str(n), True, False, False, True, False), None
    return None, f"GPIO{n} does not exist or is not usable on ESP32"


_S3_OK = set(range(0, 19)) | {21} | set(range(38, 43)) | {45, 46, 47, 48}
_S3_STRAP = {0: "boot strapping pin", 3: "boot strapping pin", 45: "boot strapping pin", 46: "boot strapping pin"}


def _esp32s3_pin(text: str):
    if not text.isdigit():
        return None, "ESP32-S3 pins are GPIO numbers, e.g. pin 5"
    n = int(text)
    if n in (43, 44):
        return None, f"GPIO{n} is UART0, which talks to the master"
    if n in (19, 20):
        return None, f"GPIO{n} is the USB port"
    if 26 <= n <= 37:
        return None, f"GPIO{n} is used by flash/PSRAM on many S3 modules"
    if n in _S3_OK:
        w = _S3_STRAP.get(n, "")
        return Pin(n, str(n), True, True, True, 1 <= n <= 10, True, f"GPIO{n} is a {w}" if w else ""), None
    return None, f"GPIO{n} does not exist on ESP32-S3"


def resolve_pin(target: str, text: str):
    """Return (Pin, None) or (None, error message)."""
    if target in ("nano", "uno"):
        return _avr_pin(target, text)
    if target == "esp32":
        return _esp32_pin(text)
    if target == "esp32s3":
        return _esp32s3_pin(text)
    return None, f"unknown target '{target}'"


# --------------------------------------------------------------------------
# Peripherals
# --------------------------------------------------------------------------
# Hardware I2C pins (SDA, SCL) per target, as normalised pin keys.
I2C_PINS = {"nano": (18, 19), "uno": (18, 19), "esp32": (21, 22), "esp32s3": (8, 9)}


def i2c_decode(value: int, n: int, signed: bool) -> int:
    """What the generated code reads back from an n-byte big-endian I2C register."""
    u = value & ((1 << (8 * n)) - 1)
    if signed and n < 4 and (u >> (8 * n - 1)) & 1:
        u -= 1 << (8 * n)
    return wrap32(u) if n == 4 else u


def pack_rgb(r: int, g: int, b: int) -> int:
    c = lambda x: max(0, min(255, x))
    return (c(r) << 16) | (c(g) << 8) | c(b)


# --------------------------------------------------------------------------
# Animations (integer-only, deterministic: the generated C++ uses the same arithmetic)
# --------------------------------------------------------------------------
ANIM_MODES = {"off": 0, "blink": 1, "pulse": 2, "chase": 3, "rainbow": 4}
ANIM_STEP_MS = 20
ANIM_MIN_PERIOD = 40
ANIM_MAX_PERIOD = 3_600_000


def _c255(v: int) -> int:
    return max(0, min(255, v))


def wheel(p: int):
    """Colour wheel 0..255 -> (r, g, b); same three segments as the generated C++."""
    if p < 85:
        return 255 - p * 3, 0, p * 3
    if p < 170:
        p -= 85
        return 0, p * 3, 255 - p * 3
    p -= 170
    return p * 3, 255 - p * 3, 0


def anim_frame(mode: int, n: int, elapsed: int, per: int, c) -> list:
    """Pixels (packed 0xRRGGBB) for one animation frame. elapsed in ms since the animation started."""
    t = (elapsed & 0xFFFFFFFF) % per
    if mode == 1:                                   # blink: first half on, second half off
        return [pack_rgb(*c) if t < per // 2 else 0] * n
    if mode == 2:                                   # pulse: triangle brightness 0..100..0
        h = per // 2
        x = t if t < h else per - t
        lvl = min((x * 100) // h, 100)
        return [pack_rgb(*[(ch * lvl) // 100 for ch in c])] * n
    if mode == 3:                                   # chase: bright head, dim tail one behind
        head = (t * n) // per
        prev = (head + n - 1) % n
        px = [0] * n
        px[prev] = pack_rgb(*[(ch * 30) // 100 for ch in c])
        px[head] = pack_rgb(*c)
        return px
    if mode == 4:                                   # rainbow: wheel spread over the strip, rotating
        base = (t * 256) // per
        return [pack_rgb(*wheel((base + (k * 256) // n) % 256)) for k in range(n)]
    return [0] * n


# --------------------------------------------------------------------------
# Protocol hash: lets the master check it is talking to the firmware it was generated for
# --------------------------------------------------------------------------
def path_interval(speed, declared):
    """Milliseconds between steps for a waypoint's `speed` (steps/s); None keeps the stepper's declared rate."""
    return declared if speed is None else max(1, 1000 // speed)


def fnv31(text) -> int:
    h = 0x811C9DC5
    for b in (text if isinstance(text, (bytes, bytearray)) else text.encode("utf-8")):
        h ^= b
        h = (h * 0x01000193) & 0xFFFFFFFF
    return h & 0x7FFFFFFF
