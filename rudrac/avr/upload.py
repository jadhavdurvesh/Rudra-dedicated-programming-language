"""Upload an Intel HEX file to an Uno/Nano through its bootloader (STK500 v1, the protocol Optiboot speaks): our own
avrdude for the one job we need. The port is any object with write(bytes), read(n) and, for the reset pulse, an optional
dtr property (pyserial's Serial is one); tests use a fake bootloader.

Not tested against a real board yet: see B5_TEST.md. Uno bootloader speaks 115200 baud; old Nano bootloaders 57600."""
import time
from typing import Dict, List

PAGE = 128                  # ATmega328P flash page, bytes
STK_OK, STK_INSYNC = 0x10, 0x14
CRC_EOP = 0x20


class UploadError(Exception):
    pass


def parse_hex(text: str) -> Dict[int, int]:
    """Intel HEX -> {byte address: value}. Checks every record's checksum."""
    mem: Dict[int, int] = {}
    base = 0
    for n, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise UploadError(f"hex line {n}: does not start with ':'")
        try:
            b = bytes.fromhex(line[1:])
        except ValueError:
            raise UploadError(f"hex line {n}: not hexadecimal") from None
        if len(b) < 5 or len(b) != b[0] + 5:
            raise UploadError(f"hex line {n}: wrong length")
        if sum(b) & 0xFF:
            raise UploadError(f"hex line {n}: checksum error")
        kind, addr, data = b[3], (b[1] << 8) | b[2], b[4:-1]
        if kind == 0:
            for i, v in enumerate(data):
                mem[base + addr + i] = v
        elif kind == 1:
            break
        elif kind == 2:
            base = ((data[0] << 8) | data[1]) << 4
        elif kind == 4:
            base = ((data[0] << 8) | data[1]) << 16
        # other record types (start address) are irrelevant for flashing
    if not mem:
        raise UploadError("the hex file holds no data")
    return mem


def pages(mem: Dict[int, int], size: int = PAGE) -> List[tuple]:
    """[(byte address of page start, bytes of one full page)], unused bytes 0xFF."""
    out = {}
    for a, v in mem.items():
        p = a - a % size
        out.setdefault(p, bytearray([0xFF] * size))[a % size] = v
    return [(a, bytes(d)) for a, d in sorted(out.items())]


class Stk500:
    def __init__(self, port, sleep=time.sleep):
        self.port, self.sleep = port, sleep

    def _read(self, n: int) -> bytes:
        got = b""
        for _ in range(3):
            got += self.port.read(n - len(got))
            if len(got) >= n:
                break
        return got

    def cmd(self, payload: bytes, extra: int = 0) -> bytes:
        self.port.write(bytes(payload) + bytes([CRC_EOP]))
        r = self._read(2 + extra)
        if len(r) < 2 or r[0] != STK_INSYNC or r[-1] != STK_OK:
            raise UploadError(f"bootloader answered {r.hex() or 'nothing'} to command 0x{payload[0]:02x} (expected 14..10)")
        return r[1:-1]

    def reset_and_sync(self, tries: int = 8):
        """Pulse DTR (resets an Uno into its bootloader), then send GET_SYNC until it answers."""
        if hasattr(self.port, "dtr"):
            self.port.dtr = False
            self.sleep(0.25)
            self.port.dtr = True
            self.sleep(0.25)
        for _ in range(tries):
            if hasattr(self.port, "reset_input_buffer"):
                self.port.reset_input_buffer()              # drop anything left over from the reset
            try:
                self.cmd(b"\x30")
                return
            except UploadError:
                self.sleep(0.05)
        raise UploadError("no answer from the bootloader. Is it an Uno/Nano on this port, with no serial monitor open? "
                          "(an old Nano needs baud 57600)")

    def program(self, mem: Dict[int, int], progress=None, verify=True):
        self.reset_and_sync()
        self.cmd(b"\x50")                                           # enter programming mode
        for k, (addr, data) in enumerate(pages(mem)):
            self.cmd(bytes([0x55, (addr >> 1) & 0xFF, (addr >> 1) >> 8]))        # word address
            self.cmd(bytes([0x64, len(data) >> 8, len(data) & 0xFF, 0x46]) + data)    # program page, flash
            if progress:
                progress(k + 1, len(pages(mem)))
        if verify:
            for addr, data in pages(mem):
                self.cmd(bytes([0x55, (addr >> 1) & 0xFF, (addr >> 1) >> 8]))
                got = self.cmd(bytes([0x74, len(data) >> 8, len(data) & 0xFF, 0x46]), extra=len(data))
                if got != data:
                    i = next(i for i in range(len(data)) if got[i] != data[i])
                    raise UploadError(f"verify failed at byte 0x{addr + i:04x}: wrote {data[i]:02x}, read {got[i]:02x}")
        self.cmd(b"\x51")                                           # leave programming mode: the bootloader starts the program


def upload(port, hex_text: str, **kw):
    Stk500(port, **kw.pop("io", {})).program(parse_hex(hex_text), **kw)
