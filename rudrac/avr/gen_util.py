"""Pin helpers shared by the AVR backend modules."""
from typing import Tuple


class AvrUnsupported(Exception):
    def __init__(self, reasons):
        self.reasons = list(reasons)
        super().__init__("; ".join(self.reasons))


def _pin(pin) -> int:
    c = str(pin.c)
    return 14 + int(c[1:]) if c[0] in "Aa" else int(c)


def _port(n: int) -> Tuple[int, int, int]:
    """(PORT io address, DDR io address, bit) of an Uno digital pin number."""
    if n <= 7:
        return 0x0B, 0x0A, n
    if n <= 13:
        return 0x05, 0x04, n - 8
    if n <= 19:
        return 0x08, 0x07, n - 14
    raise AvrUnsupported([f"pin {n} does not exist on this chip"])


