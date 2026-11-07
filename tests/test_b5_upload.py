"""rudrac upload: tested against a fake Optiboot (STK500 v1). Not against a real board."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rudrac.avr.asm import assemble, to_hex
from rudrac.avr.upload import PAGE, Stk500, UploadError, pages, parse_hex

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class FakeBootloader:
    """Speaks the Optiboot subset: sync, enter/leave programming, load address, program/read page. Anything else gets no answer."""

    def __init__(self, drop_sync=0, corrupt_read=False):
        self.flash = bytearray([0xFF] * 0x8000)
        self.out = bytearray()
        self.inp = bytearray()
        self.addr = 0
        self.prog = False
        self.left = False
        self.dtr = True
        self.dtr_log = []
        self.drop_sync = drop_sync
        self.corrupt_read = corrupt_read

    def write(self, data):
        self.inp += data
        while self.inp:
            c = self.inp[0]
            need = {0x30: 2, 0x50: 2, 0x51: 2, 0x55: 4}.get(c)
            if c in (0x64, 0x74):
                if len(self.inp) < 4:
                    return
                need = 4 + ((self.inp[1] << 8) | self.inp[2] if c == 0x64 else 0) + 1
            if need is None:
                self.inp.clear()
                return
            if len(self.inp) < need:
                return
            msg, self.inp = bytes(self.inp[:need]), self.inp[need:]
            assert msg[-1] == 0x20, "missing EOP"
            self.handle(msg)

    def handle(self, m):
        c = m[0]
        if c == 0x30:
            if self.drop_sync:
                self.drop_sync -= 1
                return
            self.out += b"\x14\x10"
        elif c == 0x50:
            self.prog = True
            self.out += b"\x14\x10"
        elif c == 0x51:
            self.left = True
            self.out += b"\x14\x10"
        elif c == 0x55:
            self.addr = (m[1] | (m[2] << 8)) * 2
            self.out += b"\x14\x10"
        elif c == 0x64:
            assert self.prog and m[3] == 0x46
            n = (m[1] << 8) | m[2]
            self.flash[self.addr:self.addr + n] = m[4:4 + n]
            self.out += b"\x14\x10"
        elif c == 0x74:
            n = (m[1] << 8) | m[2]
            data = bytearray(self.flash[self.addr:self.addr + n])
            if self.corrupt_read:
                data[3] ^= 0x01
            self.out += b"\x14" + bytes(data) + b"\x10"

    def read(self, n):
        r, self.out = bytes(self.out[:n]), self.out[n:]
        return r


class HexTests(unittest.TestCase):
    def test_roundtrip_with_assembler(self):
        words = assemble("ldi r16, 0xFF\n ret\n .org 0x40\n nop")
        mem = parse_hex(to_hex(words))
        self.assertEqual(mem[0], 0x0F)
        self.assertEqual(mem[1], 0xEF)
        self.assertEqual(mem[0x80], 0x00)

    def test_bad_checksum_and_garbage(self):
        for bad in (":040000000FEF089500\n:00000001FF", "hello", ":0400", ":zz", ""):
            with self.assertRaises(UploadError, msg=bad):
                parse_hex(bad)

    def test_extended_address_records(self):
        mem = parse_hex(":020000021000EC\n:0100000042BD\n:00000001FF")
        self.assertEqual(mem, {0x10000: 0x42})

    def test_pages_pad_with_ff(self):
        p = pages({0: 1, 130: 2})
        self.assertEqual([a for a, _ in p], [0, 128])
        self.assertEqual(p[0][1][:2], bytes([1, 0xFF]))
        self.assertEqual(p[1][1][2], 2)
        self.assertTrue(all(len(d) == PAGE for _, d in p))


class UploadTests(unittest.TestCase):
    HEX = open(os.path.join(ROOT, "examples", "avr", "blink_raw.hex")).read()

    def go(self, bl, **kw):
        slept = []
        Stk500(bl, sleep=slept.append).program(parse_hex(self.HEX), **kw)
        return slept

    def test_flash_matches_hex(self):
        bl = FakeBootloader()
        self.go(bl)
        mem = parse_hex(self.HEX)
        for a, v in mem.items():
            self.assertEqual(bl.flash[a], v, hex(a))
        self.assertTrue(bl.left)
        self.assertEqual(bl.flash[300:310], bytes([0xFF] * 10))        # beyond the program: untouched

    def test_dtr_pulsed_before_sync(self):
        bl = FakeBootloader()
        log = []
        type(bl).dtr = property(lambda s: True, lambda s, v: log.append(v))
        try:
            self.go(bl)
        finally:
            del type(bl).dtr
        self.assertEqual(log[:2], [False, True])

    def test_sync_retries(self):
        bl = FakeBootloader(drop_sync=3)
        self.go(bl)
        self.assertTrue(bl.left)

    def test_no_bootloader_is_a_clear_error(self):
        bl = FakeBootloader(drop_sync=999)
        with self.assertRaises(UploadError) as e:
            self.go(bl)
        self.assertIn("no answer from the bootloader", str(e.exception))

    def test_verify_catches_a_bad_write(self):
        bl = FakeBootloader(corrupt_read=True)
        with self.assertRaises(UploadError) as e:
            self.go(bl)
        self.assertIn("verify failed", str(e.exception))

    def test_no_verify_skips_readback(self):
        bl = FakeBootloader(corrupt_read=True)
        self.go(bl, verify=False)
        self.assertTrue(bl.left)

    def test_wrong_answer_is_reported(self):
        class Deaf(FakeBootloader):
            def handle(self, m):
                self.out += b"\x15\x10" if m[0] == 0x50 else b""
                if m[0] == 0x30:
                    self.out += b"\x14\x10"
        with self.assertRaises(UploadError) as e:
            self.go(Deaf())
        self.assertIn("0x50", str(e.exception))


if __name__ == "__main__":
    unittest.main()
