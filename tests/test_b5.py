"""Stage B5 (first slice): the AVR assembler and the ATmega328P emulator.

The assembler is checked against opcode words that appear in real avr-gcc listings and the datasheet's encoding tables;
the emulator's arithmetic flags against the datasheet's boolean formulas, written separately from the emulator's own
arithmetic and run over every pair of 8-bit operands."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rudrac.avr.asm import AsmError, assemble, to_bytes, to_hex
from rudrac.avr.emu import AVR, EmuError, SREG_C, SREG_H, SREG_N, SREG_S, SREG_V, SREG_Z

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

GOLD = [
    ("ldi r16, 0xFF", 0xEF0F), ("ret", 0x9508), ("reti", 0x9518), ("sei", 0x9478), ("cli", 0x94F8),
    ("push r28", 0x93CF), ("pop r28", 0x91CF), ("mov r24, r22", 0x2F86), ("movw r24, r22", 0x01CB),
    ("adiw r24, 1", 0x9601), ("sbiw r28, 1", 0x9721), ("cpi r24, 0x0A", 0x308A), ("sbi 0x05, 5", 0x9A2D),
    ("in r24, 0x3f", 0xB78F), ("out 0x3e, r29", 0xBFDE), ("out 0x3d, r28", 0xBFCD), ("nop", 0x0000),
    ("clr r1", 0x2411), ("eor r1, r1", 0x2411), ("lsl r24", 0x0F88), ("sub r24, r22", 0x1B86),
    ("add r24, r22", 0x0F86), ("com r24", 0x9580), ("inc r24", 0x9583), ("dec r24", 0x958A),
    ("subi r24, 1", 0x5081), ("andi r24, 0x0F", 0x708F), ("ori r24, 0x80", 0x6880), ("ld r24, Z", 0x8180),
    ("ld r24, X+", 0x918D), ("st X+, r24", 0x938D), ("ldd r24, Y+1", 0x8189), ("std Y+2, r24", 0x838A),
    ("rjmp .", 0xC000),
]


def run(src, ms=10):
    m = AVR(assemble(src))
    m.set_sp(0x8FF)
    m.run_ms(ms)
    return m


class AssemblerTests(unittest.TestCase):
    def test_known_opcodes(self):
        for text, word in GOLD:
            self.assertEqual(assemble(text)[0], word, text)

    def test_branch_and_jump_distance(self):
        self.assertEqual(assemble("l: rjmp l")[0], 0xCFFF)           # jump to itself
        self.assertEqual(assemble("l: nop\n brne l")[1], 0xF7F1)     # the avr-gcc encoding of 'brne .-4'
        self.assertEqual(assemble("breq t\n nop\n t: nop")[0], 0xF009)
        w = assemble("jmp 0x20")
        self.assertEqual((w[0], w[1]), (0x940C, 0x20))
        w = assemble("lds r24, 0x0100")
        self.assertEqual((w[0], w[1]), (0x9180, 0x0100))

    def test_labels_and_directives(self):
        w = assemble(".equ X, 5\n .org 4\n a: ldi r16, X\n .word a, 0x1234\n .byte 1, 2, 3")
        self.assertEqual(w[4], 0xE005)
        self.assertEqual((w[5], w[6]), (4, 0x1234))
        self.assertEqual((w[7], w[8]), (0x0201, 0x0003))

    def test_out_of_range_is_an_error(self):
        for bad in ("ldi r16, 256", "ldi r5, 1", "sbi 0x30, 1", "sbi 0x05, 9", "adiw r20, 1", "adiw r24, 64",
                    "brne far\n" + "nop\n" * 70 + "far: nop", "movw r3, r4", "out 64, r1", "ldd r24, Y+64",
                    "frobnicate r1", "ldi r16", "rjmp nowhere", "l: nop\n l: nop"):
            with self.assertRaises(AsmError, msg=bad):
                assemble(bad)

    def test_overlapping_org_is_an_error(self):
        with self.assertRaises(AsmError):
            assemble("nop\n nop\n .org 1\n nop")

    def test_hex_format(self):
        h = to_hex(assemble("ldi r16, 0xFF\n ret"))
        self.assertEqual(h.splitlines()[0], ":040000000FEF089561")
        self.assertEqual(h.splitlines()[-1], ":00000001FF")
        for line in h.splitlines():
            b = bytes.fromhex(line[1:])
            self.assertEqual(sum(b) & 0xFF, 0, line)


def flags(m):
    return {n: m.flag(b) for n, b in (("C", SREG_C), ("Z", SREG_Z), ("N", SREG_N), ("V", SREG_V), ("S", SREG_S), ("H", SREG_H))}


def bit(x, n):
    return (x >> n) & 1


class EmulatorFlagTests(unittest.TestCase):
    """Datasheet formulas (AVR Instruction Set Manual) evaluated per bit, independent of the emulator's integer arithmetic."""

    def test_add_adc_all_operands(self):
        m = AVR({})
        for cin in (0, 1):
            for a in range(256):
                for b in range(256):
                    m.setf(SREG_C, cin)
                    r = m.add(a, b, cin)
                    self.assertEqual(r, (a + b + cin) & 0xFF)
                    H = (bit(a, 3) & bit(b, 3)) | (bit(b, 3) & (1 - bit(r, 3))) | ((1 - bit(r, 3)) & bit(a, 3))
                    V = (bit(a, 7) & bit(b, 7) & (1 - bit(r, 7))) | ((1 - bit(a, 7)) & (1 - bit(b, 7)) & bit(r, 7))
                    C = (bit(a, 7) & bit(b, 7)) | (bit(b, 7) & (1 - bit(r, 7))) | ((1 - bit(r, 7)) & bit(a, 7))
                    f = flags(m)
                    self.assertEqual((f["H"], f["V"], f["C"], f["N"], f["Z"], f["S"]),
                                     (H, V, C, bit(r, 7), int(r == 0), bit(r, 7) ^ V), (a, b, cin))

    def test_sub_sbc_cp_all_operands(self):
        m = AVR({})
        for cin in (0, 1):
            for zin in (0, 1):
                for a in range(256):
                    for b in range(256):
                        m.setf(SREG_C, cin)
                        m.setf(SREG_Z, zin)
                        keep = cin == 1 or zin == 1 or True
                        r = m.sub(a, b, cin, True)
                        self.assertEqual(r, (a - b - cin) & 0xFF)
                        H = ((1 - bit(a, 3)) & bit(b, 3)) | (bit(b, 3) & bit(r, 3)) | (bit(r, 3) & (1 - bit(a, 3)))
                        V = (bit(a, 7) & (1 - bit(b, 7)) & (1 - bit(r, 7))) | ((1 - bit(a, 7)) & bit(b, 7) & bit(r, 7))
                        C = ((1 - bit(a, 7)) & bit(b, 7)) | (bit(b, 7) & bit(r, 7)) | (bit(r, 7) & (1 - bit(a, 7)))
                        f = flags(m)
                        Z = int(r == 0 and zin == 1)
                        self.assertEqual((f["H"], f["V"], f["C"], f["N"], f["Z"], f["S"]),
                                         (H, V, C, bit(r, 7), Z, bit(r, 7) ^ V), (a, b, cin, zin))

    def test_plain_sub_sets_z_from_result(self):
        m = AVR({})
        m.setf(SREG_Z, 0)
        m.sub(5, 5)
        self.assertEqual(m.flag(SREG_Z), 1)
        m.setf(SREG_Z, 1)
        m.sub(5, 4)
        self.assertEqual(m.flag(SREG_Z), 0)

    def test_instructions_via_programs(self):
        # 16-bit compare and subtract chains, the way generated code uses them
        m = run("ldi r24, 0x00\n ldi r25, 0x01\n ldi r22, 0x01\n ldi r23, 0x00\n sub r24, r22\n sbc r25, r23\n x: rjmp x")
        self.assertEqual((m.r[24], m.r[25]), (0xFF, 0x00))
        m = run("ldi r24, 200\n lsl r24\n ldi r25, 0\n rol r25\n x: rjmp x")      # 200*2 = 400 = 0x190
        self.assertEqual((m.r[24], m.r[25]), (0x90, 0x01))
        m = run("ldi r24, 0x81\n lsr r24\n ldi r25, 0x81\n asr r25\n ldi r26, 0x01\n sec\n ror r26\n x: rjmp x")
        self.assertEqual((m.r[24], m.r[25], m.r[26]), (0x40, 0xC0, 0x80))
        m = run("ldi r24, 7\n ldi r25, 9\n mul r24, r25\n x: rjmp x")
        self.assertEqual((m.r[0], m.r[1]), (63, 0))
        m = run("ldi r24, 5\n neg r24\n ldi r25, 0x0F\n swap r25\n x: rjmp x")
        self.assertEqual((m.r[24], m.r[25]), (251, 0xF0))
        m = run("ldi r28, 0x00\n ldi r29, 0x02\n ldi r24, 0xAB\n st Y+, r24\n st Y, r24\n ldi r26, 0\n ldi r27, 2\n"
                " ld r20, X+\n ld r21, X\n sts 0x0300, r20\n lds r22, 0x0300\n x: rjmp x")
        self.assertEqual((m.r[20], m.r[21], m.r[22], m.mem[0x200], m.mem[0x201]), (0xAB, 0xAB, 0xAB, 0xAB, 0xAB))
        m = run("ldi r30, lo8(tab*2)\n ldi r31, hi8(tab*2)\n lpm r24, Z+\n lpm r25, Z\n x: rjmp x\n tab: .byte 0x5A, 0xA5")
        self.assertEqual((m.r[24], m.r[25]), (0x5A, 0xA5))

    def test_call_ret_and_stack(self):
        m = run("ldi r24, 1\n rcall f\n ldi r25, 2\n x: rjmp x\n f: ldi r24, 9\n ret")
        self.assertEqual((m.r[24], m.r[25], m.sp()), (9, 2, 0x8FF))

    def test_skip_over_two_word_instruction(self):
        m = run("ldi r24, 5\n cpse r24, r24\n lds r25, 0x0100\n ldi r26, 7\n x: rjmp x")
        self.assertEqual((m.r[25], m.r[26]), (0, 7))
        m = run("ldi r24, 5\n sbrs r24, 0\n ldi r25, 1\n ldi r26, 7\n x: rjmp x")
        self.assertEqual((m.r[25], m.r[26]), (0, 7))
        m = run("ldi r24, 4\n sbrs r24, 0\n ldi r25, 1\n ldi r26, 7\n x: rjmp x")
        self.assertEqual((m.r[25], m.r[26]), (1, 7))

    def test_cycle_counts(self):
        m = AVR(assemble("nop\n ldi r24, 1\n lds r25, 0x0100\n rjmp .+0\n rcall .+0"))
        m.set_sp(0x8FF)
        for _ in range(5):
            m.step()
        self.assertEqual(m.cycles, 1 + 1 + 2 + 2 + 3)

    def test_unimplemented_and_wild_accesses_raise(self):
        with self.assertRaises(EmuError):
            AVR({0: 0x95C8}).step()              # lpm (no operands): not implemented, must not be guessed
        with self.assertRaises(EmuError):
            run("lds r24, 0x2000")
        with self.assertRaises(EmuError):
            run("ldi r24, 0\n out 0x3d, r24\n out 0x3e, r24\n push r24")

    def test_stack_pointer_loaded_from_ramend(self):
        m = AVR(assemble("ldi r28, lo8(0x08FF)\n ldi r29, hi8(0x08FF)\n out 0x3e, r29\n out 0x3d, r28\n push r28\n x: rjmp x"))
        m.run_ms(0.01)
        self.assertEqual((m.sp(), m.mem[0x8FF]), (0x8FE, 0xFF))


class PeripheralTests(unittest.TestCase):
    def test_port_toggle_and_pin_log(self):
        m = run("sbi 0x04, 5\n sbi 0x03, 5\n sbi 0x03, 5\n sbi 0x05, 5\n x: rjmp x", 0.01)
        self.assertEqual([(p, l) for _, p, l in m.pin_log], [("B", 5, 1)] if False else [("B5", 1), ("B5", 0), ("B5", 1)])

    def test_timer0_overflow_interrupt_period(self):
        src = (".org 0\n jmp go\n .org 0x20\n jmp isr\n go:\n eor r1, r1\n ldi r24, 3\n out 0x25, r24\n ldi r24, 1\n"
               " sts 0x6E, r24\n ldi r28, lo8(0x8FF)\n ldi r29, hi8(0x8FF)\n out 0x3e, r29\n out 0x3d, r28\n sei\n x: rjmp x\n"
               " isr: inc r10\n reti")
        m = AVR(assemble(src))
        m.run_ms(102.4)           # 100 overflows at 1.024 ms
        self.assertIn(m.r[10], (99, 100))

    def test_uart_tx_and_rx(self):
        m = AVR(assemble("ldi r24, 0x41\n sts 0xC6, r24\n w: lds r25, 0xC0\n sbrs r25, 7\n rjmp w\n lds r24, 0xC6\n"
                         " sts 0xC6, r24\n x: rjmp x"))
        m.run_ms(1, rx_events=[(0.5, b"Z")])
        self.assertEqual(m.tx_text(), "AZ")

    def test_blink_example(self):
        src = open(os.path.join(ROOT, "examples", "avr", "blink_raw.S")).read()
        m = AVR(assemble(src))
        m.run_ms(2600)
        self.assertEqual(m.tx_text(), "HI\n")
        edges = [(round(t), p, l) for t, p, l in m.pin_log]
        self.assertEqual([(p, l) for _, p, l in edges], [("B5", 1), ("B5", 0), ("B5", 1), ("B5", 0), ("B5", 1)])
        for (t1, _, _), (t2, _, _) in zip(edges, edges[1:]):
            self.assertTrue(497 <= t2 - t1 <= 502, (t1, t2))     # 488 overflows of 1.024 ms = 499.7 ms

    def test_checked_in_hex_matches_source(self):
        src = open(os.path.join(ROOT, "examples", "avr", "blink_raw.S")).read()
        self.assertEqual(open(os.path.join(ROOT, "examples", "avr", "blink_raw.hex")).read(), to_hex(assemble(src)))


if __name__ == "__main__":
    unittest.main()
