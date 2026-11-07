; Blink PB5 (Uno pin 13 / L LED) every 500 ms using Timer0 as a clock, and print "HI\n" at start. Hand-written: this is
; the first machine code rudrac's toolchain has ever produced, so it is kept deliberately small.
.equ RAMEND, 0x08FF
.equ TICKS, 0x0100          ; 32-bit count of Timer0 overflows (each 1.024 ms at 16 MHz / 64 / 256)
.equ LASTT, 0x0104

.org 0
        jmp reset
.org 0x20
        jmp t0ovf

reset:
        eor r1, r1
        out 0x3f, r1
        ldi r28, lo8(RAMEND)
        ldi r29, hi8(RAMEND)
        out 0x3e, r29
        out 0x3d, r28
        ; zero the counters
        sts TICKS, r1
        sts TICKS+1, r1
        sts TICKS+2, r1
        sts TICKS+3, r1
        sts LASTT, r1
        sts LASTT+1, r1
        sts LASTT+2, r1
        sts LASTT+3, r1
        ; PB5 output, low
        sbi 0x04, 5
        cbi 0x05, 5
        ; UART 115200, U2X, 8N1, TX on
        ldi r24, 2
        sts 0xC0, r24
        ldi r24, 16
        sts 0xC4, r24
        sts 0xC5, r1
        ldi r24, 0x06
        sts 0xC2, r24
        ldi r24, 0x08
        sts 0xC1, r24
        ; Timer0: clk/64, overflow interrupt
        ldi r24, 3
        out 0x25, r24
        ldi r24, 1
        sts 0x6E, r24
        sei
        ldi r24, 72
        rcall putc
        ldi r24, 73
        rcall putc
        ldi r24, 10
        rcall putc

main:
        ; r25:r24 = TICKS low 16 bits, read with interrupts off
        cli
        lds r24, TICKS
        lds r25, TICKS+1
        sei
        lds r22, LASTT
        lds r23, LASTT+1
        movw r20, r24
        sub r20, r22
        sbc r21, r23
        ; 488 overflows ~ 500 ms: 488 = 0x01E8
        cpi r20, 0xE8
        ldi r18, 0x01
        cpc r21, r18
        brlo main
        sts LASTT, r24
        sts LASTT+1, r25
        sbi 0x03, 5                 ; writing 1 to PINB bit 5 toggles PORTB bit 5
        rjmp main

putc:
        lds r18, 0xC0
        sbrs r18, 5
        rjmp putc
        sts 0xC6, r24
        ret

t0ovf:
        push r24
        in r24, 0x3f
        push r24
        lds r24, TICKS
        subi r24, 0xFF
        sts TICKS, r24
        lds r24, TICKS+1
        sbci r24, 0xFF
        sts TICKS+1, r24
        lds r24, TICKS+2
        sbci r24, 0xFF
        sts TICKS+2, r24
        lds r24, TICKS+3
        sbci r24, 0xFF
        sts TICKS+3, r24
        pop r24
        out 0x3f, r24
        pop r24
        reti
