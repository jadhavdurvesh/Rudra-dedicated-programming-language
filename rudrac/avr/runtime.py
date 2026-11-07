"""The fixed part of every AVR program rudrac generates: 32-bit arithmetic, serial transmit, the receive-line reader's
number parser. Written once in assembly. Register use (this is rudrac's own convention, not the avr-gcc one):

  r22:r25  A, the accumulator (little endian)     r18:r21  B, the second operand
  r26:r29  scratch / division remainder           r30:r31  Z, flash and RAM pointer
  r16 r17 r14 r15  scratch                        r2  always 0 (set at reset, never written otherwise)
"""

HELPERS = r"""
; ---------------------------------------------------------------- 32-bit arithmetic: A = A op B
add32:
        add r22, r18
        adc r23, r19
        adc r24, r20
        adc r25, r21
        ret
sub32:
        sub r22, r18
        sbc r23, r19
        sbc r24, r20
        sbc r25, r21
        ret
neg32:
        com r25
        com r24
        com r23
        neg r22
        sbci r23, 0xFF
        sbci r24, 0xFF
        sbci r25, 0xFF
        ret
negb32:
        com r21
        com r20
        com r19
        neg r18
        sbci r19, 0xFF
        sbci r20, 0xFF
        sbci r21, 0xFF
        ret
not32:                          ; A = (A == 0)
        mov r16, r22
        or r16, r23
        or r16, r24
        or r16, r25
        ldi r22, 1
        breq not32_z
        ldi r22, 0
not32_z:
        ldi r23, 0
        ldi r24, 0
        ldi r25, 0
        ret
mul32:                          ; A = A * B (low 32 bits)
        clr r26
        clr r27
        movw r28, r26
        ldi r30, 32
mul32_l:
        sbrs r22, 0
        rjmp mul32_s
        add r26, r18
        adc r27, r19
        adc r28, r20
        adc r29, r21
mul32_s:
        lsl r18
        rol r19
        rol r20
        rol r21
        lsr r25
        ror r24
        ror r23
        ror r22
        dec r30
        brne mul32_l
        movw r22, r26
        movw r24, r28
        ret
udiv32:                         ; A = A / B, r26:r29 = A % B, unsigned, B != 0
        clr r26
        clr r27
        movw r28, r26
        ldi r30, 32
udiv32_l:
        lsl r22
        rol r23
        rol r24
        rol r25
        rol r26
        rol r27
        rol r28
        rol r29
        brcs udiv32_s
        cp r26, r18
        cpc r27, r19
        cpc r28, r20
        cpc r29, r21
        brlo udiv32_n
udiv32_s:
        sub r26, r18
        sbc r27, r19
        sbc r28, r20
        sbc r29, r21
        inc r22
udiv32_n:
        dec r30
        brne udiv32_l
        ret
divs32:                         ; A = A / B, signed, truncating toward zero; x / 0 = 0
        mov r16, r25
        eor r16, r21
        mov r17, r25
        mov r30, r18
        or r30, r19
        or r30, r20
        or r30, r21
        brne divs32_nz
        clr r22
        clr r23
        clr r24
        clr r25
        ret
divs32_nz:
        sbrc r25, 7
        rcall neg32
        sbrc r21, 7
        rcall negb32
        rcall udiv32
        sbrc r16, 7
        rjmp neg32
        ret
mods32:                         ; A = A % B, signed, sign follows the dividend; x % 0 = 0
        mov r17, r25
        mov r30, r18
        or r30, r19
        or r30, r20
        or r30, r21
        brne mods32_nz
        clr r22
        clr r23
        clr r24
        clr r25
        ret
mods32_nz:
        sbrc r25, 7
        rcall neg32
        sbrc r21, 7
        rcall negb32
        rcall udiv32
        movw r22, r26
        movw r24, r28
        sbrc r17, 7
        rjmp neg32
        ret

; ---------------------------------------------------------------- serial transmit (polling)
tx_byte:                        ; r16 -> UART
        lds r17, 0xC0
        sbrs r17, 5
        rjmp tx_byte
        sts 0xC6, r16
        ret
tx_pstr:                        ; Z -> zero-terminated text in flash
        lpm r16, Z+
        tst r16
        breq tx_pstr_e
        rcall tx_byte
        rjmp tx_pstr
tx_pstr_e:
        ret
tx_crlf:
        ldi r16, 13
        rcall tx_byte
        ldi r16, 10
        rjmp tx_byte
tx_i32:                         ; A as a signed decimal
        sbrs r25, 7
        rjmp tx_i32_p
        ldi r16, 45
        rcall tx_byte
        rcall neg32
tx_i32_p:
        clr r14
tx_i32_d:
        ldi r18, 10
        clr r19
        clr r20
        clr r21
        rcall udiv32
        push r26
        inc r14
        mov r16, r22
        or r16, r23
        or r16, r24
        or r16, r25
        brne tx_i32_d
tx_i32_o:
        pop r16
        subi r16, 0xD0
        rcall tx_byte
        dec r14
        brne tx_i32_o
        ret

; ---------------------------------------------------------------- text to number, the way C's strtol does it
mul10:                          ; A = A * 10, clobbers B
        movw r18, r22
        movw r20, r24
        lsl r22
        rol r23
        rol r24
        rol r25
        lsl r18
        rol r19
        rol r20
        rol r21
        lsl r18
        rol r19
        rol r20
        rol r21
        lsl r18
        rol r19
        rol r20
        rol r21
        add r22, r18
        adc r23, r19
        adc r24, r20
        adc r25, r21
        ret
parse_i32:                      ; X -> text; A = value (saturating like strtol), X moved past the digits
        clr r22
        clr r23
        clr r24
        clr r25
        clr r15                 ; r15: 1 once the value no longer fits 32 bits
        clr r14                 ; r14: 1 for a minus sign
        ld r17, X
        cpi r17, 45
        brne parse_i32_p
        ldi r16, 1
        mov r14, r16
        adiw r26, 1
        rjmp parse_i32_d
parse_i32_p:
        cpi r17, 43
        brne parse_i32_d
        adiw r26, 1
parse_i32_d:
        ld r17, X
        subi r17, 48
        cpi r17, 10
        brsh parse_i32_e
        ldi r18, 0x9A           ; 429496730: A >= that would overflow when multiplied by 10
        ldi r19, 0x99
        ldi r20, 0x99
        ldi r21, 0x19
        cp r22, r18
        cpc r23, r19
        cpc r24, r20
        cpc r25, r21
        brlo parse_i32_ok
        ldi r16, 1
        mov r15, r16
        adiw r26, 1
        rjmp parse_i32_d
parse_i32_ok:
        rcall mul10
        add r22, r17
        adc r23, r2
        adc r24, r2
        adc r25, r2
        brcc parse_i32_n
        ldi r16, 1
        mov r15, r16
parse_i32_n:
        adiw r26, 1
        rjmp parse_i32_d
parse_i32_e:
        tst r15
        brne parse_i32_sat
        tst r14
        brne parse_i32_neg
        tst r25                 ; positive: at most 0x7FFFFFFF
        brpl parse_i32_r
parse_i32_pos:
        ldi r22, 0xFF
        ldi r23, 0xFF
        ldi r24, 0xFF
        ldi r25, 0x7F
parse_i32_r:
        ret
parse_i32_neg:                  ; negative: magnitude at most 0x80000000
        tst r25
        brpl parse_i32_neg2
        ldi r18, 0x01           ; 0x80000000 + 1 > magnitude -> too big if A >= that
        cp r22, r18
        cpc r23, r2
        cpc r24, r2
        ldi r18, 0x80
        cpc r25, r18
        brlo parse_i32_neg2
        rjmp parse_i32_min
parse_i32_neg2:
        rjmp neg32
parse_i32_sat:
        tst r14
        breq parse_i32_pos
parse_i32_min:
        clr r22
        clr r23
        clr r24
        ldi r25, 0x80
        ret
"""
