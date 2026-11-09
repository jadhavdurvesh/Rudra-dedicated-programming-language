"""AVR backend, part 2 (stage B5.5b): stepper and motor service routines, and the black box."""
from .gen_util import _port, _pin


class ExtraMixin:
    def cmp_ge(self, b, target, cond="brlo"):
        """Compare A (r22..r25, unsigned) with the constant b; jump to target when the condition holds."""
        self.const(b, 18)
        self.w("cp r22, r18")
        for k in (1, 2, 3):
            self.w(f"cpc r{22 + k}, r{18 + k}")
        self.br_far(cond, target)

    def stepper_motor_routines(self):
        w = self.w
        if self.steppers:
            w("r_steppers:")
            for d in self.steppers:
                n, pins, iv = d.name, d.cfg["pins"], d.cfg["interval"]
                skip, plus, go, up, dn = (self.lab("Ls") for _ in range(5))
                self.load("NOWC")
                self.load(f"TS_{n}", 18)
                w("call sub32")
                if n in self.step_paths:
                    self.load(f"IV_{n}", 18)
                    w("cp r22, r18")
                    for k in (1, 2, 3):
                        w(f"cpc r{22 + k}, r{18 + k}")
                    self.br_far("brlo", skip)
                    w("add r18, r18")                  # 2 * interval
                    w("adc r19, r19")
                    w("adc r20, r20")
                    w("adc r21, r21")
                    w("cp r22, r18")
                    for k in (1, 2, 3):
                        w(f"cpc r{22 + k}, r{18 + k}")
                    self.br_far("brlo", plus)
                else:
                    self.cmp_ge(iv, skip)
                    self.cmp_ge(2 * iv, plus)
                self.load("NOWC")
                self.store(f"TS_{n}")
                w(f"rjmp {go}")
                w(f"{plus}:")
                self.load(f"TS_{n}")
                if n in self.step_paths:
                    self.load(f"IV_{n}", 18)
                else:
                    self.const(iv, 18)
                w("call add32")
                self.store(f"TS_{n}")
                w(f"{go}:")
                self.load(f"SP_{n}")
                self.load(f"O_{n}", 18)
                w("cp r22, r18")
                for k in (1, 2, 3):
                    w(f"cpc r{22 + k}, r{18 + k}")
                self.br_far("breq", skip)
                dport, _, dbit = _port(_pin(pins["dir"]))
                sport, _, sbit = _port(_pin(pins["step"]))
                w(f"brlt {up}")
                w(f"cbi {dport}, {dbit}")
                w("ldi r16, 1")                    # r16 = 1: position goes down
                w(f"rjmp {dn}")
                w(f"{up}:")
                w(f"sbi {dport}, {dbit}")
                w("ldi r16, 0")
                w(f"{dn}:")
                w(f"sbi {sport}, {sbit}")
                for _ in range(16):
                    w("nop")                       # about 1 us of pulse width for the driver
                w(f"cbi {sport}, {sbit}")
                self.load(f"SP_{n}")
                lo, hi = self.lab("Ls"), self.lab("Ls")
                w("tst r16")
                w(f"breq {hi}")
                w("subi r22, 1")
                w("sbci r23, 0")
                w("sbci r24, 0")
                w("sbci r25, 0")
                w(f"rjmp {lo}")
                w(f"{hi}:")
                w("subi r22, 0xFF")
                w("sbci r23, 0xFF")
                w("sbci r24, 0xFF")
                w("sbci r25, 0xFF")
                w(f"{lo}:")
                self.store(f"SP_{n}")
                w(f"{skip}:")
            w("ret")
        if self.need_mstep:
            w("r_motors:")
            for k, d in enumerate(self.motors, 1):
                c, n = d.cfg, d.name
                if not (c.get("ramp") or c.get("timeout")):
                    continue
                sv, nxt, ramp = c["safe"], self.lab("Lm"), self.lab("Lm")
                if c.get("timeout"):
                    chk = self.lab("Lm")           # (out != safe or cur != safe) and now - last >= timeout
                    self.load(f"O_{n}")
                    self.const(sv, 18)
                    w("cp r22, r18")
                    for j in (1, 2, 3):
                        w(f"cpc r{22 + j}, r{18 + j}")
                    w(f"brne {chk}")
                    self.load(f"CUR_{n}")
                    self.const(sv, 18)
                    w("cp r22, r18")
                    for j in (1, 2, 3):
                        w(f"cpc r{22 + j}, r{18 + j}")
                    self.br_far("breq", ramp)
                    w(f"{chk}:")
                    self.load("NOWC")
                    self.load(f"ML_{n}", 18)
                    w("call sub32")
                    self.cmp_ge(c["timeout"], ramp)
                    self.const(sv)
                    self.store(f"O_{n}")
                    self.store(f"CUR_{n}")
                    w(f"call hw_{n}")
                    self.bb(5, k)
                    w(f"jmp {nxt}")
                w(f"{ramp}:")
                if c.get("ramp"):
                    step = c["ramp_step"]
                    gt, lt, fix, same, st_ = (self.lab("Lm") for _ in range(5))
                    self.load(f"O_{n}")
                    self.load(f"CUR_{n}", 18)
                    w("cp r22, r18")
                    for j in (1, 2, 3):
                        w(f"cpc r{22 + j}, r{18 + j}")
                    self.br_far("breq", nxt)
                    w("call sub32")                       # A = target - cur
                    self.const(step, 18)
                    w("cp r18, r22")                      # step < diff ?
                    for j in (1, 2, 3):
                        w(f"cpc r{18 + j}, r{22 + j}")
                    w(f"brlt {gt}")
                    self.const(-step, 18)
                    w("cp r22, r18")                      # diff < -step ?
                    for j in (1, 2, 3):
                        w(f"cpc r{22 + j}, r{18 + j}")
                    w(f"brlt {lt}")
                    self.load(f"O_{n}")
                    w(f"rjmp {fix}")
                    w(f"{gt}:")
                    self.load(f"CUR_{n}")
                    self.const(step, 18)
                    w("call add32")
                    w(f"rjmp {fix}")
                    w(f"{lt}:")
                    self.load(f"CUR_{n}")
                    self.const(step, 18)
                    w("call sub32")
                    w(f"{fix}:")
                    self.load(f"CUR_{n}", 18)
                    w("mov r16, r21")
                    w("eor r16, r25")
                    w("sbrs r16, 7")
                    w(f"rjmp {st_}")                      # same sign: no zero crossing
                    w("sbrc r25, 7")
                    w(f"rjmp {same}")
                    w("mov r16, r22")                      # nx >= 0, cur < 0: nx -> 0 unless it already is
                    w("or r16, r23")
                    w("or r16, r24")
                    w("or r16, r25")
                    w(f"breq {st_}")
                    for r in (22, 23, 24, 25):
                        w(f"clr r{r}")
                    w(f"rjmp {st_}")
                    w(f"{same}:")                          # nx < 0, cur >= 0
                    w("mov r16, r18")
                    w("or r16, r19")
                    w("or r16, r20")
                    w("or r16, r21")
                    w(f"breq {st_}")
                    for r in (22, 23, 24, 25):
                        w(f"clr r{r}")
                    w(f"{st_}:")
                    self.store(f"CUR_{n}")
                    w(f"call hw_{n}")
                w(f"{nxt}:")
            w("ret")

    def bb_routines(self):
        w, N = self.w, self.bb_n
        w("bb_log:                       ; r16 = code, A = argument; the time comes from NOWC")
        w("movw r18, r22")
        w("movw r20, r24")
        w("lds r17, BBH")
        w("mov r30, r17")
        w("clr r31")
        w("lsl r30")
        w("rol r31")
        w("lsl r30")
        w("rol r31")
        w("mov r26, r30")
        w("mov r27, r31")
        w("ldi r22, lo8(BBT)")
        w("ldi r23, hi8(BBT)")
        w("add r30, r22")
        w("adc r31, r23")
        w("ldi r22, lo8(BBA)")
        w("ldi r23, hi8(BBA)")
        w("add r26, r22")
        w("adc r27, r23")
        for k in range(4):
            w(f"lds r22, NOWC+{k}")
            w("st Z+, r22")
        for r in (18, 19, 20, 21):
            w(f"st X+, r{r}")
        w("mov r30, r17")
        w("clr r31")
        w("ldi r22, lo8(BBC)")
        w("ldi r23, hi8(BBC)")
        w("add r30, r22")
        w("adc r31, r23")
        w("st Z, r16")
        w("inc r17")
        w(f"cpi r17, {N}")
        w("brlo bbl_h")
        w("clr r17")
        w("bbl_h:")
        w("sts BBH, r17")
        w("lds r17, BBN")
        w(f"cpi r17, {N}")
        w("brsh bbl_c")
        w("inc r17")
        w("sts BBN, r17")
        w("bbl_c:")
        w("lds r22, BBTOT")
        w("subi r22, 0xFF")
        w("sts BBTOT, r22")
        for k in (1, 2, 3):
            w(f"lds r22, BBTOT+{k}")
            w("sbci r22, 0xFF")
            w(f"sts BBTOT+{k}, r22")
        w("ret")
        w("bb_ptr:                       ; Z = base (r18:r19) + 4 * BBI")
        w("lds r30, BBI")
        w("clr r31")
        w("lsl r30")
        w("rol r31")
        w("lsl r30")
        w("rol r31")
        w("add r30, r18")
        w("adc r31, r19")
        w("ret")
        w("bb_dump:")
        w("lds r16, BBH")
        w("lds r17, BBN")
        w("sts BBK, r17")
        w("sub r16, r17")
        w("brsh bbd_f")
        w(f"subi r16, {(-N) & 0xFF}")
        w("bbd_f:")
        w("sts BBI, r16")
        w("bbd_l:")
        w("lds r16, BBK")
        w("tst r16")
        w("brne bbd_g")
        w("rjmp bbd_e")
        w("bbd_g:")
        t, sp = self.string("LOG "), self.string(" ")
        w(f"ldi r30, lo8({t}*2)")
        w(f"ldi r31, hi8({t}*2)")
        w("call tx_pstr")
        w("ldi r18, lo8(BBT)")
        w("ldi r19, hi8(BBT)")
        w("call bb_ptr")
        for r in (22, 23, 24, 25):
            w(f"ld r{r}, Z+")
        w("andi r25, 0x7F")
        w("call tx_i32")
        w(f"ldi r30, lo8({sp}*2)")
        w(f"ldi r31, hi8({sp}*2)")
        w("call tx_pstr")
        w("lds r30, BBI")
        w("clr r31")
        w("ldi r18, lo8(BBC)")
        w("ldi r19, hi8(BBC)")
        w("add r30, r18")
        w("adc r31, r19")
        w("ld r22, Z")
        w("clr r23")
        w("clr r24")
        w("clr r25")
        w("call tx_i32")
        w(f"ldi r30, lo8({sp}*2)")
        w(f"ldi r31, hi8({sp}*2)")
        w("call tx_pstr")
        w("ldi r18, lo8(BBA)")
        w("ldi r19, hi8(BBA)")
        w("call bb_ptr")
        for r in (22, 23, 24, 25):
            w(f"ld r{r}, Z+")
        w("call tx_i32")
        w("call tx_crlf")
        w("lds r16, BBI")
        w("inc r16")
        w(f"cpi r16, {N}")
        w("brlo bbd_s")
        w("clr r16")
        w("bbd_s:")
        w("sts BBI, r16")
        w("lds r16, BBK")
        w("dec r16")
        w("sts BBK, r16")
        w("rjmp bbd_l")
        w("bbd_e:")
        t = self.string("LOGEND ")
        w(f"ldi r30, lo8({t}*2)")
        w(f"ldi r31, hi8({t}*2)")
        w("call tx_pstr")
        for k in range(4):
            w(f"lds r{22 + k}, BBTOT+{k}")
        w("andi r25, 0x7F")
        w("call tx_i32")
        w("call tx_crlf")
        w("ret")

    # ------------------------------------------------------------------ paths (stage B5.5c)
    def path_routines(self):
        w = self.w
        outs = {}
        for p in self.paths:
            outs.setdefault(p.out, []).append(p)
        decl = {d.name: d for d in self.outs}
        # pe_<P>: enter the waypoint PIDX of P
        for p in self.paths:
            n, d = p.name, decl[p.out]
            w(f"pe_{n}:")
            w(f"lds r16, PIDX_{n}")
            for k, pt in enumerate(p.points):
                nx = self.lab("Lp")
                w(f"cpi r16, {k}")
                w(f"breq {nx}_y")
                w(f"rjmp {nx}")
                w(f"{nx}_y:")
                if p.kind == "stepper":
                    iv = d.cfg["interval"] if pt.sp is None else max(1, 1000 // pt.sp)
                    self.const(iv)
                    self.store(f"IV_{p.out}")
                    self.const(pt.v)
                    w(f"call set_{p.out}")
                    w("ldi r16, 1")
                    w(f"sts PST_{n}, r16")
                else:
                    self.const(pt.v)
                    w(f"call set_{p.out}")
                    w("ldi r16, 2")
                    w(f"sts PST_{n}, r16")
                    self.load("NOWC")
                    self.store(f"PT_{n}")
                w("ret")
                w(f"{nx}:")
            w("ret")
        # pn_<P>: next waypoint, repeat, or finish
        for p in self.paths:
            n, d = p.name, decl[p.out]
            again, fin = self.lab("Lp"), self.lab("Lp")
            w(f"pn_{n}:")
            w(f"lds r16, PIDX_{n}")
            w("inc r16")
            w(f"sts PIDX_{n}, r16")
            w(f"cpi r16, {len(p.points)}")
            w(f"brsh {again}")
            w(f"jmp pe_{n}")
            w(f"{again}:")
            if p.forever:
                w(f"sts PIDX_{n}, r2")
                w(f"jmp pe_{n}")
            else:
                self.load(f"PREP_{n}")
                self.const(1, 18)
                w("cp r18, r22")                       # 1 < prep ?
                for j in (1, 2, 3):
                    w(f"cpc r{18 + j}, r{22 + j}")
                w(f"brlt {fin}x")
                w(f"rjmp {fin}")
                w(f"{fin}x:")
                self.const(1, 18)
                self.load(f"PREP_{n}")
                w("call sub32")
                self.store(f"PREP_{n}")
                w(f"sts PIDX_{n}, r2")
                w(f"jmp pe_{n}")
                w(f"{fin}:")
            if not p.forever:
                pass
            w(f"sts PST_{n}, r2")
            if p.kind == "stepper":
                self.const(d.cfg["interval"])
                self.store(f"IV_{p.out}")
            if n in self.dones:
                w(f"jmp h_{self.dones[n]}")
            else:
                w("ret")
        # pcancel_<out>, prun_<P>, pstop_<out>
        for out, ps in outs.items():
            d = decl[out]
            w(f"pcancel_{out}:")
            for p in ps:
                w(f"sts PST_{p.name}, r2")
            if out in self.step_paths:
                self.const(d.cfg["interval"])
                self.store(f"IV_{out}")
            w("ret")
            w(f"pstop_{out}:")
            w(f"call pcancel_{out}")
            if d.kind == "stepper":
                self.load(f"SP_{out}")
                self.store(f"O_{out}")
                w("ret")
            else:
                self.const(int(d.cfg["safe"]))
                w(f"jmp set_{out}")
        for p in self.paths:
            n = p.name
            w(f"prun_{n}:")
            w(f"call pcancel_{p.out}")
            w(f"sts PIDX_{n}, r2")
            self.const(p.repeat)
            self.store(f"PREP_{n}")
            w(f"jmp pe_{n}")
        # r_paths: advance every active path (called once per loop pass, after the event handlers)
        w("r_paths:")
        for p in self.paths:
            n = p.name
            skip, st2 = self.lab("Lq"), self.lab("Lq")
            w(f"lds r16, PST_{n}")
            w("tst r16")
            w(f"brne {n}_act")
            w(f"rjmp {skip}")
            w(f"{n}_act:")
            if p.kind == "stepper":
                w("cpi r16, 1")
                self.br_far("brne", st2)
                for k, pt in enumerate(p.points):
                    nx = self.lab("Lq")
                    w(f"lds r16, PIDX_{n}")
                    w(f"cpi r16, {k}")
                    w(f"breq {nx}_y")
                    w(f"rjmp {nx}")
                    w(f"{nx}_y:")
                    self.load(f"SP_{p.out}")
                    self.const(pt.v, 18)
                    w("call sub32")
                    w("sbrs r25, 7")
                    w(f"rjmp {nx}_p")
                    w("call neg32")
                    w(f"{nx}_p:")
                    self.const(pt.zn, 18)
                    w("cp r18, r22")                   # zone < |diff| ?  then not there yet
                    for j in (1, 2, 3):
                        w(f"cpc r{18 + j}, r{22 + j}")
                    w(f"brge {nx}_in")
                    w(f"rjmp {skip}")
                    w(f"{nx}_in:")
                    if pt.wt > 0:
                        w("ldi r16, 2")
                        w(f"sts PST_{n}, r16")
                        self.load("NOWC")
                        self.store(f"PT_{n}")
                        w(f"rjmp {skip}")
                    else:
                        w(f"call pn_{n}")
                        w(f"rjmp {skip}")
                    w(f"{nx}:")
                w(f"rjmp {skip}")
            w(f"{st2}:")
            for k, pt in enumerate(p.points):
                nx = self.lab("Lq")
                w(f"lds r16, PIDX_{n}")
                w(f"cpi r16, {k}")
                w(f"breq {nx}_y")
                w(f"rjmp {nx}")
                w(f"{nx}_y:")
                self.load("NOWC")
                self.load(f"PT_{n}", 18)
                w("call sub32")
                self.cmp_ge(pt.wt, f"{skip}")
                w(f"call pn_{n}")
                w(f"rjmp {skip}")
                w(f"{nx}:")
            w(f"{skip}:")
        w("ret")
