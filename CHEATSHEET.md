# Rudra cheat sheet (v0.6)

```
node NAME target nano|uno|esp32|esp32s3 {
  serial 9600                     // link speed
  address 3   rs485 pin 4         // optional: shared bus
  blackbox 16                     // optional: last 16 events, read with LOG
  budget 10ms                     // optional: refuse to build if one main-loop pass can take longer (see: rudrac cost)

  const X = 600     var n = 0     // constants / variables (int or bool)

  // INPUTS
  input b:   digital(pin 2, pullup, invert, debounce 30ms)
  input t:   analog(pin A0)                       // 0..1023
  input d:   ultrasonic(trig 7, echo 8)           // cm, 0 = no echo, every 60 ms
  input s:   i2c(addr 0x48, reg 0, bytes 2, signed)

  // OUTPUTS (every one needs safe ...)
  output l:  digital(pin 13, safe false)          // invert
  output f:  pwm(pin 9, safe 100)                 // 0..100
  output sv: servo(pin 10, safe 90)               // 0..180
  output hn: tone(pin 6, safe off)                // Hz, 0 = silent
  output m:  motor(pwm 5, dir 4, ramp 200, limit 80, timeout 1s, safe 0)   // or fwd 5, rev 6
  output st: stepper(step 8, dir 9, rate 400, safe 0)                      // value = target position
  output r:  neopixel(pin 6, count 8, safe off)
  output x:  i2c(addr 0x20, reg 9, bytes 2, safe 0)

  never heater && !fan            // interlock: outputs and consts only

  func f(a: int, b: bool) -> int { return a }    // pure: reads only, no assignments, no send. no recursion
  proc p(n: int) { led = true }                  // may change outputs/vars; call as a statement:  p(3)

  start { }                       // once at boot
  every 100ms { }                 // periodic
  when cond { }                   // when cond turns true
  when cond for 500ms { }         // ...and stayed true for 500 ms
  on CMD(a, b) { }                // command from master: "CMD 1 2"
  heartbeat 3s { }                // silence for 3 s -> every output goes safe

  state idle { enter { }  exit { }  every 1s { }  when c { }  on GO { goto run } }
  state run  { ... }              // first state is the start state; goto must be last in its block
}
```

**Imports:** `import "common.rudra"` before `node` · library files hold only const/enum/struct/func/proc/never · `pin FAN_PIN` works with a const.

**VM nodes:** `rudrac build --vm` · `rudrac image n.rudra` -> `.rimg` · `rudrac push n.rimg PORT` · lines `IMG off hex` / `IMGEND len sum` / `IMGRESET` / `IMGINFO` (reserved command names).

**Data:** `enum Mode { idle, scan }` (`Mode.scan`; `==`/`!=` only) · `struct P { x: int, hot: bool }` · `var a: int[8]` · `var p: P` · `var ps: P[3]` · `a[i]` · `p.x` · `ps[i].x`. Bad index: read = 0, write refused + counted in STATUS + black box code 6. Constant index out of range = compile error.

**Statements:** `let x = e` · `for i in 0..8 { ... break ... }` (constant bounds) · `return e` · `f(x)` · `x = e` · `if c { } else { }` · `send "NAME", e` · `goto state` · `fill s, r, g, b` · `pixel s, i, r, g, b` · `animate s, blink|pulse|chase, r, g, b, 500ms` · `animate s, rainbow, 3s` · `animate s, off`

**Operators:** `+ - * / %` `< <= > >= == !=` `&& || !` (or `and or not`). Division by zero = 0. `int` and `bool` never mix.

**Built-in commands:** `PING` `WHO` `STATUS` and, if used, `STATE` `LOG`. On a bus prefix every line with `@3 ` (`@0` = broadcast, nobody answers).

**Commands:** `check` `build` `cost` `link` `bus` `verify` `sim` `info` `flash` `targets` (run as `python3 -m rudrac ...`)

**Rules of thumb:** keep writing a motor in an `every` if it has a `timeout` · a stepper needs a position, not a speed · `send` on a bus only inside `on` ·
never rely on software alone for anything dangerous: add a hardware cut-off.

## Motion (v0.11)
```
path NAME for OUT { at POS [speed 1..1000] [zone 0..10000] [wait ms]  ...  repeat N | forever }
run NAME     stop OUT     done NAME { ... }     NAME  (bool: running)
```
Stepper: POS in steps. Motor: POS is a speed -100..100, only `wait` applies. A safe state or a direct write ends paths.

## Tools (v0.12)
`rudrac fmt F` tidy layout   `rudrac lsp` editor server   `rudrac conform` run the conformance suite   spec: SPEC.md
