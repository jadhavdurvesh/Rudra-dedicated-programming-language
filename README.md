# Rudra / `rudrac` v0.6

One-page summary of the whole language: [CHEATSHEET.md](CHEATSHEET.md).

A small language for **FireBot slave nodes**: Arduino Nano / Uno, ESP32, ESP32-S3 boards that drive LEDs, relays, fans,
servos, DC motors, steppers, NeoPixel strips and read simple sensors. You write a `.rudra` file; `rudrac` checks it, and generates a normal
Arduino-framework C++ sketch **and** the matching code for the master board. Nothing runs on the node except plain compiled C++.

```
                         +--> <node>.ino        (flash to the slave with arduino-cli)
.rudra --> rudrac -------+--> <node>_link.h     (main board, C++)
 (check, prove)          +--> <node>_link.py    (PC / Raspberry Pi side)
                         +--> <NODE>_PROTOCOL.md
                         +--> rudrac sim        (same logic on your PC with virtual time)
                         +--> rudrac verify     (random-tests the script against its interlocks)
```

Arduino/ESP-IDF stays underneath as the hardware layer. Rudra replaces only the code you write.
It is **not** meant for the main boards, only for small slaves that obey the master over serial.

## Why not just C++ or Rust?

For the logic itself, C++ and Rust are as capable as Rudra, and for anything big they are better: Rudra has no loops, arrays or
functions on purpose. Rudra only earns its place by giving you things you would have to build, by hand, in every project,
with nothing forcing you to remember them. Those are the five ideas:

| | what Rudra does | in C++ / Rust |
|---|---|---|
| **Safe state** | every output *must* declare one; the compiler refuses the file otherwise. Applied at boot and when the master goes quiet | you can write it; nothing makes you |
| **Interlocks** | `never heater and not fan`: the compiler **proves** the safe state obeys it, and every write is checked at runtime, so no code path (boot, failsafe, any handler) can break it | you can write guards; nothing proves them against the failsafe path |
| **Master in sync** | commands, baud rate, keep-alive time and a protocol fingerprint are generated for the master from the same file; the node answers `WHO` so a stale master is detected | written twice by hand |
| **Find the bug before you flash** | `rudrac verify` fuzzes the script with random commands, inputs and silences and reports which path *tried* to break a rule | you write the fuzzer |
| **Domain checks** | pin that cannot do PWM, servo vs PWM timer clash on AVR, ESP32 flash pins, I2C pins used as GPIO, NeoPixel update longer than one serial byte at your baud rate | datasheet reading |

What Rudra costs you: generality, and the maturity of a language with millions of users. It is a young compiler with a small test suite,
not a certified toolchain. Use plain C++ or Rust for the main boards and for anything computational. On a Nano, Rust is, as far as I know,
still experimental (nightly toolchain), and MicroPython does not run there at all. Rudra generates plain Arduino C++, so it runs on both.

**Important:** a software interlock is not a substitute for a hardware one. For anything that can hurt someone, keep a physical cut-off
(thermal fuse, relay wired so it cannot close) in addition. Rudra reduces mistakes in the software; it does not certify the machine.

## Quick start

```
python3 -m rudrac check  examples/heater_nano.rudra
python3 -m rudrac verify examples/heater_buggy.rudra        # finds the bug (exit code 1)
python3 -m rudrac sim    examples/scout_esp32.rudra --run 6s --send "1s:ALERT" --send "3s:RAINBOW"
python3 -m rudrac cost   examples/sentry_nano.rudra          # worst-case time per handler, loop pass, stack, RAM
python3 -m rudrac info   examples/scout_esp32.rudra         # wiring, commands, messages, libraries, fingerprint
python3 -m rudrac build  examples/scout_esp32.rudra         # build/scout/scout.ino
python3 -m rudrac link   examples/scout_esp32.rudra         # master-side header, Python client, protocol doc
python3 -m rudrac bus   examples/turret_esp32.rudra examples/other.rudra   # one master for nodes sharing a wire
python3 -m rudrac flash  examples/scout_esp32.rudra -p COM3 # needs arduino-cli
```
`pip install -e .` gives a plain `rudrac` command. Python 3.9+, no dependencies. Nano clones with the old bootloader: `--old-bootloader`.

## Interlocks (`never`) and `verify`

```
output heater: digital(pin 4, safe false)
output fan:    digital(pin 5, safe true)
never heater and not fan                       // the heater may never run while the fan is off
```

* A rule may use **outputs and constants only**, so the compiler can check it against the safe state. If the safe state breaks a rule, that is an error.
* Every output write goes through a check. A write that would make a rule true is **refused** (the output keeps its old value) and counted.
  The master can read the count with `STATUS`. The safe state is applied atomically (stored first, written second), so it never depends on output order.
* Refusing silently is not enough, so `rudrac verify` looks for scripts that *try*:

```
$ rudrac verify examples/heater_buggy.rudra
  FOUND: the script tries to break rule 1:  never heater and not fan
     the write 'fan = 0' was refused in 39 of 100 runs (first: run 3, t=567ms)
     what led up to it:
        t=176ms  master sends  FAN 100
        t=567ms  master sends  FAN -1
```
It is random testing, not a proof: it can show a problem, never that there is none. (It found a real bug in the first version of `heater_nano.rudra`:
the guard was `v == 0`, but `FAN -1` also turns the fan off.)

## The language

One node per file. Comments start with `//` or `#`.

```
node scout target esp32 {            // target: nano | uno | esp32 | esp32s3
  serial 115200                      // link speed to the master (default 9600)

  const HOT = 3200                   // compile-time constants
  var   sweeping = false             // runtime variables (initialiser must be constant)

  // ---- outputs: every one needs `safe ...`
  output led:   digital(pin 5, safe false)           // flag: invert (active-low)
  output fan:   pwm(pin 9, safe 100)                 // 0..100 percent
  output head:  servo(pin 25, safe 90)               // 0..180 degrees
  output ring:  neopixel(pin 26, count 8, safe 255, 0, 0)   // or  safe off ;  flag: rgb (default GRB)
  output relay: i2c(addr 0x20, reg 9, bytes 2, safe 0)      // writes a register, big-endian

  // ---- inputs
  input btn:  digital(pin 27, pullup, invert)
  input temp: analog(pin A0)                         // always 0..1023
  input t2:   i2c(addr 0x48, reg 0, bytes 2, signed) // reads a register, big-endian

  never led and fan > 50                             // interlock (outputs and constants only)

  // ---- code
  start { }                          // once at boot
  every 100ms { }                    // periodic (500ms, 2s, or a const)
  when btn { }                       // when a condition turns false -> true
  on PAN(a) { head = a }             // command from the master: "PAN 45"
  on COLOR(r, g, b) { fill ring, r, g, b }
  on ALERT { animate ring, pulse, 255, 0, 0, 800ms }
  heartbeat 2s                       // safe states after 2 s of silence (a body { } is optional)
}
```

Statements: `name = expr`, `if cond { } else if cond { } else { }`, `send "NAME", expr` (the `, expr` is optional),
`fill strip, r, g, b`, `pixel strip, i, r, g, b`, `animate strip, mode, ...`.
Operators: `+ - * / %`, `< <= > >=`, `== !=`, `&& || !` (or `and or not`). Division by zero gives 0.
Types: `int` (32-bit) and `bool`. They never mix silently: `led = 5` and `if n { }` are errors.

| kind | value |
|---|---|
| `analog` input | 0..1023 on **every** board (ESP32's 12-bit ADC is scaled down) |
| `pwm` output | 0..100 percent (clamped) |
| `servo` output | 0..180 degrees (clamped) |
| `neopixel` colours | 0..255 each (clamped). One `show()` per loop, never per statement |
| `i2c` | `bytes` 1..4, big-endian. A failed read returns the **last good value** |
| `digital` | `true` / `false`; `invert` flips the pin level for you |

### Animations

```
animate ring, blink,   255, 0, 0, 500ms     // whole strip on for half the period, off for the rest
animate ring, pulse,   255, 0, 0, 800ms     // breathing: brightness 0..100..0
animate ring, chase,     0, 120, 255, 600ms // bright head with a dim tail, once around per period
animate ring, rainbow, 3s                   // colour wheel spread over the strip, rotating
animate ring, off                           // stop, leaving the strip as it is
```
Animations run by themselves (a frame every 20 ms, drawn from elapsed time so they cannot drift) and need no `every` timer.
`fill` and `pixel` stop an animation. Periods are 40 ms to 1 hour (shorter literal periods are a compile error, runtime values are clamped).
The safe state also stops them. Frames use integer maths only, identical in the simulator and the generated C++.

## v0.6: the worst-case prover (stage A4)

Because Rudra has no recursion, only bounded loops and no heap, every handler has a worst case that can be computed *without running it*.
`rudrac cost node.rudra` prints it, and a node can refuse to build if it is too slow:

```
node sentry target nano {
  budget 10ms          // the longest this node may go without reading its serial port
  ...
}
error: the main loop can take up to 25.3 ms per pass, but the budget is 10 ms
  hint: biggest contributors: ultrasonic 'sonar' waiting for an echo (every 60 ms): 25.1 ms; on GET (slowest command): 181 us; ...
```
* **Time**: per handler (loops multiplied by their bounds, the worse branch of every `if`, called functions, the interlock check on every output write, state changes)
  and per **main-loop pass** = serial poll + timers + sensors + NeoPixel `show()` + every timer handler firing together + the slowest command. That pass is how long a command or heartbeat can wait.
* **Memory**: static RAM by item and the worst-case **stack** (known exactly because the call graph is acyclic). On a Nano an estimate above 100% of 2048 bytes is an error, above 75% a warning.
* A warning if the heartbeat is shorter than twice the worst pass (the node could trip its failsafe merely from being busy).
* **What this is and is not.** The numbers come from a cost *model* (microseconds per operation for an AVR-class and an ESP-class board, in `rudrac/cost.py`), not from your board.
  Calibrate with `rudrac cost file --model my_board.json` (e.g. `{"dwrite": 4, "aread": 112}`) after timing real code with `micros()`. It ignores interrupts, a full serial transmit
  buffer (which blocks `print`), library internals it does not model, and the fact that one pass may handle several buffered commands. It is a design aid, not a certified bound.

## v0.5: functions, locals and loops (stage A1/A2 of [ROADMAP.md](ROADMAP.md))

The model is ABB RAPID's `FUNC` / `PROC` split, which also makes the C++ and the simulator agree by construction:

```
func clamp(v: int, lo: int, hi: int) -> int {      // func: computes a value, changes NOTHING
  if v < lo { return lo }
  if v > hi { return hi }
  return v
}
proc blink(n: int) {                                // proc: may change outputs and variables, returns nothing
  for i in 0..6 {                                   // bounded loop: constant bounds, stops before the last number
    if i >= n { break }
    led = !led
  }
}
on SUM(n) { let s = 0  for k in 1..9 { s = s + k * k }  send "TOTAL", clamp(s, 0, n) }
```
* A `func` can read inputs, outputs, vars and consts but cannot assign them, `send`, touch a strip or `goto`. So a call inside an expression has no side effects and the order of evaluation can never matter. A `proc` is called as a statement.
* **No recursion** (also not through other functions): the compiler must know how deep the call stack can get. Error names the cycle.
* Every path of a `func` must `return`. `let x = ...` makes a local, visible to the end of its block. Names may not hide a global, a parameter or another local.
* Loops need constant bounds and run at most 10000 times; `goto` is not allowed inside loops or procs. Loop counters cannot be assigned.
* `never` rules and `const`s cannot call functions (the compiler has to evaluate them against the safe state).
* Arithmetic is now defined to wrap at 32 bits in the generated C++ (`+ - *` and negation go through `rt_add/rt_sub/rt_mul/rt_neg`), and `-2147483648 / -1` no longer crashes: v0.4 emitted plain C++ `int` maths there, which is undefined behaviour on overflow.

## v0.4: behaviour words

All five are compiled to plain C++ (no heap, no loops) and checked against the simulator step for step.

### States

```
state idle    { enter { drive = 0 }   on GO(speed) { drive = speed  goto driving } }
state driving { every 400ms { drive = drive }   when bump for 50ms { goto backing }   exit { send "LEFT" } }
```
The first state is the start state. `every`, `when` and `on` inside a state only run while the node is in it; ones outside every state are global.
For one command name a state's own handler wins over the global one; a command nobody handles in the current state is answered `ERR`.
`goto` must be the last statement of a block and is not allowed in `start`/`enter`/`exit`. `goto` the current state does nothing.
Entering a state restarts its timers. The compiler warns about states nothing enters and states nothing leaves. `STATE` asks the node where it is.

### Timing words

* `when cond for 500ms { }`: runs once, after `cond` has been true for 500 ms without a break; re-arms when it goes false.
* `input b: digital(pin 2, pullup, invert, debounce 30ms)`: the value the program sees changes only after the pin has been steady for 30 ms.

### Motors and steppers

```
output drive: motor(pwm 5, dir 4, ramp 200, limit 80, timeout 1s, safe 0)    // or  motor(fwd 5, rev 6, ...)
output pan:   stepper(step 8, dir 9, rate 400, safe 0)
```
* **motor**: value `-limit..limit` percent. `ramp N` changes the speed by N percent per second at most (finest: 50), always passing through 0
  when the direction flips. `timeout 1s`: if the program stops *writing* the motor for that long it goes to its safe value by itself (a dead-man's switch, so keep writing in an `every`).
  `limit` clamps. Without `ramp`, a reversal is a hard reversal: that is your H-bridge's problem, not hidden.
* **stepper**: the value is a **target position** in steps (boot position = 0); the node steps towards it at `rate` steps/s (1..1000, interval is rounded down to whole ms).
  Its `safe` value is a position too: on link loss it keeps stepping at that bounded rate back to it (it is the one thing that keeps running after the link is lost).
* Both obey `never` rules (the rule sees the target) and are proved against at the safe state like every other output.

### Sensors and a buzzer

```
input  sonar: ultrasonic(trig 7, echo 8)     // HC-SR04 style, centimetres, measured every 60 ms; 0 = no echo within ~4 m
output horn:  tone(pin 6, safe off)          // value = frequency in Hz (0..20000), 0 = silent
```
A measurement can block the loop for up to 25 ms (it waits for the echo), so on a node that also answers a bus, expect that much command latency while something is out of range.
On AVR, `tone` takes over Timer2: PWM on pins 3 and 11 is refused by the compiler. See `examples/sentry_nano.rudra`.

### Shared bus

```
node turret target esp32 { address 3   rs485 pin 4   ... }
```
Every line to the node starts with `@3 `; `@0` is the broadcast (everybody obeys, **nobody answers**, so replies never collide). A node speaks only when asked:
`send` is only allowed inside `on` handlers on a bus. With `rs485 pin N` the driver-enable pin is raised for exactly each reply. `rudrac bus a.rudra b.rudra`
generates one Python master for all nodes (duplicate addresses, missing addresses and mixed baud rates are errors) and `BUS.md`.

### Arrays, structs and enums
```
enum Mode { idle, scan, fire }          // a type of its own: compare with == and !=, no arithmetic, no ordering
struct Point { x: int, y: int, hot: bool }   // fields are int, bool or an enum; no nesting
var mode: Mode = Mode.idle
var buf: int[8]                         // fixed length 1..1024, everything starts at 0 / false / first member
var pts: Point[3]
on FILL { for k in 0..8 { buf[k] = k * k } }   // k is proven in range: plain indexing, no check
on SET(i, v) { buf[i] = v }                    // i comes from outside: checked at run time
on P(i, a) { pts[i].x = a  pts[i].hot = a > 5 }
```
All storage is static (structs become one slot per field), there is no allocation and no pointers.
An index is either **proved** in range by the compiler (a constant, or a `for` counter whose bounds fit) or **checked at run time**:
a read outside returns 0/false; a write outside is refused, counted in `STATUS` (together with writes refused by `never`) and, with `blackbox`, logged as code 6 "bad index".
A constant index outside the array is a compile error. Arrays and struct fields cannot appear in `never` rules, and a `func` cannot write them.
Only global `var`s can be arrays or structs (not `let`, parameters or results).

### Black box

`blackbox 16` keeps the last 16 events in RAM: boot, link lost, state changes, refused writes (with the rule number), motor timeouts. The master reads it with `LOG`
(`LOG <ms> <code> <arg>` lines, then `LOGEND <total>`); the generated clients have `log()` / `black_box()`. It is lost on reset or power loss: a flight recorder for the
last few seconds before a failsafe, not a permanent record.

## Talking to the master

Plain text lines over serial, one per line: `NAME` or `NAME 1 2 3`.

* `on NAME(a, b) { }` handles a command. Too few arguments or an unknown command: the node replies `ERR`. Extra arguments are ignored.
* `send "NAME", value` writes `NAME 42`. A message is always sent either with or without a value, never both (the compiler checks).
* Built in: `PING` (answers `PONG`, resets the heartbeat), `WHO` (answers `WHO <fingerprint>`), `STATUS` (answers `STATUS <refused writes>`), and when declared `STATE` and `LOG`.

`rudrac link` generates everything the master needs:

```cpp
#include "scout_link.h"                     // main board (Arduino C++)
scout_ping(Serial1);                        // call at least every SCOUT_KEEPALIVE_MS
scout_COLOR(Serial1, 0, 255, 0);
scout_msg m;  if (scout_parse(line, &m) && !strcmp(m.name, SCOUT_MSG_TEMP)) use(m.value);
```
```python
import serial, scout_link                   # PC / Raspberry Pi
link = scout_link.ScoutLink(serial.Serial("/dev/ttyUSB0", scout_link.BAUD, timeout=0.2))
assert link.verify()                        # node runs the firmware this client was generated for
link.cmd_PAN(45);  link.ping();  print(link.read_message())   # ('PONG', None)
```
The fingerprint covers the node name, baud rate, heartbeat, and every command and message with its argument count. Change any of them and
`verify()` fails until the master is regenerated and the node reflashed.

On a Nano the link is hardware serial pins 0/1, shared with the USB port. **Unplug the master's wire while uploading.**

## Heartbeat and safe states

Any valid command (or `PING`) from the master resets a timer. If it runs out, **every output is put in its declared safe state**, then the optional
`heartbeat` body runs once, and the node **freezes there**: `every` / `when` handlers and animations stop, so nothing can undo the failsafe.
The next valid command brings everything back. Safe states are also applied at boot, before anything else runs.

## What the checker catches

Unknown names (with "did you mean"), type mismatches, writing to inputs/consts, non-constant `const`s, duplicate pins,
pins that do not exist or cannot do the requested job (PWM on a non-PWM pin, `A6`/`A7` as digital, ESP32 flash pins, input-only GPIOs with `pullup`,
pin 0/1 on Nano), reserved words, duplicate commands and blocks, bad durations, **outputs without a safe state, nodes with outputs and no heartbeat,
a safe state that breaks a `never` rule, rules that use inputs or variables**, safe values out of range, pixel indexes outside the strip, bad animation periods,
servo vs. PWM timer conflicts on AVR, I2C pins used as GPIO, messages sent in two formats.
Warnings: boot-strapping pins, unused inputs/outputs, `heartbeat off`, and on AVR a NeoPixel update that blocks interrupts for longer than one
serial byte lasts at your baud rate (a command arriving during an update can be lost; e.g. more than 32 pixels at 9600 baud, even 1 pixel at 115200).

## Arduino libraries needed (`rudrac info` lists them)

NeoPixel: *Adafruit NeoPixel*. Servo: *Servo* (built in on AVR), *ESP32Servo* on ESP32.
`arduino-cli lib install "Adafruit NeoPixel" ESP32Servo`

## One IR, three readers (v0.8)

After the checker accepts a program, `rudrac/ir.py` lowers every body **once** into a small typed IR: names resolved (local / global slot / input / output),
constants and enum members folded to numbers, structs flattened to slots, every expression typed `int` or `bool`, and every array access marked
*proven* or *checked at run time*. The simulator, the C++ generator and the worst-case prover all walk that same IR, so they cannot disagree about what a
program means. `python3 -m rudrac ir file.rudra` prints it, and `ir.validate()` re-checks its invariants.

One consequence worth knowing: because sim and C++ now share the lowering, comparing them can no longer catch a mistake *in* the lowering. That is why
`tests/test_b1.py` also pins golden values (`K + 1`, wraparound, division by zero, `-7 % 3`, out-of-range reads) that were worked out independently.

## Tests

```
python3 -m unittest discover -s tests -v
```
164 tests. The heavy ones compile the **generated C++** with `g++ -Wall -Wextra -Werror` against a fake Arduino core (digital/PWM, servo, I2C, NeoPixel, serial),
replay seconds of inputs and commands, and compare every hardware effect and serial line **with its timestamp** against the simulator,
for all examples, all four animation modes on Nano and ESP32 targets, the interlock examples, and every v0.4 feature (debounce, `when..for`, scoped states, motor ramp/limit/timeout, stepper, bus addressing and RS485 pin, black box).
To check the tests are not vacuous, deliberate breakages were injected (animation maths, the interlock check, and eight v0.4 ones such as an off-by-one in debounce, ramp not passing through zero, broadcast answering, `goto` same state re-entering): all are caught.
The generated master-side C++ header is compiled and run, and the generated Python clients (single node and shared-bus master) are driven against simulated nodes on one wire, including the `WHO` handshake and a black-box readout.

## Limits of v0.4 (honest list)

* **Not tested on real boards yet.** The fake-board tests are strict, but they model what the code *asks* the hardware to do, not hardware behaviour:
  real servo/NeoPixel libraries, I2C timing, serial overruns and AVR memory limits are not covered. Flash one node and watch it before trusting it with anything that matters.
* **Not a certified safety tool.** No formal verification of the compiler, no safety standard. `verify` is random testing. Keep hardware cut-offs for anything dangerous.
* An interlock is only as good as the rule you write, and it cannot know about wiring, power or the real world.
* The serial-overrun warning is arithmetic from byte time and a 30 us/pixel update; the real threshold depends on the board's UART buffering.
* ESP32 PWM uses `analogWrite`, which needs **arduino-esp32 core 3.x or newer**.
* Timing is cooperative (`millis()` based, no interrupts). A handler must not block. Good for lights, relays and servos, not for microsecond work.
* No arrays, loops or functions on purpose: every handler runs in bounded time and uses no heap.
* Motors and steppers: the **generated pin sequences** are tested, not real H-bridges, drivers or step loss. The stepper pulse uses a 4 us `delayMicroseconds`; check your driver's minimum pulse width.
  Stepper position is open-loop and starts at 0 at every boot.
* The black box lives in RAM only. The bus is a text protocol with no checksum and no collision detection beyond "nodes only speak when asked": add a checksum or a proper bus (CAN) for noisy environments.
* v0.4 also fixed a v0.3 bug: handlers read `millis()` themselves while the loop used an earlier reading, so a 1 ms skew could wrap an unsigned subtraction (a false heartbeat trip or a garbled animation frame). Every timer now uses one clock reading per loop pass.
* No SPI, UART peripherals or non-register I2C devices (displays, streaming IMUs) yet. ESP32-S3 pin rules are deliberately conservative.
* The RAM and timing figures (`rudrac info`, `rudrac cost`) come from a model, not a measurement; see the v0.6 section for what it ignores.
