# Rudra language specification (v0.12)

This document says what a conforming implementation of Rudra must do. The reference implementation is `rudrac`
(this repository); where this text and the reference disagree, that is a bug in one of them and the conformance
suite in `conformance/` (see section 12) decides. The suite is a floor, not a proof: 34 cases cover each feature
but not every combination, and its expected outputs were produced by the reference simulator and then checked by
hand for the cases with arithmetic and interlocks.

Not in scope: the generated C++ text, the bytecode image format, the serial program-update protocol, `rudrac link`
clients, cost estimates. Those belong to the reference toolchain, not to the language.

## 1. Programs and nodes

A program is one file with at most one `node`, optionally preceded by `import` lines. A node describes one small
board (a target such as `nano` or `esp32`) that listens to a master over a serial line and drives hardware.
A node has no heap, no recursion, no unbounded loops, and its main loop never blocks on the program.

## 2. Lexical structure

* Source is UTF-8. Whitespace (space, tab, CR, LF) separates tokens and is otherwise insignificant.
  There are no statement terminators: a statement ends where the grammar says it can.
* Comments: `//` or `#` to the end of the line.
* Identifiers: `[A-Za-z_][A-Za-z0-9_]*`. Command and message names are identifiers or strings.
* Numbers: decimal, or hexadecimal `0x...`. Limit 2147483647. A number directly followed by a letter or digit is an error.
  `500ms` is a duration of 500, `2s` of 2000 (milliseconds). `50%` is a percentage literal (value 50).
* Strings: `"..."` on one line, no escapes.
* Word operators `and`, `or`, `not` are the same tokens as `&&`, `||`, `!`.
* Keywords are reserved. `path`, `run`, `stop`, `done`, `at`, `speed`, `zone`, `wait`, `repeat`, `forever`, `fill`,
  `pixel`, `animate` are contextual: they are ordinary identifiers outside the statement forms that use them.

## 3. Types and values

All values are 32-bit signed integers (`int`), booleans (`bool`), or members of an enum. Arithmetic wraps on
overflow (two's complement). `/` and `%` truncate toward zero; division or remainder by zero yields 0;
`INT_MIN / -1` wraps to `INT_MIN` and `INT_MIN % -1` yields 0. Comparison yields `bool`. `&&` and `||` short-circuit.
Booleans convert to 0 and 1 where an `int` is needed. Enums compare with `==` and `!=` only.
Conformance cases `p15` and `p16` pin these down.

## 4. Declarations

```
node NAME target BOARD {
  serial BAUD               address N            rs485 pin P        blackbox N        budget Tms
  const NAME = EXPR
  input  NAME: KIND(...)    output NAME: KIND(..., safe VALUE)
  var NAME [: TYPE] [= EXPR]            enum NAME { a, b }       struct NAME { f: TYPE, ... }
  func NAME(p: T, ...) -> T { }         proc NAME(p: T, ...) { }
  never RULE
  path NAME for OUT { at ... }
  start { }  every D { }  when COND [for D] { }  on CMD(args) { }  heartbeat D { }
  state NAME { enter { } exit { } every ... when ... on ... }
}
```

* A `const` is a compile-time integer or boolean. Declaration order does not matter for `func`/`proc`.
* Every output **must** declare `safe VALUE` (section 7). A program without one is rejected.
* `var` storage is static and starts at its initialiser, or 0 / false / the first enum member. Arrays `T[N]` have
  length 1..1024; structs hold ints, bools or enums and do not nest.
* `func` is pure: it may read inputs, outputs and variables and call other functions, but not write anything,
  send, or change state. `proc` may write variables and outputs and send. Neither may be recursive, directly or
  through other functions (call graph acyclic). Local `let` names and parameters are scoped to the body.
* `for i in LO..HI { }` has constant bounds (`LO` and `HI` constants, `HI` exclusive, at most 10000 iterations);
  `break` leaves the nearest `for`. There is no `while`.
* Array index: a constant or a `for` counter that provably fits is not checked. Any other index is checked at run
  time: a read outside returns 0 / false; a write outside is not performed, and is counted (section 6). A constant
  index outside the array is a compile error.

## 5. Execution model

The node is a single-threaded main loop over a millisecond clock `now`. In **one pass** of the loop, in this order:

1. Read serial; every complete line (up to 31 characters) is a command (section 6), run to completion in arrival order.
2. Sample inputs: debounce and I2C and sonar readings are updated.
3. Heartbeat check (section 7). If the node is in the failsafe state, steppers continue (section 8) and the pass ends.
4. In declaration order: each `every D` whose timer has elapsed, and each `when` condition, runs its body when due.
   Only the global handlers and those of the current state are active.
5. Paths (section 9).
6. Animations (every 20 ms), motor ramps and timeouts (every 20 ms), steppers.
7. Output writes accumulated during the pass reach the hardware.

Handlers run to completion; there is no preemption and no handler is interrupted by another.
`every D`: fires when `now - last >= D`; `last` then advances by `D`, or jumps to `now` if two or more periods were missed.
`when COND { }` runs once on the false-to-true change of `COND`; with `for D` it runs once after `COND` has been
continuously true for `D` ms, and re-arms when `COND` goes false.
`start { }` runs once at boot, after the safe states are applied.
Inputs: a digital input declared `debounce D` changes its visible value only after the pin has been steady for `D` ms;
`invert` flips it. An unreadable I2C input keeps its last value.

**States.** The first `state` is the start state. `goto S` must be the last statement of its block, and is not allowed in
`start`, `enter` or `exit`. Leaving a state runs its `exit`, entering runs `enter`; entering restarts the state's timers.
`goto` to the current state does nothing. A state's own `on` handler wins over a global one of the same name.

## 6. The master link

Lines of text. A command is `NAME` or `NAME a b c` (space-separated decimal integers; a token that is not a number
reads as 0). Behaviour:

* A command with a handler and enough arguments runs it (extra arguments are ignored) and counts as hearing the master.
* Too few arguments, an unknown name, or a name with no handler in the current state: the node transmits `ERR`.
* Built in, never overridden: `PING` answers `PONG`; `STATUS` answers `STATUS n` where `n` counts writes refused by
  `never` rules plus refused out-of-range array writes; `STATE` (only with states) answers `STATE i`, `i` the index of the
  current state; `LOG` (only with `blackbox`) answers one `LOG ms code arg` line per entry then `LOGEND total`;
  `WHO` answers `WHO h`, where `h` identifies the node's protocol (its value is implementation-defined and is not tested).
* `send "NAME", v` transmits `NAME v`; `send "NAME"` transmits `NAME`. A message name is used either with or without a value.
* A node with an `address` shares a bus: every command must be prefixed `@N ` (a line for another address is ignored), and `@0` is a broadcast that nobody answers. `rs485 pin P` names the transceiver driver-enable pin, which is high while the node transmits.

## 7. Safe states, heartbeat, interlocks

* On boot, before anything else, every output is driven to its `safe` value.
* `heartbeat D` (milliseconds; default none): if no valid command (including `PING`) has been heard for `D`, the node applies
  every safe state **atomically** (all stored first, then written), runs the heartbeat body once, and **freezes**:
  no `every`, `when`, path or animation runs until the next valid command. Steppers keep stepping towards their safe position.
  The next valid command ends the failsafe; handlers resume.
* `never RULE` is a boolean expression over outputs and constants. It is checked at compile time against the safe state
  (a safe state that breaks a rule is an error) and at run time on **every** write to an output: a write that would
  make any rule true is **not performed**, the output keeps its value, and the refusal is counted in `STATUS`.
  The check applies to every route to the output: direct writes, paths, animations of the output's value.

## 8. Outputs

| kind | value | hardware lines (`phys`, see section 11) |
|---|---|---|
| `digital` | bool | `W ms pin 0/1` (inverted if `invert`) |
| `pwm` | 0..100 (clamped) | `W ms pin duty`, duty = value*255/100 (255-duty if `invert`) |
| `servo` | 0..180 (clamped) | `S ms pin angle` |
| `tone` | Hz 0..20000, 0 = silent | `T ms pin hz` |
| `motor` | -limit..limit | direction line, then duty line (`W`), per style |
| `stepper` | target position in steps | per step: `W ms dir 0/1`, `W ms step 1`, `W ms step 0` |
| `i2c` | integer | `I ms addr hexbytes` |
| `neopixel` | (via `fill`, `pixel`, `animate`) | `N ms rgb rgb ...` (one packed 0xRRGGBB per pixel, in decimal) |

Every write that is not refused produces its hardware line, even when the value is unchanged. A NeoPixel strip produces at
most one `N` line per pass, after all that pass's `fill`, `pixel` and animation changes (an out-of-range pixel index is ignored).
Motor `ramp N` limits the change to N percent per second in 20 ms steps and always passes through 0 on a reversal;
`timeout T` returns the motor to `safe` when the program has not written it for `T`.
Stepper: boot position 0; one step per `1000 / rate` ms (integer division, minimum 1) towards the target.
Input kinds: `digital`, `analog`, `i2c`, `ultrasonic` (centimetres, 0 for no echo; measured every 60 ms).

## 9. Paths

```
path NAME for OUT { at POS [speed S] [zone Z] [wait T]  ...  [repeat N | forever] }
run NAME     stop OUT     done NAME { }     NAME  (bool)
```

Limits: 1..64 `at` lines; `speed` 1..1000 steps/s; `zone` 0..10000 steps; `wait` 0..3600000 ms; `repeat` 1..1000.
Motor paths take only `wait`; their `POS` is a speed -100..100. `done` is an error on a `forever` path and allowed once per path.

`run P` stops any path on the same output and starts `P` at waypoint 0. Entering a stepper waypoint writes its position as the
target and sets the step interval to `max(1, 1000 / speed)` ms (the declared rate when `speed` is absent); entering a motor
waypoint writes its speed and starts its hold. A stepper waypoint is done when `|position - POS| <= zone` (measured against
the waypoint, not the target); then it holds for `wait` ms (if > 0) and moves on. After the last waypoint the path repeats
(`repeat N` times in all, or forever); when it ends the declared step interval is restored and the `done` body runs.
`stop OUT` ends paths on `OUT`: a stepper keeps its position (target := position), a motor goes to `safe`. A direct write
to `OUT` ends its paths. A safe state ends all paths. A write refused by an interlock leaves a stepper path waiting for a
position it will not reach, and a motor path continues by time.

## 10. Imports

`import "file.rudra"` (a path relative to the importing file) brings in `const`, `enum`, `struct`, `func`, `proc` and
`never` items from a library file, which may import others. A file imported twice is read once; cycles are errors.
Unused library functions do not warn.

## 11. What is observable

A conforming run of a program against a scenario is its sequence of **hardware lines** and **transmitted lines**:

* hardware line: `TAG ms args...` as in section 8, in the order the writes happen; boot lines are at `ms = 0`.
* transmitted line: `ms TEXT`.

A scenario is a run length in ms and a list of events in time order: `rx` (a command line arrives), `set` (an input
changes: digital as true/false, analog/sonar as integer, i2c as the register's integer value), `fail` (an I2C input
stops or resumes answering). Events at time `t` are applied before pass `t`, and pass `t` is run for every `t` from 0 to the run length inclusive.

## 12. Conformance

`conformance/NAME/{prog.rudra, scenario.json, expect.json}`. `expect.json` either lists the exact hardware and transmitted lines
(`phys`, `tx`) a conforming run produces, or lists the diagnostics (`errors`: line, level, a fragment of the message) a
conforming compiler must report for a program it rejects. Diagnostic wording beyond the fragment is not specified; the
line is, and so is the set of errors and warnings (one to one).
Run `python3 -m rudrac conform` to check the reference implementation four ways: simulator, simulator driven by the
bytecode VM, generated C++ and generated `--vm` C++ (needs `g++`; compiled against a fake Arduino core in `tests/stub`).
No real board has been involved.

## 13. Compile-time guarantees

A conforming compiler rejects: use of an undefined name; assignment to an input or constant; type mismatches; recursion;
constant array indices out of range; two uses of one pin; a pin the board does not have or cannot use for the role;
an output without a `safe` state; a safe state that breaks a `never` rule; paths and path options outside their limits;
`done` on a `forever` path; and, when a `budget` is declared, a main-loop pass whose worst-case estimated time exceeds it.
The time estimate is model-based and approximate; the budget check is a guard, not a certification.
