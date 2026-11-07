# Rudra roadmap: from node script to a robot language

Model: KUKA KRL and ABB RAPID. Small, strict, readable by a technician, built around what the robot does, and
**provably bounded**: no surprise stalls, no heap, no recursion. Every stage ends with tests that compare the simulator
with the compiled output, and an honest README entry. Nothing is "done" until it is tested on a real board.

## Track A: the language

| Stage | What | Exit test |
|---|---|---|
| A0 (done, v0.4) | outputs with safe states, interlocks, states, timing words, motors, bus, black box | 101 tests |
| A1 (done, v0.5) | `func` (pure) / `proc`, typed, non-recursive, `return`, local `let` | recursion is a compile error; sim == C++ (116 tests) |
| A2 (done, v0.5) | `for i in 0..N` with constant bounds, `break` | a loop can never run unbounded; sim == C++ |
| A3 (done, v0.7) | fixed-size arrays `int[16]`, `struct`, `enum` | index proved in range or checked at run time and counted; sim == C++ (151 tests) |
| A4 (done, v0.6) | **worst-case prover**: max run time per handler and per main-loop pass, stack depth, RAM; refuse if over a declared `budget` | model-based; to be calibrated against real boards |
| A5 (done, v0.10) | `import` / shared pin maps and interlock sets | two nodes share one definition |
| A6 (done, v0.11) | motion words in the KRL/RAPID spirit: `move`, `speed`, `zone`, waypoints for steppers and motors | sequence tables, bounded |

## Track B: the compiler

| Stage | What | Exit test |
|---|---|---|
| B1 (done, v0.8) | one typed IR (`rudrac/ir.py`); simulator, C++ backend and prover all read it; `rudrac ir` prints it | one definition of "what the program means"; IR validator; golden-value tests (164 tests) |
| B2 (done, v0.9) | bytecode backend + tiny VM for Nano and ESP32; update logic over serial without reflashing | same programs, same results as C++ backend |
| B3 (done, v0.12) | formatter, language server (errors in the editor), `rudrac fmt` | formatter proves the tokens are unchanged, idempotent on every example; LSP session tested over a real pipe |
| B4 (done, v0.12) | written spec + conformance test suite | `SPEC.md` + 34 cases in `conformance/`; reference passes on simulator, VM, C++ and VM C++; expectations can fail (tested) |
| B5 (started, v0.14) | direct machine code (AVR first; Xtensa/RISC-V later); self-hosting | B5.1 done: own AVR assembler + HEX writer + ATmega328P emulator; a hand-written blink assembles to 282 bytes and runs in the emulator (flags checked exhaustively against the datasheet formulas). Waiting on: it blinking on a real Uno (`B5_TEST.md`). B5.2/B5.3 done (v0.14): runtime in assembly (32-bit math, 1 ms Timer0 clock, UART RX interrupt + line reader, PING/WHO/STATUS) and `rudrac cc`, IR to AVR code for digital outputs, `every`, `start`, `on` with arguments, `heartbeat`, `never`, `func`/`proc`, arrays, structs, `for`; seven conformance programs match the simulator on the emulator. Next: B5.4 inputs/`when`/states, then pwm/servo/motor/stepper/paths, blackbox, bus; later ESP32. |

## Order

A1 -> A2 -> A4 (the prover is the reason to exist) -> A3 -> B1 -> B2 -> A5/A6 -> B3/B4 -> B5.

## Rule that does not change

Real-board testing happens in parallel from now on: flash one node per stage and record what differed from the simulator.
