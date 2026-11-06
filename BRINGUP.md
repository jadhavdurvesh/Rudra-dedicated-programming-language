# First time on a real board: bring-up checklist

Nothing in Rudra has run on a real board yet. This is the plan for the first run, one node, one feature at a time, and
for recording what differs from the simulator. Do it with **loads disconnected** (motors, pumps, heaters, valves off the
driver, a LED or a multimeter on the pin instead) until step 6.

Tools: the board, USB cable, `arduino-cli` (for `rudrac flash`) or the Arduino IDE, a multimeter, and if you have one a
logic analyser or a cheap oscilloscope. A serial monitor at the node's baud rate.

## 0. Before you plug anything in

1. `python3 -m rudrac check node.rudra` and `python3 -m rudrac cost node.rudra` (the time numbers are model estimates:
   write down the worst-case pass it reports, you will compare it in step 5).
2. `python3 -m rudrac verify node.rudra` if the node has `never` rules.
3. `python3 -m rudrac build node.rudra` and open the sketch once, so you know what you are flashing.
4. On a Nano the serial pins 0/1 are shared with USB: **unplug the master's wire while uploading.**

## 1. Flash a minimal node first

Use `examples/led_node.rudra` (Nano) or `examples/scout_esp32.rudra` (ESP32) before your own program, so a failure is the
toolchain and not your code. `python3 -m rudrac flash examples/led_node.rudra -p /dev/ttyUSB0` (`--old-bootloader` for Nano clones; see `rudrac flash -h`).
Expected: the safe state appears on the pins at power-up (LED at its safe level), before any command.

## 2. Talk to it (serial monitor, newline-terminated lines)

| Send | Expect | Simulator says |
|---|---|---|
| `PING` | `PONG` | same |
| `WHO` | `WHO <number>` | `python3 -m rudrac sim node.rudra --run 100ms --send 10ms:WHO` prints the same number |
| `STATUS` | `STATUS 0` | same |
| `NOPE` | `ERR` | same |

If `WHO` differs from the simulator's, you flashed a different build than the one you checked.

## 3. One output at a time

For each output, run the matching simulator script and watch the pin:
`python3 -m rudrac sim node.rudra --run 3s --send 100ms:ON --send 1s:OFF` and send the same lines at the same times by hand.

| Output | Check | Record |
|---|---|---|
| digital | level on the pin matches `safe`, then follows commands; `invert` flips it | pass / fail |
| pwm | multimeter or scope: duty = value % (255 scale on AVR) | |
| servo | horn moves to `safe` angle at boot, clamps at 0..180 | |
| motor (driver, **no motor**) | PWM duty and direction pin as in the simulator log; with `ramp` the duty steps every 20 ms | |
| stepper (driver, **no motor first**) | step pulses on the step pin at `rate` steps/s; count them for a known move (`AIM 100` = 100 pulses) | |
| neopixel | colours and animation look right; first pixel order (GRB/RGB) matches | |
| i2c | `rudrac sim` prints the bytes; check them on an analyser, or that the device acts | |
| tone | frequency on the pin (phone tuner app is enough) | |

## 4. Failsafe (the reason Rudra exists)

1. Send a command that turns an output away from its safe state.
2. Stop talking. After exactly `heartbeat` the output must go to `safe` and `every`/`when` must stop. Time it with a stopwatch or the analyser; record the real delay.
3. Send `PING`: the node must come back. 
4. Pull the USB/serial wire while a stepper is mid-move: it should keep stepping to its safe position.
5. Power-cycle: pins must be at safe state before the first command.
6. For each `never` rule: try to break it by command (the node must refuse and `STATUS` must count up).

## 5. Timing: where the model may be wrong

The worst-case numbers are estimates from a cost model, not measurements. To calibrate: toggle a spare pin at the start and end of
the slowest handler (temporarily, by hand in the generated sketch), measure the pulse width on a scope, and compare with the `rudrac cost`
line for that handler. If the measurement exceeds the model, edit the model file (`rudrac cost --model FILE`, a JSON file such as `{"dwrite": 4}`) and rerun `cost`
so `budget` checks use real numbers. Do the same for the full pass (the longest gap between two serial reads).

## 6. Loads, last

Connect the real motor/pump/heater only after steps 3 and 4 pass for that output, with a current-limited supply the first time,
and keep a hardware cut-off (fuse, thermal switch, relay with its own watchdog) in the loop: the node's failsafe is software.

## 7. Record what differed

For every row that is not "pass", write one line: board, feature, what the simulator said, what the board did. Those lines are the
bug reports; send them back and each becomes a conformance case in `conformance/` so it cannot regress.

```
board:      (Nano / ESP32 variant, clock)
rudrac:     (version from SPEC.md / README, e.g. 0.12.0)
program:    (file, and the WHO number)
feature:    
simulator:  
board:      
cause:      (if known)
```
