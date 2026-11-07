# Uno test, step by step (Windows; Linux/Mac differences in brackets)

Files you need: `rudra_v0.12.zip` (everything), and this page. Nothing else is required for Parts A to C.
You need: an Arduino Uno, a USB cable, a phone stopwatch. No wires, no parts: the LED is the one on the board (marked L).

## Part A: set up (once, ~15 minutes)

1. **Python 3.** Open the Start menu, search "Microsoft Store", install "Python 3.12". Check: open PowerShell and type `python --version`. It should print 3.x.
   [Mac/Linux: use `python3` everywhere below instead of `python`.]
2. **arduino-cli.** In PowerShell: `winget install ArduinoSA.CLI`. Close and reopen PowerShell. Check: `arduino-cli version`.
   [Mac: `brew install arduino-cli`. Linux: see arduino.github.io/arduino-cli/latest/installation]
3. **Uno support (once):**
   ```
   arduino-cli core update-index
   arduino-cli core install arduino:avr
   ```
4. **Unzip** `rudra_v0.12.zip` (right-click, Extract All). Open PowerShell in the new `rudra` folder: in File Explorer open the folder, click the address bar, type `powershell`, press Enter.
5. **Check Rudra runs:** `python -m rudrac check examples/blink_uno.rudra`. Expected last line: `examples/blink_uno.rudra: ok (1 warning)` (the warning about `heartbeat off` is expected).

## Part B: put it on the Uno (~5 minutes)

6. Plug in the Uno. Find its port: `arduino-cli board list`. You will see a line like `COM3  Serial Port (USB)  Arduino Uno`.
   [Linux `/dev/ttyACM0`, Mac `/dev/cu.usbmodem...`]
7. Build and upload (use your port instead of COM3):
   ```
   python -m rudrac flash examples/blink_uno.rudra -p COM3
   ```
   It compiles, then uploads. Success ends with something like `avrdude done. Thank you.`
   If it fails, copy ALL the text and send it to me (see Part D). Common fix: close the Arduino IDE serial monitor, which holds the port.
8. **Look at the board.** The small LED marked **L** next to pin 13 should blink: 0.5 s on, 0.5 s off.

## Part C: the tests

Open the serial monitor: `arduino-cli monitor -p COM3 -c baudrate=115200`.
(Or in the Arduino IDE: Tools > Serial Monitor, bottom right: baud **115200**, line ending **Newline**.)
Type each word and press Enter. Write down what comes back.

| # | You type | Expected | What you saw |
|---|---|---|---|
| 1 | `PING` | `PONG` | |
| 2 | `STATUS` | `STATUS 0` | |
| 3 | `NOPE` | `ERR` | |
| 4 | `WHO` | `WHO 1234567` (a number) | |
| 5 | `ON` | LED stays on, steady | |
| 6 | `OFF` | LED stays off | |
| 7 | `BLINK` | blinking again | |
| 8 | `FAST` | blinks faster (about 3 times a second) | |
| 9 | `SLOW` | back to 1 per second | |

**Number 4:** to know what `WHO` should be, in a second PowerShell in the rudra folder run
`python -m rudrac sim examples/blink_uno.rudra --run 100ms --send 10ms:WHO`. The line `TX  WHO ...` shows the number. They must be equal.

**Timing test** (phone stopwatch):
- Send `BLINK`, then `SLOW`. Count **20 blinks** (LED on to on). Expected about **20 s**. Time: ______ s
- Send `FAST`. Count **20 blinks**. Expected about **6 s**. Time: ______ s

## Part D: send me the results

Copy this and fill it in:

```
Board: Arduino Uno (clone? original?)      OS: Windows/Mac/Linux
Part B step 7 worked: yes / no   (if no: paste ALL the text it printed)
LED blinked at the start: yes / no
Table rows 1-9: (paste the "what you saw" column)
WHO from the board: ______    WHO from the simulator: ______
20 blinks at SLOW: ___ s       20 blinks at FAST: ___ s
Anything strange:
```

## Part E: tools for the next stage (B5), only after Part D works

On Windows, in PowerShell as Administrator: `wsl --install`, restart, open "Ubuntu", then:
```
sudo apt update
sudo apt install gcc-avr avr-libc simavr
avr-gcc --version
simavr --help
```
Send me the first line each of the last two commands prints.
[Mac: `brew install avr-gcc simavr`. Linux: the same `apt` line.]
