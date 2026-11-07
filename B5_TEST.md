# B5 first test: machine code that rudrac's own toolchain produced

`examples/avr/blink_raw.S` is a small AVR assembly program (blink the L LED on pin 13 every ~500 ms, print `HI` on serial at 115200).
`python3 -m rudrac avr examples/avr/blink_raw.S` assembles it with **rudrac's own assembler** (no avr-gcc) into `examples/avr/blink_raw.hex`,
which is already in the zip. No C++, no Arduino core, no bootloader code of ours: just our bytes.

In the emulator I wrote it blinks every 499.7 ms and prints `HI`. That emulator is mine, so it can be wrong in the same way the assembler is.
This test is the independent check: if the real chip (or Wokwi's emulator) blinks and prints `HI`, the encodings and peripheral use are right.

## Way 0: our own uploader (new, needs `pip install pyserial`)

```
pip install pyserial
python3 -m rudrac upload examples/avr/blink_raw.hex -p COM3
```
(Use your port; close the Arduino serial monitor first.) It resets the Uno, talks to its bootloader, writes the pages, reads them back and compares.
It prints `uploaded 282 bytes to COM3 and verified them` on success. It has only been tested against a fake bootloader I wrote, so if it fails, use Way 1
and send me the text it printed.

## Way 1: real Uno, using the tool the Arduino IDE already installed

1. Open the Arduino IDE, File > Preferences, tick **Show verbose output during: upload**. Upload any sketch (for example blink) once.
2. In the black output area find the long line starting with `...avrdude.exe -C...avrdude.conf -v -V -patmega328p -carduino -PCOM3 -b115200 -D -Uflash:w:...:i`.
   Copy it.
3. In PowerShell, paste it, but replace the last part `-Uflash:w:<something>.hex:i` with
   `-Uflash:w:C:\path\to\rudra\examples\avr\blink_raw.hex:i` (the real path of the hex in the unzipped folder).
4. Press Enter. avrdude uploads the hex. The L LED should blink about twice a second.
5. Open the Arduino serial monitor at 115200 and press the Uno's reset button: you should see `HI`.
   (The serial monitor opening resets the Uno, so you will see it right away.)

Also possible with arduino-cli: `arduino-cli upload -p COM3 --fqbn arduino:avr:uno --input-file examples\avr\blink_raw.hex`.

## Way 2: Wokwi (no hardware)

In wokwi.com, new Arduino Uno project, press **F1** and look for a command like **Upload Firmware and Start Simulation**. Choose
`blink_raw.hex`. If your Wokwi has no such command, skip this way; Way 1 is the one that matters.

## What to send back

```
Way used: 1 / 2
LED blinks: yes / no   how fast (20 blinks took ___ s; expected about 10 s)
Serial shows "HI": yes / no
avrdude / Wokwi messages if anything failed (paste all text)
```

## Going back to normal

Uploading any normal sketch from the IDE (for example the blink_uno one) replaces this program. Nothing is permanently changed.
