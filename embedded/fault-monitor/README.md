# Fault monitor code

## Compiling and uploading:
Compiling and uploading this has some extra steps because of how ESP32 chips go into download mode on boot.

1. Connect a jumper between GPIO2 and GND on the board.
2. Connect the board to your laptop via USB.
3. Hold down the boot button on the board, and while holding it down, press the EN button once, then release the boot button. This will put the board into download mode.
4. Run `arduino-cli compile --fqbn esp32:esp32:esp32 ./` to compile the code.
5. Run `arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 ./` to upload the code to the board.
