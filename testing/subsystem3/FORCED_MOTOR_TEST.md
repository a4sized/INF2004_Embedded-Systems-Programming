# One-shot motor test

This is a separate USB diagnostic UF2. It bypasses the Vehicle Controller,
IR line-following permission and ultrasonic speed cap by starting only
Subsystems 1 and 2 (Subsystem 5 is initialized first so its PWM reset does
not undo the motor setup). It still sends the request through Subsystem 2's
`s2_set_ir_velocity()` and motor state machine; it does not write raw PWM.
The test does not read barcodes or follow the line.

## Flash and run

1. Build from the repository root with
   `python3 app_program/subsystem3/build_forced_motor_test.py`, then flash
   `build_make/team_s2_forced_motor_test_smp1_usb_cdc_wifi_cyw43.uf2`.
2. **Raise both drive wheels off the surface for the first run.** Turn on the
   motor power supply and connect the USB serial monitor at 115200 baud.
   Use the actual `/dev/cu.usbmodem...` port shown by
   `ls /dev/cu.* | rg usbmodem`. Keep clear of the wheels.
3. Once USB is open, expect `MOTOR_TEST,USB_READY_WAIT_3S`. Three seconds
   later, expect `MOTOR_TEST,FORWARD_50_50_1S`. Both wheels should rotate
   forward for about one second; Subsystem 2 ramps up and down, so actual
   powered time may be slightly longer. Then expect `MOTOR_TEST,STOP_0_0`.
   The pulse runs **once per boot**. Power cycle for another trial.
4. Save the complete serial output and note whether each wheel spun and
   whether its encoder tick total rose during the pulse. If one wheel turns
   backward, report that before any floor test. If the log announces
   `FORWARD_50_50_1S` but neither wheel spins, check motor supply, switch,
   motor driver wiring and Subsystem 2's output path. If both wheels spin,
   reflash the separate moving barcode test to inspect its `IR_MOTION`
   sensor and request lines.

Do not put the car on the track with this UF2 until both raised-wheel
directions have been checked. This test does not establish line tracking,
barcode decoding or production startup behaviour.
