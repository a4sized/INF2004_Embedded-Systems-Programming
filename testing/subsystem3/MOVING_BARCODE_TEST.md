# Integrated moving barcode test

This is a **USB-tethered test image**, not the competition firmware. It uses
the team Subsystems 1–5, Vehicle Controller, Subsystem 2 motor control, GP7
left line sensor, GP2 right line sensor and GP6 barcode sensor. The car
follows the line while capturing Code 39. It **does not turn at a junction**
after reading A or Z. Keep the first test track straight beyond the barcode.

## What this image changes for the test

- GP6 is still polled every 5 ms. The test image allows 8 s for the 30 barcode
  edges and up to 3 s more for trailing white. These are diagnostic margins,
  not validated competition timing limits.
- Before the first GP6 white-to-black edge, the usual line follower owns the
  request. From that edge until decode or capture timeout, the test requests
  equal forward wheel speeds through Subsystem 2, even if both line sensors
  briefly read white. It resumes normal line following afterward. This only
  engages after a valid line reading was seen with following permission; USB
  loss, stale IR data and revoked permission still prevent a forward request.
  After the first successful decode in a boot, later GP6 edges no longer
  activate this test-only straight-through override. Power cycle for each
  barcode pass. This avoids driving straight across a later junction cue.
- The Vehicle Controller still waits for IMU calibration, the first clear
  ultrasonic scan and a fresh IR snapshot. For this test image only, an open
  USB serial connection stands in for the initial Wi-Fi/START path. Once
  ready, it waits 3 s, then grants line following. No MQTT is needed.
- If USB becomes unavailable after motion starts, the Vehicle Controller
  revokes following permission and enters STOPPED. A fault or blocked
  ultrasonic path also prevents or stops following. After STOPPED, power
  cycle before another run. The physical power switch remains the immediate
  way to stop the test.
- Subsystem 1 prints Vehicle Controller state and barcode capture summaries
  over USB. The revised test image also prints `IR_MOTION` every 0.5 s with
  raw/stable line readings and Buddy 3's left/right request to Subsystem 2.
  On a successful decode it also prints `BARCODE_DECODED,letter=A;order=F`
  (or the actual letter/order) through Subsystem 1. The normal team image
  prints the same decode event when USB serial is connected. Diagnostic
  motion and startup settings remain test-only.

## Build and flash

From the repository root:

```sh
python3 app_program/subsystem3/build_moving_barcode_test.py
```

Flash `build_make/team_ir_moving_barcode_test_smp1_usb_cdc_wifi_cyw43.uf2`
through the Pico's BOOTSEL/RPI-RP2 drive. Earlier builds decoded moving Z in
both physical directions; the latest follower-state and one-decode override
changes have only been tested on the host and compiled. Keep the wheels clear of the surface
while flashing. Disconnect power, put the car at the beginning of a straight
black-line section, and ensure the USB cable has slack before powering it.

On the monitoring laptop, find and open the Pico port (use the actual port
name shown by the first command):

```sh
ls /dev/cu.* | rg usbmodem
screen /dev/cu.usbmodemMTK3PICO1 115200
```

If VS Code already has the port open, close its serial monitor first. To
leave `screen`, press Control-A, then K, then Y. Save the complete serial
output for each run, including the startup lines and all `LOG_CSV` lines.

## Run sequence

1. Start with a short straight section, one A barcode and no junction turn.
   Open the serial monitor while the car is at the start. Keep the power
   switch reachable. The car should remain still through readiness and the
   3 s test delay.
2. Look for `IR_TEST_READY,I1;U1;R1;B1;F0`: IMU, ultrasonic, IR and USB are
   ready; `F0` means no reported fault. Then expect `IR_TEST,READY_WAIT_3S`
   and `VC_STATE,FOLLOWING`. If a readiness digit remains 0 or F becomes 1,
   stop and save the log; do not bypass that gate.
   `IR_MOTION` reports `L=raw>stable;R=raw>stable;B=raw;capture=...;pos=...;mode=...;`
   `allowed=...;status=...;req=left/right` every 0.5 s. Position 0 means
   both white, 1 left black, 2 right black and 3 both black. While following
   over black, expect `allowed=1` and usually `req=51/51`. If it says
   `req=0/0`, check that at least one line sensor is over black and reports
   `1` (the sensors currently treat high as black). If it says `req=51/51`
   but the wheels stay still, check the motor power switch and supply; the
   commanded PWM may also be below the loaded motors' starting threshold.
   During the barcode, expect `capture=1` and `req=51/51`; after an
   `IR_CAPTURE` result, expect `capture=0` and normal line following.
3. Watch the car track the line across A. A successful capture prints an
   `IR_CAPTURE` line containing `count=30;duration_ms=...;overflow=0;letter=A;`
   `order=F` or `order=R`; and `result=DECODED`. Pattern order describes the
   order of bars, not the car's physical direction. The car keeps following
   straight after the decode.
4. Power off at the end of the straight section. Repeat for three A passes,
   then replace A with Z and do three Z passes. Record each pass as decoded,
   incomplete, rejected or invalid. Also note whether the car stayed on the
   line and the `duration_ms` values of complete captures.
5. If you see `result=INCOMPLETE` with fewer than 30 edges, the 8 s capture
   window may still be too short or an edge was missed. `result=REJECTED`
   means 30 edges arrived but failed Code 39/quiet checks. `INVALID` means
   malformed edges or overflow. Save that run's full log for diagnosis.

The next tuning decision uses the largest successful `duration_ms`, any
incomplete captures and the distance from the barcode to the junction. A
successful moving decode here establishes line-following plus barcode
capture; it does not establish junction navigation or a working MQTT start.

For the current circuit trial, use this USB-start UF2 while the team network
START configuration is unavailable. Keep the car on a track segment where
it can continue forward after the barcode, and stop it before a junction:
decoded A or Z does not request a turn. The standard team UF2 has the same
decode event but requires the real Wi-Fi/START path before it grants motion.

First hardware attempt (4 November 2026): the initial UF2 reached
`VC_STATE,FOLLOWING` with `I1;U1;R1;B1;F0`, but the car did not sustain
motion. Its serial output did not include line readings or motor requests,
so that run cannot distinguish a both-white zero request from a nonzero
request below the motor's starting torque. The revised image adds
`IR_MOTION` and capture-specific straight motion for that distinction;
flash the new UF2 before retesting. If `allowed=1` and `req=51/51` while
the car stays still, run the raised-wheel forced motor test next.

Moving Z forward pass (4 November 2026): in the second serial session the
IMU calibrated, the controller granted FOLLOWING, Subsystem 2 showed active
51/51 with cap 255, and both encoders advanced. GP6 decoded Z from 30 edges
in 2,800 ms with no overflow. The car reached both-white after decoding and
requested 0/0. A later test-only fix retains the last tracked side across
capture so ordinary correction/search can begin at that point; it is built
but not yet tested on the car. The first session had `I0` and no permission.

Moving Z reverse pass (4 November 2026): after IMU calibration and
`VC_STATE,FOLLOWING`, GP6 captured 30 edges in 2,679 ms, decoded Z with
pattern order R and no overflow. `IR_MOTION` showed `req=51/51` during capture
and after decode while the right line sensor still read black; `S2_PATH`
showed active 51/51 and nonzero PWM. Later, both line sensors read white and
another GP6 black edge began a new capture on the old test image. The latest
test image disallows its straight-through override after the first decoded
barcode in a boot; that change still needs a physical retest. These two Z
passes establish initial moving, bidirectional decoding, not repeatability
or junction navigation.
