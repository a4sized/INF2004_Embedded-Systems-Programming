# One-shot USB U-turn calibration

This dedicated image runs one encoder-controlled left pivot, then stays
stopped until the Pico resets. No barcode, line follower, line search,
IMU task, ultrasonic scan or MQTT START is needed. It uses Subsystem 2's
normal motor requests, ramping and reversal delay and Subsystem 1 logging.
S5 is initialized for the shared PWM block, but its tasks never start.

The current trial overrides only its own effective track to **129 mm**,
calculated as `132 * 180 / 184`, rounded, using the midpoint of the
user's reported 3–5 degree overshoot at 132 mm (about 184 degrees total).
The earlier 147 mm setting came from the initial approximately 270-degree
observation at 220 mm. The user has now accepted the 129 mm trial;
integrated firmware also uses **129 mm**.
At 129 mm the average absolute wheel target is 2007 ticks; differential
travel must reach 4014 ticks with left reverse and right forward. Requests
are -64/+64 (25%). An 8 s encoder timeout stops the trial and reports a
fault. No test result claims the physical angle is already calibrated.

## Run one trial

1. Flash `build_make/team_uturn_calibration_129mm_usb.uf2`.
2. Use the same motor power, battery condition, mounting and surface as the
   earlier pivot. Mark the initial heading and opposite 180-degree heading.
   Opening USB serial starts a **three-second countdown**, so position the
   car first, allow room for the pivot and keep the USB lead slack.
3. Open the USB serial monitor at 115200 baud and save the full output.
   `UTURN_CAL,USB_READY_WAIT_3S;target_deg=180` announces the countdown.
4. The car pivots left once. `START`/`PIVOT`/`SETTLING` logs include
   elapsed milliseconds, signed `EL`/`ER` totals, `track_mm`, wheel target
   and PWM. At encoder completion it requests zero and settles for 200 ms.
5. Measure the **final stopped heading** (a phone video helps). There is
   no subsequent following/search to confuse the measurement. Expect
   `UTURN_CAL_RESULT,DONE` and repeated `STOPPED_RESET_TO_REPEAT` totals.
   These later totals include any coast after the zero request. DONE means
   the encoder target was reached, not that physical 180 degrees was proven.
6. Record trial number, observed angle, surface, battery/power setup and the
   whole log. Reset the Pico and reposition before opening serial for the
   next trial. It cannot automatically repeat after finishing or aborting.

USB loss during rotation requests zero and ends the trial without retrying.
Faults or clock failure also stop it. `ENCODER_TIMEOUT` is a failed trial;
do not use it as a successful angle calibration. Restore the integrated
UF2 when finished with this dedicated motor image.

## Adjust using measured results

Try three runs before choosing another setting. If results are repeatable,
use `new track = current track * 180 / average measured angle`. If they vary
widely, keep the setting and investigate slip, cable drag or stopping first.

From the project repository, build the first trial with:

```sh
python3 app_program/subsystem3/build_uturn_calibration.py
```

For a later setting, for example 140 mm:

```sh
python3 app_program/subsystem3/build_uturn_calibration.py --track-mm 140
```

The filename includes the setting: `team_uturn_calibration_140mm_usb.uf2`.
The builder rejects settings outside 1..500 mm. The integrated header's
default is not edited by these trial builds. Integrate the chosen value
only after physical repeatability is accepted. The current 129 mm value
has been accepted by the user and integrated; keep this profile for
future rechecks.

## Validation status

The calibration image and unchanged default networked image compile.
The host pivot tests pass with the 129 mm override under sanitizers.
The user reports a satisfactory half-turn at 129 mm. Repeated precision
measurements, coast, cable effects, integrated handoff and USB-loss stopping
remain pending on-car tests. Keep small-barcode testing separately pending.
