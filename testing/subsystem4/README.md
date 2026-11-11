# Subsystem 4: IMU motion and terrain monitoring

The firmware is in `app_program/subsystem4/` (8 files). This folder holds what is *not* firmware:
the test log, the PC tests and simulator, the log plotter, and this guide.

| Here | What it is |
|---|---|
| `TEST_LOG.md` | Test cases and results (the team's required test record) |
| `logs/` | Saved console output and CSV logs |
| `host_test/run_sim.sh` | Runs the packing test and the simulation scenarios: the full drive, picked up, overturned, two false-alarm tests, the ghost-fault sweep and the sensor faults |
| `host_test/test_pack.c` | PC test of the minimum-bit data: limits, clipping, wrap-round, the accumulator |
| `host_test/sim/` | Mock kernel, fake sensor and car, mocks of Subsystem 1's functions (compiled against Subsystem 1's real header) |
| `tools/imu_log.py` | Captures the CSV sample stream over serial and plots it |

The on-target tests stay in `app_program/subsystem4/subsystem4_test.c`: the makefile only compiles
`app_program/*/*.c` and `app_main.c` calls `subsystem4_test()`.

## The firmware, file by file

| File | What it is |
|---|---|
| `s4_config.h` | Every number you may tune: axis signs, timing, thresholds, options |
| `lsm303d.h/.c` | The sensor driver: 5 register writes, two 6-byte reads, each value traced to ST's LSM303D datasheet. Not affected by the data requirement: it handles the chip's own register format |
| `task_imu.c` | `imu_task` (priority 11, every 20 ms): sensor, calibration, tilt, hump detector, picked-up / overturned detector, turn rate. Packs its results into the message and the snapshot word |
| `task_report.c` | `report_task` (priority 18): unpacks the messages, decides the motion state (whole numbers only, no floating point), reports to Subsystem 1 |
| `subsystem4.h` | Public functions, the packed message, the field table, the single-writer rules, clipping helpers |
| `subsystem4.c` | `subsystem4_init/start`, shared data, the fault link and the telemetry link to Subsystem 1 |
| `subsystem4_test.c` | On-target tests S4-U01 to U08, U17 to U19 and U25 (`make TEST=1`) |

## How subsystem 4 talks to the other subsystems

| Direction | Mechanism | What happens |
|---|---|---|
| S4 -> S1 (IF-01, IF-09) | `s1_set_pitch()`, `s1_set_car_state()` | Pitch (8 bits, clipped), our motion state (3 bits) and hump state (2 bits), every 100 ms. Each setter updates one field under Subsystem 1's lock, so nobody else's field is overwritten |
| S4 -> S1 (IF-07) | `s1_report_fault()` / `s1_clear_fault()` | One bit, `FAULT_IMU_SENS`, set while the IMU is not answering OR the car is picked up or overturned; cleared when neither is. While faulted it is checked every 100 ms and reported again ONLY if someone cleared it. After a release it is watched for 700 ms and cleared again if a stale report brings it back (see Open points) |
| S4 -> S1 (IF-08) | `s1_csv_log_event()` | `HUMP`, `PICKED_UP`, `OVERTURNED`, `PUT_DOWN`, `IMU_FAULT`, `RUN_SUMMARY` lines (heights in millimetres, tilt in degrees) |
| S4 -> S2 (IF-10) | `motion_trigger_safety_halt(FAULT_IMU_SENS)` | When the car is picked up or overturned: the motors are halted completely and at once (see the next section). Declared inside `subsystem4.c`: Subsystem 2's header is not needed |
| S2 -> S4 (IF-06) | `get_distance_traveled_mm()` | Distance driven, for hump height: the mean of both wheel encoders in millimetres, forward positive. Subsystem 2 owns the wheel size and the ticks per turn (`WHEEL_DIAMETER_MM`, `ENCODER_TPR` in `encoder.h`), so there is nothing to calibrate in subsystem 4. Declared inside `task_imu.c`: Subsystem 2's header is not needed. One step of more than `S4_MAX_STEP_CM` (5 cm per 20 ms) is ignored: it is a counter reset or a glitch |

## Picked up and overturned (this replaces collision detection)

Collision detection has been **removed completely**: no impact threshold, no `COLLISION` event, no `IMPACT` motion
state, no `S4_FAULT_COLLISION`, no `s4_clear_collision()`. A knock is now only shaking. In its place the car must
notice that it has been **picked up** or **overturned**, and **stop completely**.

| Sign | Rule (numbers in `s4_config.h`) | Reported as |
|---|---|---|
| overturned | gravity more than 60 deg from the car's "up" for 15 samples (0.3 s): on its side or upside down | `OVERTURNED` |
| picked up, by force | the total pull `\|a\|` is more than 0.15 g away from 1 g for 6 samples (120 ms). It is 1 g at rest and while driving, whatever the tilt; lifting pushes it above 1 g | `PICKED_UP` |
| picked up, tilted | gravity more than 35 deg from "up" for 25 samples (0.5 s) | `PICKED_UP` |

Nothing fires until the sensor is calibrated (an uncalibrated car does not know which way is up; simulated by the
wrong-mounting scenario) and 0.5 s after the first reading.

**What happens at once** (`lift_start()` in `task_imu.c`, in this order):
1. `s4_stop_motors()` calls Subsystem 2's `motion_trigger_safety_halt(FAULT_IMU_SENS)`: both wheels to PWM 0/0 immediately.
   This direct call is needed: Subsystem 2's automatic "any fault means halt" override is commented out in
   `subsystem2.c`, so raising the fault bit alone does not stop the wheels.
2. `FAULT_IMU_SENS` is raised through Subsystem 1 (the same single team bit as the lost-sensor fault).
3. The motion state becomes `PICKED_UP` (`car_state` 6) or `OVERTURNED` (`car_state` 7), and a `PICKED_UP` or `OVERTURNED` line goes to the CSV log, with the tilt.

If somebody clears the bit while the car is still lifted, the next status tick (100 ms) reports it and halts the motors again.

**Picked up versus overturned.** The two are reported separately everywhere:

| | picked up | overturned |
|---|---|---|
| motion state, and the telemetry `car_state` | `PICKED_UP` = **6** | `OVERTURNED` = **7** |
| CSV event and console line | `PICKED_UP` | `OVERTURNED` |
| `s4_get_lift()` | `S4_LIFT_PICKED_UP` | `S4_LIFT_OVERTURNED` |
| motors | stopped at once | stopped at once |
| `FAULT_IMU_SENS` | set | set (the same single bit) |

While the car is stopped the reason **follows its tilt**: past 60 deg it is overturned, and back within 45 deg of upright
(`S4_RIGHTED_COS2`, for 0.3 s) it is picked up again.

| What happens | What is reported |
|---|---|
| lifted and carried level | `PICKED_UP` until it is put down |
| tipped over on the ground | `OVERTURNED`; `PICKED_UP` once a hand turns it back up; `PUT_DOWN` after it has rested |
| lifted, then turned over in the hand | `PICKED_UP`, then `OVERTURNED`, then `PICKED_UP` again once it is turned back up |
| upside down and left there | `OVERTURNED` until it is righted and put down |

The motors are stopped once and stay stopped through every change of reason (a change sends no second halt). One limit: the
sensor only sees tilt, so a car that is **held** sideways in a hand also reads `OVERTURNED`.

**Release.** The car counts as *put down* when it is resting: upright within 20 deg, the pull within 0.08 g of 1 g, and not
shaking (`S4_STILL_VIB_MG`), all for `S4_PUTDOWN_MS` (2 s) in a row. The motors are off by then, so any shaking is a hand: a
carried car never rests. After a quick move the shaking figure needs about a second to settle, so the release comes about
`S4_PUTDOWN_MS` + 1 s after the car is set down. Two modes (`S4_LIFT_LATCH`): **0** releases by itself (default); **1** stays
latched until `s4_clear_lifted()` is called, which is manual clearance as `systemFaults.h` asks for the fatal tier. A clear is
only accepted while the car is resting level; a CLEAR sent while it is in a hand is ignored.

**How sensitive is it?** Measured in the PC simulation with about +/-0.04 g of hand tremor (`host_test`, probe mode); the real
numbers must come from the car:

| | Caught / tolerated with the defaults |
|---|---|
| a lift lasting 0.3 s | caught at 0.20 g and above; **not** caught at 0.15 g and below |
| a 0.30 g lift | caught if it lasts 120 ms or more; **not** caught at 100 ms or less |
| carried tilted, no lift pulse | caught at about 40 deg and above; not at 36 deg and below |
| rough ground, uniform shaking | no false stop up to +/-0.18 g; false stops at +/-0.22 g |
| a slope, held 3 s | no false stop up to 34 deg; a false stop from 36 deg |
| a knock, a vertical bump, a sideways shove on rough ground | none of them stops the car |

Lowering `S4_LIFT_G` to 0.12 catches lifts of 0.15 g but tolerates only +/-0.15 g of shaking; 0.10 g gains nothing more. **A
gentle lift that stays level can be missed**: record real pick-ups with `tools/imu_log.py` (`S4_PRINT_CSV 1`) and tune from them.

## Minimum-bit data (change requirement 1)

Packed, at the boundaries (what crosses a task boundary):

| Value | Range / step | Bits | Where |
|---|---|---|---|
| message type | 4 kinds | 2 | message |
| hump state | flat / up / down | 2 | message |
| time | low 16 bits of the ms clock | 16 | message |
| vibration | 0..255 mg, 1 mg | 8 | message |
| pitch, roll | -128..127 deg, 1 deg | 8 + 8 | message, `s4_snap` |
| turn rate | -256..254 deg/s, 2 deg/s | 8 | message, `s4_snap` |
| acceleration change | -1.28..1.27 g, 10 mg | 8 | message |
| hump peak | 0..127 mm, 1 mm | 7 | message |
| hump length | 0..510 mm, 2 mm | 8 | message |
| lift state | none / picked up / overturned | 2 | message (status), `s4_lift` |
| tilt at a lift event | 0..255 deg, 1 deg (180 = upside down) | 8 | message |
| fault conditions | comm lost, lifted | 2 | `s4_faults` |
| motion state | 8 states (0 to 5 driving, 6 picked up, 7 overturned) | 3 (a byte in RAM) | `s4_motion` |

Values outside a range are clipped to the limit, never wrapped.
The ranges for hump peak and length are placeholders until the real track is known.

**Not packed, on purpose:** the working values inside `imu_task` (hump height, filters, level-ground
baseline). A hump grows by about 0.4 mm per sample at 4 degrees; a 1 mm accumulator would round every step to
zero and never see a hump (`test_pack.c` shows this). Pack at the boundaries, keep the working state wide.

**Race-free access:** every shared word has one writer and is published in one store (`s4_snap`, `s4_motion`,
`s4_hump_count`, `s4_max_hump_mm`, `s4_samples`, `s4_faults`); the three flags that several tasks may set
(`calibrated`, `reset request`, `clear request`) are bits of a kernel event flag, which the kernel sets and clears
atomically (the requests are consumed with `TWF_BITCLR`); everything else goes through the message buffer.
The fault bit goes through Subsystem 1's functions, which take Subsystem 1's lock.

**Measured with the ARM compiler and the real build flags** (not estimates):

| | Before | After |
|---|---|---|
| `sizeof(S4_MSG)` | 48 B | 8 B |
| message buffer, 16 slots | 896 B | 256 B |
| subsystem 4 static RAM (data + bss) | 160 B | 137 B |
| subsystem 4 code | 6,473 B | 7,065 B (+592 B, +9%) |
| stack used, `imu_task` / `report_task` | n/a | 304 B / 192 B of 2,048 B each |

*(These figures were measured before the picked-up / overturned detector was added. The message is still exactly 8 bytes, but the code and RAM of subsystem 4 have changed: measure again before quoting them.)*

About 660 B of RAM saved in total, paid for with about 590 B more code (the shifting and clipping). The task stacks
are the biggest RAM item and are not part of this requirement. Execution time on the real board is not measured yet:
time `process_sample()` with the 1 MHz timer before quoting a number.

## Open points for the team

- **Tier 1 is "requires manual clearance"** (`systemFaults.h`), but a picked-up / overturned condition releases itself once the
  car has rested for `S4_PUTDOWN_MS`. For manual clearance set `S4_LIFT_LATCH` to 1 and have whoever handles the `CLEAR`
  command call `s4_clear_lifted()`. After a release Subsystem 2 leaves its halt only when the fault mask is clear **and** its
  commanded speeds are zero (`handle_motor_safety_halt`), so the follower has to stop commanding movement before the wheels
  can turn again.
- **`car_state` is assumed to be subsystem 4's motion state** (8 states = 3 bits). Test S4-U19 checks it; confirm with Subsystem 1. Subsystem 4 is the only writer of `car_state`, but `scripts/telemetry_decoder.py` still names the values after Subsystem 2's motor states (see `telemetry_decoder.patch`).
- **Hump height is not in the 8-byte packet**, only in the CSV log. Ask Subsystem 1 for a field.
- **The CSV log is text.** Ask the professor whether it counts as application data.
- **Ghost fault (found by the simulation, fixed in subsystem 4, root cause is in Subsystem 1).** `s1_report_fault()`
  sets the mask bit AND a bit in Subsystem 1's event flag; `s1_clear_fault()` clears only the mask. Subsystem 1's LED
  task (every `S1_TICK_MS` = 250 ms) copies leftover flag bits back into the mask, so a report that was still waiting
  brought a cleared fault back, and nothing cleared it again. Subsystem 4 now reports again only when the bit is
  missing, and watches for 700 ms after a release. The proper fix is one line in `s1_clear_fault()`: also clear the
  same bits in `flg_s1_isr_fault` (`tk_clr_flg(flg_s1_isr_fault, ~fault_bit)`). Every other subsystem that clears a
  fault through `s1_clear_fault()` has the same problem until Subsystem 1 fixes it.
- **Subsystem 2's automatic fault override is commented out** (`subsystem2.c`, "Master safety override"), so `FAULT_IMU_SENS`
  alone does not stop the wheels. Picked-up / overturned stops them with the direct call `motion_trigger_safety_halt()`
  (IF-10). A lost IMU still only raises the bit. If Subsystem 2 re-enables its override, both paths work together.
- **Pins (checked against `PINLAYOUT.md` and the code):** the encoders moved to GP17/GP16 (left A/B) and GP5/GP4 (right A/B).
  Subsystem 4 does not use them, and its own pins are unchanged: I2C1 on GP26 (SDA) and GP27 (SCL). One hardware risk:
  `PINLAYOUT.md` puts the edge sensor's (Grove 4) analog output on **GP26, the same pin as the IMU's SDA**. If the IMU
  reports `not answering` or `IMU lost` only while that sensor is plugged in, unplug its analog (yellow) wire.
- **Build state (checked on the current repo):** the plain and `TEST=1` builds succeed. The Wi-Fi/MQTT build
  (`NET=lwip MQTT=1`) fails in Buddy 1's `mqtt_utk.c` (`MQTT_TOPIC_CMD` undeclared), not in subsystem 4.

## Known limits (said plainly)

- **Acceleration looks like tilt.** (The encoder distance now gives the speed, so this could be corrected; it is not.) Speeding up or braking pushes the accelerometer the same way as tilting does.
  Drive over humps at a steady speed.
- **Hump height can only be as accurate as the distance.** It uses Subsystem 2's `WHEEL_DIAMETER_MM` (66) and `ENCODER_TPR` (2050), both marked "replace with your exact value". Height scales directly with them: a 2 % error in the distance is a 2 % error in every hump. Push the robot an exactly measured distance (1 m or more, tape measure) and compare it with the distance Subsystem 2 reports (test S4-U24). `S4_USE_ENCODER_DISTANCE 0` falls back to a fixed 20 cm/s and is only for running without encoders.
- **Short humps:** a 5 cm hump at 30 cm/s (about 8 samples) is missed with the current settings in the simulation.
  Tune `S4_HUMP_ENTER_N`, `S4_HUMP_EXIT_N` and `S4_PITCH_ALPHA` from real logs.
- **A gentle lift can be missed.** The force rule needs about 0.2 g for 120 ms; a slow, smooth lift that is then held level
  gives neither. Only a tilt of about 40 degrees or more would still catch it. Tune `S4_LIFT_G` and `S4_LIFT_N` from real
  pick-ups (see the sensitivity table above).
- **A car held perfectly still and level can be released early** in the automatic mode (it then looks like resting). A real
  hand shakes more than `S4_STILL_VIB_MG`; use `S4_LIFT_LATCH 1` if that is not enough.
- **Stopping takes a moment.** Detection needs 120 ms (force), 0.3 s (overturned) or 0.5 s (carried tilted), and the motors stop
  at the next sample after that.
- **Turn rate** comes from the compass and is disturbed by the motors: measure `S4_MAG_OFFSET_*`.
- **No recovery from a stuck I2C bus** (SDA held low): only retries.

## Wiring and axis mapping

RP2040 hardware I2C1 only allows **GP26 = SDA** and **GP27 = SCL**. The board is an LSM303D (address 0x1D, SA0 high).
Its X axis points backward and its Z axis points down, so `S4_SIGN_FWD = -1`, `S4_SIGN_LEFT = +1`, `S4_SIGN_UP = -1`.
Check: nose up gives positive pitch, left side up gives positive roll.

## Running the tests

PC (needs gcc; run from inside the repo, it compiles the firmware in `app_program/subsystem4`):

    cd testing/subsystem4/host_test
    ./run_sim.sh

On the Pico (see `testing/README.md`): `cd build_make && make TEST=1 -j8`, flash, and save the console output
to `testing/subsystem4/logs/YYYY-MM-DD_unit.txt`.

For hump logging set `S4_PRINT_CSV` to 1 and `S4_PRINT_STATUS` to 0 in `s4_config.h`, then:

    pip install pyserial matplotlib
    python tools/imu_log.py capture COM6 logs/2026-10-14_run1.csv
    python tools/imu_log.py plot logs/2026-10-14_run1.csv

**Tuning the picked-up / overturned limits on the PC.** After `run_sim.sh` has built `/tmp/s4_sim`, the probe mode tries one
value per run and prints `fired=0` or `fired=1` (did the motors get halted?):

    PROBE_SCN=lift_g    PROBE_VAL=0.20 /tmp/s4_sim probe    # a 0.3 s lift of 0.20 g
    PROBE_SCN=lift_ms   PROBE_VAL=100  /tmp/s4_sim probe    # a 0.30 g lift lasting 100 ms
    PROBE_SCN=carry_deg PROBE_VAL=40   /tmp/s4_sim probe    # carried tilted 40 deg, no lift pulse
    PROBE_SCN=rough_g   PROBE_VAL=0.18 /tmp/s4_sim probe    # rough ground, +/-0.18 g shaking (fired=1 is a FALSE stop)
    PROBE_SCN=ramp_deg  PROBE_VAL=34   /tmp/s4_sim probe    # a slope held for 3 s (fired=1 is a FALSE stop)

To try another limit, build with it (`S4_LIFT_G` and `S4_LIFT_N` can be set on the compiler line, for example
`-DS4_LIFT_G=0.12f`). The simulated noise is an assumption (about +/-0.04 g of hand tremor): use it to see the trade-off,
and take the real numbers from the car.
