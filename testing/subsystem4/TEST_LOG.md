# Subsystem 4 — test log

**Subsystem:** IMU motion and terrain monitoring  ·  **Student ID:** 2500787  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.
The PC simulator is in `host_test/`, the log plotter in `tools/`; saved console output and CSV logs go in `logs/`.

## Test cases

Define a test here before running it. Test IDs never change.

**On-target automatic tests (S4-U01 to S4-U08, S4-U17 to S4-U19, S4-U25):** flash the `make TEST=1` image with the robot **still and level**
and do not touch it for about 8 seconds. They are in `app_program/subsystem4/subsystem4_test.c`.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | the IMU answers on I2C1 (GP26 SDA, GP27 SCL) | wiring, I2C1 ownership | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | accelerometer calibrated while still | calibration | auto | `subsystem4_test.c` | `[TEST] S4-U02 PASS` |
| S4-U03 | the sensor is read about 50 times a second | 20 ms sampling | auto | `subsystem4_test.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | pitch and roll within ±5° when level | orientation | auto | `subsystem4_test.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | a still robot is classified STATIONARY | motion classification | auto | `subsystem4_test.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | no false hump and no false picked-up / overturned while still | hump / lift detection | auto | `subsystem4_test.c` | `[TEST] S4-U06 PASS` |
| S4-U07 | the IMU pitch reaches Subsystem 1's telemetry (`s1_set_pitch`) and its speed and obstacle values are not overwritten | IF-01 | auto | `subsystem4_test.c` | `[TEST] S4-U07 PASS` |
| S4-U08 | `FAULT_IMU_SENS` is set and cleared through Subsystem 1 (`s1_report_fault` / `s1_clear_fault`), other subsystems' bits untouched | IF-07 | auto | `subsystem4_test.c` | `[TEST] S4-U08 PASS` |

| S4-U17 | our two conditions (IMU not answering, picked up / overturned) share the ONE `FAULT_IMU_SENS` bit, which stays set until both are over | IF-07 | auto | `subsystem4_test.c` | `[TEST] S4-U17 PASS` |
| S4-U18 | the message between the tasks is exactly 8 bytes, and values outside a field's range are clipped, never wrapped | minimum-bit data | auto | `subsystem4_test.c` | `[TEST] S4-U18 PASS` |
| S4-U19 | Subsystem 1's `car_state` and `hump_state` fields show our motion state and hump state | IF-09 | auto | `subsystem4_test.c` | `[TEST] S4-U19 PASS` |
| S4-U25 | the picked-up / overturned limits are in a sensible order (overturned needs more tilt than leaving overturned, than carried; put down needs more upright than any of them; PICKED_UP and OVERTURNED are different 3-bit states) | lift detection | auto | `subsystem4_test.c` | `[TEST] S4-U25 PASS` |

**PC tests (no hardware):**

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U09 | the real driver and tasks on a mock kernel with a fake sensor and the real header of Subsystem 1: a full drive (calibration, a hard knock that must stop nothing, hump, turn) with the distance from `get_distance_traveled_mm()` and without encoders (fixed speed), sensor unplugged and plugged in, wrong mounting; and random 35 ms stalls | IF-01, IF-06, IF-07, IF-08, hump measurement | auto (PC) | `cd testing/subsystem4/host_test && ./run_sim.sh` | every scenario prints `ALL PASSED` |
| S4-U20 | packing: the message and the shared snapshot at the minimum, the maximum and just past the maximum of every field; clipping, 16-bit time wrap, and why the hump accumulator is not packed | minimum-bit data | auto (PC) | first step of `./run_sim.sh` (`test_pack.c`) | `21 checks, 0 failed` |
| S4-U22 | no ghost fault: a put-down released after 0.6 s while Subsystem 1's LED task (which copies leftover report bits back into the fault mask) runs at 6 different phases; and the IMU bit is still clear at the end of the full drive | IF-07 | auto (PC) | second scenario group of `./run_sim.sh` | all 6 phases print `ALL PASSED` (the unfixed code fails 4 of the 6) |
| S4-U26 | picked up: the motors are halted within 0.45 s of the lift (`motion_trigger_safety_halt` called), `FAULT_IMU_SENS` set, state `PICKED_UP` (6, never 7); a clear by someone else is undone and the motors halted again; a CLEAR while carried is ignored; automatic release after resting `S4_PUTDOWN_MS`; with `S4_LIFT_LATCH 1` it stays latched until `s4_clear_lifted()` | IF-07, IF-10, lift detection | auto (PC) | scenarios 3 and 3a of `./run_sim.sh` | both print `ALL PASSED` |
| S4-U27 | overturned onto its side and upside down: halted, state `OVERTURNED` (7, never 6 while it lies there), held while it is not upright; when set upright by hand the reason changes to `PICKED_UP` (6) with no second halt; released after it has been upright and resting | IF-07, IF-10, lift detection | auto (PC) | scenarios 3c and 3d of `./run_sim.sh` | both print `ALL PASSED` |
| S4-U28 | false alarms: rough ground (0.12 g shaking) with a 1.8 g knock, a 0.9 g vertical bump and a 0.8 g sideways shove; a 30 degree slope; and the 1.8 g knock in the full drive. None may stop the car | lift detection | auto (PC) | scenarios 1, 3e and 3f of `./run_sim.sh` | all print `ALL PASSED`; `halt_calls` stays 0 |
| S4-U33 | the reason follows the tilt: lifted (`PICKED_UP`, 6), turned over in the hand (`OVERTURNED`, 7), turned back up (`PICKED_UP`, 6), put down (`PUT_DOWN`); only one halt is sent | IF-09, IF-10, lift detection | auto (PC) | scenario 3g of `./run_sim.sh` | `ALL PASSED`; events: `PICKED_UP` twice, `OVERTURNED` once, `PUT_DOWN` once |
| S4-U29 | a wrongly mounted board (reads upside down) is not reported as overturned: with no calibration the detector is off | lift detection | auto (PC) | scenario 6 of `./run_sim.sh` | `ALL PASSED` |
| S4-U23 | the encoder counters are zeroed in the middle of a hump (as `motor_reset_ticks()` would do): the hump is still measured, because one step of more than 5 cm per 20 ms is ignored | IF-06, hump measurement | auto (PC) | scenario 2b of `./run_sim.sh` | `ALL PASSED`: the 30 mm hump reads 29 mm (without the guard it read 19 mm) |

**Manual tests (flash the normal `make` image; the `HUMP`, `PICKED_UP`, `OVERTURNED`, `PUT_DOWN` and `RUN_SUMMARY` lines come from Subsystem 1's CSV logger):**

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U10 | axis directions | mounting | manual | 1. Level and still: `pitch`/`roll` near 0. 2. Lift the nose. 3. Lower the nose. 4. Lift the left side. | nose up: pitch positive; nose down: negative; left side up: roll positive |
| S4-U11 | picked up: the motors stop completely | lift detection, IF-07, IF-10 | manual | With the car driving (Subsystem 2 commanding movement), lift it off the ground and hold it for 3 s. Repeat gently and firmly, 5 times each. | within about 0.5 s: the wheels stop, console `PICKED_UP (tilt N deg): motors stopped`, `faults now 0x2`, `LOG_CSV,PICKED_UP,tilt_deg=N`, `motion: PICKED_UP`; `FAULT_IMU_SENS` (bit 4) set in the telemetry `fault_mask`; `car_state` = 6 (never 7). Write down which lifts were missed (they set `S4_LIFT_G`) |
| S4-U12 | turn rate and TURNING | turn monitoring | manual | Turn the robot left by hand about 90° in one second, on a level table. | `turn` is positive while turning and the state shows TURNING |
| S4-U13 | hump peak height accuracy | hump measurement (project brief) | manual | Drive over humps of known, ruler-measured height, 5 runs each, with Subsystem 2's encoders counting (`S4_USE_ENCODER_DISTANCE 1`). Compare the `LOG_CSV,HUMP` `peak_mm` with the ruler. | each peak within ±20% of the measured height (team target) |
| S4-U14 | hump detection rate | hump measurement (project brief) | manual | 10 full runs of the course. Count missed humps and false humps from the `HUMP` lines. | at least 95% of humps found, at most 1 false hump per run (team targets) |
| S4-U15 | highest hump of the run is reported | project brief: report the highest hump peak | manual | Call `s4_publish_summary()` when the mission completes. | `LOG_CSV,RUN_SUMMARY,…highest_mm=` equals the largest `peak_mm` of the run |
| S4-U16 | an unplugged IMU is reported and recovers | robustness, IF-07 | manual | Run, unplug the IMU, then plug it back in. | console `IMU not answering…`; `FAULT_IMU_SENS` set; after plugging in it recalibrates (keep the robot still) and the bit clears |
| S4-U21 | the IMU fields in the 8-byte MQTT packet | IF-01, IF-09 | manual | Subscribe to the telemetry topic, tilt the robot nose up, and decode the 8-byte payload (`S1_TELEMETRY_PKT`). | `pitch_deg` follows the tilt (positive nose up); `car_state` = 4 while climbing; `hump_state` = 1 going up, 2 going down |
| S4-U30 | overturned: the motors stop and stay stopped until it is upright and resting | lift detection, IF-07, IF-10 | manual | With the car driving, tip it onto its side; leave it for 3 s; then turn it upside down. Then set it upright on the floor and leave it alone. | the wheels stop; console `OVERTURNED (tilt N deg): motors stopped` (N about 90 on its side, about 180 upside down); `motion: OVERTURNED`, `car_state` = 7; it stays stopped while it lies there, even though quiet and still; when you turn it back up by hand it reports `PICKED_UP` (`car_state` 6) until it has rested; about `S4_PUTDOWN_MS` + 1 s after it is upright and still: `put down`, `faults now 0x0`, the bit clears |
| S4-U31 | put down: release (and manual clearance) | lift detection, IF-07 | manual | Pick the car up, carry it around for 5 s, and put it down. With `S4_LIFT_LATCH 1` the release needs `s4_clear_lifted()`: call it while it is still in your hand, then again once it rests on the floor. | it does NOT release while carried (the hand shakes). Automatic mode: released once it has rested about 3 s. Latch mode: the clear in the hand is ignored (`clear ignored: the car is not resting level`), the clear on the floor releases it |
| S4-U32 | no false stops while driving | lift detection | manual | 10 full runs of the course (humps, turns, knocks against walls, rough floor). Count unwanted stops from the `PICKED_UP` / `OVERTURNED` lines. | none. The slopes and shaking of the course are well inside the limits (simulated: no false stop up to 34 deg or +/-0.18 g) |
| S4-U24 | the distance subsystem 4 uses is right | IF-06, hump measurement | manual | Mark a straight line of exactly 1.00 m (tape measure), start the robot at one end and let it stop at the other (or push it by hand with the motors off). Compare Subsystem 2's `CALCULATED DISTANCE` with the tape. | within ±2 % (hump heights are off by the same percentage) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console; **auto (PC)** = run on a laptop;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
