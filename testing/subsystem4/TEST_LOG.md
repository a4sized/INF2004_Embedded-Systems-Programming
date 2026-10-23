# Subsystem 4 — test log

**Subsystem:** IMU motion and terrain monitoring  ·  **Student ID:** <your student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.
The PC simulator is in `host_test/`, the log plotter in `tools/`; saved console output and CSV logs go in `logs/`.

## Test cases

Define a test here before running it. Test IDs never change.

**On-target automatic tests (S4-U01 to S4-U08):** flash the `make TEST=1` image with the robot **still and level**
and do not touch it for about 8 seconds. They are in `app_program/subsystem4/subsystem4_test.c`.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | the IMU answers on I2C1 (GP26 SDA, GP27 SCL) | wiring, I2C1 ownership | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | accelerometer calibrated while still | calibration | auto | `subsystem4_test.c` | `[TEST] S4-U02 PASS` |
| S4-U03 | the sensor is read about 50 times a second | 20 ms sampling | auto | `subsystem4_test.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | pitch and roll within ±5° when level | orientation | auto | `subsystem4_test.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | a still robot is classified STATIONARY | motion classification | auto | `subsystem4_test.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | no false hump and no false collision while still | hump / collision detection | auto | `subsystem4_test.c` | `[TEST] S4-U06 PASS` |
| S4-U07 | the IMU pitch reaches Subsystem 1's telemetry and its speed and obstacle values are not overwritten | IF-01 | auto | `subsystem4_test.c` | `[TEST] S4-U07 PASS` |
| S4-U08 | an IMU fault bit is set and cleared in `active_fault_mask`, other subsystems' bits untouched | IF-08 | auto | `subsystem4_test.c` | `[TEST] S4-U08 PASS` |

**PC test (no hardware):**

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U09 | the real driver and tasks on a mock kernel with a fake sensor and the real headers of Subsystems 1 and 2: a full drive (calibration, knock, hump, turn), distance from fixed speed and from S2's encoders, latched collision, sensor unplugged and plugged in, wrong mounting | IF-01, IF-07, IF-08, IF-09, hump measurement | auto (PC) | `cd testing/subsystem4/host_test && ./run_sim.sh` | all 6 scenarios print `ALL PASSED` |

**Manual tests (flash the normal `make` image; the `HUMP`, `COLLISION` and `RUN_SUMMARY` lines come from Subsystem 1's CSV logger):**

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U10 | axis directions | mounting | manual | 1. Level and still: `pitch`/`roll` near 0. 2. Lift the nose. 3. Lower the nose. 4. Lift the left side. | nose up: pitch positive; nose down: negative; left side up: roll positive |
| S4-U11 | collision is detected, stops the robot, and releases | collision detection, IF-08 | manual | Tap the robot (or drive it into an obstacle with Subsystem 2 driving the motors). Wait 3 s. | console `COLLISION x.x g`, `faults now 0x40000000`, a `LOG_CSV,COLLISION` line; bit 30 in the reported fault code; the motors halt; after 3 s `faults now 0x0` |
| S4-U12 | turn rate and TURNING | turn monitoring | manual | Turn the robot left by hand about 90° in one second, on a level table. | `turn` is positive while turning and the state shows TURNING |
| S4-U13 | hump peak height accuracy | hump measurement (project brief) | manual | Drive over humps of known, ruler-measured height, 5 runs each, with `S4_CM_PER_TICK` set and Subsystem 2's encoders counting. Compare the `LOG_CSV,HUMP` `peak_mm` with the ruler. | each peak within ±20% of the measured height (team target) |
| S4-U14 | hump detection rate | hump measurement (project brief) | manual | 10 full runs of the course. Count missed humps and false humps from the `HUMP` lines. | at least 95% of humps found, at most 1 false hump per run (team targets) |
| S4-U15 | highest hump of the run is reported | project brief: report the highest hump peak | manual | Call `s4_publish_summary()` when the mission completes. | `LOG_CSV,RUN_SUMMARY,…highest_mm=` equals the largest `peak_mm` of the run |
| S4-U16 | an unplugged IMU is reported and recovers | robustness, IF-08 | manual | Run, unplug the IMU, then plug it back in. | console `IMU not answering…`; bit 28 set and the motors halt; after plugging in it recalibrates (keep the robot still) and bit 28 clears |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console; **auto (PC)** = run on a laptop;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
