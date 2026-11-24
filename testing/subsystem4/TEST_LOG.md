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

| S4-U34 | a hump under ONE rear wheel (tricycle: front wheel stays down, nose never goes up, the car rolls), right and left: exactly one hump, height within ±20% of the true 15 mm from the roll and the track width; and the control run with the roll detector off (`S4_SIDE_ENABLE 0`) sees nothing, proving the old detector could not | IF-01, hump measurement | auto (PC) | scenarios 7, 7a and 7b of `./run_sim.sh` | all print `ALL PASSED`; the right and left humps read 14 to 15 mm, the control run reads 0 humps |

| S4-U35 | NO false hump from the roll detector: a tight curve with 0.06 g of sideways acceleration (3.4 degrees of fake roll), the car lifted and carried, tipped onto its side, turned over in the hand, a 0.8 g sideways shove on rough ground, a 30 degree slope | hump measurement | auto (PC) | scenarios 7c, 3, 3a, 3c, 3d, 3g, 3e and 3f of `./run_sim.sh` | all `ALL PASSED`; no `HUMP` line in any of them (without the turn-rate guard the curve reads a false 8 mm hump; without the lockout a tip-over read 91 mm) |



**Manual tests**



| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |

|---|---|---|---|---|---|

| S4-U10 | axis directions | mounting, turn monitoring | manual | Car on the table, powered. 1. Level and still: look at `pitch` and `roll`. 2. Lift the nose, then lower it. 3. Lift the left side.| level: both near 0. Nose up: pitch positive; nose down: negative. Left side up: roll positive. |

| S4-U11 | picked up: the wheels stop, and release after it is put down | project change: pick-up detection, IF-07, IF-10 | manual | Car driving on the floor (Subsystem 2 commanding movement). Lift it off the ground, hold it for 3 s, put it down on the floor and leave it alone. Once. | within about 0.5 s of the lift the wheels stop; console `PICKED_UP (tilt N deg): motors stopped`, `faults now 0x2`, `motion: PICKED_UP`; it stays stopped while held; about 3 s after it is put down: `put down`, `faults now 0x0` and it can drive again (with `S4_LIFT_LATCH 1` it stays stopped until `s4_clear_lifted()`) |

| S4-U30 | overturned: the wheels stop and stay stopped | project change: overturn detection, IF-07, IF-10 | manual | Car driving on the floor. Tip it onto its side, hold it there for 3 s, then set it upright on the floor and leave it alone. Once. | the wheels stop; console `OVERTURNED (tilt N deg): motors stopped` with N about 90; `motion: OVERTURNED`; it stays stopped while it lies on its side; |

| S4-U36 | hump under ONE rear wheel (tricycle chassis) | project brief: hump height | manual | Drive over the ruler-measured hump so that only the RIGHT rear wheel climbs it, once; then only the LEFT rear wheel, once. | one `HUMP` line per run, `peak_mm` within +/-20 % of the ruler; the nose does not go up and `roll` shows the climb |

| S4-U14 | all humps found from a single run, the highest one reported, no false stop | project brief: detect humps and report the highest peak | manual | One run of the course with 2 different hump heights on it. | one `HUMP` line for every hump, the larger peak is the one reported as `max_mm`, and the wheels never stop |

| S4-U16 | an unplugged IMU is reported and recovers | robustness, IF-07 | manual | Car still on the table. Unplug the IMU, wait 3 s, plug it back in and keep the car still. Once. | console `IMU not answering...`; `FAULT_IMU_SENS` set; after plugging in it recalibrates and the bit clears |



*Method:* **auto** = `TEST_CHECK` in code, result printed on the console; **auto (PC)** = run on a laptop;

**manual** = a person observes the result (LED, scope, meter, phone app…).



## Results



Add a new row for every run. Never edit or delete old rows.



| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |

|---|---|---|---|---|---|---|

| 2026-11-18 | S4-U20 | 1bdd64f | Darius | PASS | logs/2026-11-18_pc_sim.txt | packing: 21 checks, 0 failed |

| 2026-11-18 | S4-U09 | 1bdd64f | Darius | PASS | logs/2026-11-18_pc_sim.txt | scenarios 1, 2, 4, 5 all ALL PASSED (full drive with and without encoders, sensor unplugged, plugged in) |

| 2026-11-18 | S4-U23 | 1bdd64f | Darius | PASS | logs/2026-11-18_pc_sim.txt | scenario 2b: hump read 28 mm for the true 30 mm with encoders zeroed mid-hump |

| 2026-11-18 | S4-U22 | 1bdd64f | Darius | PASS | logs/2026-11-18_pc_sim.txt | scenario 3b: all 6 LED phases (0 to 200 ms) ALL PASSED, no ghost fault |

| 2026-11-18 | S4-U26 | 1bdd64f | Darius | PASS | logs/2026-11-18_pc_sim.txt | scenarios 3 and 3a (automatic release and LIFT_LATCH 1) ALL PASSED |

| 2026-11-18 | S4-U27 | 1bdd64f | Darius | PASS | logs/2026-11-18_pc_sim.txt | scenarios 3c (on its side, then set upright) and 3d (upside down) ALL PASSED |

| 2026-11-18 | S4-U28 | 1bdd64f | Darius | PASS | logs/2026-11-18_pc_sim.txt | scenarios 1, 3e and 3f ALL PASSED: knock, rough ground and 30 degree slope never stop the car |

| 2026-11-18 | S4-U33 | 1bdd64f | Darius | PASS | logs/2026-11-18_pc_sim.txt | scenario 3g: PICKED_UP twice, OVERTURNED once, one halt only |

| 2026-11-18 | S4-U29 | 1bdd64f | Darius | PASS | logs/2026-11-18_pc_sim.txt | scenario 6: not reported as overturned; the calibration FAILED line is expected |

| 2026-11-18 | S4-U01 | 1bdd64f | Darius | PASS | logs/2026-11-18_unit.txt | |

| 2026-11-18 | S4-U02 | 1bdd64f | Darius | PASS | logs/2026-11-18_unit.txt | |

| 2026-11-18 | S4-U03 | 1bdd64f | Darius | PASS | logs/2026-11-18_unit.txt | |

| 2026-11-18 | S4-U04 | 1bdd64f | Darius | PASS | logs/2026-11-18_unit.txt | |

| 2026-11-18 | S4-U05 | 1bdd64f | Darius | PASS | logs/2026-11-18_unit.txt | |

| 2026-11-18 | S4-U06 | 1bdd64f | Darius | PASS | logs/2026-11-18_unit.txt | |

| 2026-11-18 | S4-U07 | 1bdd64f | Darius | FAIL | logs/2026-11-18_unit.txt | test.c line 66. Subsystem 5 was running and rewrites the obstacle field after every reading, most likely overwriting the test's value 33. Test changed to wait for Subsystem 4's own status message and retry; re-run needed |

| 2026-11-18 | S4-U08 | 1bdd64f | Darius | PASS | logs/2026-11-18_unit.txt | |

| 2026-11-18 | S4-U17 | 1bdd64f | Darius | PASS | logs/2026-11-18_unit.txt | |

| 2026-11-18 | S4-U18 | 1bdd64f | Darius | PASS | logs/2026-11-18_unit.txt | |

| 2026-11-18 | S4-U19 | 1bdd64f | Darius | PASS | logs/2026-11-18_unit.txt | |

| 2026-11-18 | S4-U25 | 1bdd64f | Darius | PASS | logs/2026-11-18_unit.txt | |

| 2026-11-18 | S4-U07 | 1bdd64f | Darius | PASS | logs/2026-11-18_unit.txt | Re-run of the earlier FAIL. Fixed in 25a95bd: the test now waits for Subsystem 4's own status message and retries, because Subsystem 5 rewrites the obstacle field |
| 2026-11-24 | S4-U10 | 25a95bd | Darius | PASS | https://youtu.be/RMK1jYUtg2g | level pitch and roll near 0; nose up positive, nose down negative; left side up gives positive roll |
| 2026-11-24 | S4-U11 | 25a95bd | Darius | PASS | https://youtu.be/RMK1jYUtg2g | wheels stopped within about 0.5 s of the lift, stayed stopped while held, released after being put down |
| 2026-11-24 | S4-U30 | 25a95bd | Darius | PASS | https://youtu.be/SZSbGvP1IzI | wheels stopped on its side, stayed stopped; |
| 2026-11-24 | S4-U36 | 25a95bd | Darius | PASS | https://youtu.be/y29xQHxkbuE | A hump of 10mm successfully records shows up as 11 mm, which is within  20%|
| 2026-11-24 | S4-U14 | 25a95bd | Darius | PASS | https://youtu.be/y29xQHxkbuE | recorded 2 humps tested and successfully displays the higher hump |
| 2026-11-24 | S4-U16 | 25a95bd | Darius | PASS | https://youtu.be/ct8wM9yMECo | IMU not answering reported and FAULT_IMU_SENS set; recalibrated and bit cleared after replugging |
