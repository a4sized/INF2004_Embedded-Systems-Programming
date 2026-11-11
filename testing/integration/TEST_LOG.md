# Integration — test log

Tests that two or more subsystems work together. Covers the shared interfaces in `TEAM.md` Section 5 and cross-subsystem contracts (`Vehicle Controller`, fault propagation, and obstacle handoff). Both owners agree on each interface test and sign off the result.

See `testing/README.md` for how to run tests and record results.

Related integration records:
- Pending fault-contract notes: `testing/integration/FAULT_INTEGRATION_PENDING.md` (`FLT-01`–`FLT-08`)
- Vehicle Controller policy & open handoffs: `testing/integration/VEHICLE_CONTROLLER_TEST_LOG.md`
- Host policy suite: `testing/integration/vehicle_controller_policy_test.c`

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | Legacy example samples flow from S2 to S3 (≥ 5 in 2 s) | Legacy `IF-01` / `IF-02` (`mbx_samples`) | auto | `app_program/subsystem3/subsystem3_test.c` (only active when starter `consumer_task` is built instead of `BUDDY3_TEAM_IR=1`) | `[TEST] INT-01 PASS` (or N/A in `BUDDY3_TEAM_IR=1` team image) |
| INT-02 | Legacy example summary lines flow from S3 to S4 | Legacy `IF-03` (`mbf_summary` / `flg_events`) | dropped | Legacy template interface removed; `mbf_summary`/`flg_events` stubbed `0` in `app_program/subsystem4/subsystem4.c` and superseded by `INT-03`–`INT-11`. ID retained per `testing/README.md`. | Superseded by `INT-03`–`INT-11` |
| INT-03 | S4 -> S1 IMU telemetry (`s1_set_pitch`, `s1_set_car_state`, `s4_get_max_hump_mm`) updates `S1_TELEMETRY_PKT` without overwriting S2/S5 fields | `IF-01`, `IF-09` (`TEAM.md` §5) | auto + manual | Auto: flash `make TEST=1` (`subsystem4_test.c` `S4-U07`, `S4-U19`). Live: flash `make NET=lwip MQTT=1`, tilt nose up/down and watch `telemetry_decoder.py` `Pitch` and `Terrain`. | `[TEST] S4-U07 PASS` & `[TEST] S4-U19 PASS`; decoder tracks live pitch & motion state while `obstacle_cm` stays intact |
| INT-04 | S4 -> S1 IMU fault (`FAULT_IMU_SENS`) bit isolation & CSV event logging (`HUMP`, `PICKED_UP`, `OVERTURNED`, `PUT_DOWN`, `IMU_FAULT`, `RUN_SUMMARY`) | `IF-07`, `IF-08`, `IF-10` (`TEAM.md` §5) | auto + manual | Auto: `subsystem4_test.c` `S4-U08`, `S4-U17`, `S4-U25`. PC: `testing/subsystem4/host_test/run_sim.sh` (`S4-U09`, `S4-U26`–`S4-U33`). Live: `S4-U11` / `S4-U30` lift or tip chassis / `S4-U16` unplug IMU. | `[TEST] S4-U08 PASS`, `S4-U17 PASS`, `S4-U25 PASS`; lifting/overturning sets bit 4 (`0x010`), halts motors, logs `PICKED_UP` (`6`) / `OVERTURNED` (`7`), and clears after `S4_PUTDOWN_MS` (2 s) rest (`PUT_DOWN`) |
| INT-05 | S5 -> S1 ultrasonic `obstacle_cm` (`s1_set_obstacle`), `FAULT_US_SENS` streak reporting/recovery, and `S5_FAULT` CSV log | `IF-01`, Tier-2 `FAULT_US_SENS` | auto + manual | Auto: `subsystem5_test.c` `S5-U02` (sensor fitted). Live: flash `make NET=lwip MQTT=1`, hold target at 15 cm (watch decoder `Obstacle = 15 cm`), then unplug/block sensor and reconnect. | `[TEST] S5-U02 PASS`; decoder reflects live obstacle cm; 3 consecutive sensor errors raise `FAULT_US_SENS` (`0x008`) and recovery clears it |
| INT-06 | S3 -> S1 decoded Code 39 barcode reaches `barcode_id` telemetry, logs `BARCODE_DECODED`, and hands off W/Z junction turns (or `FAULT_IR_NAV` on timeout) | `IF-01`, S3->S1 barcode & nav handoff | manual | 1. Flash `make NET=lwip MQTT=1`. 2. Pass valid Code 39 barcode (`'A'`..`'Z'`) across GP6 sensor. 3. Observe USB console and `scripts/telemetry_decoder.py`. | USB prints `LOG_CSV,...,BARCODE_DECODED,letter=<ch>,order=F\|R`; `telemetry_decoder.py` shows `Barcode = '<ch>'` (W/Z arms `JUNCTION_ARMED` -> `JUNCTION_TURN`, clearing `barcode_id` to `0`) |
| INT-07 | CYW43 / lwIP MQTT <-> S1 <-> Vehicle Controller readiness & remote command gate (`INIT -> READY -> FOLLOWING -> STOPPED -> FAULT -> INIT`) | `IF-02`, `IF-03`, `IF-04`, `TEAM.md` §11 | manual | 1. Configure `config/wifi_secrets.h` and start Mosquitto. 2. Flash `make NET=lwip MQTT=1 TEST=0`. 3. Verify `VC state: INIT -> READY` after Wi-Fi link & S3/S4/S5 readiness. 4. Publish `START`, `STOP`, `ESTOP`, `CLEAR` on `robot/inf2004team15/cmd`. | `VC state` advances only after initial Wi-Fi (`vehicle_controller_set_wifi_connected(true)`) + sensors ready; `START` before `READY` is ignored; `STOP->STOPPED`, `ESTOP->FAULT`, `CLEAR->INIT->READY` |
| INT-08 | Vehicle Controller pure transition policy (readiness gates, Wi-Fi loss resilience mid-run, `VC_AVOIDING` bypass state, `STOP`/`ESTOP`/`TIER1_FATAL_MASK` priority, NULL/invalid state) | `TEAM.md` §11, `VEHICLE_CONTROLLER_TEST_LOG.md` | auto (PC) | Run `gcc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined -Itesting/subsystem3/fakes -Iapp_program testing/integration/vehicle_controller_policy_test.c app_program/vehicle_controller_policy.c -o /tmp/vc_test && /tmp/vc_test` | Exit code 0 with zero assertions failed and zero ASan/UBSan diagnostics |
| INT-09 | Fault propagation & motor safety halt (`S1 -> VC -> S2`, `FLT-01`–`FLT-08`): Tier-1 (`FAULT_ESTOP`, `FAULT_IMU_SENS`, `FAULT_IR_NAV`) stops PWM (`MOTOR_SAFETY_HALT`), Tier-2 (`FAULT_US_SENS`) stops following (`VC_STOPPED`), `FAULT_WIFI` mid-run stays autonomous | `FAULT_INTEGRATION_PENDING.md`, `TIER1_FATAL_MASK` | manual | 1. Enter `VC_FOLLOWING` with chassis propped off table. 2. Send `ESTOP`, lift/tip chassis (`FAULT_IMU_SENS`), or trigger junction timeout (`FAULT_IR_NAV`): verify wheels halt immediately and refuse `START` until fault clears + `CLEAR`. 3. Re-enter `VC_FOLLOWING` and disconnect AP/broker: confirm wheels keep running (`FAULT_WIFI` logged) and reconnect when restored. | Tier-1 forces `VC_FAULT` + `motion_trigger_safety_halt()` (PWM 0); `CLEAR` clears `FAULT_ESTOP`/`FAULT_CMD_REJECTED`/`FAULT_IR_NAV`; `FAULT_WIFI` raises `0x020` without stopping wheels |
| INT-10 | Obstacle bypass arbitration (`S5 <-> VC <-> S3 <-> S2`, `VC_AVOIDING`): S5 pauses `ir_motion_consumer`, drives bypass + line return (`s5_bypass_go_round`) via `s2_set_ir_velocity()` + S2 encoders (`IF-06`), then hands back with search direction (`subsystem5_take_line_side` -> `ir_motion_consumer_request_search`) | `IF-06`, `TEAM.md` §11 | manual | 1. Start `VC_FOLLOWING` toward an obstacle with free clearance beside it. 2. Observe S5 stop car, fine-scan (`S5_FINE_SCAN`), enter `VC_AVOIDING` (`subsystem5_set_drive_permission(true)` while S3 paused), drive `s5_bypass_go_round()`, and resume line following (`VC_FOLLOWING` + `IR_FOLLOW_SEARCH`). | Zero wheel-fighting between S3 and S5; `VC state: FOLLOWING -> AVOIDING -> FOLLOWING` cleanly bypasses and reacquires the line |
| INT-11 | S2 encoder odometry -> S4 distance & hump height scaling (`IF-06`, `get_left_encoder_ticks` / `get_right_encoder_ticks`) | `IF-06` (`TEAM.md` §5) | manual | Push or drive car forward 50 cm and traverse known ruler-measured hump (`S2-U06` + `S4-U13`). | Left & right encoder ticks both count positively moving forward; `LOG_CSV,HUMP` `peak_mm` agrees within ±20% |

*Note on `IF-05` (`tid_led`):* In the final robot application, `tid_led` is still exported as `IF-05` and verified active by `S1-U01`/`S1-U02`, while S4 communicates status and faults through `IF-01`, `IF-07`, `IF-08`, and `IF-09` instead of suspending `tid_led`.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console; **auto (PC)** = run on a host computer;
**manual** = a person observes the result (hardware, serial log, Mosquitto / `telemetry_decoder.py`).

## Results

Add a new row for every post-commit run (`git rev-parse --short HEAD`). Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-11 | INT-01 | `f4a367a` | Khoo Yi Rui (2501283) | N/A | `../subsystem1/logs/2026-11-11_s1_unit.txt` | Legacy starter S2->S3 mailbox inactive in `BUDDY3_TEAM_IR=1` team image |
| 2026-11-11 | INT-02 | `f4a367a` | Khoo Yi Rui (2501283) | N/A | — | Dropped legacy template test; superseded by `INT-03`–`INT-11` |
| 2026-11-11 | INT-03 | `f4a367a` | Khoo Yi Rui (2501283) | PASS | `../subsystem1/logs/2026-11-11_s1_serial.txt`, `../subsystem1/logs/2026-11-11_s1_u14_decoder.txt`, `../subsystem1/logs/2026-11-11_s1_unit.txt` | `S4-U19 PASS`; live `Pitch` (`+19°`), `RAMP_UP`/`RAMP_DOWN`, peak `33 mm` with `Obstacle = 511 cm` preserved (`S4-U07` only raced live `s5_sensor_task` in `TEST=1`) |
| 2026-11-11 | INT-04 | `f4a367a` | Khoo Yi Rui (2501283) | PASS | `../subsystem1/logs/2026-11-11_s1_unit.txt`, `../subsystem1/logs/2026-11-11_s1_serial.txt`, `../subsystem1/logs/2026-11-11_s1_u14_decoder.txt` | `S4-U08` & `S4-U17 PASS`; live `COLLISION 1.0 g` + `IMU_FAULT` CSV and `[0x051] FAULT_ESTOP \| FAULT_IMU_SENS \| FAULT_CMD_REJECTED` bit isolation |
| 2026-11-11 | INT-08 | `f4a367a` | Khoo Yi Rui (2501283) | PASS | `logs/2026-11-11_int08_vc_policy.txt` | `gcc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined` exited 0 (`INT-08 PASS`) |
