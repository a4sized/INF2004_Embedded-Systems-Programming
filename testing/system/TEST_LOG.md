# System — test log

Tests of the complete robot against project requirements, run on the full assembled Pico W hardware with Ground Station (`NET=lwip MQTT=1 TEST=0`, Mosquitto + `scripts/telemetry_decoder.py` + MQTTX). Run the full set before every demo and hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Cold boot readiness gate, 2 Hz GP19 LED (+ optional boot tone if `BUZZER_MUTED=0`), and steady 1 Hz MQTT telemetry over Wi-Fi for 2 minutes | Telemetry link (1 Hz), `VC_INIT -> VC_READY` safety gate | manual | 1. Start Mosquitto (`mosquitto -c mosquitto_local.conf -v`) and `python scripts/telemetry_decoder.py`. 2. Power on assembled robot on level track before start (path clear). 3. Observe for 2 minutes without sending `START`. | GP19 LED blinks at 2 Hz (`BUZZER_MUTED=1` keeps buzzer silent by default); motors stay stopped in `VC_READY`; decoder receives ~120 packets at 1.0 s ± 0.1 s with `FAULT = NONE (ALL CLEAR)`; record video clip |
| SYS-02 | Remote mission control (`START`, `STOP`, `CLEAR`) & autonomous line following on track | Remote MQTT teleoperation + IR line following | manual | 1. Place ready robot (`VC_READY`) centered on track line. 2. Publish `START` to `robot/inf2004team15/cmd`. 3. Let car track straight and curve for ≥ 10 s. 4. Publish `STOP`, confirm immediate stop, verify `START` stays locked until `CLEAR` + `START`. | Car follows black line smoothly on `START`, stops promptly on `STOP` (`VC_STOPPED`), refuses auto-restart until `CLEAR` + `START`; record video clip |
| SYS-03 | Remote emergency stop (`ESTOP`) and fault-clearance interlock | Critical safety stop (`FAULT_ESTOP`) | manual | 1. While car is driving (`VC_FOLLOWING`), publish `ESTOP` to `robot/inf2004team15/cmd`. 2. Observe motors and `telemetry_decoder.py`. 3. Attempt `START` before clearing. 4. Publish `CLEAR` then `START`. | Wheels stop immediately (`VC_FAULT` / `MOTOR_SAFETY_HALT`); decoder prints `FAULT_ESTOP`; `START` rejected while latched; `CLEAR` + `START` restores operation; record video clip |
| SYS-04 | Ultrasonic obstacle stop, fine scan, full bypass with line recovery (or hold if too big) | Autonomous obstacle detection & avoidance (`S5` + `VC` + `S3`) | manual | 1. Run `VC_FOLLOWING` toward a stationary obstacle on the track (and verify held stop if oversized). 2. Watch approach, stop at `S5_STOP_DISTANCE_CM`, servo fine scan, `VC_AVOIDING` bypass (`s5_bypass_go_round`), line search reacquisition, and `telemetry_decoder.py` `Obstacle` field. | No collision; `Obstacle` distance updates in telemetry; car stops cleanly, bypasses obstacle (`VC_AVOIDING`), and reacquires line (`VC_FOLLOWING`) without conflict (or retraces/holds if oversized); record video clip |
| SYS-05 | Live Code 39 barcode decoding & junction turn during run -> Ground Station telemetry & CSV log | Optical barcode decoding (`GP6`), `W`/`Z` junction nav (`S3 -> S1`) | manual | 1. Drive robot (`VC_FOLLOWING`) past a printed Code 39 barcode (`'A'`..`'Z'`, ≥ 5 mm bars) placed at GP6 sensor spacing before a junction. 2. Observe `telemetry_decoder.py` and serial `LOG_CSV`. | Ground Station `Barcode` updates to decoded letter (clears to `'0'` at `JUNCTION_TURN`); USB prints `BARCODE_DECODED` and `JUNCTION_ARMED`/`JUNCTION_TURN`/`JUNCTION_DONE` (or `FAULT_IR_NAV` on timeout); record video clip |
| SYS-06 | Speed hump traversal, peak height telemetry, and lifted / overturned safety halt & recovery | Hump height reporting & lift/overturn safety (`S4 -> S1` & `S2`) | manual | 1. Drive robot over a ruler-measured speed hump; check `Pitch`, `Terrain` (`RAMP_UP` / `HUMP_PEAK` / `RAMP_DOWN`), and `Peak Hump Height = ... mm` in telemetry. 2. Lift (`PICKED_UP`) or tip/overturn (`OVERTURNED`) chassis, then set upright and still on track for `S4_PUTDOWN_MS` (2 s). | Telemetry tracks ramp climb/descent and reports peak hump height within ±20%; lift/overturn raises `FAULT_IMU_SENS`, sets `State` `PICKED_UP (6)`/`OVERTURNED (7)`, halts drive immediately, and clears cleanly after put-down (`PUT_DOWN`); record video clip |
| SYS-07 | Mid-run Wi-Fi / Mosquitto disconnect resilience & automatic reconnect | Graceful degraded autonomy (`FAULT_WIFI` non-fatal) | manual | 1. With robot driving in `VC_FOLLOWING`, stop Mosquitto (`Ctrl+C`) or switch off AP for 10–15 s. 2. Confirm robot stays following track safely. 3. Restore AP / restart Mosquitto (`-c mosquitto_local.conf -v`). | Robot keeps line following autonomously without motor halt; serial shows `FAULT_WIFI` (`0x020`); link auto-rejoins when restored and telemetry stream resumes |
| SYS-08 | Full end-to-end demo circuit run (cold boot to finish `STOP`) | Complete integrated product acceptance | manual | Execute complete circuit run from cold boot (`SYS-01` -> `START` -> line + hump + barcode + obstacle -> finish `STOP`) with `telemetry_decoder.py` and track in same frame. | Robot completes entire course safely with live 1 Hz Ground Station telemetry (`seq`, `obstacle_cm`, `pitch_deg`, `hump_height_mm`, `barcode_id`, `fault_mask`); record full demo video |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (hardware behavior, USB console, Mosquitto + `telemetry_decoder.py`).

## Results

Add a new row for every post-commit run (`git rev-parse --short HEAD`). Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
