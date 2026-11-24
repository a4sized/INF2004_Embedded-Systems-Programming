# Subsystem 5 — test log

**Subsystem:** Ultrasonic obstacle detection, obstacle width, and bypass  ·  **Student ID:** 2500815  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change and are never
reused.

### Automatic tests (`make TEST=1`)

All checks are in `app_program/subsystem5/subsystem5_test.c` and call pure
Subsystem 5 functions: no hardware, motion or other subsystem is needed.
Each test passes when the console prints `[TEST] <ID> PASS`.

| ID | Area | What is tested |
|---|---|---|
| S5-U03 | Obstacle detection | Speed-cap zones: no obstacle gives `S5_FULL_SPEED_CAP`; within `S5_SLOW_DISTANCE_CM` (30 cm) gives `S5_SLOW_SPEED_CAP`; at `S5_STOP_DISTANCE_CM`, and from `S5_STOP_TRIGGER_CM` (`S5_BRAKE_MARGIN_CM` early), gives 0 (stop). |
| S5-U07 | Obstacle detection | Hysteresis: once stopped, the car stays stopped inside `S5_RESUME_GAP_CM` and resumes beyond it. |
| S5-U09 | Obstacle width | Sideways distance: straight ahead (90°) is 0 cm; 20 cm away and 30° off axis on either side is 10 cm. |
| S5-U28 | Obstacle width | Resolution: 1° apart gives a different sideways distance (100 cm at 21° and 22°: 36 and 37 cm). |
| S5-U31 | Obstacle width | Planned edges (10–20 cm width range): a 5 cm lower bound is planned at 15 cm; a measured 8 cm stays 8 cm; a lower-bound side is not planned short of its real edge when the other side reads wide (`S5_EDGE_SPREAD_CM`); measured 4 + 3 cm (under 10 cm) plans the right at 6 cm. |
| S5-U12 | Bypass direction | Obstacle gone after the scan gives `CONTINUE`. |
| S5-U13 | Bypass direction | Failed scan (no profile) gives `STOP`. |
| S5-U14 | Bypass direction | Both edges found: the closer edge wins (right at 4 cm; left at 3 cm); equally far gives `LEFT`. |
| S5-U20 | Bypass direction | Only the left edge found: go round the left, both before and after the reverses are used up. |
| S5-U22 | Bypass direction | No edge found gives `REVERSE`; with the reverses used up, `STOP`. |
| S5-U23 | Bypass direction | Room to swing out (35° turn, 12 cm `S5_OUT_LEG_MIN_CM`): from 25 cm a 15 cm edge fits and a 30 cm edge does not; from 55 cm the 30 cm edge fits. |
| S5-U25 | Bypass direction | Left too wide: go round the right. |
| S5-U26 | Bypass direction | Too wide on both sides from here: go round the narrower side (left), reversing 210 mm first. |
| S5-U27 | Bypass direction | Both sides need more than `S5_BYPASS_MAX_BACKUP_MM` (250 mm) of reversing: `STOP` (wait). |
| S5-U29 | Bypass direction | Decision from planned edges: a measured left 10 cm beats a lower-bound right (planned 13 cm); right planned 6 cm beats left planned 7 cm. |
| S5-U18 | Bypass movement | Drive maths: one wheel circumference is one encoder revolution; four 90° turns make one full turn (within 3 mm). |
| S5-U34 | Bypass movement | Reversing distance: from 10 cm a 5 cm edge needs no reversing; a 20 cm edge needs 217 mm. |

### Manual track test

| ID | Area | What is tested | Steps | Expected result |
|---|---|---|---|---|
| S5-M01 | Obstacle detection, obstacle width, bypass direction, bypass movement, line recovery | Full obstacle bypass, from stopping to handing back to line following | 1. Place a box 10–20 cm wide on a straight line, with free space on both sides. 2. Start line following toward it. 3. Let the car stop, scan, go round and rejoin the line without touching it. 4. Watch the car, the USB console and the dashboard. | 1. The car stops before the box at the set distance (`S5_STOP_TRIGGER_CM`, 10 cm): `S5_OBSTACLE,STOP`. 2. The servo sweeps: `S5_FINE_SCAN,START` then `S5_FINE_SCAN,DONE`, and `S5 obstacle width: <n> cm (left …, right …)`. 3. The dashboard shows the same width under `Obstacle Width (S5)`. 4. `S5 bypass decision: LEFT` or `RIGHT` (`S5_BYPASS,…`). 5. The car goes round the box on that side without touching it. 6. The car rejoins the line (`S5_DRIVE,DONE_ON_LINE`) and S5 hands back to line following (`S5 avoiding: done, handing back`), which carries on past the box. |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-19 | S5-M01 | `ed835e5` | Timothy Seow Yi Heng (2500815) | PASS | https://youtu.be/qFmwYBuu4pI | Box on a straight line; stopped at 10 cm, width shown on the dashboard, went round on the left, rejoined the line and handed back. |
