# Vehicle Controller integration status

Date: 3 November 2026. Profile: normal SMP Pico W build, USB CDC,
`WIFI=cyw43 NET=none MQTT=0 TEST=0`.

The application starts Subsystem 1, 2, 3, 4 and 5 tasks, then starts the
Vehicle Controller. The controller begins in INIT with Buddy 3 permission
closed. It enters READY only after S4 reports IMU calibration, S5 reports
that its first sweep and a good forward reading are complete with no
obstacle/fault, Buddy 3 publishes a fresh ready IR snapshot, and Buddy 1
has reported an initial Wi-Fi connection. START from Buddy 1 is accepted
only from READY. STOP or lost onboard sensor readiness revokes following;
ESTOP or a fatal fault enters FAULT and invokes Buddy 2's safety halt.
An established run continues after a later Wi-Fi disconnect. STOPPED and
FAULT require CLEAR and a new START; automatic resumption is disabled.
Controller state changes are logged through Buddy 1's CSV logger.
`FAULT_WIFI` remains in telemetry but is excluded from the motor hazard
mask so a later Buddy 2 fault override will not contradict the chosen
continue-following policy.

The pure policy host test passed with Clang strict warnings,
AddressSanitizer and UndefinedBehaviorSanitizer. The full ARM image
compiled and linked. These are software checks; no Pico or car test was
performed.

This gate applies to the normal `TEST=0` image. Buddy 2's existing
`TEST=1` self-test directly sends nonzero motor intent, so that test
profile must be treated separately from the Vehicle Controller contract.

## Integration points still open

| Owner | Signal | Current status |
|---|---|---|
| Buddy 1 | Call `vehicle_controller_set_wifi_connected(true)` on first confirmed link and `false` on disconnect, from task context | Implemented in `cyw43_utk.c` and `mqtt_utk.c` for networked builds. The current team IR build has `NET=none MQTT=0`, so it does not report a connection. |
| Buddy 3 | Publish `ir_line_snapshot_t.b_ready=true` after successful GPIO setup using the accepted fixed black=HIGH polarity | Implemented following the user's report that the car demo readings worked. The earlier recorded successful demo used GP7/GP4/GP16; current GP7/GP2/GP6 wiring and polarity still need a saved trace before independent validation. |
| Buddy 5 | `subsystem5_startup_scan_ready()` | Implemented; first scan and clear forward path require car verification. |
| Buddy 4 | `s4_is_calibrated()` | Existing getter, now polled; actual calibration needs car verification. |
| Team | Define obstacle takeover/rejoin permission and navigation mode transitions | The controller currently grants ordinary following only. S5 speed cap can stop at an obstacle; bypass and handback are not integrated. |

Before any moving trial, verify motor cutoff, initial readiness, START,
STOP, ESTOP, fault clearance, Wi-Fi loss during a run, and startup with
missing/failed sensors on the actual car. The current image remains
motion-closed because the network hooks are inactive and START cannot arrive
through MQTT in this `NET=none MQTT=0` profile.

## YiRui network bridge merged (3 November 2026)

Commit `26b79b3` was fast-forwarded into `rance/IR-branch`. Its Wi-Fi and
MQTT callbacks now call `vehicle_controller_set_wifi_connected()` on link
establishment and loss. This is inactive in the current team IR UF2, which
still builds with `NET=none MQTT=0`; that UF2 compiled after the merge.

An earlier local `NET=lwip MQTT=1` build stopped because `MQTT_TOPIC_CMD`
was undefined. Buddy 1 subsequently defined it as `"robot/inf2004team15/cmd"` in
`config/wifi_secrets.example.h` and added a safe `#ifndef` fallback inside
`mqtt_utk.c`. Both `NET=lwip MQTT=1` and `NET=lwip MQTT=1 TEST=1` compile and
link cleanly without errors.
