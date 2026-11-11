# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `S1-U12`, `S1-U13`, `INT-07`, `SYS-08`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console / `telemetry_decoder.py` output in view.
- **Landscape**, steady, good light. No music needed.

## Recommended clips to record (Subsystem 1, Integration, System)

| Priority | Subsystem | Test ID | Clip target (15–60 s) | What must be visible in frame |
|---|---|---|---|---|
| 1 | S1 | `S1-U12` | GP19 heartbeat LED (2 Hz) | Commit hash + Pico GP19 LED blinking evenly at 250 ms intervals (with `S1-U01`–`S1-U10` `[TEST] PASS` console if running `TEST=1`) |
| 2 | S1 | `S1-U13` | Wi-Fi join + 1 Hz telemetry decode | Commit hash + Mosquitto `-v` window + `python scripts/telemetry_decoder.py` printing `[SEQ #...]` every 1 s cleanly (`FAULT=NONE`) |
| 3 | S1 | `S1-U14` | MQTT RX commands (`PING`, `INVALID`, `CLEAR`, `ESTOP`) | MQTTX / `mosquitto_pub` sending valid + invalid payloads + decoder showing `FAULT_CMD_REJECTED` / `FAULT_ESTOP` set & cleared |
| 4 | INT | `INT-06` / `INT-07` | Barcode decode + VC state transitions (`INIT->READY->FOLLOWING`) | Sweeping barcode across GP6 showing decoder `Barcode` char update, plus MQTT `START`/`STOP`/`ESTOP`/`CLEAR` transitions |
| 5 | INT | `INT-09` / `INT-10` | Fault halt / Wi-Fi resilience + S5 obstacle handoff | Chassis propped or on track showing Tier-1 motor cutoff (`ESTOP`/collision), S5 `VC_AVOIDING` transition, or AP drop continue |
| 6 | SYS | `SYS-02`–`SYS-08` | End-to-end track circuit & Ground Station | Full car navigating track (line + hump + obstacle + barcode) with `telemetry_decoder.py` / MQTTX live in corner or split-screen |

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
