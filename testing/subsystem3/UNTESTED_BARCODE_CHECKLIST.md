# Barcode checks awaiting hardware and team decisions

Status: **OPEN / UNTESTED** (23 October 2026). Owner: Buddy 3. This list is
for checks that the available host tests and ARM compilation cannot establish.
Add dated evidence and change a row's status only after performing that check.

| ID | Check to close | Needed evidence | Status |
|---|---|---|---|
| BC-01 | GP16 black/white contrast at mounted height | Sensor model, supply/DO voltage, height, lighting, saved GPIO trace | UNTESTED |
| BC-02 | Rising/falling interrupt capture of real bars and gaps | Repeated USB traces, measured shortest pulse, missed-edge count | UNTESTED |
| BC-03 | Capture timeout, ring overflow and invalid-edge rejection on Pico | Saved traces showing valid and injected failure cases | UNTESTED |
| BC-04 | 5 ms line sampling while barcode capture and USB output run | Full 60-second gap counters with at least 99% in 4–6 ms and none above 10 ms | UNTESTED |
| BC-05 | Complete-code start/end, trailing white and clearance rules | Actual course samples scanned at final sensor height and supported speeds | UNTESTED |
| BC-06 | Approved full pattern dictionary and command mapping | Lecturer/team reference for every required command and scan direction | UNRESOLVED |
| BC-07 | Decoder tolerance and rejection of truncated, noisy and ambiguous scans | Separate tuning and validation traces, success/rejection counts by speed | UNTESTED |
| BC-08 | Duplicate suppression and rearm after strip clearance | Repeated passes and dwell over one strip with capture IDs | UNTESTED |
| BC-09 | Barcode-to-turn position and one-action transaction | Course placement rule, distance/encoder calibration, acknowledgement and turn traces | UNTESTED |
| BC-10 | Stop, fault and obstacle takeover during capture/navigation | Integrated traces showing cancellation and no stale command replay | UNTESTED |

Host checks already passed for queue bounds, edge-window timeout/wrap and
provisional duration normalization. Those are recorded in
`IR_SENSING_TEST_LOG.md`; they do not close any physical or integrated row
above. No symbol decoder or navigation transaction is implemented yet.
