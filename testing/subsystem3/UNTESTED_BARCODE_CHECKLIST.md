# Barcode checks awaiting hardware and team decisions

Status: **OPEN / PARTIAL** (4 November 2026). Owner: Buddy 3. This list is
for checks that the available host tests and ARM compilation cannot establish.
Add dated evidence and change a row's status only after performing that check.

| ID | Check to close | Needed evidence | Status |
|---|---|---|---|
| BC-01 | GP6 black/white contrast at mounted height | Sensor model, supply/DO voltage, height, lighting, saved GPIO trace | UNTESTED |
| BC-02 | 5 ms polling capture of real bar/gap transitions | Repeated USB traces, measured shortest bar/gap duration and missed-transition count at intended speed | PARTIAL |
| BC-03 | Capture timeout, ring overflow and invalid-edge rejection on Pico | Saved traces showing valid and injected failure cases | UNTESTED |
| BC-04 | 5 ms line sampling while barcode capture and USB output run | Full 60-second gap counters with at least 99% in 4–6 ms and none above 10 ms | UNTESTED |
| BC-05 | Complete-code start/end, trailing white and clearance rules in both scan directions | Actual course samples scanned left-to-right and right-to-left at final sensor height and line-following speed | PARTIAL |
| BC-06 | Hardcoded action letters and straight-through default | A=LEFT and Z=RIGHT are coded; user will edit the letters as needed. Other decoded letters should go straight, but junction handling is deferred. | PARTIAL |
| BC-07 | Decoder tolerance and rejection of truncated, noisy and ambiguous scans | Separate tuning and validation traces, success/rejection counts by speed and direction | UNTESTED |
| BC-08 | Duplicate suppression and rearm after strip clearance | Repeated passes and dwell over one strip with capture IDs | UNTESTED |
| BC-09 | One-action turn transaction after decoded letter | GP6 black-to-white junction cue, Subsystem 2 request/acknowledgement, turn-completion and timeout traces; no distance rule is requested | UNTESTED |
| BC-10 | Stop, fault and obstacle takeover during capture/navigation | Integrated traces showing cancellation and no stale command replay | UNTESTED |
| BC-11 | GP6 junction cue and later decoded turn | Mounted GP6 white-black-white trigger, in-turn GP6 white-black-white and either GP7/GP2 white-black confirmation with a current black line reading, turn-position repeatability, timeout and false-positive trials once integrated navigation is enabled | UNTESTED |

Host checks already passed for queue bounds, edge-window timeout/wrap and
provisional duration normalization. A host decoder check also passed using
run widths from the user's printed A and Z screenshots. Those are recorded
in `IR_SENSING_TEST_LOG.md`; they do not close any physical or integrated
row above. The single-letter decoder does not yet issue navigation turns.

Wiring update (1 November 2026): current barcode DO is GP6 on Grove 5,
superseding GP16 in historical logs. Verify the Grove 5 mapping before
closing BC-01 or BC-11.

Current-main allocation update (1 November 2026): right line DO moves
from GP4 to GP2. Verify GP2 black/white polarity and mounted tracking;
GP4 observations do not validate GP2. The pin layout places GP6 on
Grove 4, so check that connector against the earlier Grove 5 report.

Main merge update (4 November 2026): main disabled GP6 edge IRQ registration
because Subsystem 2 owns the bank-wide GPIO interrupt through its encoder ISR.
At that point the integrated image had no barcode capture path; the polling
update below supersedes that limitation.

Polling update (4 November 2026): both sensing-only and team IR images now
feed GP6 level changes sampled every 5 ms into the barcode edge queue without
registering a GPIO interrupt. A host test decoded A and Z from simulated
5 ms samples. BC-02 requires a physical polling check: save repeated real
edge-duration and missed-bar
traces at intended scan speed. The 5 ms loop may miss bars narrower than a
sampling interval; physical validation remains OPEN.

Isolated USB test path (4 November 2026): the sensing-only UF2 polls GP6
without Subsystem 2 and prints raw B levels, captured edge
timestamps, overflow status, and decoded Code 39 letters to USB serial.
This allows BC-01/02/05/07 bench evidence. It does not validate integrated
encoder coexistence, motor-speed scanning or junction commands.

Connector confirmation (4 November 2026): the user says the mounted barcode
sensor is plugged into Grove 5. Confirm the Grove 5 DO-to-GP6 electrical
mapping by checking that the serial `B=` raw level changes over black/white.

Bidirectional update (4 November 2026): the Code 39 decoder now checks both
scan directions using the fixed A-Z symbol table. Host tests decoded A and Z
in both directions, including simulated 5 ms GPIO polling. No per-letter
training recordings are required. The robot's actual line-following speed,
shortest observed bar/gap duration and missed-transition rate still need
physical measurement; a narrow feature below the polling interval can be
missed even though host tests pass.

Physical A log (4 November 2026): the user's USB serial trace contains 14
barcode windows: eight with all 30 edges and six incomplete, with zero queue
overflows. GP6 alternates white/black; the shortest logged bar/gap interval
is 14.34 ms. One complete reverse-direction A window was replayed through
the decoder and succeeds after a trailing quiet interval. The earlier
firmware reset complete scans at the 2 s edge-capture deadline before that
quiet interval finished; a bounded 750 ms post-capture grace now separates
these two phases. The log's maximum sensing gap is 18.214 ms, so BC-02
remains PARTIAL until the intended car speed and USB-free integrated timing
are measured. Left/right sensor readings were intentionally excluded from
this assessment because the user used a different physical test setup.

Retest of physical A (4 November 2026): the revised sensing-only UF2 printed
`Code 39 decoded: A direction=REVERSE action=LEFT` twice. Among five logged
windows, three held 30 edges, one held 28, and one held 2; all reported
`invalid=0` and zero overflow. The third complete window had the expected
symbol positions but failed the first `*` width-separation check, consistent
with uneven scan speed. This confirms GP6 capture and reverse A decoding in
the user's bench setup. Forward A, Z in both directions, repeated success
rate, final mounting and moving-speed timing remain open. Left/right sensor
readings were excluded at the user's request.

Direction clarification: the first log's pass before the serial restart was
physically forward and its pass afterward physically reverse. Decoder
FORWARD/REVERSE refers to bar-pattern order, not car travel. The second log
has two on-Pico REVERSE-pattern decodes; its complete FORWARD-pattern window
now passes host replay after the width threshold changed from 1.5 to 1.4.
Both physical travel directions and the new threshold still need repeatable
on-Pico verification at actual line-following speed.

Six-pass A bench retest (4 November 2026): the user made three physical
forward passes followed by three reverse passes. Five of six complete
windows decoded A on the Pico (forward 2/3, reverse 3/3); one forward window
had correct `*A*` positions but failed the 2:1 wide-element consistency
limit on its final `*`. Two short fragments had 2 and 4 edges. No window
reported invalid capture or overflow. Bidirectional A decoding is now
physically demonstrated at hand-scan speed, but repeatability, Z and
line-following-speed performance remain open. The log has the older
`direction=` wording, so the `pattern_order=` build is not confirmed flashed.

Physical Z bench retest (4 November 2026): before the serial-monitor
restart, the user moved the paper physically forward and the Pico decoded
`Z pattern_order=REVERSE action=RIGHT` twice. Three other captures timed
out near the 2 s edge limit at 23–25 edges, with short fragments afterward.
After restart, the user moved it physically reverse: all six complete
windows decoded `Z pattern_order=FORWARD action=RIGHT`. All 14 windows
reported no invalid capture or overflow. Z and RIGHT mapping are now
physically demonstrated both ways at manual speed. BC-02/05/07 remain open
for repeated passes at actual car speed and final mounting.

Integrated moving diagnostic prepared (4 November 2026): the dedicated
`build_moving_barcode_test.py` profile follows through the Vehicle Controller
and Subsystem 2 after USB, IMU, ultrasonic and IR readiness plus a 3 s wait.
It uses a provisional 8 s barcode capture limit and 3 s white grace, and
logs `IR_TEST_READY`/`IR_CAPTURE` through Subsystem 1. All three related
UF2 profiles built, and the slow-window host test passed. No moving car
evidence exists yet; see `MOVING_BARCODE_TEST.md` for the run procedure.

Moving-test capture motion (4 November 2026): the updated test image now
requests straight motion while a GP6 capture window is active, then resumes
normal line following. Host policy tests and ARM builds passed. A car run is
still needed to confirm the wheel request actually moves the car, GP6 enters
`capture=1` on the first black bar, Code 39 decodes at moving speed, and
following resumes after `capture=0`.

Moving Z forward pass (4 November 2026): the integrated test physically
decoded Z with 30 edges in 2,800 ms, pattern order F and no overflow while
both wheel encoders increased above 4,600 ticks. The initial serial session
had `I0` and no motion; a second session calibrated the IMU and entered
FOLLOWING. After decoding, both line sensors read white and requested 0/0.
The test-only follower-state retention fix has passed host/build checks but
still needs a physical retest. Moving A remains open.

Moving Z reverse pass (4 November 2026): the integrated moving test decoded
Z from 30 edges in 2,679 ms with pattern order R and no overflow. During
capture and immediately afterward the requested and active motor speeds were
51/51. A later GP6 black edge started another capture after both line sensors
had gone white; the revised test image prevents a second straight-through
override after a successful decode, but that change is not physically
verified. Both moving directions have one successful Z pass; BC-02, BC-05 and
BC-07 stay PARTIAL pending repeatability, moving A, final mounting and full
course testing.

Capture-only team integration (4 November 2026): the normal team image now
allows 4 s for barcode edges and up to 1.5 s more for the trailing quiet
zone, covering the observed 2,679 and 2,800 ms Z passes. It continues to
publish decoded letters through Subsystem 1. The USB test's straight-through
motor override is not in the team image; decoded A/Z do not yet trigger
junction turns. The new timing passed a host window test and the team ARM
image built, but the normal image has not decoded a moving barcode on the
car. Startup still depends on the team network/START path.
