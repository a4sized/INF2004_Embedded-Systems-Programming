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
| BC-06 | Hardcoded action letters and straight-through default | A=LEFT and Z=RIGHT now arm integrated junction turns; other decoded letters keep ordinary following. Physical turn outcome remains untested. | PARTIAL |
| BC-07 | Decoder tolerance and rejection of truncated, noisy and ambiguous scans | Separate tuning and validation traces, success/rejection counts by speed and direction | UNTESTED |
| BC-08 | Duplicate suppression and rearm after strip clearance | Repeated passes and dwell over one strip with capture IDs | UNTESTED |
| BC-09 | One-action turn transaction after decoded letter | Integrated policy compiles and has host transition tests; still needs GP6 black-to-white cue, Subsystem 2 request/acknowledgement, turn-completion and timeout traces on car | PARTIAL |
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

Marker-driven update (4 November 2026): both the team and USB moving-test
images now check the opening Code 39 `*` after its ten boundary edges and
validate the closing `*LETTER*` frame once all 30 edges arrive. A progressing
scan no longer has a fixed total-duration cutoff. A 2 s pause between edges
clears an incomplete frame; a complete frame has a 21 s fallback while its
trailing quiet is checked. The USB test's straight-motion override remains
bounded to 8 s independently of capture. Host marker, slow-scan, replay,
polling and ARM builds passed; physical moving reliability and jitter effects
remain open.

Moving Z trace follow-up (4 November 2026): a supplied moving pass captured
all 30 `*Z*` edges without overflow, but the next GP6 black arrived 675 ms
after the final white edge, before the old approximately 850 ms quiet rule
could announce a decode. The integrated sensing task now publishes the
letter immediately after validating the closing `*` on edge 30. The measured
trace passes the host frame replay and the networked ARM UF2 compiled.
Retest autonomous A and Z scans, the junction turn, and line reacquisition
on the robot; the user's observed left motor indicator with no movement
during a turn is still unresolved.

Junction crossing follow-up (4 November 2026): a moving Z decoded and armed
RIGHT, but GP6 remained black while both line sensors went white. The
follower stopped and entered SEARCH before `JUNCTION_TURN`; telemetry had no
fault because TURNING never started. The new junction policy requests 51/51
while crossing confirmed GP6 black, with a three-second stuck-black stop and
`FAULT_IR_NAV`. The subsequent user-requested power trial raises straight,
junction-crossing, turn and search requests to 77/255, and directed
correction to 51/255.
A 100 ms straight hold during barcode capture suppresses brief both-white
corrections. Retest physically; the supplied successful Z pass does not
explain the other reported failed scans.

30%-duty trace follow-up (4 November 2026): the supplied moving pass had 30
GP6 edges, no overflow, and a forward `*Z*` width pattern, but it was
rejected because one Z wide gap stretched to 275 ms during correction,
exceeding the old 2× wide-spread bound. The decoder now permits up to 4×;
a 500 ms synthetic outlier is still rejected. The user requested 25% duty:
straight, junction crossing and turn/search are now 64/255; correction
remains 51/255. Host replay passed; decode and turn at this setting still
need a physical run.

20%-base trial (7 November 2026): one networked car run at the previous
25%-base setting decoded Z and completed a right turn, then briefly lost and
reacquired the line before a longer loss. For the next trial, base travel
and junction crossing are 51/255 (20%), junction turning and line search
stay 64/255 (25%), and a both-white gap during active barcode capture may
hold equal forward requests for up to 350 ms before ordinary correction.
The networked UF2 compiled and the host line-snapshot test passed. Physical
decode repeatability, loaded movement at 20%, GP6 widths around corrections,
turn torque, and post-turn line retention are still untested at this setting.

Gentle-steering trial (9 November 2026): a lone-black line reading now adds
up to 8/255 to the outside wheel in 4/255 steps per consumer cycle. GP6
capture suppresses that bias and keeps equal requests while a line sensor
sees black. The host test and networked build passed. On the car, compare
curve jitter, line loss, and A/Z decode success with the previous 20%-base
build; save GP6 edges/windows and `IR_FOLLOW_MODE` around any failed scan.

W/Z mapping follow-up (9 November 2026): the physical run correctly decoded
W in reverse order as well as Z in forward order. W now maps to LEFT and Z
to RIGHT; A no longer requests a turn. At the start of a junction turn,
the consumed letter's Subsystem 1 telemetry field clears to zero so the next
decode is visibly distinct. Host tests and the networked build passed;
the W-left turn, zero telemetry on turn start, and subsequent Z-right turn
in one on-car run remain untested.

Curve tuning follow-up (9 November 2026): a sustained lone-black reading
now raises the outside wheel to at most 64/255 while the inside stays at
51/255; it still ramps in steps of 4 per consumer cycle. One both-white
sample holds the previous tracking request instead of briefly requesting
zero; two consecutive white samples still start the timed correction.
Host line-snapshot tests passed. On-car curve retention, search frequency,
motor load, and W/Z decode repeatability with this tuning remain untested.

Stronger-bias on-car log (9 November 2026): one forward Z decoded and
initiated a right turn. A W-shaped scan collected 28/30 edges, with an
abnormally long 600 ms black interval near its end. A later complete
Z-shaped scan was rejected because the data character's wide/narrow
separation was only 1.12, below the 1.4 threshold. No overflow was logged.
Repeated W/Z decoding and reliable line retention after junction turns
remain unverified; retain the raw edge logs for comparison with the next
physical run.

Encoder diagnostic trial (18 November 2026): the networked debug image logs
signed wheel counters beside GP6 edges and identifies rejection stages in
both pattern orders. The Code 39 host tests, including the measured Z
width-overlap rejection, passed with sanitizers and the networked UF2 built.
Encoder/GPIO alignment, logging load, small/large W/Z capture in both
directions, and movement during corrections remain untested on the car.
Distance-based acceptance is not enabled by this diagnostic change.

Superseded F sensor U-turn trial (18 November 2026): F now triggers an immediate left pivot
(-64/+64) without GP6 junction gating. Completion uses a line sensor's
white-to-black transition and 40 ms stable black, with an 8 s fault timeout.
Host tests and the networked build passed. Actual pivot direction, reverse
motor torque, early reacquisition, correct return heading, repeat scanning,
and fault/obstacle takeover on the car remain untested. No 180-degree gate
is used, at the user's explicit request.

Encoder half-turn follow-up (18 November 2026): the user reports improved
scan consistency; F was absent from the dashboard, and small-print tests
remain pending. F now completes from signed differential encoder travel
for an estimated 180-degree pivot, then settles for 200 ms and resumes
following/search. No line/GP6 completion gate remains. F is retained in
telemetry during the pivot instead of cleared immediately. Actual angle,
slip/drag calibration, dashboard delivery, takeover on the car, and
small/large W/Z/F decode coverage remain untested. Host tests and a
networked UF2 build cover software paths only.

Isolated U-turn calibration (18 November 2026): a separate USB one-shot
motor profile now overrides effective track to 147 mm, based on the user's
approximate 270-degree result at 220 mm. It waits three seconds after USB
opens, runs the shared pivot algorithm at -64/+64, then stops permanently
until reset and logs final encoder totals. No following/search can add
rotation. The integrated default remains 220 mm pending physical trials.
Both profiles compile; host pivot tests at 147 mm pass. Actual final angle,
repeatability, torque, coast and USB-loss stop remain untested. See
`testing/subsystem3/UTURN_CALIBRATION_TEST.md`. Small-print barcode testing
is still pending separately.

Small-print failed scan evidence (18 November 2026): the first supplied
small-print USB trace rejects two ten-edge windows at START_MARKER_REJECTED,
then logs a five-edge incomplete window. No letter decoded; invalid and
overflow remain zero. Captured black pulses include 5, 10 and 15 ms, near
the 5 ms poll resolution. Wheel-distance normalization still rejects the
first opening pattern despite nearly equal wheel increments. Small-print
reliability is now attempted/failed, not wholly untested; repeatable success,
letter/direction coverage and the cause of waveform distortion remain open.

1 ms small-print sampling trial (18 November 2026): a separate networked
profile stages a 1 ms tick/raw GP6 capture cadence while preserving 5 ms
line publication and the 10 ms motor consumer. The profile logs per-window
actual sample-gap statistics with nominal=1000 us, n, min/max, over2ms and
over5ms. The default image stays at 5 ms; speed, decoder rules and U-turn
calibration are unchanged. Both images compile; Code 39 regression tests
pass with sanitizers. Actual rate under system/USB load, small-print decode
success and full-car scheduling at 1 ms remain untested. See
`testing/subsystem3/BARCODE_1MS_TEST.md`.

Calibration refinement (18 November 2026): the user clarified approximately
15-degree angular overshoot on the 147 mm trial. The next calibration UF2
uses 136 mm (147 * 180 / 195 rounded); host tests and compilation pass.
Physical angle and repeatability at this setting remain pending. The
integrated U-turn setting and barcode sampling trial are unchanged.

Second calibration refinement (18 November 2026): 136 mm reportedly
overshot by approximately 5 degrees. The next one-shot calibration uses
132 mm (2054 average wheel ticks). Compilation and host pivot tests pass;
physical accuracy/repeatability remain pending. Integrated calibration
and barcode sampling profiles are unchanged.

Final requested calibration refinement (18 November 2026): reported
3–5 degree overshoot at 132 mm led to a 129 mm one-shot trial (2007 mean
wheel ticks), using a 4-degree midpoint. Build and host tests pass. Physical
angle/repeatability and acceptance for integration remain pending.

Calibration accepted/integrated (18 November 2026): the user reports that
129 mm produced the desired half-turn in the isolated build and authorised
integration. The normal firmware and separate rebuilt 1 ms sampling profile
now both use 129 mm. Ray's YiRui dashboard/runtime-mapping branch merged
without conflict. Both UF2s compile; junction/decoder host tests and 14
Python dashboard/decoder tests pass. Post-pivot follow/search on the car,
MQTT remapping, repeatability under full load and 1 ms small-print success
remain pending. The isolated pivot angle is user-accepted, not untested.

1 ms physical trace follow-up (18 November 2026): early identified
small-print capture still rejects the opening marker at ten edges. During
this pass ~986–991 intervals/s and max gaps ~3.1–3.5 ms were measured, with
no >5 ms gaps in adjacent timing windows. No overflow/invalid flags were
logged. Thus faster polling was exercised but did not establish small-print
success. Later telemetry confirms Z and RIGHT arming across a serial-output
gap; that success cannot be assigned to the early small print. The later
right turn times out independently. Further controlled sensor/print tests
remain needed; keep the 1 ms profile experimental.

Manual small-print evidence (18 November 2026): the team image records
all 30 edges per pass for offline reverse *A* and forward *Z* reconstruction,
with encoders zero. It rejects them because pooled black/white separation
ratios are 1.12–1.29, below 1.4; within-colour widths separate cleanly.
These manual captures support separate bar/gap classification as the next
fix, not a blanket threshold reduction. Firmware acceptance and moving
small-print decoding remain open. The user has suspended further 1 ms
trials; retain normal team integration as the active test baseline.

Timestamp colour-classification implementation (18 November 2026): the
user clarified removal of encoder measurements from barcode sensing, not
replacement of timestamp decoding. EL/ER reads and logging were removed
from barcode edge/sample diagnostics; U-turn encoder use remains. The
decoder now separates black and white distributions (two wide bars/one
wide gap for A-Z/*), retains 1.4 within-colour separation, bounds black
wide spread and white outliers, and validates both markers/full 30 edges.
Both measured manual small-print frames pass host replay as reverse A and
forward Z. A-Z/direction/scale/bias and rejection regressions pass under
sanitizers, as do earlier moving fixtures and the 5 ms polling pipeline.
The normal networked UF2 compiles. Actual manual re-scan and car-driven
small/large-print reliability remain pending. Default capture is 5 ms;
1 ms testing remains suspended.

Normal-print follow-up (18 November 2026): the latest supplied team-image
trace directly logs BARCODE_DECODED,letter=W;order=R after all 30 edges,
with static wheel totals. Normal reverse W is physically decoded with the
new classifier. Wi-Fi SSID failures and unready IMU keep VC in INIT; no
turn/dashboard delivery is validated by that scan. Small-print re-scan
and moving-car reliability remain pending.

Small A/Z manual repeat (18 November 2026): neither decoded in the next
supplied trace. The two passes have 18 and 22 total transitions (9 and 11
black bars), below 30/15 required for *LETTER*. Start-marker width/pattern
checks reject, followed by incomplete-window timeouts; overflow/invalid
remain zero. Unlike earlier complete manual captures accepted in host
replay, these waveforms are incomplete. Controlled sensor/print alignment
and contrast checks remain needed; small-print physical success and
repeatability are still open.

Z repeat evidence (18 November 2026): two complete 30-edge passes, one
DATA_WIDTH rejection for a forward Z-shaped frame (white separation
103/75=1.373 versus required 1.4) and one actual reverse Z decode. First
pass captured all markers/edges and fell about 2 ms short of the required
wide-gap duration. No overflow/invalid flag. Repeatability and borderline
timing tolerance remain unresolved; no threshold changed during review.

W/F framing evidence (18 November 2026): actual log successes are W F/R/F
and F forward. Host probing of raw edges recovers three more complete F
frames (F/R/R) preceded by 8/4/2 unwanted transitions; current sensing
resets every failed ten-edge opening group and loses their alignment.
All recovered frames pass the existing classifier when aligned, without
threshold changes. Rolling start-marker resynchronisation remains to be
implemented/tested. Rejected windows are not independent physical scans.
