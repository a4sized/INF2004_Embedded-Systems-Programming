# Buddy 3 three-sensor readout evidence

Date: 13 October 2026, Asia/Singapore.

## Scope

Sensing-only image for left GP7, right GP4 and barcode GP16 digital DO.
No motion, barcode decoding, network or MQTT calls. The image uses the
professor's RTOS and USB console. The original one-sensor bench and example
subsystems remain separate.

## Build

Command: `python3 app_program/subsystem3/build_sensing.py` from the fresh
starter repository root. Profile: SMP=1, CONSOLE=usb_cdc, WIFI=none,
NET=none, MQTT=0, CRASH_REPORT=0, TEST=0. The standalone app files compile
with `-Wall -Wextra -Werror -Wconversion -Wshadow`. A fresh UF2 was built
on 13 October 2026 at `build_make/buddy3_ir_sensing_smp1_usb_cdc_wifi_none.uf2`.
The build used temporary staging and did not run the standard team image.

## Reported black/white levels (21 October 2026)

After adjusting the line modules' variable resistors, the user reported
the same digital levels for all three sensors:

| Sensor | GPIO | Black | White |
|---|---:|---:|---:|
| Left IR | GP7 | 1 | 0 |
| Right IR | GP4 | 1 | 0 |
| Barcode IR | GP16 | 1 | 0 |

The standalone image now maps HIGH to BLACK on each channel. These are
user-reported observations; a saved USB trace at mounted height and moving
robot trials have not been supplied. Recheck raw levels after changes to
height, lighting or module adjustment.

A successful compilation is not evidence of physical contrast, 5 ms
sampling, barcode edge capture or robot operation.

## Line-level stability filter (21 October 2026)

The user elected to keep the two line modules together at the centre and
filter brief line-level changes in Buddy 3. `ir_line_filter.c` now qualifies
left and right changes using observed elapsed milliseconds. The standalone
diagnostic uses a provisional 10 ms stability interval and prints
`raw>stable` for each line channel. Barcode DO is passed through unchanged.
No motion requests or queue integration were added.

Fresh checks: the host test passed under Clang AddressSanitizer and
UndefinedBehaviorSanitizer for a short pulse, sustained independent left/
right changes, barcode passthrough and 32-bit timer wrap. The isolated ARM
UF2 build passed with strict application warnings. No moving-robot trial or
5 ms sampling/jitter measurement has been performed.

## Isolated 5 ms scheduling build (21 October 2026)

The build script now copies only `config/` into its temporary tree and changes
that copy's kernel period from 10 ms to 5 ms. The repository's shared
`config/config.h` stays at 10 ms. A cyclic handler wakes the sensing task
every 5 ms; the task reads all three GPIO levels once per wake and discards
queued wakeups after each reading rather than replaying delayed samples.
The USB summary remains near 200 ms at the nominal rate. The ARM UF2 builds.
The diagnostic measures consecutive GPIO-read starts with RP2040's
free-running microsecond timer. Each USB line reports minimum and maximum
gaps, the count in 4000–6000 us, and the count above 10,000 us. Counters
reset after 12,000 intervals (about 60 seconds at the target rate). To check
NFR9, keep the serial monitor open and inspect a full window: at least 99%
of intervals must be in range and none may exceed 10,000 us. These counters
reflect this standalone sensing build with USB enabled, not integrated load.

The user subsequently reported that the timing result reached the requested
threshold. Exact `ok` and `over10ms` values or a serial trace were not saved
here, so record this as a reported standalone Pico check. Timing with motion,
network and the other subsystems remains untested.

## Four-state line decision (21 October 2026)

The user found a narrow centered placement where GP7 and GP4 both read
black. The standalone diagnostic now maps filtered levels as follows:

| GP7 left | GP4 right | Position | Latest correction |
|---:|---:|---|---|
| 1 | 0 | Left black | Left |
| 0 | 1 | Right black | Right |
| 1 | 1 | Centered | None |
| 0 | 0 | Both white | None |

The latest decision is overwritten at each 5 ms sample. Both white lasting
30 ms sets a diagnostic loss flag; after loss, both black lasting 20 ms
clears it. The host state test passed under AddressSanitizer and
UndefinedBehaviorSanitizer, including side changes, both-black centering,
timed loss/reacquisition and timer wrap. The standalone ARM build passed.
The user then reported that all diagnostic readings work on the Pico. Exact
serial values for each position were not saved in this log. This is a
stationary bench observation; moving-robot behaviour remains untested. No
motion requests are sent.

## GP16 interrupt edge capture build (22 October 2026)

The standalone profile now timestamps both GP16 edge types in a short GPIO
callback using the microsecond timer. A 64-entry single-producer ring passes
events to the sensing task; no allocation, printing or RTOS call occurs in
the callback. The sensing task stores at most 64 edges starting with a
white-to-black transition. After two seconds it prints the edge levels and
successive durations. Queue overflow or a non-alternating/ambiguous edge
marks the window invalid. The final edge's duration is unknown and is
printed as zero. A window is a diagnostic trace, not a verified complete
barcode; framing and trailing-white criteria require real sample data.

The diagnostic build now uses `WIFI=cyw43, NET=none` because the starter's
`WIFI=none` build omits the Pico SDK GPIO interrupt implementation. The
standalone task installs the callback on core 1; normal team startup still
uses the example consumer. This is not an agreed integrated IRQ ownership
contract. USB printing a 64-edge window can delay line samples, so rerun
the 60-second gap check during real scans before treating the timing target
as met with capture enabled.

Fresh checks: `ir_barcode_capture_test.c` passed with Clang strict warnings,
AddressSanitizer and UndefinedBehaviorSanitizer. It verifies bounded full
queue behavior, overflow counting, FIFO order and ring wrap. The isolated
ARM UF2 compiled at `build_make/buddy3_ir_sensing_smp1_usb_cdc_wifi_cyw43.uf2`.
No Pico flash, USB edge trace, pulse-width measurement or integrated robot
test was performed in this change.

## Host-side edge-window verification (22 October 2026)

The two-second task-side collection rules were moved into
`ir_barcode_window.c` so they can be checked independently of Pico hardware.
`ir_barcode_window_test.c` passed with strict Clang warnings, AddressSanitizer
and UndefinedBehaviorSanitizer. It exercises leading white edges, alternating
edge durations, the exact timeout boundary, 32-bit microsecond timer wrap,
duplicate/ambiguous edges, explicit invalidation and the 64-edge limit.
The isolated ARM UF2 rebuilt with this module. This verifies host logic and
compilation only; the open physical capture and timing checks above remain.

## Provisional bar/gap proportions (22 October 2026)

`ir_barcode_segments.c` now derives edge-to-edge bar and internal-white-gap
durations and normalizes them to parts per million of the observed span.
It excludes unmeasured leading/trailing white areas. It rejects invalid,
odd-ended, zero-duration and over-two-second windows. The standalone USB
trace labels structurally valid vectors `PROVISIONAL`; this is not a complete
barcode decision and cannot produce a symbol or motion request.

The host test passed with strict Clang warnings, AddressSanitizer and
UndefinedBehaviorSanitizer for speed scaling, 32-bit timer wrap, incomplete
and invalid windows, zero duration and excessive span. The isolated ARM UF2
rebuilt. No recorded physical scan exists, so end framing, minimum pulse
width, tolerances, dictionary, and scan-speed range remain unverified.

The complete pending barcode list is maintained in
`UNTESTED_BARCODE_CHECKLIST.md`. Host test passes do not mark those hardware
and integration checks complete.

## Local IR fault status (23 October 2026)

`ir_fault_status.c` now latches a failed GPIO initialization until explicit
reset and counts sample gaps above 10,000 us. The isolated USB readout shows
the local initialization flag and cumulative late-sample count. This is
fault-source evidence only: it does not set a shared system bit, call Buddy
1 telemetry, or request motor stop. Barcode overflow remains an invalid
capture, not an automatic system fault.

The host test passed with strict Clang warnings, AddressSanitizer and
UndefinedBehaviorSanitizer for initialization failure, reset, exact gap
boundary and timer wrap. The isolated ARM UF2 rebuilt. No Buddy 1 integration,
fault arbitration, physical stop or reset test has run.

## Gatekeeper fault wiring (23 October 2026)

The isolated sensing image now links the actual Subsystem 1 implementation
and initializes its kernel objects before starting the IR sensing task. If
GPIO initialization fails, the task reports `FAULT_IR_SENS` with
`s1_report_fault`, logs `IR_SENSOR_FAULT` through `s1_csv_log_event`, and
reasserts the fault every 250 ms while sensing remains stopped. The call is
task-context only. No IR fault is raised from the barcode GPIO interrupt.
The late-sample counter remains a timing diagnostic, not a sensor fault.

Local checks: the fault-status and provisional barcode-segment host tests
passed with strict Clang warnings, AddressSanitizer and
UndefinedBehaviorSanitizer. `python3 app_program/subsystem3/build_sensing.py`
produced the isolated ARM UF2 with strict application warnings and linked
Buddy 1's sources. No fault injection on Pico, cloud observation, robot stop,
or clear/reset test was performed. See the pending cases in
`testing/integration/FAULT_INTEGRATION_PENDING.md`.

## Latest IR snapshot and 20 ms host consumer (28 October 2026)

`ir_line_snapshot_shared.c` now keeps one complete latest IR reading behind
a micro T-Kernel mutex. The isolated sensing task initializes the mutex
before starting, then publishes raw and filtered levels, interpreted
surfaces, line position/correction, loss state, sequence and sample time
after each 5 ms wake. Readers take one coherent copy. The source does not
queue old steering decisions or request motor movement.

The isolated image publishes `b_gpio_ready=true` after successful GPIO
configuration but leaves `b_ready=false`. Sensor readiness needs a
mounted black/white verification or an agreed fixed-polarity acceptance
procedure. `ir_line_snapshot_steering_status()` also requires explicit
following permission and a caller-supplied freshness limit. Both-white,
confirmed loss, mismatched position/correction and stale readings are
rejected; centered returns an available reading with no correction.

A host test simulates 5 ms publication and a 20 ms consumer, verifies each
read sees the newest sequence, exercises readiness, permission, missing
line, exact age boundary, 32-bit timer wrap and explicit invalidation, and runs concurrent
producer/reader copies through a pthread-backed mutex substitute. It passed
with strict Clang warnings, AddressSanitizer and UndefinedBehaviorSanitizer.
The isolated ARM UF2 and standard team ARM image compiled after the change.

Host command from the repository root:

```sh
clang -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow \
  -fsanitize=address,undefined -fno-omit-frame-pointer -pthread \
  -I testing/subsystem3/fakes -I app_program/subsystem3 \
  testing/subsystem3/ir_line_snapshot_test.c \
  app_program/subsystem3/ir_line_snapshot.c \
  app_program/subsystem3/ir_line_snapshot_shared.c \
  app_program/subsystem3/ir_motion_handoff.c \
  -o /tmp/ir_line_snapshot_test && /tmp/ir_line_snapshot_test
```

Open physical/integration checks: run the reader on the Pico under the real
5 ms sensing load, save a trace showing coherent latest values and age,
measure the effect of the new mutex on timing, define and perform startup
sensor readiness verification, and replace the normal example consumer
through a coordinated team integration. These are not claimed by the host
test or ARM build.

## Opt-in team IR and Buddy 2 zero-intent handoff (28 October 2026)

`build_team_ir.py` stages the team sources with a 5 ms kernel tick and
`BUDDY3_TEAM_IR`. That profile selects the IR sensing service in
`subsystem3_init/start` and starts a 20 ms latest-snapshot consumer. The
consumer calls Buddy 2's current `s2_set_ir_velocity()` API. Following
permission defaults false and sensing keeps `b_ready=false`, so it still
sends zero velocity intent. The
normal team build still starts the example consumer at the 10 ms tick.

The host test checks zero targets for paused, stale and unready data, plus
bounded left/right targets with simulated permission and speeds. It passed
with strict Clang warnings and ASan/UBSan. The opt-in team UF2, isolated
sensing UF2 and ordinary team ARM image compiled. No integrated image ran
on the Pico.

Open checks: agree Vehicle Controller permission ownership and revocation,
startup readiness/calibration, wheel orientation and safe speeds with Buddy
2, and the production freshness limit. Then measure 5 ms sensing and 20 ms
handoff timing on the Pico, inject stale/unready/lost readings, and verify
zero intent and fault behavior before allowing nonzero targets.

## Provisional left/right movement requests (28 October 2026)

The user specified provisional slow base speed and low correction, with
correction ending when the white sensor sees black again. The first-run
values are base=50 and correction=15 in Buddy 2's 0..255 units. With an
eligible snapshot, left-black/right-white maps to left=35/right=65;
right-black/left-white maps to left=65/right=35; both black maps to 50/50.
These values are starting guesses, not motor calibration results.
The runtime setter currently rejects higher speed, correction or age
settings while these starting limits are unvalidated.

The future Vehicle Controller can call `ir_motion_consumer_set_following()`
with permission and target settings. Permission defaults false; revocation
immediately sends zero to Buddy 2 and is serialized with consumer writes.
The 20 ms consumer also requires a new snapshot sequence on every cycle and
rejects samples older than 20 ms. The sensor still publishes `b_ready=false`,
so this image cannot issue a nonzero request yet.

The strict host policy test and opt-in team ARM build passed. There was no
Pico, motor, direction, STOP/ESTOP, or timeout test. Before movement, the
team must assign the controller owner, verify both mounted sensors, confirm
wheel direction and tune speeds. Buddy 2 must confirm what happens if the
IR consumer stops running while its last nonzero target is stored.

## Three-second boot-delay line-following demo image (28 October 2026)

The user chose a temporary power-switch demonstration: flash, power off,
place the untethered car on the track, switch on, wait 3 seconds, then
follow the black line. `build_line_follow_demo.py` builds a separate 5 ms
team UF2 with `BUDDY3_LINE_FOLLOW_DEMO`. That flag enables provisional
readiness after GPIO setup using the user-observed black=1 polarity. The
consumer writes zero before its 3-second delay, then grants its own
provisional 50/15 follow settings. It still requires a new, at-most-20-ms
snapshot each 20 ms cycle; both-white and lost readings request zero.
Buddy 2 receives requests through `s2_set_ir_velocity()`, and its fault
mask gate remains active. The ordinary and non-moving opt-in images retain
their prior behavior.

The demo and non-moving opt-in ARM images compiled. No Pico, wheels-raised,
track, switch/reset, stopping-distance or motor-current test has run. The
demo deliberately bypasses FR11 startup polarity calibration and does not
establish full FR8–FR10 motor compliance. Buddy 2's source accepts signed
left/right targets and has a 20 ms ramp/reverse state machine, but its
published test log contains no motor-direction evidence; its current source
does not update encoder tick counters or implement measured over-current
or stall shutdown, and PWM wrap is fixed at 255. The user's report of
Buddy 2's physical direction check is not a recorded trace here.

## USB input and request trace for the demo (28 October 2026)

The user will first power the Pico through USB to inspect sensor readings
and movement requests. The demo image now emits a Buddy 1 CSV `IR_TRACE`
event about every 200 ms while the USB console is connected. The event's
details include `GP7`, `GP4`, `GP16` raw digital levels; `stable7` and
`stable4` filtered line levels; the named line position; `ready`, `allow`,
`used`, steering `status`, reading `age_ms`; and `sentL`/`sentR`, the exact
values passed to Buddy 2's `s2_set_ir_velocity()` on that consumer cycle.
The reported sensor polarity is 1=BLACK, 0=WHITE. `GP16` is shown for
input inspection only and does not affect steering.

After the consumer's three-second startup delay, fresh readings should
produce 50/50 when both line sensors see black, 35/65 when only GP7 sees
black, 65/35 when only GP4 sees black, and 0/0 when both see white. The
stable levels require about 10 ms before following a raw transition.
`sentL`/`sentR` record requests issued to Buddy 2, not measured motor PWM,
wheel speed, or an acknowledgement from Buddy 2. A USB serial trace and
physical motion test are still pending.

## First demo boot investigation (28 October 2026)

The user reported that the demo produced no `/dev/cu.usbmodem*` port and
the wheels did not spin. BOOTSEL mounted as `RPI-RP2`, then disappeared
after the UF2 copy. The IR sensors and IMU were connected. The application
USB CDC path is not yet verified.
Static review found two independent reasons a wheel can remain stopped:
the demo intentionally sends zero for its first three seconds and whenever
the line reading is both white, stale, or unusable; an IMU communication
fault causes Subsystem 4 to report `FAULT_IMU_SENS`, which Buddy 2 treats
as a halt even when the module is physically connected.

There is also an integration defect in initialization order: the ordinary
team `app_main.c` initializes Buddy 2's motor PWM before Buddy 5's servo,
but Buddy 5 resets the whole PWM peripheral during servo initialization.
This clears the motor slice configuration. The demo build now stages a
temporary `app_main.c` copy that initializes the servo before the motor;
the repository's shared `app_main.c` and Buddy 5 sources are unchanged.
Buddy 5 and Buddy 2 should agree on a permanent shared PWM initialization
contract. The revised demo and an isolated sensing USB image both compiled,
and their UF2 records passed structural checks. Neither image has been
reflashed or observed on the Pico after this correction. The USB CDC
failure remains unresolved pending an on-Pico comparison with the isolated
sensing image and the demo image.

## Direct motor and line diagnostic image (28 October 2026)

At the user's explicit request to bypass subsystem motion permissions for
diagnosis, `build_direct_motor_demo.py` builds a separate standalone image
from `bench/direct_motor/src/main.c`.
It omits the team application and its Buddy 1/2/4/5 startup paths, disables
unused device drivers only in staged build configuration, releases and
configures PWM slices for GP8–GP11, and reads GP7/GP4 directly. It waits
three seconds, applies a 120/255 forward PWM pulse to both wheels for
400 ms regardless of IR readings, stops for 200 ms, then follows the black
line using 80/80 when both sensors are high, 60/100 when only GP7 is high,
100/60 when only GP4 is high, and 0/0 when both are low. USB CDC prints
`DIRECT_IR` input/output values about every 200 ms when a serial terminal
is attached. This image has no Buddy 1 fault handling, Buddy 2 ramping,
Buddy 4 IMU, or Buddy 5 obstacle checks; the short pulse is intended for
a wheels-raised diagnostic before any floor trial.

The standalone ARM image compiled with strict application warnings, its
linker symbols were checked, and its UF2 blocks passed structural checks.
The source is outside `app_program/`, and the ordinary team image still
compiled after the new diagnostic was added.
The host currently has neither a Pico application serial port nor a BOOTSEL
drive, so it was not flashed or tested on the car. Motor power, PWM pins,
wheel direction, startup pulse, and USB enumeration remain open hardware
checks. This image is local and uncommitted.

## Reduced direct-demo speed after user trial (28 October 2026)

The user reported that the direct diagnostic ran fast enough to need a
large speed reduction. This is evidence that the flashed direct PWM path
moved the car, but no serial trace, measured speed, wheel direction or
line-following result was supplied. The diagnostic PWM values were halved:
the startup pulse is now 60/255 on each wheel for the same 400 ms; the
line-following base is 40/255 with a 10/255 correction. The targets are
40/40 for both black, 30/50 for only GP7 black, 50/30 for only GP4 black,
and 0/0 for both white. The pulse log now prints the compiled constants.
The revised standalone ARM image compiled; the slower image has not yet
been tested on the car.

## Further 80% direct-demo PWM reduction (28 October 2026)

At the user's request, all direct-demo PWM values were reduced by 80%
from the previous revision. The startup pulse is now 12/255 per wheel;
line following uses 8/8 for both black, 6/10 for only GP7 black, 10/6 for
only GP4 black, and 0/0 for both white. The 3-second wait and 400 ms pulse
duration remain unchanged. The image compiled, but these low duties may
not overcome the motors' starting friction; physical movement and line
following at this setting remain untested.

## Revised to 30% PWM reduction (28 October 2026)

The user replaced the requested 80% reduction with a 30% reduction from
the preceding 60/40/10 settings. The direct demo now uses a 42/255 startup
pulse, a 28/255 straight line-following target and a 7/255 correction.
Resulting requests are 28/28 for both black, 21/35 for only GP7 black,
35/21 for only GP4 black and 0/0 for both white. The three-second wait
and 400 ms pulse duration remain. The ARM UF2 compiled; this revision has
not yet been physically tested.

## Active-low line-follow and lost-line search (28 October 2026)

The user reported that the prior direct demo stopped when both line sensors
read white and did not turn effectively. They asked to switch the line
sensor polarity and keep searching after line loss. The direct diagnostic
now interprets GP7/GP4 DO LOW as black and HIGH as white, and disables pad
pulls because the modules drive DO. It removes the unconditional startup
motor pulse. After the three-second wait, it stays at 0/0 until a black
reading has occurred. Both black request 28/28 forward; left black alone
requests 0/35 for a stronger left turn; right black alone requests 35/0.
After a line has been seen, both white requests a continuous 0/35 or 35/0
pivot toward the last side that saw black; reacquiring black immediately
returns to the matching follow command. If the first black state was both
sensors, the initial search direction defaults left. The USB trace prints
raw pin levels, interpreted black flags, mode and applied PWM.

The standalone ARM build passed with strict warnings and `git diff --check`
passed. The switched polarity, turn direction, search behaviour and
appropriate PWM duty remain untested on the car. The earlier 400 ms startup
pulse is intentionally absent in this revision.

## Restored polarity and slower curve handling (28 October 2026)

The user asked to restore the previous IR polarity and slow the direct
demo for curves and U-turns. GP7/GP4 HIGH again means black, LOW means
white. The three-second wait, no unconditional pulse, black-before-motion
rule, one-wheel turn and last-seen-side search remain. Straight PWM is
now 20/20; a left correction or search uses 0/30, and a right correction
or search uses 30/0. The sensing/control loop now runs every 10 ms instead
of 20 ms, while USB trace output remains about every 200 ms. The ARM UF2
compiled with strict warnings and `git diff --check` passed. Wheel torque
at these PWM values, curve tracking and U-turn recovery still need a car
test; the 10 ms loop is a nominal RTOS interval, not measured jitter.

## Reduced turn and confirmed both-white search entry (28 October 2026)

The user asked to decrease turning rate and prevent line search unless
both sensors detect white together; the user confirmed simultaneous white
readings. The direct diagnostic retains HIGH=black
and 20/20 straight PWM, but lowers the one-wheel turn and search PWM from
30 to 22. Search now requires that the car previously saw black and that
GP7 and GP4 both read white for two consecutive 10 ms control samples.
The first both-white sample requests 0/0 while confirming; the second and
later samples pivot toward the last black side at 0/22 or 22/0. Both-white
at startup remains 0/0 without search. Any black reading clears the
both-white counter. The USB trace includes `white_n` for inspection.
The standalone ARM image compiled with strict warnings and the diff check
passed. Actual turn rate, motor torque at 22/255 and search trigger still
need on-car verification.

## Explicit follow and search states (28 October 2026)

The user clarified that line following and line searching are separate.
The direct diagnostic now has WAIT, FOLLOW and SEARCH states. FOLLOW handles
one-black corrections or both-black straight driving. SEARCH starts only
after a previously seen line is absent from both sensors for two consecutive
10 ms samples, and pivots toward the last black side until either sensor
reads black. The first both-white sample stops while confirming the loss;
both-white at startup remains WAIT. The serial trace reports `state` and
`action` independently. The standalone ARM UF2 compiled and `git diff
--check` passed; transitions, turn direction, and line reacquisition still
need a car test.

## Zero-output pause before line search (28 October 2026)

The direct diagnostic now commands both motor PWM outputs to zero as soon
as both GP7 and GP4 read white while following. After two consecutive
both-white readings it stays in STOPPING with zero outputs for ten more
nominal 10 ms cycles, then SEARCH may pivot on the next cycle. Any black
reading during STOPPING resumes FOLLOW. The trace includes `stop_n` and
`state=STOPPING`. The standalone ARM UF2 compiled and `git diff --check`
passed; whether the car actually
stops during this pause and then searches requires physical testing.

## One-second stop and immediate black reacquisition (28 October 2026)

The direct diagnostic's zero-PWM STOPPING phase was lengthened from ten
to 100 nominal 10 ms control cycles (about one second). A black reading
on either sensor during STOPPING or SEARCH returns to FOLLOW and applies
its line-following motor output in that same cycle. The two-sample
both-white search gate remains. The standalone ARM UF2 compiled and
`git diff --check` passed. On-car timing and reacquisition remain open.

## Integrated demo steering and search policy (29 October 2026)

The team IR consumer now requests 20/20 straight, 0/22 left and 22/0
right through Buddy 2. Two consecutive both-white 10 ms consumer samples
after tracking begin a one-second zero-request pause; then search pivots
toward the last one-black side. Either black sensor resumes FOLLOW on the
next eligible filtered snapshot, including during the pause or search.
The line-loss flag now clears on either black sensor. Permission starts
disabled in both normal and temporary team builds; the temporary three-
second auto-grant was removed. IMU calibration status exists, but first
ultrasonic scan and Wi-Fi connection readiness contracts are missing, so
the all-subsystems-ready grant remains open. The current team build also
sets NET=none/MQTT=0. Continuous search conflicts with the original
three-second ordinary search failure requirement and needs team review.
The user confirmed that permission stays closed until the team controller
grants it, and a later Wi-Fi disconnect should not revoke onboard
line following.

The line-state and snapshot/controller host tests passed under Clang with
AddressSanitizer, UndefinedBehaviorSanitizer, and strict warnings. Normal
and temporary team ARM UF2 profiles compiled. Physical motor stopping,
search direction, black reacquisition, and startup gate remain untested.

## Smoother direct demo steering trial (29 October 2026)

The user reported left/right oscillation in the standalone direct demo.
For this trial, its 10 ms sensor period stays unchanged; ordinary left
and right corrections now request 16/24 and 24/16 PWM, keeping both wheels
driven. A direct reversal of correction direction needs two consecutive
one-black readings on the new side, with 20/20 during the first. Both-white
still immediately requests 0/0, the one-second stop and search pivot are
unchanged, and black reacquisition still resumes following in the same
cycle. The USB trace adds 200 ms counts `bb_n`, `left_n`, `right_n`, and
`ww_n` to reveal fast sensor changes that a single trace sample could miss.

The standalone ARM UF2 compiled with strict warnings and `git diff --check`
passed. Physical smoothness, the minimum PWM that moves each motor, and
whether the sensor outputs chatter at track edges remain untested. This
trial changes only the standalone demo, not the team motion policy.

## Loss-triggered correction trial (29 October 2026)

After the user's report that the gentle correction still jitters and
loses curves, the direct demo now requests 20/20 whenever either line
sensor reads black. Two consecutive both-white 10 ms readings start a
250 ms directed correction: GP7 as the last lone black sensor selects
0/30 toward the left, and GP4 selects 30/0 toward the right. Any black
reading immediately returns to straight FOLLOW. If the directed
correction finds no black, the demo commands 0/0 for about one second,
then searches with the previous last-side pivot at 0/22 or 22/0. With
no prior lone-black side, the demo skips correction and takes the
stop/search path, whose search direction still defaults left. The
10 ms sensing period and 200 ms trace sample-count fields remain.

The standalone ARM UF2 compiled with strict warnings and `git diff
--check` passed. On-car curve recovery, timing, motor response and
sensor stability remain untested. This change has not been copied to
team integration.

## One-shot GP16 junction turn demo (29 October 2026)

The standalone direct-motor profile now samples GP16 along with GP7/GP4
at nominal 10 ms intervals. After the three-second boot wait, two white
GP16 samples arm a one-shot junction trigger; two subsequent black GP16
samples start a turn. `DEMO_JUNCTION_TURN` defaults LEFT and can be edited
to RIGHT. The motor request is 0/30 left or 30/0 right. During the first
250 ms the line sensors cannot confirm success. Afterward, LEFT requires
GP4 white then two black samples, while RIGHT requires GP7 white then two
black samples. Success resumes ordinary following; three seconds without
confirmation latches 0/0 until reboot. Trace fields include `raw16`,
`junction`, `turn_n`, `confirm_white` and `confirm_black_n`.

The user reports GP16 normally white on the ordinary line and black at
the junction, but that contrast and physical lead distance remain
unverified. The standalone ARM UF2 compiled with strict warnings; trigger
position, turn direction, confirmation, timeout and false positives all
remain open physical tests. Team integration was not changed.

## Junction exit trigger and slower turn trial (29 October 2026)

The direct-motor demo now treats two black GP16 readings as a junction
mark and keeps following until GP16 returns to white for two consecutive
10 ms readings. It then begins the same one-shot turn, with junction PWM
reduced from 30 to 24. `junction=WAIT_EXIT` in the USB trace shows that
the black mark was captured; `exit_white_n` shows consecutive exit-white
samples. The 250 ms ignore interval, white-then-stable-black turn
confirmation, and three-second timeout are unchanged. Only the demo was
changed. On-car timing, turn direction and motor response at PWM 24 need
testing.

## Immediate either-sensor junction confirmation (29 October 2026)

The user reported unreliable turn confirmation in the standalone demo.
The turn now checks GP7 and GP4 immediately, without a 250 ms ignore
period. Each sensor must have read white during the turn; the first
subsequent black sample on either sensor completes the turn and returns
to ordinary line following. The USB trace reports `seen_white7` and
`seen_white4`. The GP16 black-then-white trigger, PWM 24 and three-second
timeout remain. Physical timing, false confirmations, direction and
motor response need retesting. Team integration is unchanged.

## Dual-confirmation junction turn and right trim (29 October 2026)

The standalone demo applies `RIGHT_TRIM_PERCENT` to right PWM; it is set
to 100, so the output is presently unchanged. Junction completion now
requires both a GP7 or GP4 white-to-black transition during the turn and
a GP16 white-to-black-to-white sequence during the turn. At completion,
GP7 or GP4 must currently read black. The trace adds `line_confirmed`
and `barcode_phase` (0 waiting white, 1 waiting black, 2 waiting final
white, 3 complete). Three-second timeout is unchanged. These sequences,
turn completion, and motor balance remain untested on the car. Team
integration was not changed.
The standalone ARM UF2 compiled with strict warnings and `git diff
--check` passed.

## Main pin layout and retired demo migration (1 November 2026)

The active right line input is now GP2, matching the latest team pin
layout; GP7 remains left and GP6 barcode. The GP4 polarity and steering
observations above describe earlier hardware and do not validate GP2.
The temporary direct-motor and line-follow demo source, build scripts and
UF2 images were moved to `../demo_archive_2026-11-01` outside this Git
repository. No automatic junction turn was added to the team image;
barcode symbol decoding and a validated navigation instruction are still
required before that behavior can run.

The normal team build now compiles Buddy 3's IR service on a 5 ms kernel
tick. Its motion policy requests 20/20 whenever either line sensor reads
black. After two both-white control samples, it requests 0/30 toward the
last GP7-only black side or 30/0 toward the last GP2-only black side for
250 ms. If neither sensor finds black, it requests zero for one second
before searching at 0/22 or 22/0. Any black reading returns to straight
following in that cycle. Without a last single-black side it skips the
directed correction. Motor requests still use Buddy 2's
`s2_set_ir_velocity()`. Subsystem 5 is initialized before Subsystem 2 so
its PWM reset cannot erase motor setup.

The controller host test passed with strict warnings, AddressSanitizer
and UndefinedBehaviorSanitizer. The ordinary team ARM UF2, team IR UF2,
and sensing-only UF2 compiled. `git diff --check` passed. Physical GP2
polarity, line reacquisition, encoder coexistence, PWM behavior, and
system startup have not been tested. Following permission remains false,
and published mounted-sensor readiness remains false until validation.

## Barcode edge timestamp hardening (2 November 2026)

The task-side barcode window now marks a capture invalid when adjacent
edges have the same timestamp, an edge goes backward in time, or its
timestamp exceeds the two-second window. Unsigned elapsed-time arithmetic
still accepts normal 32-bit timer wrap. Invalid windows cannot become
provisional normalized segments or navigation input.

The barcode-window host test passed with Clang strict warnings,
AddressSanitizer and UndefinedBehaviorSanitizer, including duplicate,
backward, overlong and wraparound timestamps. The isolated sensing ARM UF2
compiled and `git diff --check` passed. No GP6 hardware scan, line timing
measurement or complete barcode decoding was performed.

## User-accepted mounted IR readiness (3 November 2026)

The user reports that the car moved and followed the line in the demo,
and instructed that mounted IR readings be treated as ready. The sensing
service now publishes `b_ready=true` after successful IR GPIO setup; a
failed setup still reports `FAULT_IR_SENS` and publishes an unavailable
snapshot. This is a fixed-polarity acceptance based on the reported car
trial, not an automatic black/white calibration routine.

Earlier recorded successful demo evidence used GP7/GP4/GP16. The current
team pin map is GP7 left, GP2 right and GP6 barcode; a saved trace for
those current pins remains open. The Vehicle Controller's separate
initial Wi-Fi and START gates still prevent the normal image from moving.
The team IR and sensing-only ARM UF2 images compiled. The existing
snapshot/handoff host test passed with strict warnings and sanitizers;
`git diff --check` passed. No new car run was performed for this change.

## Single-letter Code 39 image checks (3 November 2026)

The user supplied two screenshots of printed bars. Their mid-row run widths
match standard `*A*` and `*Z*` respectively, with no fourth character for
an optional check digit. A pure Code 39 decoder now accepts exactly one
A-Z data character between start/stop markers after a trailing white
interval. The fixed action table maps A to LEFT and Z to RIGHT; other
letters have no action. The sensing service publishes a decoded letter to
Subsystem 1's barcode telemetry setter. Junction/motor commands are not
connected to this result yet.

The host test used run widths measured from both supplied images and passed
with strict warnings, AddressSanitizer and UndefinedBehaviorSanitizer. The
team IR and sensing-only ARM UF2 images compiled. Image analysis is not a
GP6 sensor scan: mounted contrast, scan speed, noise tolerance, leading
quiet zone, duplicate suppression and turn placement remain untested.
