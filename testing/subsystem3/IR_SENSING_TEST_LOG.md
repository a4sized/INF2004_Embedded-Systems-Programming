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
consumer calls Buddy 2's current `s2_set_ir_velocity()` API, but uses
following permission=false and base/correction speeds=0. The sensing task
also keeps `b_ready=false`, so this only sends zero velocity intent. The
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
