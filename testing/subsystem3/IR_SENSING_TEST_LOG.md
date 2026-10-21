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
