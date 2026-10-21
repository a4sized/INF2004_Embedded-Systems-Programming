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

## Hardware checks pending

The user reported that the barcode output read 1 over black and that both
line modules changed between black and white after their variable resistors
were adjusted. These reports have no saved per-sensor black/white level table
or timing trace. All fixed black-level settings therefore remain UNKNOWN.
Record the actual DO levels shown by USB for each sensor held over black and
white at the mounted height, plus the module LED response and lighting.

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
5 ms sampling/jitter measurement has been performed. The starter tick is
still 10 ms, so this image requests a 10 ms sampling delay rather than
claiming the design's 5 ms rate.
