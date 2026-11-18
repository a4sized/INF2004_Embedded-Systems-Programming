# 1 ms GP6 sampling trial

Flash `build_make/team_barcode_1ms_sampling_wifi_mqtt.uf2` for this test.
It uses the normal networked team startup, hotspot configuration and MQTT
START command, with line following and barcode decoding active together.
It is different from the separate one-shot U-turn calibration UF2.

The dedicated build stages a 1 ms kernel tick and wakes IR sensing every
1 ms. Raw GP6 polling, edge capture and decoding run on those wakes; line
filter/state updates and IR snapshot publication remain at approximately
5 ms, and motor requests remain at 10 ms. No GPIO IRQ is claimed. The
default integrated build still uses a 5 ms tick/capture period, car speed
and decoder acceptance rules are unchanged. The rebuilt trial now includes
the accepted 129 mm U-turn setting and Ray's dashboard/mapping merge.
Defaults are W LEFT, Z RIGHT and F UTURN; the dashboard can remap letters.
The faster sampling remains a test profile pending physical validation.

## Capture the comparison

1. Flash this sampling-test UF2 and open USB serial at 115200 baud, saving
   the whole session from startup.
2. Use the usual MQTT START workflow. Keep car speed, sensor height,
   lighting and the small print the same as in the failed 5 ms test.
3. Pass over the same small barcode. Repeat three times, recording its
   letter, print size and scan direction for each pass. Compare with the
   large counterpart in the same setup. Do not combine hand-moved and
   autonomous results as if they were the same test.
4. Send the full log, including `IR_BARCODE_TIMING`, `IR_BARCODE_EDGE`,
   `IR_BARCODE_WINDOW`, `BARCODE_DECODED`, and any faults or state changes.

The new line appears about once per second while serial is open:

```text
IR_BARCODE_TIMING,nominal_us=1000;n=...;min_us=...;max_us=...;over2ms=...;over5ms=...
```

`n` is the number of sample intervals in that reporting window, roughly
1000 if the requested rate is maintained. Minimum/maximum gaps are actual
elapsed microseconds between sensing-loop samples, not requested delays.
`over2ms` and `over5ms` count intervals strictly longer than those limits.
Startup and logging may add jitter. The timing log itself and edge output
can add load; their effect appears in subsequent measured intervals.
These counters are not proof that every physical pulse was captured.

Success is a valid complete decode of repeated small-print passes, with
the timing rows showing how well 1 ms sampling held during each pass.
If START_MARKER_REJECTED remains despite short measured sampling gaps,
inspect the bar waveform, sensor spot/height/threshold and print before
relaxing matching. Queue overflow and capture-invalid flags remain useful.

Build again from the repository with:

```sh
python3 app_program/subsystem3/build_barcode_1ms_test.py
```

Compilation and the existing Code 39 host tests pass. Actual sampling
cadence, full-system scheduling load, small-print decoding and on-car
movement at this tick setting remain pending physical validation.
