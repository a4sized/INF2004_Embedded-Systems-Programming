# Integrated W/Z junction test

Build the networked team image with:

```sh
python3 app_program/subsystem3/build_team_network.py
```

Flash `build_make/team_ir_handoff_smp1_usb_cdc_wifi_lwip_mqtt.uf2`.
It uses the existing local Wi-Fi/MQTT settings and still waits for the
Vehicle Controller's readiness checks and MQTT START command. Use MQTTX
with the team's existing command topic and binary payload format. The
USB serial monitor shows the readable event trail.

This networked image also prints GP6 diagnostics while sensing runs:
`IR_BARCODE_SAMPLE` about every 200 ms, `IR_BARCODE_EDGE` on each captured
GP6 transition, and `IR_BARCODE_WINDOW` when an incomplete/rejected capture
is cleared. In their details, `B=1` means GP6 sees black and `B=0` means white;
`L` and `R` are the raw line sensor readings, `n` is the current barcode
edge count, and `t` is milliseconds since boot. A complete single-letter
Code 39 scan has 30 edges and prints `BARCODE_DECODED` if valid. When no
decode occurs, check whether B ever changes and whether n reaches 30.
`IR_BARCODE_WINDOW` also prints `span_ms`, the time from the first to last
captured edge. The team image checks the start and stop `*` patterns and has
no fixed total scan duration. A 2 s interedge pause clears incomplete
captures; a complete frame gets its calculated trailing white, with a 21 s
fallback limit. Keep the serial monitor open until the window result appears,
then save the GP6 edge lines and nearby left/right readings from the moving
run.
The edge logs may add USB timing load, so measure capture reliability with
these diagnostics enabled before treating them as final course settings.

The networked debug image also prints `IR_FOLLOW_MODE` on transitions:
`TRACK` → `CORRECT` (250 ms) → `STOPPING` (250 ms) → `SEARCH`.
Any fresh black line reading returns it to `TRACK`. If it reaches `SEARCH`
but the wheels do not pivot, check the Subsystem 2 motor output. `JUNCTION_TURN`
owns the motor request during a junction turn; after `JUNCTION_TIMEOUT`,
the failed junction owns a zero request until the team's fault reset.

The fixed letter mapping is in `app_program/subsystem3/ir_code39.c` in
`ir_barcode_action_for`: W means LEFT; Z means RIGHT. Other decoded letters
continue ordinary line following. The turn speed is the Vehicle Controller's
current 64/255 request on one wheel, with the other requested at zero. This
is a torque trial after 22/255 lit the motor indicator without moving the
car. Ordinary line search shares this one-wheel speed. Straight travel and
junction crossing now request 51/51 (20%); directed correction requests 51
on one wheel. During an active barcode capture, a brief both-white line
reading holds an even 51/51 request for up to 350 ms before
correction/search resumes. This trial reduces GP6 width distortion from
short steering corrections while still recovering from a sustained line
loss. The one-wheel junction turn and ordinary search remain at 64/255
(25%) because the right-turn run succeeded at that request.

For this curve trial, ordinary following adds an outside-wheel bias while
exactly one line sensor sees black: left black asks up to 51/64, right black
up to 64/51. The bias changes by at most 4/255 per 10 ms consumer update
and returns gradually to equal speeds when both sensors see black. The first
simultaneous white sample retains the last bounded tracking request; a
second consecutive white sample begins the existing 250 ms one-wheel
correction, followed by the 250 ms stop and line search if black remains
absent. GP6 capture requests equal wheel speeds while a black line is
visible; the existing 350 ms both-white capture hold remains. Compare W/Z
decoding and `IR_FOLLOW_MODE` transitions on the same curve with the prior
51/59 bias build. This tuning passed the host line-snapshot test; car behavior
remains to be measured.

On a clear test track with one barcode and its junction, look for:

1. `BARCODE_DECODED,letter=W` or `letter=Z`, then `JUNCTION_ARMED,LEFT/RIGHT`.
   The decode should now appear as soon as GP6 crosses the closing `*` bar's
   black-to-white edge. It no longer waits for a long trailing white region;
   the measured moving Z pass reached the next black area 675 ms later.
2. GP6 white, black and white after the decode. `JUNCTION_TURN,LEFT/RIGHT`
   appears only after the last white reading. `JUNCTION_CROSSING,FORWARD`
   appears after the junction black is confirmed; the car requests 51/51
   through Subsystem 2 to cross it even when both line sensors are white.
   If GP6 never clears within three seconds, expect
   `JUNCTION_TIMEOUT,GP6_NOT_CLEARED` and `FAULT_IR_NAV`.
3. During the turn, GP6 must see another white, black and white sequence.
   Either line sensor must see white then black, and at least one must still
   read black. Then `JUNCTION_DONE,LINE_FOUND` marks return to following.
   At `JUNCTION_TURN`, Subsystem 1's barcode telemetry field clears to 0;
   a later W or Z decode publishes its own letter and arms the next turn.
4. If confirmation is missing for three seconds, the motion request becomes
   zero, `JUNCTION_TIMEOUT,LINE_NOT_FOUND` appears, and Subsystem 1 receives
   `FAULT_IR_NAV`. A new run requires the team's normal fault clearing flow.

Start with the wheels lifted to confirm the W/Z requested direction before
trying the printed junction. Stop via MQTTX if the car points away from the
track. Save the USB event trail and note which UF2 was flashed. Host tests
and ARM compilation do not establish physical turn accuracy or reacquisition.
