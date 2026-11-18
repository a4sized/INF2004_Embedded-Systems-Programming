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

## Timestamp decoder with separate bar/gap widths (18 November 2026)

The active build uses normal 5 ms GPIO polling and timestamp widths. The
user removed the encoder-distance barcode proposal. Barcode edge/sample
logs now contain B/L/R/n/t, without EL/ER; wheel encoders still support the
calibrated U-turn in the separate motion controller. Earlier paired logs
are historical evidence, not the current output contract.

For each A-Z character and each * marker, the decoder ranks the five black
bars and four white spaces independently. A-Z and * each have two wide
bars and one wide space. Narrow/wide separation must be at least 1.4 within
each colour. Wide black bars retain a 4x spread limit; the single wide white
gap is capped at 6x the largest narrow white gap to reject extreme outliers
while allowing the measured 275/60 ms correction. Intercharacter gaps and
legacy quiet-zone checks use narrow WHITE widths. The integrated path
still accepts immediately at a valid closing * without waiting for quiet.
Exactly 30 alternating edges, both markers and a supported single letter
remain mandatory. The decoder recognises A-Z only; it is not a decoder for
arbitrary digits/punctuation or strings.

The two manually moved small-print recordings from 18 November now pass
host replay as reverse A and forward Z, including opening and closing
markers. Synthetic coverage checks all letters in both directions at
multiple scales and opposite black/white biases; ambiguous widths,
unsupported patterns, missing/wrong stops, zero durations and extreme
outliers remain rejected. Earlier A/Z/W fixtures and the 5 ms polling
pipeline also pass. These are host results, not new physical scans.

Flash the current normal networked UF2. First repeat the same stationary
manual small-print pass and save the full log. Expect BARCODE_DECODED for
A or Z if the waveform repeats. A is unmapped by default, so it can decode
without requesting a turn; W/Z/F defaults and Ray's dashboard mapping still
apply. Then test actual car-driven small and large prints at the same
height/lighting. Keep the paper-motion and moving-car results labelled
separately. Save IR_BARCODE_EDGE, IR_BARCODE_WINDOW and BARCODE_DECODED.

Window reasonF/reasonR still identify START/DATA/STOP_WIDTH failures,
marker pattern failures, unsupported data, bad gaps and invalid/incomplete
capture. At ten edges a rejected start is START_MARKER_REJECTED. The
reason describes the attempted pattern order; it does not establish a
physical missed bar. The 1 ms profile is suspended at the user's request.

## F immediate encoder half-turn (18 November 2026)

Default mapping: W LEFT, Z RIGHT, F UTURN. Ray's merged dashboard/MQTT
mapping can assign different letters before a run. A fresh F decode starts an immediate
left pivot through Subsystem 2 (-64/+64, 25% duty), without a junction cue.
Neither GP6 nor either line sensor determines U-turn completion.
Subsystem 2's signed encoder totals are sampled at start and on each
consumer cycle, without resets. Differential travel must reach the
half-turn target with left backward and right forward. Straight movement
and wrong-direction rotation cannot satisfy it. At the target, zero is
requested for 200 ms to settle; then normal following resumes, or SEARCH
starts if both line sensors are white. Permission, stale IR and obstacle
handoffs still apply throughout.

The integrated target now uses the user's accepted 129 mm effective track,
2050 ticks/revolution and 207 mm wheel circumference: 2007 average wheel
ticks, 4014 differential ticks. The user confirmed the separate 129 mm
one-shot build produced the desired half-turn after trials at 147, 136 and
132 mm. This replaces the earlier 220 mm value borrowed from S5; S5's own
geometry is unchanged. It remains an encoder estimate, not absolute heading.
Check the first full integrated run for stopping and post-turn search. Adjust `IR_UTURN_EFFECTIVE_TRACK_MM` in `ir_junction.h`: raise
it if the turn falls short, lower it if it overshoots. Do not change the
barcode letter patterns or encoder tick totals to tune rotation.

Flash the current networked UF2 and use the normal MQTT START workflow.
Scan F on a clear section. Look for `BARCODE_DECODED,letter=F`,
`JUNCTION_TURN,UTURN`, then `JUNCTION_DONE,UTURN_180_ENCODER`. Verify a
complete half-turn despite early black/white transitions, followed by a
brief stop and following/search. Record whether it is short or overshoots,
its approximate angle and the full serial trace. If encoder travel does
not reach the target within 8 s, it latches zero, logs
`JUNCTION_TIMEOUT,UTURN_ENCODER_TIMEOUT`, and reports FAULT_IR_NAV to S1.
A timeout does not claim successful rotation.

F telemetry is retained during its immediate pivot so MQTT has a chance
to publish it; it clears on completion. Previously it was cleared within
one consumer cycle of decoding and could disappear before a 1 Hz publish.
The public telemetry parser already accepts ASCII F. No new dashboard
source is present in this checkout, so any additional dashboard mapping
still needs checking in that application if F remains absent.

Host junction tests cover straight-motion rejection, early line transition
rejection, target boundaries, stop/settle timing, timeout, cancellation,
missing odometry and encoder rollover. Physical pivot angle/torque,
post-turn search, F dashboard visibility, and small-print scans are pending.
