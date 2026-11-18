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

## Encoder comparison trial (18 November 2026)

Flash the newly built networked team UF2. Use the existing hotspot/MQTT
START workflow and save the entire USB serial session. Test W and Z, each
in the small and large print sizes and both scan directions. Include smooth
passes and passes with an ordinary steering correction. Record which print,
size and direction belongs to each pass; do not move the paper by hand for
this moving-car comparison.

`IR_BARCODE_EDGE` retains `B`, `L`, `R`, `n` and millisecond `t`, and adds
signed cumulative `EL` and `ER` encoder ticks. The encoder getters are read
beside the GPIO sample before decoding/logging, with no GPIO interrupt
changes. They are sequential reads, not an atomic wheel pair. For adjacent
edges in the SAME capture, compare elapsed time with signed wheel tick
differences; `(delta_EL + delta_ER) / 2` estimates wheel-midpoint travel.
Keep each wheel's delta too: unequal or opposite deltas identify steering
where the midpoint estimate may differ from GP6 travel. Avoid interpreting
counter resets, zero movement or wheel slip as valid barcode distances.

Rejected `IR_BARCODE_WINDOW` lines now include `reasonF` and `reasonR` for
the forward and reverse pattern order. `INCOMPLETE` means fewer than 30
edges; `START_MARKER_REJECTED` identifies rejection at the first ten edges.
`START_WIDTH`, `DATA_WIDTH` and `STOP_WIDTH` identify character width
classification failure (zero widths, narrow/wide overlap, excessive wide
spread or tied wide elements). `START_PATTERN`, `STOP_PATTERN`,
`DATA_PATTERN`, `CHARACTER_GAP`, `EDGE_LEVEL`, `INVALID_CAPTURE`, and
`EXTRA_EDGES` identify the other checks. A frame may fail at different
stages in each orientation. `VALID` here excludes the optional trailing
quiet-zone check; normal integrated decoding accepts a valid complete frame
at the final black-to-white edge.

The existing timestamp decoder still determines `BARCODE_DECODED` and
junction turns. Encoder-based decoding, separate black/white width
classification, and revised steering during capture are pending analysis of
these paired measurements. Keep complete *LETTER* validation enabled.

## F immediate sensor-confirmed U-turn (18 November 2026)

Mapping in `ir_barcode_action_for`: W LEFT, Z RIGHT, F UTURN. F starts
immediately after a fresh decode while following is permitted; it does not
wait for GP6 junction white/black/white. It requests -64/+64 (left backward,
right forward) through Subsystem 2, whose reversal delay still applies.
The consumed telemetry letter clears on turn start as for the other turns.

A line sensor must be observed white after the pivot starts, then that
sensor must see black. Either line sensor can satisfy this sequence. At
reacquisition the controller requests zero while checking for 40 ms of
stable black in distinct fresh publications; a white flicker resumes the
pivot. Completion resets ordinary following and resumes its requests.
There is no angle or encoder-distance gate. This can finish on an earlier
branch or the original line if it is encountered again; the user explicitly
chose sensor confirmation rather than an approximately 180-degree gate.

Flash the networked UF2 and START through the existing MQTT workflow. On a
clear line, scan F and observe `BARCODE_DECODED,letter=F`, then
`JUNCTION_TURN,UTURN` without a junction cue. Check that the wheels rotate
in opposite directions, that black present before the turn does not end it,
and that white then stable black yields `JUNCTION_DONE,LINE_FOUND` followed
by normal following. Save the full USB log and note whether the car actually
faces back along the track. A turn with no confirmed line stops at 8 s,
logs `JUNCTION_TIMEOUT,UTURN_NOT_CONFIRMED`, and reports FAULT_IR_NAV through
Subsystem 1. Permission/stale-IR/obstacle handoffs still apply. Repeat W and
Z afterwards to check the unchanged junction behavior. This U-turn has host
coverage and compiles; on-car torque, geometry, confirmation and timeout
behavior are untested.
