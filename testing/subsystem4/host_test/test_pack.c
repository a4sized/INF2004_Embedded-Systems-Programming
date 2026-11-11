/*
 * test_pack.c -- PC test of subsystem 4's minimum-bit data (change requirement 1).
 *
 * Checks, at the minimum, the maximum and JUST PAST the maximum of every field:
 *   - the message is exactly 8 bytes and every field survives a trip through memory,
 *   - values outside the range are clipped, never wrapped,
 *   - the packed snapshot word keeps three signed fields apart,
 *   - the 16-bit time stamp subtracts correctly across its wrap,
 *   - and WHY the hump accumulator is not packed (see the last test).
 *
 *   cd testing/subsystem4/host_test && ./run_sim.sh      (runs this first)
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "tk/tkernel.h"
#include "app.h"
#include "subsystem4/subsystem4.h"

static int fails, checks;
#define CHK(cond, desc) do { checks++; if (!(cond)) { fails++; printf("  FAIL  %s\n", desc); } } while (0)

int main(void)
{
    /* ---- size ---- */
    CHK(sizeof(S4_MSG) == 8, "S4_MSG is exactly 8 bytes");

    /* ---- clipping: never wrap ---- */
    CHK(s4_clip_s(127, 8) == 127 && s4_clip_s(128, 8) == 127 && s4_clip_s(300, 8) == 127, "signed 8-bit: top clips at 127");
    CHK(s4_clip_s(-128, 8) == -128 && s4_clip_s(-129, 8) == -128 && s4_clip_s(-300, 8) == -128, "signed 8-bit: bottom clips at -128");
    CHK(s4_clip_s(0, 8) == 0 && s4_clip_s(-1, 8) == -1, "signed 8-bit: small values unchanged");
    CHK(s4_clip_u(0, 7) == 0 && s4_clip_u(127, 7) == 127 && s4_clip_u(128, 7) == 127 && s4_clip_u(5000, 7) == 127, "unsigned 7-bit clips at 127");
    CHK(s4_clip_u(255, 8) == 255 && s4_clip_u(256, 8) == 255 && s4_clip_u(-1, 8) == 0 && s4_clip_u(-300, 8) == 0, "unsigned 8-bit clips at 0 and 255");
    CHK(s4_round(2.4f) == 2 && s4_round(2.5f) == 3 && s4_round(-2.4f) == -2 && s4_round(-2.5f) == -3 && s4_round(-0.2f) == 0, "rounding is symmetric");

    /* ---- the snapshot word: three signed bytes that must not disturb each other ---- */
    int snap_ok = 1;
    for (int v = -128; v <= 127; v++) {
        UW w0 = s4_snap_pack(v, 0, 0), w1 = s4_snap_pack(0, v, 0), w2 = s4_snap_pack(0, 0, v);
        if (s4_snap_field(w0, 0) != v || s4_snap_field(w0, 1) != 0 || s4_snap_field(w0, 2) != 0) snap_ok = 0;
        if (s4_snap_field(w1, 1) != v || s4_snap_field(w1, 0) != 0 || s4_snap_field(w1, 2) != 0) snap_ok = 0;
        if (s4_snap_field(w2, 2) != v || s4_snap_field(w2, 0) != 0 || s4_snap_field(w2, 1) != 0) snap_ok = 0;
    }
    CHK(snap_ok, "snapshot: every value -128..127 in each field comes back, the other two untouched");
    UW all = s4_snap_pack(-128, 127, -1);
    CHK(s4_snap_field(all, 0) == -128 && s4_snap_field(all, 1) == 127 && s4_snap_field(all, 2) == -1, "snapshot: mixed signs");

    /* ---- the message: the kernel copies it as bytes, so a memcpy must give it back unchanged ---- */
    S4_MSG a = {0}, b;
    a.type = S4_MSG_STATUS; a.hump_state = 2; a.time16 = 0xFFFF; a.vibration = 255; a.lift = 2;
    a.p.status.pitch = -128; a.p.status.roll = 127; a.p.status.turn = -128; a.p.status.accel = 127;
    memcpy(&b, &a, sizeof a);
    CHK(b.type == S4_MSG_STATUS && b.hump_state == 2 && b.time16 == 0xFFFF && b.vibration == 255 && b.lift == 2, "STATUS header survives at maximum values (including the 2-bit lift state)");
    CHK(b.p.status.pitch == -128 && b.p.status.roll == 127 && b.p.status.turn == -128 && b.p.status.accel == 127, "STATUS payload survives at the limits");
    S4_MSG h = {0}, h2;
    h.type = S4_MSG_HUMP; h.time16 = 1; h.p.hump.peak = 127; h.p.hump.length = 255;
    memcpy(&h2, &h, sizeof h);
    CHK(h2.type == S4_MSG_HUMP && h2.p.hump.peak == 127 && h2.p.hump.length == 255, "HUMP: 7-bit peak and 8-bit length at maximum");
    S4_MSG c = {0}, c2;
    c.type = S4_MSG_LIFT; c.p.lift.reason = S4_LIFT_OVERTURNED; c.p.lift.tilt = 255;
    memcpy(&c2, &c, sizeof c);
    CHK(c2.type == S4_MSG_LIFT && c2.p.lift.reason == S4_LIFT_OVERTURNED && c2.p.lift.tilt == 255, "LIFT: 2-bit reason and 8-bit tilt survive at maximum");
    {   /* every tilt 0..255 for every reason: the two fields must not disturb each other */
        int lift_ok = 1;
        for (unsigned reason = 0; reason <= 2; reason++) for (unsigned tilt = 0; tilt <= 255; tilt++) {
            S4_MSG x = {0}, y;
            x.type = S4_MSG_LIFT; x.p.lift.reason = reason; x.p.lift.tilt = tilt;
            memcpy(&y, &x, sizeof x);
            if (y.type != S4_MSG_LIFT || y.p.lift.reason != reason || y.p.lift.tilt != tilt) lift_ok = 0;
        }
        CHK(lift_ok, "LIFT: every reason (none / picked up / overturned) with every tilt 0..255 comes back unchanged");
    }
    CHK(s4_clip_u(180, 8) == 180 && s4_clip_u(300, 8) == 255, "a tilt of 180 degrees (upside down) fits the 8-bit field; anything above 255 is clipped");
    S4_MSG f = {0}, f2;
    f.type = S4_MSG_FAULT; f.p.fault.faults = 3;
    memcpy(&f2, &f, sizeof f);
    CHK(f2.type == S4_MSG_FAULT && f2.p.fault.faults == 3, "FAULT: both condition bits survive");
    CHK((unsigned)S4_MSG_FAULT <= 3U && (unsigned)S4_LIFT_OVERTURNED <= 3U && (unsigned)S4_LIFTED <= 7U, "type fits 2 bits, lift state fits 2 bits, motion state fits 3 bits");

    /* ---- the 16-bit time stamp ---- */
    UH t1 = 65530, t2 = 10;
    CHK((UH)(t2 - t1) == 16, "time16 difference is right across the wrap (65530 -> 10 = 16 ms)");
    CHK((UH)(5 - 4) == 1, "time16 difference, no wrap");

    /* ---- why the hump accumulator is NOT packed ----
     * At 30 cm/s and 20 ms a sample moves 0.6 cm.  At the 4 degree "climb" angle each sample adds
     * 0.6 * sin(4 deg) = 0.042 cm = 0.42 mm of height.  The OUTPUT step is 1 mm. */
    float exact = 0.0f; int quantised_mm = 0;
    for (int i = 0; i < 50; i++) {
        float step_mm = 6.0f * (float)sin(4.0 * M_PI / 180.0);       /* 0.42 mm */
        exact += step_mm;
        quantised_mm += s4_round(step_mm);                            /* what a 1 mm accumulator would add */
    }
    CHK(exact > 20.0f, "float accumulator reaches about 21 mm in 50 samples");
    CHK(quantised_mm == 0, "a 1 mm accumulator would stay at 0 (every step rounds away): this is why it is not packed");

    printf("%d checks, %d failed\n%s\n", checks, fails, fails ? "FAILED" : "ALL PASSED");
    return fails ? 1 : 0;
}
