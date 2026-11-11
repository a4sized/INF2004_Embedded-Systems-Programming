/**
 * @file    s5_bypass.c
 * @brief   Subsystem 5, stages 4 and 5: going round an obstacle and back to
 *          the line.
 *
 * The decision (s5_avoid.c) gives the side, the PLANNED edge on it (how far
 * it is from the car's centre line, from the fine scan and the known width
 * range, s5_avoid_plan_extent) and how far to reverse first for there to be
 * room to swing out round it (s5_avoid_plan_backup_mm, often 0). The car:
 *   0. reverses that far, straight back along the line (no new scan: that
 *      would not change the obstacle's width);
 *   1. turns out S5_BYPASS_TURN_DEG toward that side, the sensor looking
 *      along the original heading, straight at the obstacle's front;
 *   2. drives out on that slant far enough that, once it turns back, its
 *      side clears the planned edge by S5_BYPASS_CLEAR_MARGIN_CM:
 *      (planned edge + half the car's width + margin) / sin(turn). Meanwhile
 *      it watches the front: closer than S5_OUT_LEG_MIN_CM and the obstacle
 *      is wider than planned;
 *   3. turns back to the original heading;
 *   4. drives along the obstacle, the sensor turned toward its side
 *      (S5_PASS_LOOK_OFF_DEG off straight ahead, within the fine scan's
 *      limits), until it ends (its length is not known from the front).
 *      The sensor, at the front of the car, is then about level with the
 *      back corner, so the car drives S5_PASS_CLEAR_MM (about one car's
 *      length) more, until the whole car is past it. If the side is never
 *      seen, it is taken as a short obstacle: just S5_PASS_CLEAR_MM;
 *   5. turns in S5_RETURN_TURN_DEG (a gentler slant than going out) and
 *      drives back, far enough to cover the same sideways distance as the
 *      out leg, until Subsystem 3's IR sensors find the line, plus
 *      S5_IR_TO_AXLE_MM so the drive wheels are on it; turns back to the
 *      original heading (S5_RETURN_TURN_DEG again). If the
 *      IR sensors are not over the line then, the line follower searches
 *      for it once it has the car back, toward the side it is on
 *      (subsystem5_take_line_side).
 *
 * Too big: if the obstacle's front comes within S5_OUT_LEG_MIN_CM on the out
 * leg (too wide: the car's front corner would hit it) or its side has not
 * ended after S5_EDGE_MAX_MM (too long), the bypass is NOT carried on. Every
 * move made so far after step 0 is undone in reverse order (forward legs
 * driven back, turns turned back), so the car ends up on the line where it
 * turned out, and S5_GO_ROUND_TOO_BIG is returned.
 *
 * Casts in this file: (UB *) only retypes a string literal for the T-Kernel
 * tm_printf() API; (unsigned int) widens values that always fit; (void)
 * discards a return value that is not needed. The other casts are commented
 * where they are made.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "../app.h"
#include "s5_config.h"
#include "s5_log.h"
#include "s5_report.h"
#include "s5_guard.h"
#include "s5_profile.h"
#include "s5_avoid.h"
#include "s5_drive.h"
#include "s5_edge.h"
#include "s5_sense.h"
#include "s5_bypass.h"
#include "../subsystem3/ir_line_snapshot_shared.h"

#define HALF_CAR_CM         ((S5_CAR_WIDTH_CM + 1u) / 2u)   /* rounded up */

/* Most moves in one bypass: turn out, out, turn in, pass, turn in, back to
 * the line, turn out. */
#define MAX_LEGS            (7u)

_Static_assert((S5_RETURN_TURN_DEG >= 5u) &&
               (S5_RETURN_TURN_DEG <= S5_BYPASS_TURN_DEG),
               "the return turn must be 5 degrees up to the out turn (the "
               "return distance divides by sin(return turn))");
_Static_assert((S5_BYPASS_TURN_DEG >= 5u) && (S5_BYPASS_TURN_DEG <= 90u),
               "bypass turns must be 5..90 degrees (the out-leg clearance "
               "divides by sin(turn))");
/* The servo never turns further than the fine scan (s5_sense_servo_to
 * clamps to S5_FINE_MIN_ANGLE..S5_FINE_MAX_ANGLE), so both bypass look
 * angles must be within those limits, or the sensor would not look where
 * the code thinks it does. */
_Static_assert((S5_PASS_LOOK_OFF_DEG >= 1u) &&
               ((S5_FORWARD_DEG + S5_PASS_LOOK_OFF_DEG) <=
                S5_FINE_MAX_ANGLE) &&
               ((S5_FORWARD_DEG - S5_PASS_LOOK_OFF_DEG) >=
                S5_FINE_MIN_ANGLE),
               "the pass look angle must be within the fine scan's limits");
_Static_assert(((S5_FORWARD_DEG + S5_BYPASS_TURN_DEG) <= S5_FINE_MAX_ANGLE) &&
               ((S5_FORWARD_DEG - S5_BYPASS_TURN_DEG) >= S5_FINE_MIN_ANGLE),
               "the out-leg look angle (90 +/- the turn) must be within the "
               "fine scan's limits");
_Static_assert(0u == (S5_PING_INTERVAL_MS % S5_DRIVE_POLL_MS),
               "ping interval must be a multiple of the drive poll");

/* Written by the vehicle controller task, read by the scanning task:
 * atomic loads and stores. */
static bool gb_drive_permitted = false;

void s5_bypass_set_permitted (const bool b_allowed)
{
    __atomic_store_n(&gb_drive_permitted, b_allowed, __ATOMIC_RELEASE);
}

bool s5_bypass_permitted (void)
{
    return __atomic_load_n(&gb_drive_permitted, __ATOMIC_ACQUIRE);
}

/* ------------------------------------------------------------------------
 * One move: the shared drive loop
 * ------------------------------------------------------------------------ */

/* How one move ended. */
typedef enum
{
    RUN_DONE = 0,       /* reached its distance / angle                    */
    RUN_HOOK_STOP,      /* a hook asked to stop (it says why)              */
    RUN_FAILED          /* permission withdrawn, something ahead, or timed
                         * out (s5_bypass_run says which)                  */
} run_result_t;

/* Optional per-move hooks. p_on_poll runs every S5_DRIVE_POLL_MS and
 * p_on_ping with every reading; either returns true to stop the move.
 * NULL = not used. */
typedef struct
{
    bool (*p_on_poll) (void * const p_ctx);
    bool (*p_on_ping) (const bool b_good, const uint16_t distance_mm,
                       void * const p_ctx);
    void * p_ctx;
} run_hooks_t;

/*
 * Drive one move and wait for it to end, checking the encoders every
 * S5_DRIVE_POLL_MS and pinging every S5_PING_INTERVAL_MS (readings handled
 * for where the servo is facing). A forward move with the sensor facing
 * ahead stops if the obstacle guard sees something within the stop
 * distance. The wheels are stopped and given S5_DRIVE_SETTLE_MS to ramp
 * down before returning.
 */
static run_result_t s5_bypass_run (const s5_move_t move, const uint16_t amount,
                                   const run_hooks_t * const p_hooks)
{
    const s5_look_t look          = s5_sense_look();
    const bool      b_watch_ahead = ((S5_MOVE_FORWARD == move) &&
                                     (S5_LOOK_AHEAD == look));
    uint32_t        elapsed_ms    = 0u;
    bool            b_blocked     = false;
    bool            b_hook_stop   = false;
    run_result_t    result        = RUN_DONE;

    s5_drive_start(move, amount);

    while ((!s5_drive_is_done()) && (!b_blocked) && (!b_hook_stop) &&
           (elapsed_ms < S5_DRIVE_MOVE_TIMEOUT_MS) && s5_bypass_permitted())
    {
        (void)tk_dly_tsk(S5_DRIVE_POLL_MS);
        elapsed_ms += S5_DRIVE_POLL_MS;
        s5_drive_nudge(elapsed_ms);   /* raise a turn that has not started */

        if ((NULL != p_hooks) && (NULL != p_hooks->p_on_poll))
        {
            b_hook_stop = p_hooks->p_on_poll(p_hooks->p_ctx);
        }

        if (0u == (elapsed_ms % S5_PING_INTERVAL_MS))
        {
            uint16_t distance_mm = NO_OBSTACLE;
            bool     b_good      = s5_sense_reading(look, false,
                                                    &distance_mm);

            b_blocked = (b_watch_ahead && s5_guard_is_stopped());
            if ((!b_hook_stop) && (NULL != p_hooks) &&
                (NULL != p_hooks->p_on_ping))
            {
                b_hook_stop = p_hooks->p_on_ping(b_good, distance_mm,
                                                 p_hooks->p_ctx);
            }
        }
    }

    bool b_done = s5_drive_is_done();

    s5_drive_print_result(move, amount, b_done, elapsed_ms);   /* verbose */
    s5_drive_stop();
    s5_sense_watch_for(look, S5_DRIVE_SETTLE_MS);

    if (!s5_bypass_permitted())
    {
        result = RUN_FAILED;
        S5_LOG((UB *)"S5 drive: permission withdrawn, move stopped\n");
    }
    else if (b_blocked)
    {
        result = RUN_FAILED;
        S5_LOG((UB *)"S5 drive: something ahead, move stopped\n");
    }
    else if (b_hook_stop)
    {
        result = RUN_HOOK_STOP;   /* the hook has said why */
    }
    else if (!b_done)
    {
        result = RUN_FAILED;
        S5_LOG((UB *)"S5 drive: move timed out (wheels or encoders?)\n");
    }
    else
    {
        /* RUN_DONE */
    }

    return result;
}

/* A plain move (straight or turn). True if it was completed. */
static bool s5_bypass_move (const s5_move_t move, const uint16_t amount)
{
    return (RUN_DONE == s5_bypass_run(move, amount, NULL));
}

bool s5_bypass_back (const uint16_t distance_mm)
{
    bool b_done;

    s5_guard_set_drive(true);
    b_done = s5_bypass_move(S5_MOVE_BACK, distance_mm);
    s5_guard_set_drive(false);

    return b_done;
}

/* ------------------------------------------------------------------------
 * Driving along the obstacle
 * ------------------------------------------------------------------------ */

/* How a leg beside the obstacle (out, or passing it) ended. */
typedef enum
{
    ALONG_DONE = 0,     /* driven as planned (passing: it ended, and the
                         * extra distance driven)                          */
    ALONG_TOO_BIG,      /* out: front too close; passing: it did not end   */
    ALONG_FAILED        /* blocked, timed out or permission withdrawn       */
} along_result_t;

/* ------------------------------------------------------------------------
 * Driving out (step 2)
 * ------------------------------------------------------------------------ */

/* Every reading on the out leg: track where the obstacle's front ends (for
 * the width check) and stop if it is so close that the car's front corner
 * would hit it. */
static bool s5_bypass_out_ping (const bool b_good, const uint16_t distance_mm,
                                void * const p_ctx)
{
    bool * const   p_b_ended   = (bool *)p_ctx;
    const uint16_t distance_cm = s5_report_mm_to_cm(distance_mm);
    bool           b_stop      = false;

    *p_b_ended = s5_edge_feed(b_good, distance_cm, s5_drive_progress_ticks());

    if (b_good && (distance_cm <= S5_OUT_LEG_MIN_CM))
    {
        b_stop = true;
        S5_LOG((UB *)"S5 out: obstacle front only %u cm away, wider than "
               "planned\n", (unsigned int)distance_cm);
        s5_log_event("S5_OUT", "TOO_WIDE");
    }

    return b_stop;
}

/*
 * Drive out on the slant distance_mm, the sensor turned to look along the
 * original heading at the obstacle's front (look_deg). *p_driven_mm
 * receives how far the car went, *p_b_front_ended whether the sensor saw
 * the front end on the way (for the width check).
 */
static along_result_t s5_bypass_out (const uint16_t look_deg,
                                     const uint16_t distance_mm,
                                     uint16_t * const p_driven_mm,
                                     bool * const p_b_front_ended)
{
    const run_hooks_t hooks  = { NULL, s5_bypass_out_ping, p_b_front_ended };
    along_result_t    result = ALONG_DONE;
    run_result_t      run;

    *p_b_front_ended = false;
    s5_sense_servo_to(look_deg, S5_LOOK_SIDE);
    s5_edge_start(false);
    run = s5_bypass_run(S5_MOVE_FORWARD, distance_mm, &hooks);
    *p_driven_mm = s5_drive_mm_for_ticks(s5_drive_progress_ticks());

    if (RUN_HOOK_STOP == run)
    {
        result = ALONG_TOO_BIG;   /* too wide: front too close */
    }
    else if (RUN_FAILED == run)
    {
        result = ALONG_FAILED;
    }
    else
    {
        S5_LOG((UB *)"S5 out: drove %u mm\n", (unsigned int)*p_driven_mm);
    }

    return result;
}

/* ------------------------------------------------------------------------
 * Passing it (step 4)
 * ------------------------------------------------------------------------ */

typedef struct
{
    uint16_t extra_mm;       /* drive this far once the side has ended      */
    bool     b_edge_found;   /* the obstacle has (for now) ended            */
} along_ctx_t;

/* Every reading while passing it: follow the obstacle's side, and set the
 * move's end once it has ended (or take that back if the obstacle shows up
 * beside the car again). */
static bool s5_bypass_along_ping (const bool b_good,
                                  const uint16_t distance_mm,
                                  void * const p_ctx)
{
    along_ctx_t * const p_along     = (along_ctx_t *)p_ctx;
    const uint16_t      distance_cm = s5_report_mm_to_cm(distance_mm);
    const bool          b_ended     = s5_edge_feed(b_good, distance_cm,
                                                   s5_drive_progress_ticks());

    if (b_ended && (!p_along->b_edge_found))
    {
        p_along->b_edge_found = true;
        if (!s5_edge_was_seen())
        {
            S5_LOG((UB *)"S5 along: obstacle NOT seen beside the car, taken "
                   "as a short one (or the servo is looking the wrong way)\n");
            s5_log_event("S5_ALONG", "NOT_SEEN");
        }
        s5_drive_retarget(s5_edge_last_seen_ticks() +
                          s5_drive_ticks_for_mm(p_along->extra_mm));
        S5_LOG((UB *)"S5 along: obstacle ends after %u mm, driving %u mm "
               "more\n",
               (unsigned int)s5_drive_mm_for_ticks(s5_edge_last_seen_ticks()),
               (unsigned int)p_along->extra_mm);
    }
    else if ((!b_ended) && p_along->b_edge_found)
    {
        /* Seen beside the car again: not past it yet. Keep going (up to
         * S5_EDGE_MAX_MM) until it ends again. */
        p_along->b_edge_found = false;
        s5_drive_retarget(s5_drive_ticks_for_mm(S5_EDGE_MAX_MM));
        S5_LOG((UB *)"S5 along: obstacle still beside the car at %u mm, "
               "carrying on\n",
               (unsigned int)s5_drive_mm_for_ticks(s5_drive_progress_ticks()));
    }
    else
    {
        /* No change: still beside it, or still past it. */
    }

    return false;   /* never stops the move: it only moves its end */
}

/*
 * Drive straight past the obstacle with the sensor turned to look at its
 * side (look_deg) until it has ended, then extra_mm further. If its side is
 * not in view, it is taken as a short obstacle that has already ended (just
 * extra_mm), unless it comes into view on the way. *p_driven_mm receives
 * how far the car went. The servo turns to look ahead again afterwards.
 */
static along_result_t s5_bypass_along (const uint16_t look_deg,
                                       const uint16_t extra_mm,
                                       uint16_t * const p_driven_mm)
{
    along_ctx_t       along  = { extra_mm, false };
    const run_hooks_t hooks  = { NULL, s5_bypass_along_ping, &along };
    along_result_t    result = ALONG_DONE;
    run_result_t      run;

    s5_sense_servo_to(look_deg, S5_LOOK_SIDE);
    s5_edge_start(true);
    run = s5_bypass_run(S5_MOVE_FORWARD, S5_EDGE_MAX_MM, &hooks);

    *p_driven_mm = s5_drive_mm_for_ticks(s5_drive_progress_ticks());
    s5_sense_servo_to(S5_FORWARD_DEG, S5_LOOK_AHEAD);

    if (RUN_DONE != run)
    {
        result = ALONG_FAILED;
    }
    else if (!along.b_edge_found)
    {
        result = ALONG_TOO_BIG;   /* too long: never ended */
        S5_LOG((UB *)"S5 along: obstacle did not end within %u mm, too "
               "long to get round\n", (unsigned int)S5_EDGE_MAX_MM);
        s5_log_event("S5_ALONG", "TOO_LONG");
    }
    else
    {
        S5_LOG((UB *)"S5 along: drove %u mm\n", (unsigned int)*p_driven_mm);
    }

    return result;
}

/* ------------------------------------------------------------------------
 * Back to the line (Stage 5)
 * ------------------------------------------------------------------------ */

/*
 * True if Subsystem 3's latest IR reading is fresh (S5_LINE_MAX_AGE_MS) and
 * either line sensor is over the black line.
 */
static bool s5_bypass_line_seen (void)
{
    ir_line_snapshot_t ir     = { 0 };
    SYSTIM             now    = { 0 };
    bool               b_seen = false;

    if (ir_line_snapshot_shared_read(&ir) && (E_OK == tk_get_tim(&now)) &&
        ir.b_ready && ir.b_gpio_ready)
    {
        /* Unsigned subtraction: the millisecond clock may wrap. */
        const uint32_t age_ms = (uint32_t)now.lo - ir.sampled_at_ms;

        b_seen = ((age_ms <= S5_LINE_MAX_AGE_MS) &&
                  (IR_LINE_BOTH_WHITE != ir.position));
    }

    return b_seen;
}

typedef struct
{
    uint32_t look_from_ticks;   /* only look for the line from here       */
    uint16_t expected_mm;       /* where the IR sensors should reach it   */
    bool     b_found;           /* the IR sensors have seen the line      */
} line_ctx_t;

/* Every poll on the way back: once the line is seen, end the move
 * S5_IR_TO_AXLE_MM later, so the drive wheels are on the line. */
static bool s5_bypass_line_poll (void * const p_ctx)
{
    line_ctx_t * const p_line = (line_ctx_t *)p_ctx;

    if ((!p_line->b_found) &&
        (s5_drive_progress_ticks() >= p_line->look_from_ticks) &&
        s5_bypass_line_seen())
    {
        const uint32_t seen_ticks = s5_drive_progress_ticks();

        p_line->b_found = true;
        s5_drive_retarget(seen_ticks +
                          s5_drive_ticks_for_mm(S5_IR_TO_AXLE_MM));
        S5_LOG((UB *)"S5 line: seen after %u mm (expected about %u)\n",
               (unsigned int)s5_drive_mm_for_ticks(seen_ticks),
               (unsigned int)p_line->expected_mm);
    }

    return false;   /* never stops the move: it only moves its end */
}

/*
 * Drive straight back towards the line, about distance_mm away (the
 * distance the car drove away from it), with the servo facing ahead. From
 * S5_LINE_SEARCH_MM before the expected point up to S5_LINE_SEARCH_MM past
 * it the IR sensors are watched. If the line is never seen, the car backs
 * up to the distance measured by the encoders. Returns false if the car
 * could not complete the move.
 */
static bool s5_bypass_to_line (const uint16_t distance_mm)
{
    /* Casts are safe: the sums stay far below 65535 mm. */
    const uint16_t ahead_mm = (uint16_t)(S5_IR_TO_AXLE_MM + S5_LINE_SEARCH_MM);
    const uint16_t look_mm  = (distance_mm > ahead_mm) ?
                              (uint16_t)(distance_mm - ahead_mm) : 0u;
    const uint16_t limit_mm = (uint16_t)(distance_mm + S5_LINE_SEARCH_MM);
    line_ctx_t     line     =
    {
        s5_drive_ticks_for_mm(look_mm),
        (distance_mm > S5_IR_TO_AXLE_MM) ?
            (uint16_t)(distance_mm - S5_IR_TO_AXLE_MM) : 0u,
        false
    };
    const run_hooks_t hooks = { s5_bypass_line_poll, NULL, &line };
    bool              b_done;

    b_done = (RUN_DONE == s5_bypass_run(S5_MOVE_FORWARD, limit_mm, &hooks));

    if (!b_done)
    {
        /* Blocked, timed out or permission withdrawn: already reported. */
    }
    else if (line.b_found)
    {
        S5_LOG((UB *)"S5 line: wheels on the line after %u mm\n",
               (unsigned int)s5_drive_mm_for_ticks(
                   s5_drive_progress_ticks()));
    }
    else
    {
        /* Line not seen: go back to where the encoders say it should be. */
        const uint16_t driven_mm = s5_drive_mm_for_ticks(
                                       s5_drive_progress_ticks());

        S5_LOG((UB *)"S5 line: NOT seen within %u mm, backing to %u mm\n",
               (unsigned int)limit_mm, (unsigned int)distance_mm);
        if (driven_mm > (uint16_t)(distance_mm + S5_RETURN_TOL_MM))
        {
            /* Cast is safe: the if keeps the difference positive. */
            b_done = s5_bypass_move(S5_MOVE_BACK,
                                    (uint16_t)(driven_mm - distance_mm));
        }
    }

    return b_done;
}

/* ------------------------------------------------------------------------
 * The whole bypass
 * ------------------------------------------------------------------------ */

/* The moves made so far, so they can be undone if the obstacle turns out
 * too big. */
typedef struct
{
    s5_move_t move[MAX_LEGS];
    uint16_t  amount[MAX_LEGS];   /* mm, or degrees for a turn */
    uint8_t   count;
} leg_log_t;

static void s5_bypass_log_leg (leg_log_t * const p_log, const s5_move_t move,
                               const uint16_t amount)
{
    if ((0u != amount) && (p_log->count < MAX_LEGS))
    {
        p_log->move[p_log->count]   = move;
        p_log->amount[p_log->count] = amount;
        p_log->count++;
    }
}

/* The move that undoes 'move'. */
static s5_move_t s5_bypass_undo_move (const s5_move_t move)
{
    s5_move_t undo;

    switch (move)
    {
        case S5_MOVE_BACK:       undo = S5_MOVE_FORWARD;    break;
        case S5_MOVE_TURN_LEFT:  undo = S5_MOVE_TURN_RIGHT; break;
        case S5_MOVE_TURN_RIGHT: undo = S5_MOVE_TURN_LEFT;  break;
        case S5_MOVE_FORWARD:
        default:                 undo = S5_MOVE_BACK;       break;
    }

    return undo;
}

/* Undo every logged move, last first, so the car is back where the bypass
 * started (on the line, facing along it). Only ground the car has just
 * driven over is driven again. Returns true if every move was undone. */
static bool s5_bypass_retrace (const leg_log_t * const p_log)
{
    bool b_ok = true;

    S5_LOG((UB *)"S5 drive: obstacle too big to get round, going back to "
           "the line (%u moves)\n", (unsigned int)p_log->count);
    s5_log_event("S5_DRIVE", "RETRACE");

    for (uint8_t i = p_log->count; b_ok && (i > 0u); i--)
    {
        /* Cast is safe: i - 1 is a valid index below MAX_LEGS. */
        const uint8_t leg = (uint8_t)(i - 1u);

        b_ok = s5_bypass_move(s5_bypass_undo_move(p_log->move[leg]),
                              p_log->amount[leg]);
    }

    return b_ok;
}

/*
 * Second measurement of the edge on the chosen side, printed for tuning
 * S5_EDGE_TRIM_DEG: on the out leg the sensor looks along the original
 * heading, square-on to the obstacle's front, so the sideways position at
 * which it last saw the front is where the edge is. It is sensor-to-axle +
 * distance driven (to that point) along the slant, times sin(turn). Call
 * right after the out leg, only if the front was seen to end. Casts are
 * safe: a few hundred mm at most.
 */
static void s5_bypass_width_check (const bool b_left, const uint16_t plan_cm)
{
    if (s5_edge_was_seen())
    {
        const uint32_t along_mm = (S5_SENSOR_TO_AXLE_CM * S5_MM_PER_CM) +
                                  (uint32_t)s5_drive_mm_for_ticks(
                                      s5_edge_last_seen_ticks());
        const uint32_t side_mm  = (along_mm * s5_profile_sin_q14(
                                       S5_BYPASS_TURN_DEG)) >> 14;

        S5_LOG((UB *)"S5 width check: %s edge about %u cm from the car's "
               "centre line (planned: %u cm)\n", b_left ? "left" : "right",
               (unsigned int)((side_mm + (S5_MM_PER_CM / 2u)) / S5_MM_PER_CM),
               (unsigned int)plan_cm);
    }
}

s5_go_round_t s5_bypass_go_round (const s5_bypass_t side,
                                  const uint16_t plan_cm,
                                  const uint16_t backup_mm)
{
    const bool      b_left   = (S5_BYPASS_LEFT == side);
    const s5_move_t turn_out = b_left ? S5_MOVE_TURN_LEFT : S5_MOVE_TURN_RIGHT;
    const s5_move_t turn_in  = b_left ? S5_MOVE_TURN_RIGHT : S5_MOVE_TURN_LEFT;
    /* The obstacle is on the car's right after a left turn out, and the
     * other way round; S5_HIGH_ANGLE_IS_LEFT says which servo angles look
     * which way. Casts are safe: the static assert keeps 90 +/-
     * S5_PASS_LOOK_OFF_DEG within the servo's 0..180. */
    const bool      b_high_is_right = (0u == S5_HIGH_ANGLE_IS_LEFT);
    const uint16_t  look_deg = (b_left == b_high_is_right) ?
                               (uint16_t)(S5_FORWARD_DEG +
                                          S5_PASS_LOOK_OFF_DEG) :
                               (uint16_t)(S5_FORWARD_DEG -
                                          S5_PASS_LOOK_OFF_DEG);
    /* On the out leg the obstacle's front is S5_BYPASS_TURN_DEG off the
     * car's heading, on the same side as look_deg: look straight at it.
     * Casts are safe: the result stays within 0..180. */
    const uint16_t  out_look_deg = (look_deg > S5_FORWARD_DEG) ?
                    (uint16_t)(S5_FORWARD_DEG + S5_BYPASS_TURN_DEG) :
                    (uint16_t)(S5_FORWARD_DEG - S5_BYPASS_TURN_DEG);
    /* Out leg (step 2): the car turns about its rear axle, so the axle is
     * still on the line after turning out; every mm then driven on the
     * slant moves it sin(turn) mm sideways. Once it turns back its side is
     * half its width from the axle, so it must move the planned edge + half
     * its width + the margin sideways:
     *   out = (plan + half width + margin) / sin(turn).
     * Passing it (step 4, straight): when the sensor loses the side the
     * car drives S5_PASS_CLEAR_MM (about one car's length) more, so the
     * rest of the car is past the corner before it turns in.
     * The cast is safe: the cm values are at most a few hundred, so it fits
     * 16 bits easily (the divisor is at least sin(5 deg), static assert). */
    const uint16_t  out_mm_plan = (uint16_t)(
                    ((((uint32_t)plan_cm + HALF_CAR_CM +
                       S5_BYPASS_CLEAR_MARGIN_CM) * S5_MM_PER_CM) << 14) /
                    s5_profile_sin_q14(S5_BYPASS_TURN_DEG));
    leg_log_t       legs          = { { S5_MOVE_FORWARD }, { 0u }, 0u };
    along_result_t  along         = ALONG_DONE;
    uint16_t        out_mm        = 0u;
    uint16_t        pass_mm       = 0u;
    bool            b_front_ended = false;
    bool            b_ok;
    s5_go_round_t   result        = S5_GO_ROUND_FAILED;

    S5_LOG((UB *)"S5 drive: going round on the %s, edge planned %u cm "
           "out: reversing %u mm, then driving out %u mm\n",
           b_left ? "left" : "right", (unsigned int)plan_cm,
           (unsigned int)backup_mm, (unsigned int)out_mm_plan);
    s5_guard_set_drive(true);

    /* Step 0: only as far back as the out leg needs (not retraced: the car
     * is on the line either way). */
    b_ok = ((0u == backup_mm) || s5_bypass_move(S5_MOVE_BACK, backup_mm));

    /* The servo stays on the obstacle from the turn out until the car has
     * passed it, then faces ahead again for the way back to the line. */
    if (b_ok)
    {
        s5_sense_servo_to(out_look_deg, S5_LOOK_SIDE);
        b_ok = s5_bypass_move(turn_out, S5_BYPASS_TURN_DEG);
    }
    if (b_ok)
    {
        s5_bypass_log_leg(&legs, turn_out, S5_BYPASS_TURN_DEG);
        along = s5_bypass_out(out_look_deg, out_mm_plan, &out_mm,
                              &b_front_ended);
        if ((ALONG_DONE == along) && b_front_ended)
        {
            s5_bypass_width_check(b_left, plan_cm);
        }
        s5_bypass_log_leg(&legs, S5_MOVE_FORWARD, out_mm);
        b_ok  = (ALONG_DONE == along);
    }
    if (b_ok)
    {
        b_ok = s5_bypass_move(turn_in, S5_BYPASS_TURN_DEG);
    }
    if (b_ok)
    {
        s5_bypass_log_leg(&legs, turn_in, S5_BYPASS_TURN_DEG);
        along = s5_bypass_along(look_deg, S5_PASS_CLEAR_MM, &pass_mm);
        s5_bypass_log_leg(&legs, S5_MOVE_FORWARD, pass_mm);
        b_ok  = (ALONG_DONE == along);
    }

    if (ALONG_TOO_BIG == along)
    {
        /* Do not carry on round something this big: go back to the line
         * (only reversing and turning, so the servo may stay turned). */
        if (s5_bypass_retrace(&legs))
        {
            result = S5_GO_ROUND_TOO_BIG;
        }
    }
    else
    {
        /* Back to the line on a gentler slant than going out: to cover
         * the out leg's sideways distance (out x sin(out turn)) it drives
         * out x sin(out turn) / sin(return turn). Widened to 32 bits:
         * 65535 x 16384 fits; the cast back is safe for any real leg (the
         * ratio is at most sin(90) / sin(5), and out legs are well under
         * a metre). */
        const uint16_t back_mm = (uint16_t)(
            ((uint32_t)out_mm * s5_profile_sin_q14(S5_BYPASS_TURN_DEG)) /
            s5_profile_sin_q14(S5_RETURN_TURN_DEG));

        if (b_ok)
        {
            b_ok = s5_bypass_move(turn_in, S5_RETURN_TURN_DEG);
        }
        if (b_ok)
        {
            b_ok = s5_bypass_to_line(back_mm);
        }
        if (b_ok)
        {
            b_ok = s5_bypass_move(turn_out, S5_RETURN_TURN_DEG);
        }
        if (b_ok)
        {
            result = S5_GO_ROUND_DONE;
        }
    }

    /* Always face ahead again (also after a bypass that stopped part-way):
     * the obstacle guard only uses readings taken straight ahead. */
    s5_sense_servo_to(S5_FORWARD_DEG, S5_LOOK_AHEAD);
    s5_guard_set_drive(false);

    if (S5_GO_ROUND_DONE == result)
    {
        /* Stage 5 check: the IR sensors should now be over the line. If
         * not, the line follower is handed the car in its search state,
         * turning toward the side the line is on (subsystem5_take_line_side,
         * passed on by the vehicle controller). */
        const bool b_on_line = s5_bypass_line_seen();

        S5_LOG((UB *)"S5 drive: bypass done, %s\n", b_on_line ?
               "IR sensors on the line" :
               "line NOT under the IR sensors (line follower searches)");
        s5_log_event("S5_DRIVE", b_on_line ? "DONE_ON_LINE" : "DONE_NO_LINE");
        s5_drive_resume();
    }
    else if (S5_GO_ROUND_TOO_BIG == result)
    {
        S5_LOG((UB *)"S5 drive: obstacle too big, back on the line\n");
        s5_log_event("S5_DRIVE", "TOO_BIG");
    }
    else
    {
        S5_LOG((UB *)"S5 drive: bypass stopped part-way, car stays\n");
        s5_log_event("S5_DRIVE", "ABORTED");
    }

    return result;
}
