/**
 * @file    s5_drive.c
 * @brief   Subsystem 5: encoder-measured moves for driving a bypass.
 *
 * See s5_drive.h. Progress of a move = the average of how far each wheel
 * has turned since the move started (either direction counts, so turns on
 * the spot and a wheel whose encoder counts backwards both work).
 * The encoder counters are read, never reset: Subsystem 4 also uses them.
 *
 * Casts in this file are commented where they are made.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "../app.h"
#include "../subsystem2/subsystem2.h"
#include "s5_config.h"
#include "s5_log.h"
#include "s5_drive.h"

/* Turning on the spot by angle a: each wheel rolls pi x track x a / 360.
 * pi is taken as 355 / 113 (error under 0.00001 %). */
#define PI_NUMERATOR        (355u)
#define PI_DENOMINATOR      (113u)
#define DEGREES_PER_TURN    (360u)

/* Minimum-bit move state. The encoder start counts are the full 32-bit
 * counters from Subsystem 2. A move is at most a few metres, so its target
 * fits in 16 bits (65535 ticks = about 6.6 m). */
typedef struct
{
    int32_t   start_left;     /* left encoder count when the move began  */
    int32_t   start_right;    /* right encoder count when the move began */
    uint16_t  target_ticks;   /* ticks per wheel for this move           */
    s5_move_t move;           /* the move in progress                    */
} move_state_t;

static move_state_t g_move = { 0, 0, 0u, S5_MOVE_FORWARD };

/* Distance in encoder ticks between two counts, either direction. The
 * casts are safe: the difference is taken in 64 bits, made positive, and a
 * real move is far smaller than 2^31 ticks. */
static uint32_t s5_drive_ticks_between (const int32_t from, const int32_t to)
{
    int64_t difference = (int64_t)to - (int64_t)from;

    if (difference < 0)
    {
        difference = -difference;
    }

    return (uint32_t)difference;
}

/* Send wheel speeds to Subsystem 2 (positive = forward). */
static void s5_drive_set_wheels (const int16_t left_speed,
                                 const int16_t right_speed)
{
    s2_set_ir_velocity(left_speed, right_speed);
}

uint16_t s5_drive_ticks_for_mm (const uint16_t distance_mm)
{
    /* Widened to 32 bits: 65535 x 2050 fits. The cast back is safe for
     * any real move (up to about 6.6 m). */
    return (uint16_t)(((uint32_t)distance_mm * S5_ENCODER_TICKS_PER_REV) /
                      S5_WHEEL_CIRCUM_MM);
}

uint16_t s5_drive_turn_mm (const uint16_t angle_deg)
{
    /* Widened to 32 bits: 110 x 355 x 360 fits easily. The result is at
     * most pi x track (about 350 mm for a full turn), so it fits 16 bits. */
    return (uint16_t)(((uint32_t)S5_WHEEL_TRACK_MM * PI_NUMERATOR *
                       angle_deg) / (PI_DENOMINATOR * DEGREES_PER_TURN));
}

/* True for the two turn-on-the-spot moves. */
static bool s5_drive_is_turn (const s5_move_t move)
{
    return ((S5_MOVE_TURN_LEFT == move) || (S5_MOVE_TURN_RIGHT == move));
}

/* Turn state, used only by the scanning task.  g_turn_speed starts at
 * S5_TURN_SPEED for every move and may be raised by s5_drive_nudge().
 * g_last_moved and g_speed_x16 follow the car's speed (ticks per poll, x16
 * for precision); g_coast_ticks is the early-stop amount last worked out;
 * g_b_done latches once the move has reached its target. */
static int16_t  g_turn_speed   = S5_TURN_SPEED;
static uint32_t g_last_moved   = 0u;
static uint32_t g_speed_x16    = 0u;
static uint32_t g_coast_ticks  = 0u;
static bool     g_b_done       = false;

/* How many milliseconds of the car's current speed to allow for coasting. */
static uint32_t s5_drive_coast_ms (const s5_move_t move)
{
    uint32_t coast_ms = S5_FORWARD_COAST_MS;

    if (S5_MOVE_BACK == move)
    {
        coast_ms = S5_BACK_COAST_MS;
    }
    else if (s5_drive_is_turn(move))
    {
        coast_ms = S5_TURN_COAST_MS;
    }
    else
    {
        /* S5_MOVE_FORWARD */
    }

    return coast_ms;
}

/* How far the move has gone: the mean of both wheels, in ticks. */
static uint32_t s5_drive_moved_ticks (void)
{
    uint32_t left_ticks  = s5_drive_ticks_between(g_move.start_left,
                                                  get_left_encoder_ticks());
    uint32_t right_ticks = s5_drive_ticks_between(g_move.start_right,
                                                  get_right_encoder_ticks());

    return ((left_ticks + right_ticks) / 2u);
}

/* Drive the wheels as the current move needs. The (int16_t) casts only
 * restore the type after a negation; the speeds are far below 32767. */
static void s5_drive_wheels_for_move (void)
{
    switch (g_move.move)
    {
        case S5_MOVE_BACK:
            s5_drive_set_wheels(-S5_REVERSE_SPEED, -S5_REVERSE_SPEED);
        break;
        case S5_MOVE_TURN_LEFT:
            s5_drive_set_wheels((int16_t)(-g_turn_speed), g_turn_speed);
        break;
        case S5_MOVE_TURN_RIGHT:
            s5_drive_set_wheels(g_turn_speed, (int16_t)(-g_turn_speed));
        break;
        default:
            /* S5_MOVE_FORWARD */
            s5_drive_set_wheels(S5_DRIVE_SPEED, S5_DRIVE_SPEED);
        break;
    }
}

void s5_drive_start (const s5_move_t move, const uint16_t amount)
{
    uint16_t distance_mm = amount;

    if (s5_drive_is_turn(move))
    {
        distance_mm = s5_drive_turn_mm(amount);   /* amount is degrees */
    }

    g_move.move         = move;
    g_move.start_left   = get_left_encoder_ticks();
    g_move.start_right  = get_right_encoder_ticks();
    g_move.target_ticks = s5_drive_ticks_for_mm(distance_mm);

    g_turn_speed  = S5_TURN_SPEED;
    g_last_moved  = 0u;
    g_speed_x16   = 0u;
    g_coast_ticks = 0u;
    g_b_done      = false;

    s5_drive_wheels_for_move();
}

/*
 * Called once per poll.  A move is done a little EARLY: the car's speed is
 * followed (ticks since the last poll, smoothed over two polls), and the ticks
 * it will still cover while Subsystem 2 ramps the wheels down (the move's
 * coast allowance, in milliseconds of that speed) are counted as already done.
 * Once true it stays true, so the call after the loop agrees with the loop.
 */
bool s5_drive_is_done (void)
{
    uint32_t moved = s5_drive_moved_ticks();

    if (!g_b_done)
    {
        uint32_t coast_ticks = 0u;

        {
            uint32_t step_ticks = (moved >= g_last_moved) ?
                                  (moved - g_last_moved) : 0u;

            g_speed_x16   = (g_speed_x16 + (step_ticks * 16u)) / 2u;
            g_last_moved  = moved;
            coast_ticks   = (g_speed_x16 * s5_drive_coast_ms(g_move.move)) /
                            (S5_DRIVE_POLL_MS * 16u);
            g_coast_ticks = coast_ticks;
        }

        if ((moved + coast_ticks) >= g_move.target_ticks)
        {
            g_b_done = true;
        }
    }

    return g_b_done;
}

void s5_drive_nudge (const uint32_t elapsed_ms)
{
    if (s5_drive_is_turn(g_move.move) && (!g_b_done) && (elapsed_ms > 0u) &&
        (0u == (elapsed_ms % S5_TURN_START_MS)) &&
        (g_turn_speed < S5_DRIVE_SPEED) &&
        (s5_drive_moved_ticks() <
         ((g_move.target_ticks * elapsed_ms) / S5_TURN_MAX_MS)))
    {
        g_turn_speed = (int16_t)(g_turn_speed + S5_TURN_SPEED_STEP);
        if (g_turn_speed > S5_DRIVE_SPEED)
        {
            g_turn_speed = S5_DRIVE_SPEED;
        }
        s5_drive_wheels_for_move();
        S5_LOG((UB *)"S5 turn: wheels not moving, speed raised to %d\n",
               (int)g_turn_speed);
    }
}

void s5_drive_print_result (const s5_move_t move, const uint16_t amount,
                            const bool b_done, const uint32_t elapsed_ms)
{
    static const char * const names[] = { "FORWARD mm", "BACK mm",
                                          "TURN LEFT deg", "TURN RIGHT deg" };

    S5_LOG((UB *)"S5 leg %s %u: %s after %u ms, wheels turned "
           "L=%d R=%d ticks (needed %u)\n",
           names[(unsigned int)move], (unsigned int)amount,
           b_done ? "DONE" : "NOT DONE", (unsigned int)elapsed_ms,
           (int)(get_left_encoder_ticks() - g_move.start_left),
           (int)(get_right_encoder_ticks() - g_move.start_right),
           (unsigned int)g_move.target_ticks);
    S5_LOG((UB *)"S5 move: stopped %u ticks early to allow for coasting"
           "%s\n", (unsigned int)g_coast_ticks,
           s5_drive_is_turn(move) ? " (turn speed shown below)" : "");
    if (s5_drive_is_turn(move))
    {
        S5_LOG((UB *)"S5 turn: speed %d\n", (int)g_turn_speed);
    }
}

uint32_t s5_drive_progress_ticks (void)
{
    return s5_drive_moved_ticks();
}

uint32_t s5_drive_ticks_for_distance (const uint16_t distance_mm)
{
    return (uint32_t)s5_drive_ticks_for_mm(distance_mm);
}

uint16_t s5_drive_mm_for_ticks (const uint32_t ticks)
{
    return (uint16_t)((ticks * S5_WHEEL_CIRCUM_MM) / S5_ENCODER_TICKS_PER_REV);
}

void s5_drive_retarget (const uint32_t target_ticks)
{
    g_move.target_ticks = (target_ticks > 65535u) ?
                          (uint16_t)65535u : (uint16_t)target_ticks;
}

void s5_drive_stop (void)
{
    s5_drive_set_wheels(0, 0);
}

void s5_drive_resume (void)
{
#if (0u != S5_RESUME_AFTER_BYPASS)
    s5_drive_set_wheels(S5_DRIVE_SPEED, S5_DRIVE_SPEED);
#else
    s5_drive_set_wheels(0, 0);
#endif
}
