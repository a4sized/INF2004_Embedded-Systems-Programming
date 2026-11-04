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
    uint8_t   speed;          /* wheel speed for this move, 1..255       */
    uint16_t  stop_ticks;     /* ticks covered when told to stop         */
    uint8_t   stop_rate;      /* ticks per 20 ms when told to stop       */
} move_state_t;

#define MAX_SPEED           (255u)
#define MOVE_KINDS          (4u)      /* forward, back, turn left, right */
#define COAST_Q             (4u)      /* coast factors in 1/16 steps     */
#define COAST_FACTOR_MAX    (255u)    /* 8-bit factor: up to 15.9 polls  */
#define RATE_MAX            (255u)

static move_state_t g_move = { 0, 0, 0u, S5_MOVE_FORWARD, 0u, 0u, 0u };

/* Learned coast factor per kind of move, in 1/16 of a 20 ms poll at the
 * stopping rate (0 = not learned yet). Scanning task only. */
static uint8_t g_coast_x16[MOVE_KINDS] = { 0u, 0u, 0u, 0u };

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

/* Drive the wheels as the current move needs. */
static void s5_drive_wheels_for_move (void)
{
    int16_t speed = (int16_t)g_move.speed;   /* 0..255 always fits */

    switch (g_move.move)
    {
        case S5_MOVE_BACK:
            s5_drive_set_wheels(-speed, -speed);
        break;

        case S5_MOVE_TURN_LEFT:
            s5_drive_set_wheels(-speed, speed);
        break;

        case S5_MOVE_TURN_RIGHT:
            s5_drive_set_wheels(speed, -speed);
        break;

        default:
            /* S5_MOVE_FORWARD */
            s5_drive_set_wheels(speed, speed);
        break;
    }
}

uint8_t s5_drive_boost (void)
{
    uint32_t limit     = (S5_MOVE_BACK == g_move.move) ?
                         S5_REVERSE_MAX_SPEED : MAX_SPEED;
    uint8_t  new_speed = 0u;

    if (g_move.speed < limit)
    {
        /* Cast is safe: the sum is limited to limit (at most 255). */
        uint32_t raised = (uint32_t)g_move.speed + S5_DRIVE_BOOST_STEP;

        g_move.speed = (uint8_t)((raised > limit) ? limit : raised);
        new_speed    = g_move.speed;
        s5_drive_wheels_for_move();
    }

    return new_speed;
}

uint16_t s5_drive_mm_done (void)
{
    /* Widened: ticks x 207 fits 32 bits for any real move; the result (a
     * few hundred mm) fits 16 bits. */
    return (uint16_t)((s5_drive_ticks_done() * S5_WHEEL_CIRCUM_MM) /
                      S5_ENCODER_TICKS_PER_REV);
}

void s5_drive_start (const s5_move_t move, const uint16_t amount)
{
    uint16_t distance_mm = amount;

    if ((S5_MOVE_TURN_LEFT == move) || (S5_MOVE_TURN_RIGHT == move))
    {
        distance_mm = s5_drive_turn_mm(amount);   /* amount is degrees */
    }

    g_move.move         = move;
    g_move.speed        = (S5_MOVE_BACK == move) ? S5_REVERSE_SPEED :
                                                   S5_DRIVE_SPEED;
    g_move.start_left   = get_left_encoder_ticks();
    g_move.start_right  = get_right_encoder_ticks();
    g_move.target_ticks = s5_drive_ticks_for_mm(distance_mm);

    s5_drive_wheels_for_move();
}



uint32_t s5_drive_ticks_done (void)
{
    uint32_t left_ticks  = s5_drive_ticks_between(g_move.start_left,
                                                  get_left_encoder_ticks());
    uint32_t right_ticks = s5_drive_ticks_between(g_move.start_right,
                                                  get_right_encoder_ticks());

    return ((left_ticks + right_ticks) / 2u);
}

uint16_t s5_drive_ticks_target (void)
{
    return g_move.target_ticks;
}

bool s5_drive_is_done (void)
{
    return (s5_drive_ticks_done() >= g_move.target_ticks);
}

bool s5_drive_is_close (void)
{
    return ((s5_drive_ticks_done() + S5_DRIVE_TOLERANCE_TICKS) >=
            g_move.target_ticks);
}

void s5_drive_resume_move (void)
{
    s5_drive_wheels_for_move();
}

bool s5_drive_should_stop (const uint32_t ticks_per_poll)
{
    uint32_t factor_x16 = g_coast_x16[g_move.move];
    uint32_t done       = s5_drive_ticks_done();

    if (0u == factor_x16)
    {
        /* Not learned yet: estimate from Subsystem 2's ramp-down. */
        factor_x16 = ((((uint32_t)g_move.speed / S5_S2_RAMP_STEP) + 1u) <<
                      COAST_Q) / 2u;
    }

    /* Remember the stopping point and rate for s5_drive_learn_coast().
     * Casts are safe: both are limited to their field sizes first. */
    g_move.stop_ticks = (uint16_t)((done > UINT16_MAX) ? UINT16_MAX : done);
    g_move.stop_rate  = (uint8_t)((ticks_per_poll > RATE_MAX) ?
                                  RATE_MAX : ticks_per_poll);

    return ((done + ((ticks_per_poll * factor_x16) >> COAST_Q)) >=
            g_move.target_ticks);
}

void s5_drive_stop (void)
{
    s5_drive_set_wheels(0, 0);
}

uint32_t s5_drive_ramp_down_ms (void)
{
    return ((((uint32_t)g_move.speed / S5_S2_RAMP_STEP) + 1u) *
            S2_PERIOD_MS);
}

void s5_drive_learn_coast (void)
{
    uint32_t done = s5_drive_ticks_done();

    if ((0u != g_move.stop_rate) && (done >= g_move.stop_ticks))
    {
        uint32_t factor_x16 = ((done - g_move.stop_ticks) << COAST_Q) /
                              g_move.stop_rate;
        uint32_t old_x16    = g_coast_x16[g_move.move];

        if (0u != old_x16)
        {
            factor_x16 = (factor_x16 + old_x16 + 1u) / 2u;   /* smooth */
        }
        if (0u == factor_x16)
        {
            factor_x16 = 1u;   /* stops dead: still mark it as learned */
        }

        /* Cast is safe: limited to COAST_FACTOR_MAX (8 bits) first. */
        g_coast_x16[g_move.move] = (uint8_t)((factor_x16 > COAST_FACTOR_MAX)
                                             ? COAST_FACTOR_MAX : factor_x16);
    }
}

void s5_drive_resume (void)
{
#if (0u != S5_RESUME_AFTER_BYPASS)
    s5_drive_set_wheels(S5_DRIVE_SPEED, S5_DRIVE_SPEED);
#else
    s5_drive_set_wheels(0, 0);
#endif
}
