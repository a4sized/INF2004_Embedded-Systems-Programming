/**
 * @file    s5_drive.h
 * @brief   Subsystem 5: encoder-measured moves for driving a bypass.
 *
 * With S5_DRIVE_BYPASS = 1, Subsystem 5 carries out its bypass decision
 * through Subsystem 2's s2_set_ir_velocity(). Each move (forward, back,
 * turn on the spot) is measured with the wheel encoders: s5_drive_start()
 * sets the wheel speeds, the caller polls s5_drive_is_done(), then calls
 * s5_drive_stop().
 *
 * Call everything only from the scanning task, except the two pure
 * functions, which any task (and the tests) may call.
 */
#ifndef S5_DRIVE_H
#define S5_DRIVE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    S5_MOVE_FORWARD = 0,   /* amount in mm                       */
    S5_MOVE_BACK,          /* amount in mm                       */
    S5_MOVE_TURN_LEFT,     /* amount in degrees, on the spot     */
    S5_MOVE_TURN_RIGHT     /* amount in degrees, on the spot     */
} s5_move_t;

/* Start a move: set the wheel speeds and remember where the encoders are. */
void s5_drive_start (const s5_move_t move, const uint16_t amount);

/* True once both wheels together have covered the move's distance. */
bool s5_drive_is_done (void);

/* True once the move is within S5_DRIVE_TOLERANCE_TICKS of its distance. */
bool s5_drive_is_close (void);

/* Drive the wheels again for the move in progress (a correction pass
 * after it stopped short). The distance still counts from the start. */
void s5_drive_resume_move (void);

/* True once the move will reach its distance if the wheels are told to
 * stop now: covered + coasting >= target. ticks_per_poll = encoder ticks
 * in the last 20 ms (Subsystem 2's period). Coasting = ticks_per_poll x a
 * coast factor: measured on earlier moves of the same kind
 * (s5_drive_learn_coast), or at first estimated from Subsystem 2's
 * ramp-down: (speed / S5_S2_RAMP_STEP + 1) / 2. */
bool s5_drive_should_stop (const uint32_t ticks_per_poll);

/* Time (ms) Subsystem 2 needs to ramp the current move's speed down to 0
 * after s5_drive_stop(): (speed / S5_S2_RAMP_STEP + 1) x 20 ms. */
uint32_t s5_drive_ramp_down_ms (void);

/* Call once the wheels have come to rest after s5_drive_stop(): measures
 * how far the car coasted and remembers it (per kind of move) for the
 * next s5_drive_should_stop(). */
void s5_drive_learn_coast (void);

/* Encoder ticks covered so far in the current move (average of both
 * wheels), and the ticks the move needs. For diagnostics. */
uint32_t s5_drive_ticks_done (void);
uint16_t s5_drive_ticks_target (void);

/* Raise the current move's speed by S5_DRIVE_BOOST_STEP (up to 255, or
 * S5_REVERSE_MAX_SPEED when reversing) when the wheels are not turning.
 * Returns the new speed, 0 if already at the limit. */
uint8_t s5_drive_boost (void);

/* Distance (mm) covered so far in the current move, from the encoders.
 * For diagnostics (checks S5_ENCODER_TICKS_PER_REV on the real car). */
uint16_t s5_drive_mm_done (void);

/* Stop both wheels (end of a move). */
void s5_drive_stop (void);

/* After a successful bypass: with S5_RESUME_AFTER_BYPASS drive straight
 * ahead again, otherwise stay stopped for whoever drives the car. */
void s5_drive_resume (void);

/* Encoder ticks for one wheel to roll distance_mm. Pure function. */
uint16_t s5_drive_ticks_for_mm (const uint16_t distance_mm);

/* Distance (mm) each wheel rolls to turn the car angle_deg on the spot.
 * Pure function. */
uint16_t s5_drive_turn_mm (const uint16_t angle_deg);

#endif /* S5_DRIVE_H */
