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
