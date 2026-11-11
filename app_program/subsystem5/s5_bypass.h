/**
 * @file    s5_bypass.h
 * @brief   Subsystem 5, stage 4 and 5: driving round an obstacle and back
 *          to the line, through Subsystem 2 (s2_set_ir_velocity).
 *
 * Moves are measured with the wheel encoders (s5_drive.c). Every move stops
 * at once if the vehicle controller withdraws the drive permission. Call
 * everything except the permission functions only from the scanning task.
 */
#ifndef S5_BYPASS_H
#define S5_BYPASS_H

#include <stdbool.h>
#include <stdint.h>
#include "s5_avoid.h"

/* How going round an obstacle ended. */
typedef enum
{
    S5_GO_ROUND_DONE = 0,   /* round it and back at the line (the IR sensors
                             * over it, or the line follower searches)      */
    S5_GO_ROUND_TOO_BIG,    /* it turned out too wide or too long to get
                             * round: the car retraced its moves and is back
                             * on the line where it started, facing it      */
    S5_GO_ROUND_FAILED      /* stopped part-way (blocked, timed out, or the
                             * permission was withdrawn): off the line      */
} s5_go_round_t;

/* Drive permission, set by the vehicle controller (any task). */
void s5_bypass_set_permitted (const bool b_allowed);
bool s5_bypass_permitted (void);

/* Reverse distance_mm in a straight line (to scan again from further
 * away). Returns true if the whole distance was driven. */
bool s5_bypass_back (const uint16_t distance_mm);

/* Go round the obstacle on side (S5_BYPASS_LEFT or S5_BYPASS_RIGHT), then
 * back to the line. plan_cm is the planned edge on that side, in cm from
 * the car's centre line (s5_avoid_plan_extent): the car drives out far
 * enough to clear it. It first reverses backup_mm straight back along the
 * line (s5_avoid_plan_backup_mm; 0 = not at all), for room to swing out.
 * The servo faces ahead again afterwards. */
s5_go_round_t s5_bypass_go_round (const s5_bypass_t side,
                                  const uint16_t plan_cm,
                                  const uint16_t backup_mm);

#endif /* S5_BYPASS_H */
