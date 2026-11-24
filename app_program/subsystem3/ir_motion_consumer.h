/** @file ir_motion_consumer.h @brief Integrated IR consumer task. */
#ifndef IR_MOTION_CONSUMER_H
#define IR_MOTION_CONSUMER_H
#include <tk/tkernel.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool b_follow_allowed;
    uint8_t base_speed;
    uint8_t turn_speed;
    uint32_t max_age_ms;
} ir_motion_settings_t;

/* Provisional first-run targets in Buddy 2's 0..255 units. */
#define IR_START_BASE_SPEED 51u  /* Trial: 20% of 0..255 PWM. */
#define IR_START_TURN_SPEED 64u  /* Keep 25% for junction/search torque. */
#define IR_START_MAX_AGE_MS 20u

/** Create the task-side permission mutex before tasks start. */
ER ir_motion_consumer_init (void);

/** Vehicle Controller handoff; revocation writes zero to Buddy 2 immediately.
 *  Task context only. The caller owns START/STOP/ESTOP policy.
 */
ER ir_motion_consumer_set_following (ir_motion_settings_t const * p_settings);

/** Vehicle Controller handoff for obstacle avoidance. While paused the task
 *  does not write to Buddy 2 at all, so Subsystem 5 can drive the bypass
 *  without being overwritten. Task context only.
 */
ER ir_motion_consumer_set_paused (bool b_paused);

/** Vehicle Controller handoff after an obstacle bypass: the next cycle that
 *  follows the line starts in the SEARCH state, turning toward the line
 *  (b_line_on_left, else right), instead of waiting; if either sensor
 *  already sees the line it tracks as usual, and still remembers that side
 *  for its next correction. Call after following is permitted again; a
 *  revocation cancels it. Task context only.
 */
ER ir_motion_consumer_request_search (bool b_line_on_left);

void ir_motion_consumer_task (INT stacd, void * exinf);
#endif
