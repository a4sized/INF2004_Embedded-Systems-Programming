/** @file ir_junction.c @brief Bounded GP6 junction sequence from the demo. */
#include "ir_junction.h"

#include <stddef.h>

static bool is_fresh (ir_line_snapshot_t const * p_snapshot,
                      uint32_t now_ms, uint32_t max_age_ms)
{
    return (NULL != p_snapshot) && p_snapshot->b_ready &&
           p_snapshot->b_gpio_ready &&
           ((uint32_t)(now_ms - p_snapshot->sampled_at_ms) <= max_age_ms);
}

bool ir_junction_update (ir_junction_t * p_junction,
                         ir_line_snapshot_t const * p_snapshot,
                         uint32_t now_ms, uint32_t max_age_ms,
                         bool b_follow_allowed, uint8_t base_speed,
                         uint8_t turn_speed,
                         ir_motion_target_t * p_target,
                         ir_junction_event_t * p_event)
{
    if ((NULL == p_junction) || (NULL == p_target) || (NULL == p_event))
    {
        return false;
    }
    *p_event = IR_JUNCTION_NO_EVENT;

    /* Remember each decode even if the car is stopped, so a previous
     * letter cannot arm a turn on a later START command.
     */
    if ((NULL != p_snapshot) &&
        (!p_junction->b_seen_decode_sequence ||
         (p_snapshot->barcode_decode_sequence !=
          p_junction->seen_decode_sequence)))
    {
        p_junction->seen_decode_sequence =
            p_snapshot->barcode_decode_sequence;
        p_junction->b_seen_decode_sequence = true;
        if ((0u != p_snapshot->barcode_decode_sequence) &&
            b_follow_allowed && is_fresh(p_snapshot, now_ms, max_age_ms) &&
            (IR_JUNCTION_IDLE == p_junction->mode))
        {
            ir_barcode_action_t const action =
                ir_barcode_action_for(p_snapshot->decoded_letter);
            if (IR_BARCODE_NO_ACTION != action)
            {
                p_junction->action = action;
                p_junction->mode = IR_JUNCTION_WAIT_CLEAR;
                p_junction->level_samples = 0u;
                *p_event = IR_JUNCTION_ARMED;
            }
        }
    }

    if (!b_follow_allowed || !is_fresh(p_snapshot, now_ms, max_age_ms))
    {
        p_junction->mode = IR_JUNCTION_IDLE;
        p_junction->action = IR_BARCODE_NO_ACTION;
        return false;
    }

    bool const b_barcode_black =
        (IR_SURFACE_BLACK == p_snapshot->barcode_surface);
    switch (p_junction->mode)
    {
        case IR_JUNCTION_WAIT_CLEAR:
        case IR_JUNCTION_WAIT_EXIT:
            p_junction->level_samples = b_barcode_black ? 0u :
                (uint8_t)(p_junction->level_samples + 1u);
            if (p_junction->level_samples >= IR_JUNCTION_LEVEL_SAMPLES)
            {
                p_junction->level_samples = 0u;
                if (IR_JUNCTION_WAIT_CLEAR == p_junction->mode)
                {
                    p_junction->mode = IR_JUNCTION_WAIT_BLACK;
                }
                else
                {
                    p_junction->mode = IR_JUNCTION_TURNING;
                    p_junction->turn_started_ms = now_ms;
                    p_junction->white_seen_mask = 0u;
                    p_junction->barcode_phase = 0u;
                    p_junction->b_line_confirmed = false;
                    *p_event = IR_JUNCTION_TURN_STARTED;
                }
            }
            break;
        case IR_JUNCTION_WAIT_BLACK:
            p_junction->level_samples = b_barcode_black ?
                (uint8_t)(p_junction->level_samples + 1u) : 0u;
            if (p_junction->level_samples >= IR_JUNCTION_LEVEL_SAMPLES)
            {
                p_junction->mode = IR_JUNCTION_WAIT_EXIT;
                p_junction->crossing_started_ms = now_ms;
                p_junction->level_samples = 0u;
                *p_event = IR_JUNCTION_CROSSING;
            }
            break;
        case IR_JUNCTION_TURNING:
        {
            bool const b_left_black =
                (IR_SURFACE_BLACK == p_snapshot->left_surface);
            bool const b_right_black =
                (IR_SURFACE_BLACK == p_snapshot->right_surface);
            if ((b_left_black && (0u != (p_junction->white_seen_mask & 1u))) ||
                (b_right_black && (0u != (p_junction->white_seen_mask & 2u))))
            {
                p_junction->b_line_confirmed = true;
            }
            if (!b_left_black)
            {
                p_junction->white_seen_mask |= 1u;
            }
            if (!b_right_black)
            {
                p_junction->white_seen_mask |= 2u;
            }
            if ((0u == p_junction->barcode_phase) && !b_barcode_black)
            {
                p_junction->barcode_phase = 1u;
            }
            else if ((1u == p_junction->barcode_phase) && b_barcode_black)
            {
                p_junction->barcode_phase = 2u;
            }
            else if ((2u == p_junction->barcode_phase) && !b_barcode_black)
            {
                p_junction->barcode_phase = 3u;
            }
            if ((3u == p_junction->barcode_phase) &&
                p_junction->b_line_confirmed &&
                (b_left_black || b_right_black))
            {
                p_junction->mode = IR_JUNCTION_IDLE;
                p_junction->action = IR_BARCODE_NO_ACTION;
                *p_event = IR_JUNCTION_TURN_DONE;
                return false;
            }
            if ((uint32_t)(now_ms - p_junction->turn_started_ms) >=
                IR_JUNCTION_TIMEOUT_MS)
            {
                p_junction->mode = IR_JUNCTION_FAILED;
                *p_event = IR_JUNCTION_TURN_TIMEOUT;
            }
            break;
        }
        case IR_JUNCTION_IDLE:
        case IR_JUNCTION_FAILED:
        default:
            break;
    }

    if (IR_JUNCTION_WAIT_EXIT == p_junction->mode)
    {
        if ((uint32_t)(now_ms - p_junction->crossing_started_ms) >=
            IR_JUNCTION_CROSSING_TIMEOUT_MS)
        {
            p_junction->mode = IR_JUNCTION_FAILED;
            *p_event = IR_JUNCTION_CROSSING_TIMEOUT;
        }
        else
        {
            /* GP6 is over the black junction cue. Ordinary both-white
             * line loss must not stop the car before GP6 reaches the exit
             * white edge that starts the turn.
             */
            *p_target = (ir_motion_target_t)
            {
                .left = (int16_t)base_speed,
                .right = (int16_t)base_speed,
                .status = IR_STEERING_AVAILABLE
            };
            return true;
        }
    }

    if ((IR_JUNCTION_TURNING == p_junction->mode) ||
        (IR_JUNCTION_FAILED == p_junction->mode))
    {
        *p_target = (ir_motion_target_t){0};
        if (IR_JUNCTION_TURNING == p_junction->mode)
        {
            if (IR_BARCODE_TURN_LEFT == p_junction->action)
            {
                p_target->right = (int16_t)turn_speed;
            }
            else
            {
                p_target->left = (int16_t)turn_speed;
            }
            p_target->status = IR_STEERING_AVAILABLE;
        }
        return true;
    }
    return false;
}
