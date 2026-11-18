/**
 * @file ir_line_filter.h
 * @brief Suppress brief left/right digital changes before line decisions.
 */
#ifndef IR_LINE_FILTER_H
#define IR_LINE_FILTER_H

#include "drv_ir.h"

#include <stdint.h>

typedef struct
{
    bool b_initialized;
    bool b_stable_high;
    bool b_candidate_high;
    uint32_t candidate_since_ms;
} ir_line_channel_t;

typedef struct
{
    ir_line_channel_t left;
    ir_line_channel_t right;
} ir_line_filter_t;

/** @brief Clear both channel histories before the first sample. */
void ir_line_filter_reset (ir_line_filter_t * p_filter);

/**
 * @brief Return stable line levels, while leaving barcode raw and immediate.
 * @details A change needs two observations separated by stable_ms or more.
 * The time subtraction tolerates a wrapping 32-bit millisecond counter.
 */
drv_ir_levels_t ir_line_filter_update (
    ir_line_filter_t * p_filter,
    drv_ir_levels_t raw,
    uint32_t now_ms,
    uint32_t stable_ms);

#endif

/* End of file. */
