/**
 * @file ir_sensing.c
 * @brief Interpret DO levels without changing module sensitivity.
 */
#include "ir_sensing.h"

static ir_surface_t classify (bool b_high, ir_black_level_t black_level);

ir_sensing_result_t ir_sensing_read (
    drv_ir_levels_t raw,
    ir_sensing_map_t const * p_map)
{
    ir_sensing_result_t result = {0};

    result.raw = raw;
    result.left = classify(raw.b_left_high, p_map->left);
    result.right = classify(raw.b_right_high, p_map->right);
    result.barcode = classify(raw.b_barcode_high, p_map->barcode);

    return result;
}

static ir_surface_t classify (bool b_high, ir_black_level_t black_level)
{
    ir_surface_t surface = IR_SURFACE_UNMAPPED;

    if (IR_BLACK_LEVEL_HIGH == black_level)
    {
        surface = b_high ? IR_SURFACE_BLACK : IR_SURFACE_WHITE;
    }
    else if (IR_BLACK_LEVEL_LOW == black_level)
    {
        surface = b_high ? IR_SURFACE_WHITE : IR_SURFACE_BLACK;
    }

    return surface;
}

/* End of file. */
