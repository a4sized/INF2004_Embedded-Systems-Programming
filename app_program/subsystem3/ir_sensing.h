/**
 * @file ir_sensing.h
 * @brief Fixed per-channel polarity and black/white interpretation.
 */
#ifndef IR_SENSING_H
#define IR_SENSING_H

#include "drv_ir.h"

typedef enum
{
    IR_BLACK_LEVEL_UNKNOWN = 0,
    IR_BLACK_LEVEL_LOW,
    IR_BLACK_LEVEL_HIGH
} ir_black_level_t;

typedef enum
{
    IR_SURFACE_UNMAPPED = 0,
    IR_SURFACE_WHITE,
    IR_SURFACE_BLACK
} ir_surface_t;

typedef struct
{
    ir_black_level_t left;
    ir_black_level_t right;
    ir_black_level_t barcode;
} ir_sensing_map_t;

typedef struct
{
    drv_ir_levels_t raw;
    ir_surface_t left;
    ir_surface_t right;
    ir_surface_t barcode;
} ir_sensing_result_t;

/** @brief Interpret one simultaneous set of digital sensor readings. */
ir_sensing_result_t ir_sensing_read (
    drv_ir_levels_t raw,
    ir_sensing_map_t const * p_map);

#endif

/* End of file. */
