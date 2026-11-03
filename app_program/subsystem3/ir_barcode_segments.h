/** @file ir_barcode_segments.h @brief Provisional bar/gap duration vector. */
#ifndef IR_BARCODE_SEGMENTS_H
#define IR_BARCODE_SEGMENTS_H

#include "ir_barcode_window.h"

#define IR_BARCODE_NORMAL_SCALE 1000000u
#define IR_BARCODE_MAX_SEGMENTS (IR_BARCODE_EDGE_CAPACITY - 1u)

typedef struct
{
    uint32_t fractions[IR_BARCODE_MAX_SEGMENTS];
    uint32_t total_us;
    uint32_t count;
} ir_barcode_segments_t;

/**
 * @brief Prepare bar and internal-gap proportions from an edge window.
 * @return True for a structurally valid, provisional segment vector.
 *
 * A true result does not establish barcode completion or identify a symbol.
 * At least two black bars with a white gap must be bounded by edges. Leading
 * and trailing white areas have no measured duration and are excluded.
 */
bool ir_barcode_segments_make (ir_barcode_window_t const * p_window,
                                ir_barcode_segments_t * p_segments);

#endif
