/** @file ir_barcode_window.h @brief Bounded task-side barcode edge window. */
#ifndef IR_BARCODE_WINDOW_H
#define IR_BARCODE_WINDOW_H

#include "ir_barcode_capture.h"

#define IR_BARCODE_WINDOW_US 2000000u

typedef struct
{
    ir_barcode_edge_t edges[IR_BARCODE_EDGE_CAPACITY];
    uint32_t count;
    uint32_t start_us;
    bool b_invalid;
} ir_barcode_window_t;

void ir_barcode_window_reset (ir_barcode_window_t * p_window);
void ir_barcode_window_invalidate (ir_barcode_window_t * p_window);
bool ir_barcode_window_expired (ir_barcode_window_t const * p_window,
                                uint32_t now_us);
void ir_barcode_window_feed (ir_barcode_window_t * p_window,
                             ir_barcode_edge_t edge);

#endif
