/** @file ir_barcode_capture.h @brief Bounded single-producer barcode edge transport. */
#ifndef IR_BARCODE_CAPTURE_H
#define IR_BARCODE_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

#define IR_BARCODE_EDGE_CAPACITY 64u

typedef struct
{
    uint32_t time_us;
    bool b_high;
    bool b_ambiguous;
} ir_barcode_edge_t;

typedef struct
{
    ir_barcode_edge_t edges[IR_BARCODE_EDGE_CAPACITY];
    uint32_t head;
    uint32_t tail;
    uint32_t overflow_count;
} ir_barcode_queue_t;

/** Call only before enabling the GPIO interrupt. */
void ir_barcode_queue_reset (ir_barcode_queue_t * p_queue);

/** Called by the one GP16 ISR producer. Returns false when full. */
bool ir_barcode_queue_push_isr (ir_barcode_queue_t * p_queue,
                                ir_barcode_edge_t edge);

/** Called by the one sensing-task consumer. */
bool ir_barcode_queue_pop (ir_barcode_queue_t * p_queue,
                            ir_barcode_edge_t * p_edge);

uint32_t ir_barcode_queue_overflows (ir_barcode_queue_t const * p_queue);

#endif
