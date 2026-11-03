/** @file ir_barcode_capture.c @brief ISR-to-task single-producer ring. */
#include "ir_barcode_capture.h"

void ir_barcode_queue_reset (ir_barcode_queue_t * p_queue)
{
    __atomic_store_n(&p_queue->head, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&p_queue->tail, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&p_queue->overflow_count, 0u, __ATOMIC_RELAXED);
}

bool ir_barcode_queue_push_isr (ir_barcode_queue_t * p_queue,
                                ir_barcode_edge_t edge)
{
    uint32_t const head = __atomic_load_n(&p_queue->head, __ATOMIC_RELAXED);
    uint32_t const tail = __atomic_load_n(&p_queue->tail, __ATOMIC_ACQUIRE);

    if ((head - tail) >= IR_BARCODE_EDGE_CAPACITY)
    {
        uint32_t const count =
            __atomic_load_n(&p_queue->overflow_count, __ATOMIC_RELAXED);
        __atomic_store_n(&p_queue->overflow_count, count + 1u,
                         __ATOMIC_RELAXED);
        return false;
    }

    p_queue->edges[head % IR_BARCODE_EDGE_CAPACITY] = edge;
    __atomic_store_n(&p_queue->head, head + 1u, __ATOMIC_RELEASE);
    return true;
}

bool ir_barcode_queue_pop (ir_barcode_queue_t * p_queue,
                            ir_barcode_edge_t * p_edge)
{
    uint32_t const tail = __atomic_load_n(&p_queue->tail, __ATOMIC_RELAXED);
    uint32_t const head = __atomic_load_n(&p_queue->head, __ATOMIC_ACQUIRE);

    if (tail == head)
    {
        return false;
    }

    *p_edge = p_queue->edges[tail % IR_BARCODE_EDGE_CAPACITY];
    __atomic_store_n(&p_queue->tail, tail + 1u, __ATOMIC_RELEASE);
    return true;
}

uint32_t ir_barcode_queue_overflows (ir_barcode_queue_t const * p_queue)
{
    return __atomic_load_n(&p_queue->overflow_count, __ATOMIC_RELAXED);
}
