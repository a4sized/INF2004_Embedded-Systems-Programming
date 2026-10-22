#include "ir_barcode_capture.h"

#include <assert.h>
#include <stdint.h>

int main (void)
{
    ir_barcode_queue_t queue = {0};
    ir_barcode_edge_t edge = {0};

    ir_barcode_queue_reset(&queue);
    for (uint32_t index = 0u; index < IR_BARCODE_EDGE_CAPACITY; ++index)
    {
        ir_barcode_edge_t const input = {index * 7u, (0u == (index & 1u)), false};
        assert(ir_barcode_queue_push_isr(&queue, input));
    }
    assert(!ir_barcode_queue_push_isr(&queue, edge));
    assert(1u == ir_barcode_queue_overflows(&queue));

    for (uint32_t index = 0u; index < IR_BARCODE_EDGE_CAPACITY; ++index)
    {
        assert(ir_barcode_queue_pop(&queue, &edge));
        assert(index * 7u == edge.time_us);
        assert((0u == (index & 1u)) == edge.b_high);
    }
    assert(!ir_barcode_queue_pop(&queue, &edge));

    for (uint32_t index = 0u; index < IR_BARCODE_EDGE_CAPACITY * 3u; ++index)
    {
        ir_barcode_edge_t const input = {index, true, false};
        assert(ir_barcode_queue_push_isr(&queue, input));
        assert(ir_barcode_queue_pop(&queue, &edge));
        assert(index == edge.time_us);
    }
    return 0;
}
