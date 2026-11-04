/** @file ir_barcode_gpio.c @brief GP6 transition capture from sensing samples. */
#include "ir_barcode_gpio.h"

static ir_barcode_queue_t barcode_queue;
static bool b_have_level;
static bool b_previous_high;

void ir_barcode_gpio_start (void)
{
    ir_barcode_queue_reset(&barcode_queue);
    b_have_level = false;
    b_previous_high = false;
}

void ir_barcode_gpio_poll (bool b_high, uint32_t sampled_at_us)
{
    if (!b_have_level)
    {
        b_previous_high = b_high;
        b_have_level = true;
    }
    else if (b_high != b_previous_high)
    {
        ir_barcode_edge_t const edge =
        {
            .time_us = sampled_at_us,
            .b_high = b_high,
            .b_ambiguous = false
        };

        b_previous_high = b_high;
        (void)ir_barcode_queue_push_isr(&barcode_queue, edge);
    }
}

bool ir_barcode_gpio_pop (ir_barcode_edge_t * p_edge)
{
    return ir_barcode_queue_pop(&barcode_queue, p_edge);
}

uint32_t ir_barcode_gpio_overflows (void)
{
    return ir_barcode_queue_overflows(&barcode_queue);
}
