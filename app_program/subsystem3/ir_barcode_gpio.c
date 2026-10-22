/** @file ir_barcode_gpio.c @brief Standalone Pico GPIO edge adapter. */
#include "ir_barcode_gpio.h"

#include "drv_ir.h"
#include "ir_time.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#include "hardware/gpio.h"
#pragma GCC diagnostic pop

static ir_barcode_queue_t barcode_queue;

static void barcode_irq (uint gpio, uint32_t events)
{
    if (DRV_IR_BARCODE_PIN == gpio)
    {
        ir_barcode_edge_t edge = {0};

        edge.time_us = ir_time_us();
        if (0u != (events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)))
        {
            edge.b_high = (0u != (events & GPIO_IRQ_EDGE_RISE));
            edge.b_ambiguous =
                ((events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) ==
                 (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL));
            (void)ir_barcode_queue_push_isr(&barcode_queue, edge);
        }
    }
}

void ir_barcode_gpio_start (void)
{
    ir_barcode_queue_reset(&barcode_queue);
    gpio_set_irq_enabled_with_callback(DRV_IR_BARCODE_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, barcode_irq);
}

bool ir_barcode_gpio_pop (ir_barcode_edge_t * p_edge)
{
    return ir_barcode_queue_pop(&barcode_queue, p_edge);
}

uint32_t ir_barcode_gpio_overflows (void)
{
    return ir_barcode_queue_overflows(&barcode_queue);
}
