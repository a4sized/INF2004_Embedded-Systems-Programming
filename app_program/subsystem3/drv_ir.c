/**
 * @file drv_ir.c
 * @brief GPIO access owned by Buddy 3's sensing task.
 */
#include "drv_ir.h"

#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#define DRV_IR_PULL_MASK 0x0cu

static bool configure_input (UINT pin);

bool drv_ir_init (void)
{
    bool const b_left_ready = configure_input(DRV_IR_LEFT_PIN);
    bool const b_right_ready = configure_input(DRV_IR_RIGHT_PIN);
    bool const b_barcode_ready = configure_input(DRV_IR_BARCODE_PIN);

    return b_left_ready && b_right_ready && b_barcode_ready;
}

drv_ir_levels_t drv_ir_read (void)
{
    drv_ir_levels_t levels = {0};

    levels.b_left_high = (0u != gpio_get_val(DRV_IR_LEFT_PIN));
    levels.b_right_high = (0u != gpio_get_val(DRV_IR_RIGHT_PIN));
    levels.b_barcode_high = (0u != gpio_get_val(DRV_IR_BARCODE_PIN));

    return levels;
}

static bool configure_input (UINT pin)
{
    bool const b_ready = (E_OK == gpio_set_pin(pin, GPIO_MODE_IN));

    if (b_ready)
    {
        /* DO is driven by the module, so disable the Pico pad pulls. */
        clr_w(GPIO(pin), DRV_IR_PULL_MASK);
    }

    return b_ready;
}

/* End of file. */
