/**
 * @file drv_ir.c
 * @brief Use the professor's BSP GPIO driver for the standalone sensor.
 * @details VCC is supplied from 3V3, never from a GPIO output. AO is unused.
 */
#include "drv_ir.h"

#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#define DRV_IR_PULL_MASK 0x0cu

bool drv_ir_init (void)
{
    bool const b_ready = (E_OK == gpio_set_pin(DRV_IR_PIN, GPIO_MODE_IN));

    if (b_ready)
    {
        /* PAD bits 3:2 select pulls. Do not bias the module's DO output. */
        clr_w(GPIO(DRV_IR_PIN), DRV_IR_PULL_MASK);
    }

    return b_ready;
}

bool drv_ir_is_high (void)
{
    return (0u != gpio_get_val(DRV_IR_PIN));
}

/* End of file. */

