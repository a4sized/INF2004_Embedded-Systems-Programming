/** @file ir_fault_status.c @brief Local status; no global fault mask writes. */
#include "ir_fault_status.h"

void ir_fault_status_reset (ir_fault_status_t * p_status)
{
    p_status->b_gpio_configured = false;
    p_status->b_gpio_init_failed = false;
    p_status->b_have_sample = false;
    p_status->last_sample_us = 0u;
    p_status->gaps_over_limit = 0u;
}

void ir_fault_status_note_gpio_init (ir_fault_status_t * p_status,
                                     bool b_success)
{
    if (!b_success)
    {
        p_status->b_gpio_init_failed = true;
        p_status->b_gpio_configured = false;
    }
    else if (!p_status->b_gpio_init_failed)
    {
        p_status->b_gpio_configured = true;
    }
}

void ir_fault_status_note_sample (ir_fault_status_t * p_status,
                                  uint32_t sample_us)
{
    if (!p_status->b_gpio_configured)
    {
        return;
    }

    if (p_status->b_have_sample &&
        ((sample_us - p_status->last_sample_us) >
         IR_FAULT_SAMPLE_GAP_LIMIT_US))
    {
        ++p_status->gaps_over_limit;
    }

    p_status->last_sample_us = sample_us;
    p_status->b_have_sample = true;
}
