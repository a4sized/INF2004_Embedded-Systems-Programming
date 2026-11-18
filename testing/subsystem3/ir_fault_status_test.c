#include "ir_fault_status.h"

#include <assert.h>
#include <stdint.h>

int main (void)
{
    ir_fault_status_t status = {0};

    ir_fault_status_reset(&status);
    ir_fault_status_note_sample(&status, 100u);
    assert(!status.b_have_sample);

    ir_fault_status_note_gpio_init(&status, true);
    ir_fault_status_note_sample(&status, UINT32_MAX - 5000u);
    ir_fault_status_note_sample(&status, 4999u);
    assert(status.b_have_sample);
    assert(0u == status.gaps_over_limit);
    ir_fault_status_note_sample(&status, 15000u);
    assert(1u == status.gaps_over_limit);

    ir_fault_status_note_gpio_init(&status, false);
    assert(status.b_gpio_init_failed);
    assert(!status.b_gpio_configured);
    ir_fault_status_note_gpio_init(&status, true);
    assert(status.b_gpio_init_failed);
    assert(!status.b_gpio_configured);
    ir_fault_status_note_sample(&status, 50000u);
    assert(1u == status.gaps_over_limit);

    ir_fault_status_reset(&status);
    assert(!status.b_gpio_init_failed);
    assert(!status.b_have_sample);
    assert(0u == status.gaps_over_limit);
    return 0;
}
