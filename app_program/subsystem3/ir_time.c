/**
 * @file ir_time.c
 * @brief Pico SDK timer access kept separate from micro T-Kernel headers.
 */
#include "ir_time.h"

#include "hardware/structs/timer.h"

uint32_t ir_time_us (void)
{
    return timer_hw->timerawl;
}

/* End of file. */
