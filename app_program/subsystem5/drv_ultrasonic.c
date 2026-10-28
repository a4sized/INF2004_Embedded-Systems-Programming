/**
 * @file    drv_ultrasonic.c
 * @brief   HC-SR04 style ultrasonic ranger.
 *
 * The echo is timed by polling the 1 MHz hardware timer. The wait is
 * bounded by S5_ECHO_TIMEOUT_US (about 9 ms) so it never hangs.
 * Kernel ticks that preempt the poll add a few mm of jitter.
 */
#include "drv_ultrasonic.h"
#include "s5_config.h"
#include <bsp/libbsp.h>
#include <stdbool.h>

static uint32_t drv_us_elapsed(const uint32_t t_start)
{
    return (uint32_t)(in_w(TIMER_TIMERAWL) - t_start);
}

ER drv_ultrasonic_init(void)
{
    ER er;

    er = gpio_set_pin(S5_PIN_TRIG, GPIO_MODE_OUT);
    if (er < E_OK)
    {
        return er;
    }
    er = gpio_set_val(S5_PIN_TRIG, 0u);
    if (er < E_OK)
    {
        return er;
    }

    return gpio_set_pin(S5_PIN_ECHO, GPIO_MODE_IN);
}

uint16_t drv_ultrasonic_ping_mm(void)
{
    uint16_t result = S5_DIST_NONE;
    uint32_t t_trig;
    uint32_t t_rise;
    uint32_t width_us;
    bool     b_ok = true;

    /* 10 us trigger pulse. */
    (void)gpio_set_val(S5_PIN_TRIG, 1u);
    t_trig = (uint32_t)in_w(TIMER_TIMERAWL);
    while (drv_us_elapsed(t_trig) < S5_TRIG_PULSE_US)
    {
        /* wait */
    }
    (void)gpio_set_val(S5_PIN_TRIG, 0u);

    /* Wait for echo to go high. */
    t_trig = (uint32_t)in_w(TIMER_TIMERAWL);
    while (gpio_get_val(S5_PIN_ECHO) == 0u)
    {
        if (drv_us_elapsed(t_trig) > S5_ECHO_START_TIMEOUT_US)
        {
            b_ok = false;
            break;
        }
    }

    /* Time the echo pulse. */
    t_rise = (uint32_t)in_w(TIMER_TIMERAWL);
    while (b_ok && gpio_get_val(S5_PIN_ECHO) != 0u)
    {
        if (drv_us_elapsed(t_rise) > S5_ECHO_TIMEOUT_US)
        {
            b_ok = false;
        }
    }

    if (b_ok)
    {
        width_us = drv_us_elapsed(t_rise);
        result = (uint16_t)((width_us * S5_US_SOUND_NUM) / S5_US_SOUND_DEN);
    }
    return result;
}
