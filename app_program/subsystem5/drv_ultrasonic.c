/**
 * @file    drv_ultrasonic.c
 * @brief   HC-SR04 ultrasonic distance sensor driver.
 *
 * How the sensor works:
 *   1. We send a 10 us HIGH pulse on the trigger pin.
 *   2. The sensor sends out a sound burst and sets the echo pin HIGH.
 *   3. The echo pin goes LOW again when the sound bounces back.
 *   4. How long echo stayed HIGH tells us how far away the obstacle is.
 *
 * Timing uses the RP2040's free-running 1 MHz timer (1 count = 1 us).
 * Every wait has a timeout, so a missing sensor can never hang the task.
 */
#include "drv_ultrasonic.h"
#include "s5_config.h"
#include <bsp/libbsp.h>
#include <stdbool.h>

/* Length of the trigger pulse the sensor needs. */
#define TRIGGER_PULSE_US        (10u)

/* How long to wait for the echo pin to go HIGH after triggering. */
#define ECHO_START_TIMEOUT_US   (3000u)

/* Distance (mm) = echo time (us) * 343 / 2000.
 * Sound travels 0.343 mm per us, and we divide by 2 because the
 * sound goes to the obstacle and back. */
#define SOUND_SPEED_NUMERATOR   (343u)
#define SOUND_SPEED_DENOMINATOR (2000u)

/* After a ping with nothing in range, some HC-SR04 boards keep echo HIGH
 * for up to ~200 ms. HIGH for longer than this after the last trigger means
 * the pin is stuck, not busy. */
#define ECHO_BUSY_LIMIT_US      (250000u)

/* When the sensor was last triggered (for telling "busy" from "stuck").
 * Minimum-bit: 31 bits of timer + 1 flag = one 32-bit word. 31 bits of a
 * 1 MHz timer wrap every ~35 minutes, far longer than ECHO_BUSY_LIMIT_US,
 * so elapsed time is still exact when worked out modulo 2^31.
 * Only the scanning task touches it, so no lock. */
#define TRIGGER_TIME_MASK       (0x7FFFFFFFu)

typedef struct
{
    uint32_t last_trigger_us : 31;   /* low 31 bits of the timer */
    uint32_t b_triggered     : 1;    /* 1 = triggered at least once */
} trigger_state_t;

static trigger_state_t g_trigger = { 0u, 0u };

/* Read the current time from the 1 MHz hardware timer, in microseconds. */
static uint32_t current_time_us(void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

/* Microseconds that have passed since start_time_us. */
static uint32_t time_since_us(const uint32_t start_time_us)
{
    return current_time_us() - start_time_us;
}

ER drv_ultrasonic_init(void)
{
    ER result;

    /* Trigger pin is an output, starting LOW. */
    result = gpio_set_pin(S5_PIN_TRIG, GPIO_MODE_OUT);
    if (result < E_OK)
    {
        return result;
    }
    result = gpio_set_val(S5_PIN_TRIG, 0u);
    if (result < E_OK)
    {
        return result;
    }

    /* Echo pin is an input that the sensor drives. */
    return gpio_set_pin(S5_PIN_ECHO, GPIO_MODE_IN);
}

ER drv_ultrasonic_measure(uint16_t * const p_distance_mm)
{
    uint32_t start_time_us;
    uint32_t echo_duration_us;
    bool     b_in_range = true;

    if (NULL == p_distance_mm)
    {
        return E_PAR;
    }
    *p_distance_mm = NO_OBSTACLE;

    /* Step 0: echo must be LOW while idle. HIGH soon after a ping means the
     * sensor is still busy (not a fault); HIGH for long means it is stuck
     * or mis-wired. */
    if (0u != gpio_get_val(S5_PIN_ECHO))
    {
        if ((0u != g_trigger.b_triggered) &&
            (((current_time_us() - (uint32_t)g_trigger.last_trigger_us) &
              TRIGGER_TIME_MASK) < ECHO_BUSY_LIMIT_US))
        {
            return E_BUSY;
        }
        return E_IO;
    }

    /* Step 1: send the 10 us trigger pulse. */
    (void)gpio_set_val(S5_PIN_TRIG, 1u);
    start_time_us = current_time_us();
    g_trigger.last_trigger_us = start_time_us & TRIGGER_TIME_MASK;
    g_trigger.b_triggered     = 1u;
    while (time_since_us(start_time_us) < TRIGGER_PULSE_US)
    {
        /* busy-wait 10 us */
    }
    (void)gpio_set_val(S5_PIN_TRIG, 0u);

    /* Step 2: wait for the echo pin to go HIGH (sensor started listening).
     * A working sensor always does this, obstacle or not. */
    start_time_us = current_time_us();
    while (0u == gpio_get_val(S5_PIN_ECHO))
    {
        if (time_since_us(start_time_us) > ECHO_START_TIMEOUT_US)
        {
            return E_TMOUT;   /* sensor never responded: hardware fault */
        }
    }

    /* Step 3: measure how long the echo pin stays HIGH. */
    start_time_us = current_time_us();
    while (b_in_range && (0u != gpio_get_val(S5_PIN_ECHO)))
    {
        if (time_since_us(start_time_us) > ECHO_TIMEOUT_US)
        {
            b_in_range = false;   /* nothing within range: not a fault */
        }
    }

    if (b_in_range)
    {
        /* Step 4: convert the echo time into a distance. */
        echo_duration_us = time_since_us(start_time_us);
        *p_distance_mm = (uint16_t)((echo_duration_us * SOUND_SPEED_NUMERATOR) /
                                    SOUND_SPEED_DENOMINATOR);
    }
    return E_OK;
}

uint16_t drv_ultrasonic_read_distance_mm(void)
{
    uint16_t distance_mm = NO_OBSTACLE;

    (void)drv_ultrasonic_measure(&distance_mm);
    return distance_mm;
}
