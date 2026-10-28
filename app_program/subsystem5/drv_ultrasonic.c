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

uint16_t drv_ultrasonic_read_distance_mm(void)
{
    uint32_t start_time_us;
    uint32_t echo_duration_us;
    bool     echo_received = true;

    /* Step 1: send the 10 us trigger pulse. */
    (void)gpio_set_val(S5_PIN_TRIG, 1u);
    start_time_us = current_time_us();
    while (time_since_us(start_time_us) < TRIGGER_PULSE_US)
    {
        /* busy-wait 10 us */
    }
    (void)gpio_set_val(S5_PIN_TRIG, 0u);

    /* Step 2: wait for the echo pin to go HIGH (sensor started listening). */
    start_time_us = current_time_us();
    while (gpio_get_val(S5_PIN_ECHO) == 0u)
    {
        if (time_since_us(start_time_us) > ECHO_START_TIMEOUT_US)
        {
            echo_received = false;   /* sensor never responded */
            break;
        }
    }

    /* Step 3: measure how long the echo pin stays HIGH. */
    start_time_us = current_time_us();
    while (echo_received && (gpio_get_val(S5_PIN_ECHO) != 0u))
    {
        if (time_since_us(start_time_us) > ECHO_TIMEOUT_US)
        {
            echo_received = false;   /* nothing within range */
        }
    }

    if (!echo_received)
    {
        return NO_OBSTACLE;
    }

    /* Step 4: convert the echo time into a distance. */
    echo_duration_us = time_since_us(start_time_us);
    return (uint16_t)((echo_duration_us * SOUND_SPEED_NUMERATOR) /
                      SOUND_SPEED_DENOMINATOR);
}
