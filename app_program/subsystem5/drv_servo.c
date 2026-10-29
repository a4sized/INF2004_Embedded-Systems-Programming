/**
 * @file    drv_servo.c
 * @brief   Hobby servo driver (50 Hz PWM, 0-180 degrees).
 *
 * A hobby servo is controlled by a pulse sent every 20 ms (50 Hz).
 * The pulse width sets the angle:
 *     500 us = 0 degrees,  1500 us = 90 degrees,  2500 us = 180 degrees.
 *
 * We clock the PWM at 1 MHz so that 1 PWM count = 1 microsecond,
 * which lets us write pulse widths directly in microseconds.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include <stdbool.h>
#include "drv_servo.h"
#include "s5_config.h"

/* PWM clock: divide the 125 MHz system clock down to 1 MHz. */
#define SYSTEM_CLOCK_HZ         (125000000u)
#define PWM_CLOCK_HZ            (1000000u)

/* One full PWM cycle = 20000 us = 20 ms = 50 Hz. */
#define PWM_PERIOD_US           (20000u)

/* Pulse widths for the two ends of the servo's travel. */
#define PULSE_AT_0_DEGREES_US   (500u)
#define PULSE_AT_180_DEGREES_US (2500u)
#define MAX_ANGLE_DEGREES       (180u)
#define CENTRE_ANGLE_DEGREES    (90u)

/* How many loops to wait for the PWM hardware to come out of reset. */
#define RESET_WAIT_LOOPS        (100000u)

/* The RP2040 has 8 PWM "slices"; each drives 2 pins. Find the slice for our
 * pin and the address of its clock-divider register. */
#define SERVO_PWM_SLICE         ((S5_PIN_SERVO >> 1u) & 0x07u)
#define SERVO_CLOCK_DIVIDER_REGISTER \
    (PWM_BASE + PWM_CHx_DIV + (SERVO_PWM_SLICE * 0x14u))

/* Minimum-bit servo state (9 bits): angle 0..180 needs 8 bits, ready 1 bit.
 * Written and read only by the scanning task (after init), so no lock. */
typedef struct
{
    uint16_t angle_degrees : 8;   /* 0..180 */
    uint16_t b_ready       : 1;   /* 1 = drv_servo_init() succeeded */
} servo_state_t;

static servo_state_t g_servo = { CENTRE_ANGLE_DEGREES, 0u };

/* Turn an angle (0-180) into a pulse width in microseconds.
 * SERVO_TRIM_DEGREES shifts every angle to correct the mounting. */
static uint16_t angle_to_pulse_width_us(const uint16_t angle_degrees)
{
    int32_t trimmed_angle = (int32_t)angle_degrees + SERVO_TRIM_DEGREES;

    /* Keep within the servo's 0-180 degree range. */
    if (trimmed_angle < 0)
    {
        trimmed_angle = 0;
    }
    if (trimmed_angle > (int32_t)MAX_ANGLE_DEGREES)
    {
        trimmed_angle = (int32_t)MAX_ANGLE_DEGREES;
    }

    /* Linear mapping: 0 deg -> 500 us, 180 deg -> 2500 us. */
    return (uint16_t)(PULSE_AT_0_DEGREES_US +
                      ((uint32_t)trimmed_angle *
                       (PULSE_AT_180_DEGREES_US - PULSE_AT_0_DEGREES_US)) /
                      MAX_ANGLE_DEGREES);
}

/*
 * The board support package only takes the PWM hardware out of reset when
 * USE_PTMR is 1, and config.h sets it to 0. While in reset the PWM ignores
 * every register write, so we release it ourselves here.
 */
static ER release_pwm_from_reset(void)
{
    uint32_t loops_left = RESET_WAIT_LOOPS;

    set_w(RESETS_RESET, RESETS_RESET_PWM);    /* hold in reset   */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);    /* then release it */

    /* Wait until the hardware reports that the reset has finished. */
    while (((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0u) &&
           (loops_left > 0u))
    {
        loops_left--;
    }
    return (loops_left > 0u) ? E_OK : E_TMOUT;
}

ER drv_servo_init(void)
{
    ER       result;
    uint32_t clock_divider = SYSTEM_CLOCK_HZ / PWM_CLOCK_HZ;   /* = 125 */

    /* The divider register only holds 1..255. */
    if ((clock_divider == 0u) || (clock_divider > 255u))
    {
        return E_PAR;
    }

    result = release_pwm_from_reset();
    if (result < E_OK)
    {
        return result;
    }

    /* Route the servo pin to the PWM hardware, keep it off while we set up. */
    result = pwm_set_pin(S5_PIN_SERVO);
    if (result < E_OK)
    {
        return result;
    }
    result = pwm_set_enabled(S5_PIN_SERVO, FALSE);
    if (result < E_OK)
    {
        return result;
    }

    /* Slow the PWM clock to 1 MHz. The register has 8 whole-number bits
     * followed by 4 fraction bits, hence the shift by 4. */
    out_w(SERVO_CLOCK_DIVIDER_REGISTER, clock_divider << 4u);

    /* Count 0..19999 then restart: one 20 ms cycle. */
    result = pwm_set_wrap(S5_PIN_SERVO, PWM_PERIOD_US - 1u);
    if (result < E_OK)
    {
        return result;
    }

    /* Start at the centre position, then switch the PWM on. */
    result = pwm_set_cc(S5_PIN_SERVO,
                        angle_to_pulse_width_us(g_servo.angle_degrees));
    if (result < E_OK)
    {
        return result;
    }
    result = pwm_set_enabled(S5_PIN_SERVO, TRUE);
    if (result < E_OK)
    {
        return result;
    }

    g_servo.b_ready = 1u;
    tm_printf((UB*)"S5 servo ready at %u degrees\n",
              (unsigned int)g_servo.angle_degrees);
    return E_OK;
}

ER drv_servo_set_angle(const uint16_t angle_degrees)
{
    uint16_t new_angle = angle_degrees;
    ER       result;

    if (0u == g_servo.b_ready)
    {
        return E_OBJ;   /* drv_servo_init() has not been called */
    }

    if (new_angle > MAX_ANGLE_DEGREES)
    {
        new_angle = MAX_ANGLE_DEGREES;
    }

    /* Changing the compare value changes the pulse width = new angle. */
    result = pwm_set_cc(S5_PIN_SERVO, angle_to_pulse_width_us(new_angle));
    if (result == E_OK)
    {
        g_servo.angle_degrees = new_angle;
    }
    return result;
}

uint16_t drv_servo_get_angle(void)
{
    return (uint16_t)g_servo.angle_degrees;
}
