/**
 * @file    drv_servo.c
 * @brief   Hobby-servo driver (50 Hz PWM).
 *
 * Uses the project BSP for PWM pin, wrap, compare, and enable operations.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include <stdbool.h>
#include "drv_servo.h"
#include "s5_config.h"

#define S5_SERVO_SYS_CLK_HZ          (125000000u)
#define S5_SERVO_TICK_HZ             (1000000u)
#define S5_SERVO_PERIOD_US           (20000u)
#define S5_SERVO_MIN_US              (500u)
#define S5_SERVO_MAX_US              (2500u)
#define S5_SERVO_INVERT              (0)
#define S5_SERVO_MID_US              (1500u)
#define S5_ANGLE_MAX_DEG             (180u)
#define S5_ANGLE_CENTRE_DEG          (90u)
#define S5_SERVO_SETTLE_BASE_MS      (20u)
#define S5_SERVO_SETTLE_PER_DEG_MS   (3u)

static uint16_t s_angle_deg = S5_SERVO_HOME_DEG;
static bool     s_initialized;

#define S5_PWM_RESET_WAIT_LOOPS      (100000u)

#define S5_SERVO_SLICE \
    ((S5_PIN_SERVO >> 1u) & 0x07u)
#define S5_SERVO_DIV_REG \
    (PWM_BASE + PWM_CHx_DIV + (S5_SERVO_SLICE * 0x14u))

static uint16_t drv_servo_pulse_us(const uint16_t angle_deg)
{
    int32_t offset = (int32_t)angle_deg - (int32_t)S5_SERVO_HOME_DEG;
    int32_t pulse;

#if S5_SERVO_INVERT
    offset = -offset;
#endif
    pulse = (int32_t)S5_SERVO_MID_US +
            ((offset * (int32_t)(S5_SERVO_MAX_US - S5_SERVO_MIN_US)) /
             180);
    if (pulse < (int32_t)S5_SERVO_MIN_US)
    {
        pulse = (int32_t)S5_SERVO_MIN_US;
    }
    if (pulse > (int32_t)S5_SERVO_MAX_US)
    {
        pulse = (int32_t)S5_SERVO_MAX_US;
    }
    return (uint16_t)pulse;
}

/*
 * The BSP only releases the PWM block from reset when USE_PTMR is 1, and
 * config.h sets it to 0. A block held in reset ignores every register write,
 * so release it here (harmless if it is already running).
 */
static ER drv_servo_release_pwm_reset(void)
{
    uint32_t wait = S5_PWM_RESET_WAIT_LOOPS;

    set_w(RESETS_RESET, RESETS_RESET_PWM);
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while (((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0u) &&
           (wait > 0u))
    {
        wait--;
    }
    return (wait > 0u) ? E_OK : E_TMOUT;
}

ER drv_servo_init(void)
{
    ER     er;
    uint32_t divider;

    divider = S5_SERVO_SYS_CLK_HZ / S5_SERVO_TICK_HZ;
    if ((divider == 0u) || (divider > 255u))
    {
        return E_PAR;
    }

    er = drv_servo_release_pwm_reset();
    if (er < E_OK)
    {
        return er;
    }

    er = pwm_set_pin(S5_PIN_SERVO);
    if (er < E_OK)
    {
        return er;
    }
    er = pwm_set_enabled(S5_PIN_SERVO, FALSE);
    if (er < E_OK)
    {
        return er;
    }

    /* RP2040 DIV uses 8 integer bits and 4 fractional bits. */
    out_w(S5_SERVO_DIV_REG, divider << 4u);
    er = pwm_set_wrap(S5_PIN_SERVO, S5_SERVO_PERIOD_US - 1u);
    if (er < E_OK)
    {
        return er;
    }
    er = pwm_set_cc(S5_PIN_SERVO, drv_servo_pulse_us(s_angle_deg));
    if (er < E_OK)
    {
        return er;
    }
    er = pwm_set_enabled(S5_PIN_SERVO, TRUE);
    if (er < E_OK)
    {
        return er;
    }

    s_initialized = true;
    tm_printf((UB*)"S5 servo initialized at %u degrees\n",
              (unsigned int)s_angle_deg);
    return E_OK;
}

ER drv_servo_set_angle(const uint16_t angle_deg)
{
    uint16_t a = angle_deg;
    ER       er;

    if (!s_initialized)
    {
        return E_OBJ;
    }

    if (a > S5_ANGLE_MAX_DEG)
    {
        a = S5_ANGLE_MAX_DEG;
    }
    er = pwm_set_cc(S5_PIN_SERVO, drv_servo_pulse_us(a));
    if (er == E_OK)
    {
        s_angle_deg = a;
    }
    return er;
}

uint16_t drv_servo_get_angle(void)
{
    return s_angle_deg;
}

uint16_t drv_servo_settle_ms(const uint16_t target_deg)
{
    uint16_t delta;
    uint16_t settle = 0u;

    if (target_deg >= s_angle_deg)
    {
        delta = (uint16_t)(target_deg - s_angle_deg);
    }
    else
    {
        delta = (uint16_t)(s_angle_deg - target_deg);
    }
    if (delta > 0u)
    {
        settle = (uint16_t)(S5_SERVO_SETTLE_BASE_MS +
                            (delta * S5_SERVO_SETTLE_PER_DEG_MS));
    }
    return settle;
}