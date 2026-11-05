/**
 * @file ir_main.c
 * @brief USB-guided black/white calibration; no movement or network calls.
 * @details The starter discards USB input, so prompts use timed countdowns.
 */
#include "ir_main.h"
#include "drv_ir.h"
#include "ir_calib.h"

#include <tm/tmonitor.h>

#define BENCH_PRIORITY       16
#define BENCH_STACK_BYTES    2048
#define BENCH_LINK_READY     3u
#define BENCH_POLL_MS        200u
#define BENCH_SAMPLE_MS      20u
#define BENCH_COUNTDOWN_SEC  8u
#define BENCH_SECOND_MS      1000u

/* These existing port functions have no public header in the starter. */
extern UW tm_usb_state (void);

static void ir_bench_task (INT start_code, void * p_extra);
static void bench_halt (void);
static bool bench_wait (uint32_t delay_ms);
static bool bench_capture (char const * p_surface,
                           ir_calib_window_t * p_window);
static void bench_monitor (ir_calib_result_t const * p_result);

INT usermain (void)
{
    T_CTSK task_config = {0};

    /* FP is the kernel's generic callback type; the task ABI is unchanged. */
    task_config.task    = (FP)ir_bench_task;
    task_config.itskpri = BENCH_PRIORITY;
    task_config.stksz   = BENCH_STACK_BYTES;
    task_config.tskatr  = TA_HLNG | TA_RNG0;

#if TK_SUPPORT_SMP
    task_config.tskatr |= TA_ASSPRC;
    task_config.assprc  = TP_PRC1;
#endif

    ID const h_task = tk_cre_tsk(&task_config);

    if (h_task <= E_OK)
    {
        bench_halt();
    }

    if (E_OK != tk_sta_tsk(h_task, 0))
    {
        bench_halt();
    }

    bench_halt();
    return 0;
}

/** @brief Yield forever on an unrecoverable RTOS or GPIO error. */
static void bench_halt (void)
{
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

/** @brief Yield, and abort the current session if the terminal disconnects. */
static bool bench_wait (uint32_t delay_ms)
{
    if (E_OK != tk_dly_tsk(delay_ms))
    {
        bench_halt();
    }

    return (BENCH_LINK_READY == tm_usb_state());
}

/** @brief Train repeatedly until successful, then display learned readings. */
static void ir_bench_task (INT start_code, void * p_extra)
{
    (void)start_code;
    (void)p_extra;

    if (!drv_ir_init())
    {
        bench_halt();
    }

    for (;;)
    {
        while (BENCH_LINK_READY != tm_usb_state())
        {
            (void)bench_wait(BENCH_POLL_MS);
        }

        /* UB is an unsigned byte; the port reads these string literals. */
        tm_printf((UB const *)"\nIR calibration bench v1: DO=GP6, AO unused.\n"
                  "Keep height/lighting fixed. No keyboard input needed.\n");

        ir_calib_window_t black  = {0};
        ir_calib_window_t white  = {0};
        ir_calib_result_t result = {0};

        bool const b_black_done = bench_capture("BLACK", &black);
        bool       b_white_done = false;

        if (b_black_done)
        {
            b_white_done = bench_capture("WHITE", &white);
        }

        if (b_white_done)
        {
            ir_calib_status_t const status =
                ir_calib_evaluate(&black, &white, &result);

            if (IR_CALIB_OK == status)
            {
                bench_monitor(&result);
            }
            else
            {
                /* Status fits int; UB preserves the literal's bytes. */
                tm_printf((UB const *)"FAIL status=%d (2=noisy, 3=same level)."
                          "\nAdjust pot/height/wiring. Retrying.\n",
                          (int)status);
            }
        }
    }
}

/** @brief Give positioning time, collect a window, and print its evidence. */
static bool bench_capture (char const * p_surface,
                           ir_calib_window_t * p_window)
{
    ir_calib_reset(p_window);

    /* UB preserves literal bytes; p_surface is a null-terminated string. */
    tm_printf((UB const *)"\nPlace sensor over %s and hold still.\n",
              p_surface);

    for (uint32_t seconds = BENCH_COUNTDOWN_SEC; seconds > 0u; --seconds)
    {
        /* UW matches T-Monitor's %lu ABI and contains this bounded value. */
        tm_printf((UB const *)"Capture in %lu...\n", (UW)seconds);

        if (!bench_wait(BENCH_SECOND_MS))
        {
            return false;
        }
    }

    /* UB preserves literal bytes without discarding const qualification. */
    tm_printf((UB const *)"Sampling %s: HOLD STILL.\n", p_surface);

    SYSTIM started = {0};
    SYSTIM ended   = {0};

    if (E_OK != tk_get_tim(&started))
    {
        bench_halt();
    }

    for (uint32_t sample = 0u; sample < IR_CALIB_SAMPLES; ++sample)
    {
        if (!bench_wait(BENCH_SAMPLE_MS))
        {
            return false;
        }

        (void)ir_calib_add(p_window, drv_ir_is_high());
    }

    if (E_OK != tk_get_tim(&ended))
    {
        bench_halt();
    }

    /* UW is 32-bit on target; all counts are <=100. The span is <49 days. */
    tm_printf((UB const *)"%s: high=%lu/100, transitions=%lu, elapsed=%lu ms\n",
              p_surface, (UW)p_window->high_count,
              (UW)p_window->transitions, ended.lo - started.lo);

    return true;
}

/** @brief Print instantaneous calibrated levels until the terminal closes. */
static void bench_monitor (ir_calib_result_t const * p_result)
{
    /* UW matches T-Monitor's %lu; the Boolean conversion yields 0 or 1. */
    tm_printf((UB const *)"\nPASS: BLACK=%lu, WHITE=%lu. Calibration in RAM.\n"
              "Move between surfaces. Reset/reopen terminal to recalibrate.\n",
              (UW)p_result->b_black_high, (UW)!p_result->b_black_high);

    while (BENCH_LINK_READY == tm_usb_state())
    {
        bool const b_high  = drv_ir_is_high();
        bool const b_black = (b_high == p_result->b_black_high);

        /* UW holds 0 or 1; UB preserves the string literal's byte values. */
        tm_printf((UB const *)"raw=%lu surface=%s (instantaneous)\n",
                  (UW)b_high, b_black ? "BLACK" : "WHITE");
        (void)bench_wait(BENCH_POLL_MS);
    }
}

/* End of file. */

