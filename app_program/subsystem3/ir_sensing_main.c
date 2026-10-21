/**
 * @file ir_sensing_main.c
 * @brief USB readout for three digital sensors, without motion or network.
 */
#include "ir_sensing_main.h"

#if defined(BUDDY3_SENSING_STANDALONE)

#include "drv_ir.h"
#include "ir_line_filter.h"
#include "ir_sensing.h"

#include <stdint.h>
#include <tm/tmonitor.h>

#define IR_TASK_PRIORITY   16
#define IR_TASK_STACK      2048
#define IR_SAMPLE_DELAY_MS 10u
#define IR_PRINT_EVERY     20u
#define IR_USB_READY       3u
#define IR_LINE_STABLE_MS  10u

/* Set each value after measuring DO over black and white on the robot. */
#define IR_LEFT_BLACK_LEVEL    IR_BLACK_LEVEL_UNKNOWN
#define IR_RIGHT_BLACK_LEVEL   IR_BLACK_LEVEL_UNKNOWN
#define IR_BARCODE_BLACK_LEVEL IR_BLACK_LEVEL_UNKNOWN

/* The USB console port currently has no public state header. */
extern UW tm_usb_state (void);

static void sensing_task (INT start_code, void * p_extra);
static void halt_task (void);
static char const * surface_name (ir_surface_t surface);

INT usermain (void)
{
    T_CTSK task_config = {0};

    task_config.task = (FP)sensing_task;
    task_config.itskpri = IR_TASK_PRIORITY;
    task_config.stksz = IR_TASK_STACK;
    task_config.tskatr = TA_HLNG | TA_RNG0;

#if TK_SUPPORT_SMP
    task_config.tskatr |= TA_ASSPRC;
    task_config.assprc = TP_PRC1;
#endif

    ID const task_id = tk_cre_tsk(&task_config);

    if (task_id <= E_OK)
    {
        halt_task();
    }

    if (E_OK != tk_sta_tsk(task_id, 0))
    {
        halt_task();
    }

    halt_task();
    return 0;
}

static void sensing_task (INT start_code, void * p_extra)
{
    ir_sensing_map_t const map =
    {
        IR_LEFT_BLACK_LEVEL,
        IR_RIGHT_BLACK_LEVEL,
        IR_BARCODE_BLACK_LEVEL
    };
    uint32_t samples = 0u;
    bool b_console_was_ready = false;
    ir_line_filter_t filter = {0};

    (void)start_code;
    (void)p_extra;

    if (!drv_ir_init())
    {
        halt_task();
    }

    ir_line_filter_reset(&filter);

    for (;;)
    {
        SYSTIM now = {0};

        if (E_OK != tk_get_tim(&now))
        {
            halt_task();
        }

        drv_ir_levels_t const raw = drv_ir_read();
        drv_ir_levels_t const stable = ir_line_filter_update(
            &filter, raw, now.lo, IR_LINE_STABLE_MS);
        ir_sensing_result_t const result = ir_sensing_read(stable, &map);
        bool const b_console_ready = (IR_USB_READY == tm_usb_state());

        if (b_console_ready && !b_console_was_ready)
        {
            tm_printf((UB const *)"Buddy 3 IR sensing: GP7 left, GP4 right,"
                      " GP16 barcode.\n"
                      "Move each sensor over black, then white; note raw 0/1."
                      " ? means polarity has not been measured.\n"
                      "L/R show raw>stable; B remains raw.\n");
        }

        if (b_console_ready &&
            ((0u == (samples % IR_PRINT_EVERY)) || !b_console_was_ready))
        {
            tm_printf((UB const *)
                      "t=%u ms L=%u>%u/%s R=%u>%u/%s B=%u/%s\n",
                      now.lo,
                      (UW)raw.b_left_high,
                      (UW)stable.b_left_high,
                      surface_name(result.left),
                      (UW)raw.b_right_high,
                      (UW)stable.b_right_high,
                      surface_name(result.right),
                      (UW)raw.b_barcode_high,
                      surface_name(result.barcode));
        }

        b_console_was_ready = b_console_ready;
        ++samples;

        if (E_OK != tk_dly_tsk(IR_SAMPLE_DELAY_MS))
        {
            halt_task();
        }
    }
}

static void halt_task (void)
{
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

static char const * surface_name (ir_surface_t surface)
{
    char const * p_name = "?";

    if (IR_SURFACE_BLACK == surface)
    {
        p_name = "BLACK";
    }
    else if (IR_SURFACE_WHITE == surface)
    {
        p_name = "WHITE";
    }

    return p_name;
}

#endif

/* End of file. */
