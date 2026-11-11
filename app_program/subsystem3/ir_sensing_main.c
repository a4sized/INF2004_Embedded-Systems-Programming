/**
 * @file ir_sensing_main.c
 * @brief Three-sensor IR service and diagnostic readout.
 */
#include "ir_sensing_main.h"

#if defined(BUDDY3_SENSING_STANDALONE) || defined(BUDDY3_TEAM_IR)

#include "../subsystem1/subsystem1.h"
#include "../systemFaults.h"
#include "drv_ir.h"
#include "ir_fault_status.h"
#include "ir_barcode_capture.h"
#include "ir_barcode_gpio.h"
#include "ir_barcode_segments.h"
#include "ir_barcode_window.h"
#include "ir_code39.h"
#include "ir_line_filter.h"
#include "ir_line_snapshot_shared.h"
#include "ir_line_state.h"
#include "ir_sensing.h"
#include "ir_time.h"

#include <stdint.h>
#include <config.h>
#include <tm/tmonitor.h>

#define IR_TASK_PRIORITY   16
#define IR_TASK_STACK      2048
#define IR_SAMPLE_PERIOD_MS 5u
#define IR_PRINT_EVERY     40u
#define IR_USB_READY       3u
#define IR_LINE_STABLE_MS  10u
#define IR_GAP_MIN_US      4000u
#define IR_GAP_MAX_US      6000u
#define IR_GAP_LIMIT_US    10000u
#define IR_STATS_WINDOW    12000u
#define IR_FAULT_REASSERT_MS 250u
#if defined(BUDDY3_BARCODE_DEBUG)
#define IR_BARCODE_DEBUG_PERIOD_MS 200u
#endif

#if CNF_TIMER_PERIOD != IR_SAMPLE_PERIOD_MS
#error "Buddy 3 sensing needs a 5 ms kernel tick."
#endif

/* User-observed DO levels: black = 1, white = 0 on all three modules. */
#define IR_LEFT_BLACK_LEVEL    IR_BLACK_LEVEL_HIGH
#define IR_RIGHT_BLACK_LEVEL   IR_BLACK_LEVEL_HIGH
#define IR_BARCODE_BLACK_LEVEL IR_BLACK_LEVEL_HIGH

/* The USB console port currently has no public state header. */
extern UW tm_usb_state (void);

static void sensing_task (INT start_code, void * p_extra);
static void sensing_tick (void * p_extra);
static void print_capture (ir_barcode_window_t const * p_window);
static void halt_task (void);
#if defined(BUDDY3_BARCODE_DEBUG)
static char * barcode_debug_u32 (char * p_out, uint32_t value);
static void barcode_debug_reading (char const * p_event,
                                   drv_ir_levels_t raw, uint32_t count,
                                   uint32_t now_ms);
#endif
#if defined(BUDDY3_IR_MOVING_TEST)
static void moving_log_window (ir_barcode_window_t const * p_window,
                               char letter, char order,
                               char const * p_result);
#endif
#if defined(BUDDY3_SENSING_STANDALONE)
static char const * surface_name (ir_surface_t surface);
static char const * position_name (ir_line_position_t position);
static char const * steer_name (ir_line_steer_t steer);
#endif
static ID sensing_task_id;
static ID sensing_tick_id;
#if defined(BUDDY3_SENSING_STANDALONE)
EXPORT ID fault_flag;

EXPORT ID app_start_task (FP entry, PRI priority, UW processor,
                          char const * p_name)
{
    T_CTSK config = {0};
    ID task_id;
    ER start_status;

    (void)p_name;
    config.task = entry;
    config.itskpri = priority;
    config.stksz = TASK_STACK_SIZE;
    config.tskatr = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
    if (0u != processor)
    {
        config.tskatr |= TA_ASSPRC;
        config.assprc = processor;
    }
#else
    (void)processor;
#endif
    task_id = tk_cre_tsk(&config);
    if (task_id < E_OK)
    {
        return task_id;
    }
    start_status = tk_sta_tsk(task_id, 0);
    if (E_OK != start_status)
    {
        return start_status;
    }
    return task_id;
}

INT usermain (void)
{
    T_CFLG fault_config = {0};

    fault_config.flgatr = TA_TFIFO | TA_WMUL;
    fault_flag = tk_cre_flg(&fault_config);
    if (fault_flag < E_OK)
    {
        halt_task();
    }
    if (E_OK != subsystem1_init())
    {
        halt_task();
    }
    if (E_OK != ir_sensing_service_init())
    {
        halt_task();
    }
    if ((E_OK != subsystem1_start()) ||
        (E_OK != ir_sensing_service_start()))
    {
        halt_task();
    }

    halt_task();
    return 0;
}
#endif

ER ir_sensing_service_init (void)
{
    T_CTSK task_config = {0};
    T_CCYC tick_config = {0};
    ER status = ir_line_snapshot_shared_init();

    if (E_OK != status)
    {
        return status;
    }
    task_config.task = (FP)sensing_task;
    task_config.itskpri = IR_TASK_PRIORITY;
    task_config.stksz = IR_TASK_STACK;
    task_config.tskatr = TA_HLNG | TA_RNG0;

#if TK_SUPPORT_SMP
    task_config.tskatr |= TA_ASSPRC;
    task_config.assprc = TP_PRC1;
#endif

    sensing_task_id = tk_cre_tsk(&task_config);

    if (sensing_task_id <= E_OK)
    {
        return sensing_task_id;
    }

    tick_config.cycatr = TA_HLNG;
#if TK_SUPPORT_SMP
    tick_config.cycatr |= TA_ASSPRC;
    tick_config.assprc = TP_PRC1;
#endif
    tick_config.cychdr = (FP)sensing_tick;
    tick_config.cyctim = IR_SAMPLE_PERIOD_MS;
    tick_config.cycphs = IR_SAMPLE_PERIOD_MS;

    sensing_tick_id = tk_cre_cyc(&tick_config);

    if (sensing_tick_id <= E_OK)
    {
        return sensing_tick_id;
    }

    return E_OK;
}

ER ir_sensing_service_start (void)
{
    ER status = tk_sta_tsk(sensing_task_id, 0);
    if (E_OK != status)
    {
        return status;
    }

    status = tk_sta_cyc(sensing_tick_id);
    if (E_OK != status)
    {
        return status;
    }
    return E_OK;
}

static void sensing_task (INT start_code, void * p_extra)
{
    ir_sensing_map_t const map =
    {
        IR_LEFT_BLACK_LEVEL,
        IR_RIGHT_BLACK_LEVEL,
        IR_BARCODE_BLACK_LEVEL
    };
    uint32_t snapshot_sequence = 0u;
#if defined(BUDDY3_SENSING_STANDALONE)
    uint32_t samples = 0u;
    uint32_t previous_sample_us = 0u;
    uint32_t min_gap_us = UINT32_MAX;
    uint32_t max_gap_us = 0u;
    uint32_t intervals = 0u;
    uint32_t in_range = 0u;
    uint32_t over_limit = 0u;
    bool b_have_previous = false;
    bool b_console_was_ready = false;
#endif
    ir_line_filter_t filter = {0};
    ir_line_state_t line = {0};
    ir_fault_status_t fault_status = {0};
    ir_barcode_window_t window = {0};
    uint32_t seen_overflows = 0u;
    bool b_barcode_decoded = false;
    uint32_t barcode_decode_sequence = 0u;
    char last_decoded_letter = '\0';
#if defined(BUDDY3_BARCODE_DEBUG)
    uint32_t last_barcode_debug_ms = 0u;
#endif

    (void)start_code;
    (void)p_extra;

    ir_fault_status_reset(&fault_status);
    bool const b_ir_ready = drv_ir_init();
    ir_fault_status_note_gpio_init(&fault_status, b_ir_ready);
    if (!b_ir_ready)
    {
        ir_line_snapshot_t const unavailable = {0};

        (void)ir_line_snapshot_shared_publish(&unavailable);
        s1_report_fault(FAULT_IR_SENS);
        s1_csv_log_event("IR_SENSOR_FAULT", "GPIO_INIT_FAILED");
        for (;;)
        {
            (void)tk_dly_tsk(IR_FAULT_REASSERT_MS);
            s1_report_fault(FAULT_IR_SENS);
        }
    }

    /* GP6 transitions are captured by this task without claiming GPIO IRQ 13. */
    ir_barcode_gpio_start();
    ir_barcode_window_reset(&window);

    ir_line_filter_reset(&filter);
    ir_line_state_reset(&line);

    for (;;)
    {
        if (E_OK != tk_slp_tsk(TMO_FEVR))
        {
            halt_task();
        }

        uint32_t const sample_us = ir_time_us();
        ir_fault_status_note_sample(&fault_status, sample_us);
        drv_ir_levels_t const raw = drv_ir_read();
        ir_barcode_gpio_poll(raw.b_barcode_high, sample_us);
        SYSTIM now = {0};
        ir_barcode_edge_t edge = {0};
        char decoded_letter = '\0';
        ir_code39_direction_t scan_direction = IR_CODE39_FORWARD;
        bool b_complete_frame = false;

        if (ir_barcode_gpio_overflows() != seen_overflows)
        {
            seen_overflows = ir_barcode_gpio_overflows();
            ir_barcode_window_invalidate(&window);
        }

        while (ir_barcode_gpio_pop(&edge))
        {
            if ((IR_BARCODE_COMPLETE_EDGES == window.count) ||
                ir_barcode_window_expired(&window, edge.time_us))
            {
                print_capture(&window);
                ir_barcode_window_reset(&window);
            }
            ir_barcode_window_feed(&window, edge);
#if defined(BUDDY3_BARCODE_DEBUG)
            drv_ir_levels_t edge_levels = raw;
            edge_levels.b_barcode_high = edge.b_high;
            barcode_debug_reading("IR_BARCODE_EDGE", edge_levels, window.count,
                                  edge.time_us / 1000u);
#endif
            if (10u == window.count)
            {
                if (!ir_code39_start_marker_seen(&window))
                {
                    print_capture(&window);
                    ir_barcode_window_reset(&window);
                }
            }
            else if (IR_BARCODE_COMPLETE_EDGES == window.count)
            {
                if (!ir_code39_frame_single_with_direction(
                        &window, &decoded_letter, &scan_direction))
                {
                    print_capture(&window);
                    ir_barcode_window_reset(&window);
                }
                else
                {
                    /* Both * markers and the single letter are validated.
                     * Commit before a nearby junction black bar can arrive;
                     * waiting for a long trailing quiet gap lost valid Z
                     * frames in the moving car trace.
                     */
                    b_complete_frame = true;
                    break;
                }
            }
        }

        if (ir_barcode_gpio_overflows() != seen_overflows)
        {
            seen_overflows = ir_barcode_gpio_overflows();
            ir_barcode_window_invalidate(&window);
        }

        if ((b_complete_frame && !window.b_invalid) ||
            ir_code39_decode_single_with_direction(
                &window, ir_time_us(), &decoded_letter, &scan_direction))
        {
            b_barcode_decoded = true;
            ++barcode_decode_sequence;
            last_decoded_letter = decoded_letter;
            s1_set_barcode_id((UB)decoded_letter);
            if (IR_USB_READY == tm_usb_state())
            {
                char details[] = "letter=?;order=?";
                details[7] = decoded_letter;
                details[15] = (IR_CODE39_FORWARD == scan_direction) ?
                              'F' : 'R';
                s1_csv_log_event("BARCODE_DECODED", details);
            }
#if defined(BUDDY3_IR_MOVING_TEST)
            moving_log_window(&window, decoded_letter,
                              (IR_CODE39_FORWARD == scan_direction) ?
                                  'F' : 'R', "DECODED");
#endif
#if defined(BUDDY3_SENSING_STANDALONE)
            if (IR_USB_READY == tm_usb_state())
            {
                ir_barcode_action_t const action =
                    ir_barcode_action_for(decoded_letter);
                char const * p_action =
                    (IR_BARCODE_TURN_LEFT == action) ? "LEFT" :
                    (IR_BARCODE_TURN_RIGHT == action) ? "RIGHT" : "UNMAPPED";
                tm_printf((UB const *)
                          "Code 39 decoded: %c pattern_order=%s action=%s\n",
                          decoded_letter,
                          (IR_CODE39_FORWARD == scan_direction) ?
                              "FORWARD" : "REVERSE",
                          p_action);
                print_capture(&window);
            }
#endif
            ir_barcode_window_reset(&window);
        }

        if (ir_barcode_window_expired(&window, ir_time_us()))
        {
            print_capture(&window);
            ir_barcode_window_reset(&window);
        }

#if defined(BUDDY3_SENSING_STANDALONE)
        if (b_have_previous)
        {
            uint32_t const gap_us = sample_us - previous_sample_us;

            min_gap_us = (gap_us < min_gap_us) ? gap_us : min_gap_us;
            max_gap_us = (gap_us > max_gap_us) ? gap_us : max_gap_us;
            ++intervals;
            if ((gap_us >= IR_GAP_MIN_US) && (gap_us <= IR_GAP_MAX_US))
            {
                ++in_range;
            }
            if (gap_us > IR_GAP_LIMIT_US)
            {
                ++over_limit;
            }
        }
        previous_sample_us = sample_us;
        b_have_previous = true;
#endif

        if (E_OK != tk_get_tim(&now))
        {
            halt_task();
        }

        drv_ir_levels_t const stable = ir_line_filter_update(
            &filter, raw, now.lo, IR_LINE_STABLE_MS);
        ir_sensing_result_t const result = ir_sensing_read(stable, &map);
        ir_line_state_update(&line, stable, now.lo);
        ir_line_snapshot_t const snapshot =
        {
            .sequence = ++snapshot_sequence,
            .sampled_at_ms = now.lo,
            .raw = raw,
            .stable = stable,
            .left_surface = result.left,
            .right_surface = result.right,
            .barcode_surface = result.barcode,
            .position = line.position,
            .steer = line.latest_steer,
            .b_lost = line.b_lost,
            .b_barcode_capturing = (0u != window.count),
            .b_barcode_decoded = b_barcode_decoded,
            .barcode_decode_sequence = barcode_decode_sequence,
            .decoded_letter = last_decoded_letter,
            .b_gpio_ready = fault_status.b_gpio_configured,
            /* Mounted readings were accepted in the user-reported car trial. */
            .b_ready = b_ir_ready
        };

        if (!ir_line_snapshot_shared_publish(&snapshot))
        {
            s1_report_fault(FAULT_IR_SENS);
            halt_task();
        }
#if defined(BUDDY3_BARCODE_DEBUG)
        if ((uint32_t)(now.lo - last_barcode_debug_ms) >=
            IR_BARCODE_DEBUG_PERIOD_MS)
        {
            barcode_debug_reading("IR_BARCODE_SAMPLE", raw, window.count,
                                  now.lo);
            last_barcode_debug_ms = now.lo;
        }
#endif
#if defined(BUDDY3_SENSING_STANDALONE)
        bool const b_console_ready = (IR_USB_READY == tm_usb_state());

        if (b_console_ready && !b_console_was_ready)
        {
            tm_printf((UB const *)"Buddy 3 IR sensing: GP7 left, GP2 right,"
                      " GP6 barcode.\n"
                      "Black=1, white=0 for all three sensors."
                      " Check raw levels if lighting or height changes.\n"
                      "L/R show raw>stable; B remains raw."
                      " Timing range is 4000..6000 us.\n"
                      "Line decisions are diagnostic; no motion request"
                      " is sent. GP6 edge windows last 2 s.\n");
        }

        if (b_console_ready &&
            ((0u == (samples % IR_PRINT_EVERY)) || !b_console_was_ready))
        {
            tm_printf((UB const *)
                      "t=%u ms L=%u>%u/%s R=%u>%u/%s B=%u/%s"
                      " pos=%s steer=%s lost=%u"
                      " gap=%u..%u us ok=%u/%u over10ms=%u"
                      " ir_init_fault=%u late_total=%u\n",
                      now.lo,
                      (UW)raw.b_left_high,
                      (UW)stable.b_left_high,
                      surface_name(result.left),
                      (UW)raw.b_right_high,
                      (UW)stable.b_right_high,
                      surface_name(result.right),
                      (UW)raw.b_barcode_high,
                      surface_name(result.barcode),
                      position_name(line.position),
                      steer_name(line.latest_steer),
                      (UW)line.b_lost,
                      min_gap_us,
                      max_gap_us,
                      in_range,
                      intervals,
                      over_limit,
                      (UW)fault_status.b_gpio_init_failed,
                      fault_status.gaps_over_limit);
        }

        b_console_was_ready = b_console_ready;
        ++samples;

        if (intervals >= IR_STATS_WINDOW)
        {
            intervals = 0u;
            in_range = 0u;
            over_limit = 0u;
            min_gap_us = UINT32_MAX;
            max_gap_us = 0u;
        }
#endif

        if (tk_can_wup(TSK_SELF) < E_OK)
        {
            halt_task();
        }
    }
}

static void sensing_tick (void * p_extra)
{
    (void)p_extra;
    (void)tk_wup_tsk(sensing_task_id);
}

static void print_capture (ir_barcode_window_t const * p_window)
{
#if defined(BUDDY3_SENSING_STANDALONE)
    ir_barcode_segments_t segments = {0};
    bool const b_candidate = ir_barcode_segments_make(p_window, &segments);

    if (IR_USB_READY != tm_usb_state())
    {
        return;
    }

    tm_printf((UB const *)"Barcode edge window: count=%u invalid=%u"
              " overflows=%u segments=%s\n",
              p_window->count, (UW)p_window->b_invalid,
              ir_barcode_gpio_overflows(),
              b_candidate ? "PROVISIONAL" : "NONE");
    if (p_window->b_invalid)
    {
        return;
    }

    for (uint32_t index = 0u; index < p_window->count; ++index)
    {
        uint32_t const duration_us = (index + 1u < p_window->count) ?
            (p_window->edges[index + 1u].time_us -
             p_window->edges[index].time_us) : 0u;
        uint32_t const fraction_ppm =
            (b_candidate && (index < segments.count)) ?
            segments.fractions[index] : 0u;
        tm_printf((UB const *)"  edge=%u t=%u us level=%u"
                  " duration=%u us fraction=%u ppm\n",
                  index, p_window->edges[index].time_us,
                  (UW)p_window->edges[index].b_high,
                  duration_us, fraction_ppm);
    }
#elif defined(BUDDY3_IR_MOVING_TEST)
    moving_log_window(p_window, '-', '-',
                      p_window->b_invalid ? "INVALID" :
                      ((IR_BARCODE_COMPLETE_EDGES == p_window->count) ?
                           "REJECTED" : "INCOMPLETE"));
#elif defined(BUDDY3_BARCODE_DEBUG)
    if ((0u != p_window->count) && (IR_USB_READY == tm_usb_state()))
    {
        char details[80];
        char * p_out = details;
        char const * p_prefix = "edges=";
        while ('\0' != *p_prefix)
        {
            *p_out++ = *p_prefix++;
        }
        p_out = barcode_debug_u32(p_out, p_window->count);
        p_prefix = ";invalid=";
        while ('\0' != *p_prefix)
        {
            *p_out++ = *p_prefix++;
        }
        *p_out++ = p_window->b_invalid ? '1' : '0';
        p_prefix = ";overflow=";
        while ('\0' != *p_prefix)
        {
            *p_out++ = *p_prefix++;
        }
        p_out = barcode_debug_u32(p_out, ir_barcode_gpio_overflows());
        p_prefix = ";span_ms=";
        while ('\0' != *p_prefix)
        {
            *p_out++ = *p_prefix++;
        }
        p_out = barcode_debug_u32(
            p_out, (p_window->edges[p_window->count - 1u].time_us -
                    p_window->edges[0].time_us) / 1000u);
        *p_out = '\0';
        s1_csv_log_event("IR_BARCODE_WINDOW", details);
    }
#else
    (void)p_window;
#endif
}

#if defined(BUDDY3_BARCODE_DEBUG)
static char * barcode_debug_u32 (char * p_out, uint32_t value)
{
    char digits[10];
    uint32_t count = 0u;
    do
    {
        digits[count++] = (char)('0' + (char)(value % 10u));
        value /= 10u;
    } while (0u != value);
    while (0u != count)
    {
        *p_out++ = digits[--count];
    }
    return p_out;
}

static void barcode_debug_reading (char const * p_event,
                                   drv_ir_levels_t raw, uint32_t count,
                                   uint32_t now_ms)
{
    if (IR_USB_READY == tm_usb_state())
    {
        char details[64];
        char * p_out = details;
        *p_out++ = 'B';
        *p_out++ = '=';
        *p_out++ = raw.b_barcode_high ? '1' : '0';
        *p_out++ = ';';
        *p_out++ = 'L';
        *p_out++ = '=';
        *p_out++ = raw.b_left_high ? '1' : '0';
        *p_out++ = ';';
        *p_out++ = 'R';
        *p_out++ = '=';
        *p_out++ = raw.b_right_high ? '1' : '0';
        *p_out++ = ';';
        *p_out++ = 'n';
        *p_out++ = '=';
        p_out = barcode_debug_u32(p_out, count);
        *p_out++ = ';';
        *p_out++ = 't';
        *p_out++ = '=';
        p_out = barcode_debug_u32(p_out, now_ms);
        *p_out = '\0';
        s1_csv_log_event(p_event, details);
    }
}
#endif

#if defined(BUDDY3_IR_MOVING_TEST)
static char * moving_append_text (char * p_out, char const * p_text)
{
    while ('\0' != *p_text)
    {
        *p_out++ = *p_text++;
    }
    return p_out;
}

static char * moving_append_u32 (char * p_out, uint32_t value)
{
    char digits[10];
    uint32_t count = 0u;
    do
    {
        digits[count++] = (char)('0' + (char)(value % 10u));
        value /= 10u;
    } while (0u != value);
    while (0u != count)
    {
        *p_out++ = digits[--count];
    }
    return p_out;
}

static void moving_log_window (ir_barcode_window_t const * p_window,
                               char letter, char order,
                               char const * p_result)
{
    if ((IR_USB_READY != tm_usb_state()) || (0u == p_window->count))
    {
        return;
    }
    char details[96];
    char * p_out = moving_append_text(details, "count=");
    p_out = moving_append_u32(p_out, p_window->count);
    p_out = moving_append_text(p_out, ";duration_ms=");
    p_out = moving_append_u32(
        p_out, (p_window->edges[p_window->count - 1u].time_us -
                p_window->start_us) / 1000u);
    p_out = moving_append_text(p_out, ";overflow=");
    p_out = moving_append_u32(p_out, ir_barcode_gpio_overflows());
    p_out = moving_append_text(p_out, ";letter=");
    *p_out++ = letter;
    p_out = moving_append_text(p_out, ";order=");
    *p_out++ = order;
    p_out = moving_append_text(p_out, ";result=");
    p_out = moving_append_text(p_out, p_result);
    *p_out = '\0';
    s1_csv_log_event("IR_CAPTURE", details);
}
#endif

static void halt_task (void)
{
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

#if defined(BUDDY3_SENSING_STANDALONE)
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

static char const * position_name (ir_line_position_t position)
{
    char const * p_name = "WHITE_WHITE";

    if (IR_LINE_LEFT_BLACK == position)
    {
        p_name = "LEFT_BLACK";
    }
    else if (IR_LINE_RIGHT_BLACK == position)
    {
        p_name = "RIGHT_BLACK";
    }
    else if (IR_LINE_CENTERED == position)
    {
        p_name = "CENTERED";
    }

    return p_name;
}

static char const * steer_name (ir_line_steer_t steer)
{
    char const * p_name = "NONE";

    if (IR_LINE_STEER_LEFT == steer)
    {
        p_name = "LEFT";
    }
    else if (IR_LINE_STEER_RIGHT == steer)
    {
        p_name = "RIGHT";
    }

    return p_name;
}
#endif

#endif

/* End of file. */
