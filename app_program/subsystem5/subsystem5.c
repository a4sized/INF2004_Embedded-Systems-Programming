/**
 * @file    subsystem5.c
 * @brief   Subsystem 5: servo-mounted ultrasonic obstacle scanner.
 *
 * This file is the scanning task and Subsystem 5's public interface. The
 * work is split over:
 *   s5_sense.c    readings and servo moves
 *   s5_guard.c    speed cap for Subsystem 2: slow down / stop
 *   s5_scan.c     fine scan of a stopped-for obstacle (stages 2 to 4)
 *   s5_profile.c  obstacle profile from the fine scan
 *   s5_avoid.c    bypass decision from the profile
 *   s5_bypass.c   driving round it and back to the line (stages 4 and 5)
 *   s5_drive.c    encoder-measured moves;  s5_edge.c  where it ends
 *   s5_report.c   telemetry and sensor faults to Subsystem 1
 *
 * Start-up (only once, when the car powers up)
 *   The car is held still (speed cap 0) while the servo turns to face the
 *   front of the car (90 degrees), then the hold is released. Subsystem 5
 *   is ready (subsystem5_startup_scan_ready) once a good straight-ahead
 *   reading has also been taken, with no sensor fault and the path ahead
 *   clear.
 *
 * Normal running (stage 1)
 *   The servo faces ahead and the sensor is pinged every
 *   S5_PING_INTERVAL_MS; the guard slows the car near an obstacle and stops
 *   it S5_STOP_DISTANCE_CM away. Readings further than S5_DETECT_RANGE_CM
 *   count as "no obstacle". A reading is printed every S5_FORWARD_PRINT_MS.
 *
 * Each time the car stops for an obstacle
 *   Subsystem 5 reports "avoiding" (subsystem5_is_avoiding); the vehicle
 *   controller pauses the line follower and gives it the drive permission
 *   (subsystem5_set_drive_permission). The car fine scans the obstacle from
 *   where it stopped and decides (reversing and scanning again only while
 *   no edge was found, REVERSE). Then (S5_DRIVE_BYPASS):
 *     LEFT / RIGHT  reverse only as far as needed for room to swing out,
 *                   go round it and back to the line, and hand back,
 *                   saying which side the line is on (the line follower
 *                   searches toward it if the IR sensors are not over
 *                   it, subsystem5_take_line_side);
 *     CONTINUE      the obstacle has gone: hand back;
 *     STOP          too big to get round, or the scan failed: wait
 *                   (held, below).
 *   If going round shows the obstacle is too big after all (too wide or too
 *   long), the car retraces its moves back to the line and waits there.
 *   Losing the permission stops any move at once.
 *
 * Held (could not get past)
 *   The car is held still (speed cap 0) and S5 keeps reporting "avoiding",
 *   so the line follower stays paused. On the line it waits for the
 *   obstacle to be removed: the hold ends once the path ahead reads clear
 *   S5_HOLD_CLEAR_READINGS times in a row. Only if a bypass stopped
 *   part-way (off the line: blocked, a wheel or encoder fault) does it wait
 *   for the operator. Either way, withdrawing the drive permission
 *   (operator STOP or a fault) also ends it.
 *
 * Clearance test (S5_CLEARANCE_TEST = 1)
 *   The car stops in front of the first obstacle and fine scans it; only
 *   the fine scan (each angle, the width and edges) and the decision are
 *   printed. The car is then held still and nothing more is driven or
 *   handled until reset.
 *
 * Casts in this file: (UB *) only retypes a string literal for the T-Kernel
 * tm_printf() API; (unsigned int) and (int) widen values that always fit;
 * (void) discards a return value that is not needed; (FP) and (ER) are
 * explained where they are made.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tk/smp.h>
#include "../app.h"
#include "subsystem5.h"
#include "s5_config.h"
#include "s5_log.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "s5_report.h"
#include "s5_guard.h"
#include "s5_profile.h"
#include "s5_avoid.h"
#include "s5_drive.h"
#include "s5_sense.h"
#include "s5_scan.h"
#include "s5_bypass.h"

_Static_assert((S5_STOP_TRIGGER_CM < S5_DETECT_RANGE_CM) &&
               (S5_SLOW_DISTANCE_CM <= S5_DETECT_RANGE_CM) &&
               (S5_DETECT_RANGE_CM <= S5_MAX_RANGE_CM),
               "slow distance <= detect range <= sensor range");

/* Minimum-bit scanner state: 5 bits. b_hardware_ready is set once in init
 * (before the scanning task exists); the others are used only by the
 * scanning task, so they never race. */
typedef struct
{
    uint8_t b_hardware_ready : 1;   /* 1 = servo and sensor both started    */
    uint8_t b_profile_done   : 1;   /* 1 = current obstacle already handled */
    uint8_t b_test_done      : 1;   /* 1 = clearance test over: car held    */
    uint8_t b_held           : 1;   /* 1 = could not get past: car held     */
    uint8_t b_held_off_line  : 1;   /* 1 = held part-way round: only the
                                     *     operator releases it             */
} scanner_state_t;

static scanner_state_t g_scanner = { 0u, 0u, 0u, 0u, 0u };

/* While held: good straight-ahead readings in a row with the path clear.
 * Scanning task only. */
static uint8_t g_hold_clear_count = 0u;

/* Read by other tasks, written only by the scanning task (and once in
 * init, before it exists); atomic loads and stores.
 *   gb_facing_front    start-up done: the servo faces the front of the car
 *   gb_avoiding        handling an obstacle (or held): line follower paused
 *   g_line_side        which side of the car the line is on after a bypass,
 *                      waiting for the vehicle controller (LINE_SIDE_*);
 *                      swapped whole with the kernel's atomic_xchg(), which
 *                      works across both cores (the Cortex-M0+ has no
 *                      atomic read-modify-write of its own) */
#define LINE_SIDE_NONE      (0u)   /* nothing waiting */
#define LINE_SIDE_LEFT      (1u)
#define LINE_SIDE_RIGHT     (2u)

static bool gb_facing_front = false;
static bool gb_avoiding     = false;
/* A good straight-ahead reading has been taken since start-up (part of
 * subsystem5_startup_scan_ready). Written by the scanning task only. */
static bool gb_forward_reading_seen = false;
static UW   g_line_side     = LINE_SIDE_NONE;   /* 32-bit aligned (smp.h) */

/* ------------------------------------------------------------------------
 * Held: could not get past
 * ------------------------------------------------------------------------ */

/*
 * Hold the car still and keep "avoiding", so the vehicle controller does
 * NOT hand it back to the line follower (reversing to scan again may have
 * opened the gap past the guard's resume distance, so the line follower
 * would otherwise drive straight back at the obstacle). b_off_line = the
 * car is part-way round: only the operator releases it.
 */
static void s5_enter_held (const bool b_off_line, const char * const p_why)
{
    s5_drive_stop();
    s5_guard_set_hold(true);
    g_scanner.b_held          = 1u;
    g_scanner.b_held_off_line = b_off_line ? 1u : 0u;
    g_hold_clear_count        = 0u;
    S5_LOG((UB *)"S5 held: %s; %s\n", p_why, b_off_line ?
           "part-way round, send STOP to take over" :
           "waiting on the line for the obstacle to be removed");
    s5_log_event("S5_HELD", b_off_line ? "OFF_LINE" : "ON_LINE");
}

/* Release the hold and hand the car back to the vehicle controller. */
static void s5_leave_held (const char * const p_why)
{
    g_scanner.b_held          = 0u;
    g_scanner.b_held_off_line = 0u;
    g_hold_clear_count        = 0u;
    s5_guard_set_hold(false);
    __atomic_store_n(&gb_avoiding, false, __ATOMIC_RELEASE);
    S5_LOG((UB *)"S5 held: released (%s), handing back\n", p_why);
    s5_log_event("S5_HELD", "RELEASED");
}

/*
 * One pass while held, every S5_PING_INTERVAL_MS: keep the sensor facing
 * ahead and pinging (sensor health and telemetry carry on; the speed cap
 * stays 0), and release the hold when the operator takes over or, on the
 * line, when the obstacle has been removed.
 */
static void s5_watch_held (void)
{
    uint16_t distance_mm = NO_OBSTACLE;

    if (S5_FORWARD_DEG != drv_servo_get_angle())
    {
        s5_sense_servo_to(S5_FORWARD_DEG, S5_LOOK_AHEAD);
    }

    /* A failed or busy reading says nothing: it neither counts nor resets. */
    if (s5_sense_reading(S5_LOOK_AHEAD, false, &distance_mm))
    {
        const bool b_clear = ((NO_OBSTACLE == distance_mm) ||
                              (distance_mm >
                               (S5_STILL_BLOCKED_CM * S5_MM_PER_CM)));

        if (!b_clear)
        {
            g_hold_clear_count = 0u;
        }
        else if (g_hold_clear_count < S5_HOLD_CLEAR_READINGS)
        {
            g_hold_clear_count++;
        }
        else
        {
            /* Saturated: enough clear readings already. */
        }
    }

    if (!s5_bypass_permitted())
    {
        /* The vehicle controller left AVOIDING (operator STOP or a fault):
         * the car is no longer ours to hold. */
        s5_leave_held("drive permission withdrawn");
    }
    else if ((0u == g_scanner.b_held_off_line) &&
             (g_hold_clear_count >= S5_HOLD_CLEAR_READINGS))
    {
        s5_leave_held("obstacle removed");
    }
    else
    {
        /* Still held. */
    }
}

/* ------------------------------------------------------------------------
 * Handling one obstacle
 * ------------------------------------------------------------------------ */

/*
 * Carry the decision out (S5_DRIVE_BYPASS, with the drive permission):
 * reverse and scan again while it is REVERSE, then go round or wait.
 * Returns the reason to hold the car, or NULL to hand it back; *p_off_line
 * is set if the car is held part-way round.
 */
static const char * s5_carry_out (s5_bypass_t decision,
                                  s5_profile_t * const p_profile,
                                  bool * const p_off_line)
{
    const char * p_hold     = NULL;
    bool         b_reversed = false;

    while ((S5_BYPASS_REVERSE == decision) && s5_bypass_permitted())
    {
        S5_LOG((UB *)"S5 reverse: backing %u mm to scan again\n",
               (unsigned int)S5_REVERSE_MM);
        (void)s5_bypass_back(S5_REVERSE_MM);
        b_reversed = true;
        decision   = s5_scan_obstacle(p_profile);
    }

    if (!s5_bypass_permitted())
    {
        /* Not following the line (or stopped by the operator): decide
         * only, never move the car on our own. */
        S5_LOG((UB *)"S5 avoiding: no drive permission, decision only\n");
    }
    else if ((S5_BYPASS_LEFT == decision) || (S5_BYPASS_RIGHT == decision))
    {
        const bool          b_left    = (S5_BYPASS_LEFT == decision);
        const s5_go_round_t result    = s5_bypass_go_round(
            decision, s5_avoid_plan_extent(p_profile, b_left),
            s5_avoid_plan_backup_mm(p_profile, b_left));

        if (S5_GO_ROUND_TOO_BIG == result)
        {
            p_hold = "obstacle too big to get round (found going round it)";
        }
        else if (S5_GO_ROUND_FAILED == result)
        {
            p_hold      = "bypass stopped part-way";
            *p_off_line = true;
        }
        else
        {
            /* Round it and back at the line. The car went round on
             * b_left, so the line is on the other side: hand that on, so
             * the line follower searches toward it if the IR sensors are
             * not over it. Published before gb_avoiding is cleared, so the
             * vehicle controller sees it when it takes the car back. */
            (void)atomic_xchg(&g_line_side, b_left ? LINE_SIDE_RIGHT :
                                                     LINE_SIDE_LEFT);
        }
    }
    else if (S5_BYPASS_CONTINUE == decision)
    {
        if (b_reversed)
        {
            S5_LOG((UB *)"S5 drive: path clear after reversing\n");
            s5_drive_resume();
        }
    }
    else
    {
        p_hold = "no way round the obstacle (too big, or not measured)";
    }

    return p_hold;
}

/*
 * The car has stopped for an obstacle: fine scan it from where it stopped,
 * decide and (S5_DRIVE_BYPASS) carry the decision out. Then hand the car
 * back, or hold it if it could not get past.
 */
static void s5_handle_obstacle (void)
{
    s5_profile_t profile    = { 0u };
    s5_bypass_t  decision;
    const char * p_hold     = NULL;
    bool         b_off_line = false;

    /* Tell the vehicle controller first, so it pauses the line follower
     * and grants the drive permission while the fine scan runs. A line
     * side left over from an earlier bypass no longer applies. */
    (void)atomic_xchg(&g_line_side, LINE_SIDE_NONE);
    __atomic_store_n(&gb_avoiding, true, __ATOMIC_RELEASE);
    S5_LOG((UB *)"S5 avoiding: obstacle ahead, taking over\n");

    decision = s5_scan_obstacle(&profile);

    if (0u != S5_CLEARANCE_TEST)
    {
        /* Clearance and decision have been printed: hold the car still
         * for good (nothing else sets the speed cap after this). */
        s5_guard_set_hold(true);
        g_scanner.b_test_done = 1u;
    }
    else if (0u != S5_DRIVE_BYPASS)
    {
        p_hold = s5_carry_out(decision, &profile, &b_off_line);
    }
    else
    {
        /* Decide only. */
    }

    /* The scan and the bypass both end with the servo facing ahead, so the
     * line follower always starts with the sensor watching the path. */
    if (0u != g_scanner.b_test_done)
    {
        /* Clearance test over: stays avoiding for good. */
    }
    else if ((NULL != p_hold) && s5_bypass_permitted())
    {
        s5_enter_held(b_off_line, p_hold);
    }
    else
    {
        /* The vehicle controller hands the car back to the line follower
         * (while it is still stopped for the obstacle it stays avoiding,
         * see subsystem5_is_avoiding). */
        __atomic_store_n(&gb_avoiding, false, __ATOMIC_RELEASE);
        S5_LOG((UB *)"S5 avoiding: done, handing back\n");
    }
}

/*
 * One pass of normal running, every S5_PING_INTERVAL_MS: ping straight
 * ahead, print a reading now and then, and handle each obstacle the car
 * stops for. *p_since_print_ms counts the time since the last print.
 */
static void s5_watch_ahead (uint32_t * const p_since_print_ms)
{
    *p_since_print_ms += S5_PING_INTERVAL_MS;

    bool b_print = (*p_since_print_ms >= S5_FORWARD_PRINT_MS);

    if (b_print)
    {
        *p_since_print_ms = 0u;
    }

    /* The guard ignores readings not taken at exactly 90 degrees: put the
     * sensor back if anything left it turned. */
    if (S5_FORWARD_DEG != drv_servo_get_angle())
    {
        S5_LOG((UB *)"S5 servo at %u deg, turning back to %u\n",
               (unsigned int)drv_servo_get_angle(),
               (unsigned int)S5_FORWARD_DEG);
        s5_sense_servo_to(S5_FORWARD_DEG, S5_LOOK_AHEAD);
    }
    if (s5_sense_reading(S5_LOOK_AHEAD, b_print, NULL))
    {
        __atomic_store_n(&gb_forward_reading_seen, true, __ATOMIC_RELEASE);
    }

    if (!s5_guard_is_stopped())
    {
        if (0u != g_scanner.b_profile_done)
        {
            s5_avoid_new_obstacle();   /* fresh reverse count */
        }
        g_scanner.b_profile_done = 0u;   /* re-arm for the next one */
    }
    else if (0u == g_scanner.b_profile_done)
    {
        s5_handle_obstacle();
        g_scanner.b_profile_done = 1u;
    }
    else
    {
        /* Still stopped for an obstacle that is already handled. */
    }
}

/*
 * The scanning task: face the front of the car (the car is held still
 * until then), release the hold, then forever watch the path ahead (or,
 * while held, wait to be released).
 */
static void s5_sensor_task (INT start_code, void * p_extra_info)
{
    uint32_t since_print_ms = 0u;

    (void)start_code;     /* not used */
    (void)p_extra_info;   /* not used */

    s5_sense_servo_to(S5_FORWARD_DEG, S5_LOOK_AHEAD);
    s5_guard_set_hold(false);
    __atomic_store_n(&gb_facing_front, true, __ATOMIC_RELEASE);
    S5_LOG((UB *)"S5 start-up: sensor facing front, ready\n");
    s5_log_event("S5_STARTUP", "READY");

    for (;;)
    {
        (void)tk_dly_tsk(S5_PING_INTERVAL_MS);

        if (0u != g_scanner.b_test_done)
        {
            /* Clearance test over: idle, the car stays held. */
        }
        else if (0u != g_scanner.b_held)
        {
            s5_watch_held();
        }
        else
        {
            s5_watch_ahead(&since_print_ms);
        }
    }
}

/* ------------------------------------------------------------------------
 * Public interface (subsystem5.h)
 * ------------------------------------------------------------------------ */

EXPORT bool subsystem5_startup_scan_ready (void)
{
    return (__atomic_load_n(&gb_facing_front, __ATOMIC_ACQUIRE) &&
            __atomic_load_n(&gb_forward_reading_seen, __ATOMIC_ACQUIRE) &&
            (!s5_report_is_faulted()) && (!s5_guard_is_stopped()));
}

EXPORT bool subsystem5_is_avoiding (void)
{
    return (__atomic_load_n(&gb_avoiding, __ATOMIC_ACQUIRE) ||
            (__atomic_load_n(&gb_facing_front, __ATOMIC_ACQUIRE) &&
             s5_guard_is_stopped()));
}

EXPORT bool subsystem5_take_line_side (bool * const p_b_line_on_left)
{
    /* Take it and clear it in one step, so it is used once. */
    const UW   side      = atomic_xchg(&g_line_side, LINE_SIDE_NONE);
    const bool b_pending = (LINE_SIDE_NONE != side);

    if (b_pending && (NULL != p_b_line_on_left))
    {
        *p_b_line_on_left = (LINE_SIDE_LEFT == side);
    }

    return b_pending;
}

EXPORT void subsystem5_set_drive_permission (const bool b_allowed)
{
    s5_bypass_set_permitted(b_allowed);
}

EXPORT ER subsystem5_init (void)
{
    ER result = drv_servo_init();   /* also turns the servo to 90 degrees */

    __atomic_store_n(&gb_facing_front, false, __ATOMIC_RELAXED);
    __atomic_store_n(&gb_forward_reading_seen, false, __ATOMIC_RELAXED);
    __atomic_store_n(&gb_avoiding, false, __ATOMIC_RELAXED);
    (void)atomic_xchg(&g_line_side, LINE_SIDE_NONE);
    s5_bypass_set_permitted(false);

    if (result < E_OK)
    {
        S5_LOG((UB *)"S5 servo init failed: %d\n", (int)result);
    }
    else
    {
        result = drv_ultrasonic_init();
        if (result < E_OK)
        {
            S5_LOG((UB *)"S5 ultrasonic init failed: %d\n", (int)result);
        }
        else
        {
            g_scanner.b_hardware_ready = 1u;
        }
    }

    return result;
}

EXPORT ER subsystem5_start (void)
{
    ER result = E_OBJ;

    if (0u == g_scanner.b_hardware_ready)
    {
        /* Without the scanner the car cannot see obstacles, so tell
         * Subsystem 1 (the car then stays paused on FAULT_US_SENS). */
        s5_report_scanner_failed();
    }
    else
    {
        /* Hold the car still until the servo faces the front of the car
         * (Subsystem 2 otherwise starts with no speed limit). */
        s5_guard_set_hold(true);

        /* (FP) is the generic task-entry type app_start_task() expects;
         * the task really has the T-Kernel (INT, void *) signature. */
        ID task_id = app_start_task((FP)s5_sensor_task,
                                    SCANNER_TASK_PRIORITY, 0u, "s5_sensor");

        /* ID and ER are both INT, so the cast cannot change the value. */
        result = (task_id < E_OK) ? (ER)task_id : E_OK;
    }

    return result;
}
