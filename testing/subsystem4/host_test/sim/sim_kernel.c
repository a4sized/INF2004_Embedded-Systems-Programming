/*
 * sim_kernel.c -- runs subsystem 4's REAL code (driver, both tasks) on a PC.
 *
 * Around it: a mock micro T-Kernel (threads), a fake LSM303D with a fake car
 * driving over a hump, and mocks of the Subsystem 1/2 functions we call
 * (compiled against their REAL headers, so a wrong prototype would not compile).
 *
 * The fake chip is strict on purpose: it only produces data if the driver
 * switched it on and chose the ranges, it scales the data by the range the
 * driver chose, and it only returns 6-byte bursts if the driver set the
 * auto-increment bit.  A wrong register value in the driver fails a test.
 *
 *   cd testing/subsystem4/host_test && ./run_sim.sh
 */
#define _GNU_SOURCE
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <unistd.h>

#include "tk/tkernel.h"
#include "app.h"
#include "subsystem4/subsystem4.h"
#include "dev_i2c.h"
#include "systemFaults.h"
#include "subsystem1/subsystem1.h"
#include "subsystem2/subsystem2.h"

/* ------------------------------------------------------------------ time */
static struct timespec t0;
static double now_s(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (t.tv_sec - t0.tv_sec) + (t.tv_nsec - t0.tv_nsec) / 1e9;
}
UW s4_sim_timer_us(void) { return (UW)(now_s() * 1e6); }
ER tk_get_otm(SYSTIM *t) { t->hi = 0; t->lo = (UW)(now_s() * 1000.0); return E_OK; }
ER tk_dly_tsk(RELTIM ms) { usleep((useconds_t)ms * 1000U); return E_OK; }

int tm_printf(const UB *fmt, ...)
{
    va_list a; va_start(a, fmt);
    if (fmt[0] == (UB)'[') printf("%7.3f ", now_s());
    int n = vprintf((const char*)fmt, a); va_end(a); return n;
}
INT tm_sprintf(UB *str, const UB *fmt, ...)
{
    va_list a; va_start(a, fmt); int n = vsnprintf((char*)str, 96, (const char*)fmt, a); va_end(a); return n;
}

/* ------------------------------------------------------------ msg buffer */
typedef struct { int used, cap, maxmsz, n, head; unsigned char *buf; int *sz; pthread_mutex_t m; pthread_cond_t c; } MBF;
static MBF mbfs[8];
ID tk_cre_mbf(const T_CMBF *p)
{
    for (int i = 1; i < 8; i++) if (!mbfs[i].used) {
        MBF *b = &mbfs[i]; b->used = 1; b->maxmsz = p->maxmsz;
        b->cap = p->bufsz / (p->maxmsz + 8);               /* same sizing rule as subsystem4_init() */
        b->buf = malloc((size_t)b->cap * (size_t)p->maxmsz); b->sz = calloc((size_t)b->cap, sizeof(int));
        pthread_mutex_init(&b->m, NULL); pthread_cond_init(&b->c, NULL); return i;
    }
    return E_IO;
}
ER tk_snd_mbf(ID id, const void *msg, INT sz, TMO tmo)
{
    MBF *b = &mbfs[id]; (void)tmo; if (sz > b->maxmsz) return -17;
    pthread_mutex_lock(&b->m);
    if (b->n == b->cap) { pthread_mutex_unlock(&b->m); return E_TMOUT; }
    int slot = (b->head + b->n) % b->cap;
    memcpy(b->buf + (size_t)slot * (size_t)b->maxmsz, msg, (size_t)sz); b->sz[slot] = sz; b->n++;
    pthread_cond_signal(&b->c); pthread_mutex_unlock(&b->m); return E_OK;
}
INT tk_rcv_mbf(ID id, void *msg, TMO tmo)
{
    MBF *b = &mbfs[id];
    pthread_mutex_lock(&b->m);
    while (b->n == 0) {
        if (tmo == TMO_POL) { pthread_mutex_unlock(&b->m); return E_TMOUT; }
        pthread_cond_wait(&b->c, &b->m);                   /* TMO_FEVR */
    }
    int sz = b->sz[b->head]; memcpy(msg, b->buf + (size_t)b->head * (size_t)b->maxmsz, (size_t)sz);
    b->head = (b->head + 1) % b->cap; b->n--; pthread_mutex_unlock(&b->m); return sz;
}

/* ------------------------------------------------------------------ task */
typedef struct { FP fn; } TSK;
static void *thr(void *a) { ((void(*)(INT, void*))((TSK*)a)->fn)(0, NULL); return NULL; }
ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
    static int n = 0; (void)pri; (void)prc; (void)name; pthread_t t; TSK *k = malloc(sizeof(*k)); k->fn = entry;
    pthread_create(&t, NULL, thr, k); pthread_detach(t); return ++n;
}
ER dev_init_i2c(UW unit) { return unit == 1 ? E_OK : E_NOSPT; }

/* ============================================================ fake car */
static double frand(void) { return ((double)rand() / RAND_MAX) * 2.0 - 1.0; }
static int    g_stall = 0, g_plugin = 0, g_sensor_present = 1, g_quiet = 0;   /* g_quiet: the car stays still (no shaking) */
/* How the fake board is mounted: raw sensor axis = sign * car axis.  The default is the real car
 * (sensor X backward, Z down).  'wrongmount' flips Z so the firmware's mapping is wrong. */
static double g_sx = -1.0, g_sy = 1.0, g_sz = -1.0;
static const double BIAS[3] = { 0.030, -0.020, 0.010 };   /* offsets that calibration must remove */

#define T_START  2.0        /* the car starts moving            */
#define T_RAMP   1.0        /* seconds to reach cruise speed    */
#define V_CRUISE 20.0       /* cm/s                             */
#define HUMP_AT  100.0      /* cm                               */
#define HUMP_L   30.0
#define HUMP_H   3.0
#define T_BUMP   5.0        /* a hard knock                     */
#define T_TURN   10.0       /* then the car turns left at 60 deg/s for 1 s */
#define TURN_DPS 60.0

static double car_dist(double t)
{
    if (t < T_START) return 0.0;
    double u = t - T_START;
    if (u < T_RAMP) return 0.5 * (V_CRUISE / T_RAMP) * u * u;
    return 0.5 * V_CRUISE * T_RAMP + V_CRUISE * (u - T_RAMP);
}
static double car_acc_g(double t) { return (t >= T_START && t - T_START < T_RAMP) ? V_CRUISE / T_RAMP / 980.665 : 0.0; }
static double car_heading_deg(double t) { return t < T_TURN ? 0.0 : (t < T_TURN + 1.0 ? TURN_DPS * (t - T_TURN) : TURN_DPS); }

/* acceleration in CAR axes, then mounted onto the raw sensor axes */
static void car_accel(double t, double a[3])
{
    double d = car_dist(t), pitch = 0.0;
    double noise = (t < T_START || g_quiet) ? 0.004 : 0.04;           /* a moving car shakes a lot more than a still one */
    if (d > HUMP_AT && d < HUMP_AT + HUMP_L) pitch = atan(HUMP_H * (M_PI / HUMP_L) * sin(2 * M_PI * (d - HUMP_AT) / HUMP_L));
    a[0] = sin(pitch) + car_acc_g(t) + BIAS[0] + noise * frand();
    a[1] = BIAS[1] + noise * frand();
    a[2] = cos(pitch) + BIAS[2] + noise * frand();
    if (fabs(t - T_BUMP) < 0.012) a[0] -= 1.8;             /* the knock (in car axes) */
    a[0] *= g_sx; a[1] *= g_sy; a[2] *= g_sz;              /* mount the board last */
}
/* the Earth's field is 0.25 gauss, horizontal, pointing along the car at t=0.  When the car turns left
 * by theta the field turns RIGHT in car axes: (0.25 cos, -0.25 sin) in (fwd, left). */
static void car_mag(double t, double m[3])
{
    double th = car_heading_deg(t) * M_PI / 180.0;
    m[0] = 0.25 * cos(th) * g_sx;  m[1] = -0.25 * sin(th) * g_sy;  m[2] = -0.40 * g_sz;
}

/* ============================================= fake LSM303D at 0x1D, strict */
static unsigned char r_ctrl1, r_ctrl2, r_ctrl5, r_ctrl6, r_ctrl7;
static void put16(unsigned char *p, int v) { p[0] = (unsigned char)(v & 0xFF); p[1] = (unsigned char)((v >> 8) & 0xFF); }

ID tk_opn_dev(const UB *name, UINT mode) { (void)mode; return strcmp((const char*)name, "iicb") == 0 ? 1 : E_NOSPT; }
ER tk_cls_dev(ID dd, UINT opt) { (void)dd; (void)opt; return E_OK; }

#define E_PAR_SIM (-17)
ER tk_swri_dev(ID dd, INT dn, const void *buf, SZ sz, SZ *asz)
{
    (void)dd; *asz = 0;
    if (!g_sensor_present || (g_plugin && now_s() < 3.0)) { usleep(1000); return E_IO; }   /* nobody acknowledges */

    if (dn >= 0) {                                         /* plain write: dn = slave address */
        const unsigned char *b = buf;
        if (sz < 2 || dn != 0x1D) return dn != 0x1D ? E_IO : E_PAR_SIM;
        switch (b[0]) { case 0x20: r_ctrl1 = b[1]; break; case 0x21: r_ctrl2 = b[1]; break; case 0x24: r_ctrl5 = b[1]; break;
                        case 0x25: r_ctrl6 = b[1]; break; case 0x26: r_ctrl7 = b[1]; break; default: break; }
        *asz = sz; return E_OK;
    }
    /* the real kernel driver's rules for the "execute" call */
    if (dn != TDN_I2C_EXEC || sz < (SZ)sizeof(T_I2C_EXEC)) return E_PAR_SIM;
    const T_I2C_EXEC *e = buf;
    if (e->snd_size <= 0 || e->snd_size > 100 || e->rcv_size <= 0 || e->rcv_size > 100) return E_PAR_SIM;
    if (e->sadr != 0x1D) return E_IO;                      /* no other address answers */

    unsigned char reg = e->snd_data[0], r = reg & 0x7F;
    int burst = (reg & 0x80) != 0;                         /* auto-increment bit */
    if (r == 0x0F) { e->rcv_data[0] = 0x49; return E_OK; } /* WHO_AM_I */

    if (r == 0x28 && e->rcv_size == 6) {                   /* accelerometer X,Y,Z */
        static const double mg[5] = { 0.061, 0.122, 0.183, 0.244, 0.732 };
        int afs = (r_ctrl2 >> 3) & 7, on = ((r_ctrl1 >> 4) != 0) && ((r_ctrl1 & 7) == 7);
        if (g_stall && now_s() > 1.5 && (rand() % 90) == 0) usleep(35000);     /* being pre-empted */
        double a[3]; car_accel(now_s(), a);
        for (int i = 0; i < 6; i += 2) {
            int v = (on && afs < 5 && burst) ? (int)lround(a[i / 2] * 1000.0 / mg[afs]) : 0;
            if (!burst && i) v = (int)(short)(e->rcv_data[0] | (e->rcv_data[1] << 8));   /* no auto-increment: same register again */
            put16(e->rcv_data + i, v);
        }
        return E_OK;
    }
    if (r == 0x08 && e->rcv_size == 6) {                   /* magnetometer X,Y,Z */
        static const double mgauss[4] = { 0.08, 0.16, 0.32, 0.479 };
        int mfs = (r_ctrl6 >> 5) & 3, on = ((r_ctrl7 & 3) == 0);
        double m[3]; car_mag(now_s(), m);
        for (int i = 0; i < 6; i += 2) put16(e->rcv_data + i, (on && burst) ? (int)lround(m[i / 2] * 1000.0 / mgauss[mfs]) : 0);
        return E_OK;
    }
    for (SZ i = 0; i < e->rcv_size; i++) e->rcv_data[i] = 0;
    return E_OK;
}

/* =========================== mocks of subsystem 1 and 2 (real prototypes) */
volatile uint32_t active_fault_mask = 0;
static S1_TELEMETRY g_s1 = { 0, 123, 45, 0, 0, "BC1" };      /* speed 123, obstacle 45: must survive */
static int s1_updates, s1_pitch_max = -999, s1_pitch_min = 999;
static int csv_hump, csv_collision, csv_fault, csv_summary;
static char csv_hump_details[96];
static pthread_mutex_t s1_m = PTHREAD_MUTEX_INITIALIZER;

void s1_update_telemetry(INT speed_mm_s, INT obstacle_cm, INT pitch_deg, UW fault_code, const UB *barcode)
{
    pthread_mutex_lock(&s1_m);
    g_s1.speed_mm_s = speed_mm_s; g_s1.obstacle_cm = obstacle_cm; g_s1.pitch_deg = pitch_deg; g_s1.fault_code = fault_code;
    if (barcode) { strncpy((char*)g_s1.barcode, (const char*)barcode, 15); g_s1.barcode[15] = 0; }
    s1_updates++; if (pitch_deg > s1_pitch_max) s1_pitch_max = pitch_deg; if (pitch_deg < s1_pitch_min) s1_pitch_min = pitch_deg;
    pthread_mutex_unlock(&s1_m);
}
void s1_get_telemetry(S1_TELEMETRY *out) { pthread_mutex_lock(&s1_m); *out = g_s1; pthread_mutex_unlock(&s1_m); }
void s1_csv_log_event(const char *e, const char *d)
{
    pthread_mutex_lock(&s1_m);
    if (!strcmp(e, "HUMP")) { csv_hump++; snprintf(csv_hump_details, sizeof csv_hump_details, "%s", d); }
    else if (!strcmp(e, "COLLISION")) csv_collision++;
    else if (!strcmp(e, "IMU_FAULT")) csv_fault++;
    else if (!strcmp(e, "RUN_SUMMARY")) csv_summary++;
    pthread_mutex_unlock(&s1_m);
    printf("%7.3f LOG_CSV,%s,%s\n", now_s(), e, d);
}
int32_t get_left_encoder_ticks(void)  { return (int32_t)(car_dist(now_s()) / 0.05); }   /* 0.05 cm per tick */
int32_t get_right_encoder_ticks(void) { return (int32_t)(car_dist(now_s()) / 0.05); }

/* ================================================================ checks */
static int fails;
#define CHECK_SIM(cond, desc) do { int ok_ = ((cond) != 0); printf("  %s  %s\n", ok_ ? "PASS" : "FAIL", desc); if (!ok_) fails++; } while (0)
#define WAIT_UNTIL(sec) do { while (now_s() < (sec)) usleep(5000); } while (0)
#define WIFI_BIT (1U << 7)

static int finish(void) { printf("%s\n", fails ? "FAILED" : "ALL PASSED"); return fails ? 1 : 0; }

int main(int argc, char **argv)
{
    const char *mode = argc > 1 ? argv[1] : "drive";
    g_stall = !strcmp(mode, "stall"); g_plugin = !strcmp(mode, "plugin"); g_quiet = g_plugin;   /* re-plugging is done with the car standing still */
    g_sensor_present = strcmp(mode, "nosensor") != 0;
    if (!strcmp(mode, "wrongmount")) g_sz = 1.0;           /* Z flipped: the firmware's mapping is now wrong */
    clock_gettime(CLOCK_MONOTONIC, &t0); srand(7);
    active_fault_mask |= WIFI_BIT;                         /* another subsystem's fault: must survive */

    if (subsystem4_init() != E_OK || subsystem4_start() != E_OK) { printf("init/start failed\n"); return 1; }

    if (!strcmp(mode, "nosensor")) {
        WAIT_UNTIL(3.5);
        printf("\n--- sensor unplugged ---\n");
        CHECK_SIM(s4_get_faults() & S4_FAULT_COMM, "COMM fault raised");
        CHECK_SIM(active_fault_mask & S4_FAULT_COMM, "bit 28 is in the shared mask, so the motors halt");
        CHECK_SIM(active_fault_mask & WIFI_BIT, "another subsystem's bit untouched");
        CHECK_SIM(!s4_calibrated && s4_samples == 0, "nothing was read");
        return finish();
    }
    if (!strcmp(mode, "wrongmount")) {
        WAIT_UNTIL(6.0);
        printf("\n--- board mounted the other way up, mapping NOT changed ---\n");
        CHECK_SIM(!s4_calibrated, "calibration refuses to run (and says why)");
        CHECK_SIM(!(s4_get_faults() & S4_FAULT_COMM), "the sensor itself is fine: no fault");
        return finish();
    }
    if (!strcmp(mode, "plugin")) {
        WAIT_UNTIL(2.0);
        int comm_before = (active_fault_mask & S4_FAULT_COMM) != 0;
        WAIT_UNTIL(7.0);
        printf("\n--- sensor plugged in after 3 s (cable fixed while running) ---\n");
        CHECK_SIM(comm_before, "COMM fault was raised while the sensor was missing");
        CHECK_SIM(s4_calibrated, "calibrated by itself once the sensor appeared");
        CHECK_SIM(!(active_fault_mask & S4_FAULT_COMM), "COMM fault removed from the shared mask");
        CHECK_SIM(active_fault_mask & WIFI_BIT, "another subsystem's bit untouched");
        CHECK_SIM(s4_samples > 40, "readings flowing");
        return finish();
    }

    /* ---- the full scripted drive.  still 0-2 s, accelerate, knock at 5.0, hump 7.6-9.1, turn 10-11 ---- */
    int seen_stationary = 0, seen_moving = 0, seen_impact = 0, seen_turning = 0;
    float peak_turn = 0;
    int latch = !strcmp(mode, "latch");
    double t;
    while ((t = now_s()) < 13.0) {
        S4_MOTION m = s4_get_motion();
        if (t > 1.5 && t < 2.0 && m == S4_STATIONARY) seen_stationary = 1;
        if (t > 4.0 && t < 4.9 && m == S4_MOVING) seen_moving = 1;
        if (t > 5.0 && t < 5.6 && m == S4_IMPACT) seen_impact = 1;
        if (t > 10.2 && t < 11.3) { if (m == S4_TURNING) seen_turning = 1; if (s4_get_turn_rate() > peak_turn) peak_turn = s4_get_turn_rate(); }
        static int sampled_a, sampled_b, sampled_c; static int l1, g1, w1, l2, g2, l3, g3;
        if (!sampled_a && t > 5.6) { sampled_a = 1; l1 = (s4_get_faults() & S4_FAULT_COLLISION) != 0; g1 = (active_fault_mask & S4_FAULT_COLLISION) != 0; w1 = (active_fault_mask & WIFI_BIT) != 0; }
        if (!sampled_b && t > 9.0) { sampled_b = 1; l2 = (s4_get_faults() & S4_FAULT_COLLISION) != 0; g2 = (active_fault_mask & S4_FAULT_COLLISION) != 0;
                                     if (latch) s4_clear_collision(); }
        if (!sampled_c && t > 9.6) { sampled_c = 1; l3 = (s4_get_faults() & S4_FAULT_COLLISION) != 0; g3 = (active_fault_mask & S4_FAULT_COLLISION) != 0;
            /* stash for the report */
            seen_impact |= 0; (void)l3; (void)g3; }
        usleep(5000);
        if (t > 12.9) {            /* report */
            S1_TELEMETRY tl; s1_get_telemetry(&tl);
            int n = 0, peak_mm = 0; sscanf(csv_hump_details, "n=%d,peak_mm=%d", &n, &peak_mm);
            float tol = (S4_CM_PER_TICK > 0.0f) ? 0.6f : 1.2f;
            printf("\n--- results (%s) ---\n", mode);
            printf("samples %u, S1 pitch updates %d (max %d, min %d deg), humps %u, run max %.2f cm, turn peak %.1f dps\n",
                   (unsigned)s4_samples, s1_updates, s1_pitch_max, s1_pitch_min, (unsigned)s4_get_hump_count(), s4_get_max_hump_cm(), peak_turn);
            CHECK_SIM(s4_calibrated, "calibration succeeded (offsets removed, car still)");
            CHECK_SIM(s4_samples > 400, "sensor read about 50 times a second");
            CHECK_SIM(seen_stationary && seen_moving, "STATIONARY while standing still, MOVING while cruising");
            CHECK_SIM(s4_get_hump_count() == 1, "exactly one hump detected");
            CHECK_SIM(fabsf(s4_get_max_hump_cm() - 3.0f) < tol, S4_CM_PER_TICK > 0.0f ? "peak height within +/-20% of the true 3 cm (distance from subsystem 2's encoders)" : "peak height within 1.2 cm of the true 3 cm (fixed-speed assumption)");
            CHECK_SIM(csv_hump == 1 && abs(peak_mm - 30) <= (int)(tol * 10), "hump logged once through s1_csv_log_event(\"HUMP\"), in mm");
            printf("  -- subsystem 1 (IF-01, IF-09) --\n");
            CHECK_SIM(s1_updates >= 50, "pitch pushed to s1_update_telemetry() regularly");
            CHECK_SIM(s1_pitch_max >= 10 && s1_pitch_min <= -10, "nose up gives POSITIVE pitch, nose down NEGATIVE, in the telemetry");
            CHECK_SIM(tl.speed_mm_s == 123 && tl.obstacle_cm == 45 && !strcmp((char*)tl.barcode, "BC1"), "speed, obstacle and barcode set by others are NOT overwritten");
            printf("  -- collision + shared fault mask (IF-08) --\n");
            CHECK_SIM(csv_collision == 1 && seen_impact, "one collision: logged, and motion state IMPACT shown");
            CHECK_SIM(l1 && g1, "collision fault set locally AND as bit 30 of active_fault_mask");
            CHECK_SIM(w1 && (active_fault_mask & WIFI_BIT), "another subsystem's bit (WIFI) never disturbed");
            if (latch) {
                CHECK_SIM(l2 && g2, "S4_COLLISION_HOLD_MS = 0: still latched 4 s later");
                CHECK_SIM(!l3 && !g3, "s4_clear_collision() releases it");
            } else {
                CHECK_SIM(!l2 && !g2, "released by itself after the hold time (3 s)");
            }
            printf("  -- turning --\n");
            CHECK_SIM(seen_turning && peak_turn > 40.0f && peak_turn < 80.0f, "TURNING seen; turn rate about +60 dps (left is positive)");
            CHECK_SIM(!(s4_get_faults() & S4_FAULT_COMM), "no sensor fault");
            s4_reset_run(); usleep(200000);
            CHECK_SIM(s4_get_hump_count() == 0 && s4_get_max_hump_cm() == 0.0f, "s4_reset_run() clears the highest hump and the count");
            s4_publish_summary(); usleep(100000);
            CHECK_SIM(csv_summary == 1, "run summary logged");
            return finish();
        }
    }
    return finish();
}
