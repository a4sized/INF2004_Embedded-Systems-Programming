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

/* ------------------------------------------------------------ event flag */
typedef struct { int used; unsigned ptn; pthread_mutex_t m; } FLG;
static FLG flgs[8];
ID tk_cre_flg(const T_CFLG *p)
{
    for (int i = 1; i < 8; i++) if (!flgs[i].used) { flgs[i].used = 1; flgs[i].ptn = p->iflgptn; pthread_mutex_init(&flgs[i].m, NULL); return i; }
    return E_IO;
}
ER tk_set_flg(ID id, UINT ptn) { pthread_mutex_lock(&flgs[id].m); flgs[id].ptn |= ptn; pthread_mutex_unlock(&flgs[id].m); return E_OK; }
ER tk_wai_flg(ID id, UINT ptn, UINT mode, UINT *out, TMO tmo)
{
    (void)tmo;                                           /* the firmware only polls (TMO_POL) */
    pthread_mutex_lock(&flgs[id].m);
    if (!(flgs[id].ptn & ptn)) { pthread_mutex_unlock(&flgs[id].m); return E_TMOUT; }
    *out = flgs[id].ptn;
    if (mode & TWF_CLR) flgs[id].ptn = 0;                /* real kernel: TWF_CLR clears the WHOLE pattern */
    else if (mode & TWF_BITCLR) flgs[id].ptn &= ~ptn;    /* TWF_BITCLR clears only the bits waited for     */
    pthread_mutex_unlock(&flgs[id].m); return E_OK;
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
/* Which scripted scene the fake car plays.  SCN_DRIVE is the full drive; the others are handled by hand or on rough ground. */
enum { SCN_DRIVE = 0, SCN_LIFT, SCN_OVERTURN, SCN_FLIP, SCN_ROUGH, SCN_RAMP };
static int    g_scn = SCN_DRIVE;
static double g_lift_g = 0.30, g_lift_s = 0.30, g_rough_g = 0.12, g_ramp_deg = 30.0, g_carry_deg = 12.0;   /* the probe mode changes these */
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
    if (g_scn != SCN_DRIVE) return 0.0;             /* the scenes do not drive: the wheels never turn */
    if (t < T_START) return 0.0;
    double u = t - T_START;
    if (u < T_RAMP) return 0.5 * (V_CRUISE / T_RAMP) * u * u;
    return 0.5 * V_CRUISE * T_RAMP + V_CRUISE * (u - T_RAMP);
}
static double car_acc_g(double t) { return (t >= T_START && t - T_START < T_RAMP) ? V_CRUISE / T_RAMP / 980.665 : 0.0; }
static double car_heading_deg(double t) { return t < T_TURN ? 0.0 : (t < T_TURN + 1.0 ? TURN_DPS * (t - T_TURN) : TURN_DPS); }

static void mount(double a[3]) { a[0] *= g_sx; a[1] *= g_sy; a[2] *= g_sz; }      /* car axes -> raw sensor axes */

/* The scenes, in CAR axes (the board is mounted afterwards).  A reading is gravity (1 g up) plus what the scene adds.
 *   LIFT      on the floor 0-2.0 s | lifted 2.0-2.3 (+0.30 g up) | carried 2.3-3.6, tilted 12 deg, hand tremor |
 *             lowered 3.6-3.8 (-0.20 g) | the descent stops 3.8-3.95 (+0.35 g) | resting on the floor from 3.95 s
 *   OVERTURN  tips over onto its side 2.0-2.5 (lands with a 0.8 g thud) | lies there until 4.5 | set upright 4.5-5.0
 *   FLIP      turns upside down 2.0-2.6 and stays there
 *   ROUGH     driving over rough ground 2.0-7.0 s (0.12 g shaking), with a 1.8 g knock at 3.0, a 0.9 g vertical bump at
 *             4.0 and a 0.8 g sideways shove at 5.0: nothing here may stop the car
 *   RAMP      climbs a 30 degree slope 2.0-3.0, stays on it until 6.0, comes down by 7.0: not "picked up" */
static void scene_accel(double t, double a[3])
{
    double pitch = 0.0, phi = 0.0, up_extra = 0.0, left_extra = 0.0, fwd_extra = 0.0, noise = 0.004;
    const double D2R = M_PI / 180.0;

    switch (g_scn) {
    case SCN_LIFT:
        if      (t >= 2.0 && t < 2.0 + g_lift_s) { up_extra = g_lift_g;  noise = 0.04; }
        else if (t >= 2.0 + g_lift_s && t < 3.6)  { pitch = g_carry_deg * D2R; noise = 0.04; }
        else if (t >= 3.6 && t < 3.8)  { up_extra = -0.20; noise = 0.04; }
        else if (t >= 3.8 && t < 3.95) { up_extra = 0.35;  noise = 0.04; }
        break;
    case SCN_OVERTURN:
        if      (t >= 2.0 && t < 2.5)  { phi = 90.0 * (t - 2.0) / 0.5; noise = 0.05; }
        else if (t >= 2.5 && t < 4.5)  { phi = 90.0; }
        else if (t >= 4.5 && t < 5.0)  { phi = 90.0 * (1.0 - (t - 4.5) / 0.5); noise = 0.04; }
        if (t >= 2.5 && t < 2.53) left_extra = -0.8;                                /* the thud of landing on its side */
        break;
    case SCN_FLIP:
        if      (t >= 2.0 && t < 2.6)  { phi = 180.0 * (t - 2.0) / 0.6; noise = 0.05; }
        else if (t >= 2.6)             { phi = 180.0; }
        break;
    case SCN_ROUGH:
        if (t >= 2.0 && t < 7.0) noise = g_rough_g;
        if (fabs(t - 3.0) < 0.012) fwd_extra = -1.8;
        if (fabs(t - 4.0) < 0.015) up_extra = 0.9;
        if (fabs(t - 5.0) < 0.020) left_extra = 0.8;
        break;
    case SCN_RAMP:
        if      (t >= 2.0 && t < 3.0)  pitch = g_ramp_deg * D2R * (t - 2.0);
        else if (t >= 3.0 && t < 6.0)  pitch = g_ramp_deg * D2R;
        else if (t >= 6.0 && t < 7.0)  pitch = g_ramp_deg * D2R * (1.0 - (t - 6.0));
        if (t >= 2.0 && t < 7.0) noise = 0.04;
        break;
    default: break;
    }
    phi *= D2R;     /* the scenes above give 'phi' in degrees; sin() and cos() want radians */
    /* gravity: tilted forward by 'pitch' (nose up) and about the forward axis by 'phi' (on its side / upside down) */
    a[0] = sin(pitch) + fwd_extra + BIAS[0] + noise * frand();
    a[1] = -sin(phi) + left_extra + BIAS[1] + noise * frand();
    a[2] = cos(pitch) * cos(phi) + up_extra + BIAS[2] + noise * frand();
}

/* acceleration in CAR axes, then mounted onto the raw sensor axes */
static void car_accel(double t, double a[3])
{
    if (g_scn != SCN_DRIVE) { scene_accel(t, a); mount(a); return; }
    double d = car_dist(t), pitch = 0.0;
    double noise = (t < T_START || g_quiet) ? 0.004 : 0.04;           /* a moving car shakes a lot more than a still one */
    if (d > HUMP_AT && d < HUMP_AT + HUMP_L) pitch = atan(HUMP_H * (M_PI / HUMP_L) * sin(2 * M_PI * (d - HUMP_AT) / HUMP_L));
    a[0] = sin(pitch) + car_acc_g(t) + BIAS[0] + noise * frand();
    a[1] = BIAS[1] + noise * frand();
    a[2] = cos(pitch) + BIAS[2] + noise * frand();
    /* The knock (in car axes).  It is 24 ms long.  In the 'stall' runs a task is pre-empted for 35 ms now and then,
     * and a 22 ms sampling loop can then step right over a 24 ms knock: that is a real limit of 20 ms sampling,
     * not a bug.  So in the stall runs the knock is made longer than the worst gap (80 ms). */
    if (fabs(t - T_BUMP) < (g_stall ? 0.040 : 0.012)) a[0] -= 1.8;
    mount(a);                                              /* mount the board last */
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

/* ============== mocks of subsystem 1 (REAL prototypes) and subsystem 2 ===== */
/* Subsystem 1's real functions each take a lock; one mutex here does the same. */
static S1_TELEMETRY_PKT g_s1;
static int  s1_pitch_updates, s1_pitch_max = -999, s1_pitch_min = 999;
static unsigned s1_car_states_seen, s1_hump_states_seen;     /* bit n set = value n was reported */
static int  csv_hump, csv_picked, csv_over, csv_put_down, csv_fault, csv_summary;
static char csv_hump_details[96];
static pthread_mutex_t s1_m = PTHREAD_MUTEX_INITIALIZER;

void s1_set_speed(INT v)    { pthread_mutex_lock(&s1_m); g_s1.speed_mm_s = v;  pthread_mutex_unlock(&s1_m); }
void s1_set_obstacle(INT v) { pthread_mutex_lock(&s1_m); g_s1.obstacle_cm = v; pthread_mutex_unlock(&s1_m); }
void s1_set_barcode_id(UB v){ pthread_mutex_lock(&s1_m); g_s1.barcode_id = v;  pthread_mutex_unlock(&s1_m); }
void s1_set_line_mask(UW v) { pthread_mutex_lock(&s1_m); g_s1.line_mask = v;   pthread_mutex_unlock(&s1_m); }
void s1_set_pitch(INT pitch_deg)
{
    pthread_mutex_lock(&s1_m);
    g_s1.pitch_deg = pitch_deg;                      /* an 8-bit signed field, like the real one */
    s1_pitch_updates++;
    if (pitch_deg > s1_pitch_max) s1_pitch_max = pitch_deg;
    if (pitch_deg < s1_pitch_min) s1_pitch_min = pitch_deg;
    pthread_mutex_unlock(&s1_m);
}
void s1_set_car_state(UW state, UW hump)
{
    pthread_mutex_lock(&s1_m);
    g_s1.car_state = state; g_s1.hump_state = hump;  /* 3-bit and 2-bit fields */
    s1_car_states_seen |= 1u << state; s1_hump_states_seen |= 1u << hump;
    pthread_mutex_unlock(&s1_m);
}
/* BUDDY 1'S REAL BEHAVIOUR (task_led.c): s1_report_fault() sets BOTH the telemetry mask and the event flag flg_s1_isr_fault;
 * the LED task, every S1_TICK_MS = 250 ms, reads that flag (clearing it) and ORs its bits back into the mask. */
static unsigned s1_isr_flag;
void s1_report_fault(UW bit) { pthread_mutex_lock(&s1_m); s1_isr_flag |= bit; g_s1.fault_mask |= bit;  pthread_mutex_unlock(&s1_m); }   /* a 12-bit field */
static void *led_thread(void *a) { (void)a; { const char *ph = getenv("LED_PHASE_MS"); if (ph) usleep((useconds_t)atoi(ph) * 1000U); } for (;;) { usleep(250000); pthread_mutex_lock(&s1_m); if (s1_isr_flag) { g_s1.fault_mask |= s1_isr_flag; s1_isr_flag = 0; } pthread_mutex_unlock(&s1_m); } return NULL; }
void s1_clear_fault(UW bit)  { pthread_mutex_lock(&s1_m); g_s1.fault_mask &= ~bit; pthread_mutex_unlock(&s1_m); }
void s1_get_telemetry(S1_TELEMETRY_PKT *out) { pthread_mutex_lock(&s1_m); *out = g_s1; pthread_mutex_unlock(&s1_m); }
void s1_csv_log_event(const char *e, const char *d)
{
    pthread_mutex_lock(&s1_m);
    if (!strcmp(e, "HUMP")) { csv_hump++; snprintf(csv_hump_details, sizeof csv_hump_details, "%s", d); }
    else if (!strcmp(e, "PICKED_UP")) csv_picked++;
    else if (!strcmp(e, "OVERTURNED")) csv_over++;
    else if (!strcmp(e, "PUT_DOWN")) csv_put_down++;
    else if (!strcmp(e, "IMU_FAULT")) csv_fault++;
    else if (!strcmp(e, "RUN_SUMMARY")) csv_summary++;
    pthread_mutex_unlock(&s1_m);
    printf("%7.3f LOG_CSV,%s,%s\n", now_s(), e, d);
}
/* Subsystem 2's emergency override (REAL behaviour, subsystem2.c): it reports the fault bit to Subsystem 1 itself and halts the
 * motors at once; the halt then lasts while a fault bit is set.  Here: count the calls and note when the first one came. */
static int    halt_calls = 0;
static double halt_first_t = -1.0;
void motion_trigger_safety_halt(UINT fault_bit)
{
    s1_report_fault(fault_bit);
    if (halt_first_t < 0.0) halt_first_t = now_s();
    halt_calls++;
}

/* Subsystem 2's distance driven: the mean of both wheels in mm, forward positive.  In mode "reset" the
 * counters are zeroed at 8.0 s, in the middle of the hump, exactly as motor_reset_ticks() would do. */
static int    g_reset_mode = 0, enc_reset_done = 0;
static double enc_offset_cm = 0.0;
float get_distance_traveled_mm(void)
{
    double t = now_s(), d = car_dist(t);
    if (g_reset_mode && !enc_reset_done && t > 8.0) { enc_reset_done = 1; enc_offset_cm = d; }
    return (float)((d - enc_offset_cm) * 10.0);
}

static int imu_bit(void) { S1_TELEMETRY_PKT t; s1_get_telemetry(&t); return (t.fault_mask & FAULT_IMU_SENS) != 0; }
static int wifi_bit(void) { S1_TELEMETRY_PKT t; s1_get_telemetry(&t); return (t.fault_mask & FAULT_WIFI) != 0; }

/* ================================================================ checks */
static int fails;
#define CHECK_SIM(cond, desc) do { int ok_ = ((cond) != 0); printf("  %s  %s\n", ok_ ? "PASS" : "FAIL", desc); if (!ok_) fails++; } while (0)
#define WAIT_UNTIL(sec) do { while (now_s() < (sec)) usleep(5000); } while (0)

static int finish(void) { printf("%s\n", fails ? "FAILED" : "ALL PASSED"); return fails ? 1 : 0; }

int main(int argc, char **argv)
{
    const char *mode = argc > 1 ? argv[1] : "drive";
    g_reset_mode = !strcmp(mode, "reset");
    g_stall = !strcmp(mode, "stall"); g_plugin = !strcmp(mode, "plugin"); g_quiet = g_plugin;   /* re-plugging is done with the car standing still */
    g_sensor_present = strcmp(mode, "nosensor") != 0;
    if (!strcmp(mode, "wrongmount")) g_sz = 1.0;           /* Z flipped: the firmware's mapping is now wrong */
    if (!strcmp(mode, "probe")) {                          /* PROBE_SCN = lift | rough | ramp,  PROBE_VAL = the number to try */
        const char *sc = getenv("PROBE_SCN"); double v = getenv("PROBE_VAL") ? atof(getenv("PROBE_VAL")) : 0.0;
        if (sc && !strcmp(sc, "lift_g"))    { g_scn = SCN_LIFT;  g_lift_g = v; }       /* how strong a 0.3 s lift */
        if (sc && !strcmp(sc, "lift_ms"))   { g_scn = SCN_LIFT;  g_lift_s = v / 1000.0; g_lift_g = 0.30; }   /* how short a 0.30 g lift */
        if (sc && !strcmp(sc, "carry_deg")) { g_scn = SCN_LIFT;  g_lift_g = 0.0; g_carry_deg = v; }          /* carried tilted, no lift pulse at all */
        if (sc && !strcmp(sc, "rough_g"))   { g_scn = SCN_ROUGH; g_rough_g = v; }
        if (sc && !strcmp(sc, "ramp_deg"))  { g_scn = SCN_RAMP;  g_ramp_deg = v; }
    }
    if (!strcmp(mode, "lift") || !strcmp(mode, "ghost")) g_scn = SCN_LIFT;
    else if (!strcmp(mode, "overturn")) g_scn = SCN_OVERTURN;
    else if (!strcmp(mode, "flip"))     g_scn = SCN_FLIP;
    else if (!strcmp(mode, "rough"))    g_scn = SCN_ROUGH;
    else if (!strcmp(mode, "ramp"))     g_scn = SCN_RAMP;
    clock_gettime(CLOCK_MONOTONIC, &t0); srand(7);

    /* what the other subsystems have already put in Subsystem 1's packet: it must all survive */
    s1_set_speed(123); s1_set_obstacle(45); s1_set_barcode_id(7);
    s1_report_fault(FAULT_WIFI);                           /* another subsystem's fault */

    { pthread_t lt; pthread_create(&lt, NULL, led_thread, NULL); pthread_detach(lt); }   /* Buddy 1's LED task */
    if (subsystem4_init() != E_OK || subsystem4_start() != E_OK) { printf("init/start failed\n"); return 1; }

    if (!strcmp(mode, "nosensor")) {
        WAIT_UNTIL(3.5);
        printf("\n--- sensor unplugged ---\n");
        CHECK_SIM(s4_get_faults() & S4_FAULT_COMM, "COMM condition raised");
        CHECK_SIM(imu_bit(), "FAULT_IMU_SENS is set in subsystem 1's fault mask");
        CHECK_SIM(wifi_bit(), "another subsystem's fault bit untouched");
        CHECK_SIM(!s4_is_calibrated() && s4_samples == 0, "nothing was read");
        return finish();
    }
    if (!strcmp(mode, "probe")) {
        const char *sc = getenv("PROBE_SCN"); int rough_or_ramp = sc && (!strcmp(sc, "rough_g") || !strcmp(sc, "ramp_deg"));
        WAIT_UNTIL(rough_or_ramp ? 6.0 : 3.4);
        printf("PROBE %s=%s fired=%d first_halt_at=%.2f s\n", sc, getenv("PROBE_VAL"), halt_calls > 0, halt_first_t);
        return 0;
    }
    if (!strcmp(mode, "lift") || !strcmp(mode, "ghost")) {
        /* Picked up at 2.0 s, carried, set down (resting from 3.95 s).  S4_PUTDOWN_MS is how long it must rest before it
         * counts as put down; the 0.4 s is the shaking of the hand dying away.  In 'ghost' runs S4_PUTDOWN_MS is 600 and
         * Subsystem 1's LED task (every 250 ms) is run at a different phase each time: a stale report left in its event
         * flag must not bring the fault back after the release. */
        const double t_rest = 3.95, t_put_down = t_rest + 0.4 + S4_PUTDOWN_MS / 1000.0;
        WAIT_UNTIL(1.9);
        int quiet_before = (s4_get_lift() == S4_LIFT_NONE) && !imu_bit() && halt_calls == 0;
        WAIT_UNTIL(2.5);
        int picked = (s4_get_lift() == S4_LIFT_PICKED_UP), halted_in_time = (halt_first_t >= 2.0 && halt_first_t < 2.45), bit_up = imu_bit();
        WAIT_UNTIL(2.7);
        int shown_lifted = (s4_get_motion() == S4_LIFTED);
        s1_clear_fault(FAULT_IMU_SENS);                        /* somebody else clears our bit while the car is still in the air */
        WAIT_UNTIL(3.0);
        int stopped_again = imu_bit() && halt_calls >= 2;
        s4_clear_lifted();                                     /* a CLEAR sent while the car is still being carried */
        WAIT_UNTIL(3.5);
        int clear_ignored = (s4_get_lift() == S4_LIFT_PICKED_UP) && imu_bit();
        int not_early = 1, latched_after = 1;
        if (S4_PUTDOWN_MS >= 1500) { WAIT_UNTIL(t_rest + 1.0); not_early = (s4_get_lift() != S4_LIFT_NONE) && imu_bit(); }
#if S4_LIFT_LATCH
        WAIT_UNTIL(t_put_down + 0.6);
        latched_after = (s4_get_lift() != S4_LIFT_NONE) && imu_bit();
        s4_clear_lifted();                                     /* a CLEAR while the car is resting level */
        WAIT_UNTIL(t_put_down + 1.2);
#else
        (void)latched_after;
        WAIT_UNTIL(t_put_down + 0.8);
#endif
        int released = (s4_get_lift() == S4_LIFT_NONE) && !imu_bit();
        WAIT_UNTIL(t_put_down + 2.0);
        int no_ghost = !imu_bit();
        printf("\n--- %s: picked up at 2.0 s, carried, put down at 3.95 s  (S4_PUTDOWN_MS %d, S4_LIFT_LATCH %d, LED task phase %s ms) ---\n",
               mode, (int)S4_PUTDOWN_MS, (int)S4_LIFT_LATCH, getenv("LED_PHASE_MS") ? getenv("LED_PHASE_MS") : "0");
        CHECK_SIM(quiet_before, "nothing fires while the car sits on the floor, and no halt was sent");
        CHECK_SIM(picked, "picked up: the detector says PICKED_UP");
        CHECK_SIM(halted_in_time, "the motors were halted within 0.45 s of the lift (motion_trigger_safety_halt called)");
        CHECK_SIM(bit_up, "FAULT_IMU_SENS is set through subsystem 1");
        CHECK_SIM(shown_lifted, "the motion state shown is LIFTED");
        CHECK_SIM(stopped_again, "a clear by someone else while it is still lifted is undone, and the motors are halted again");
        CHECK_SIM(clear_ignored, "a CLEAR sent while the car is still being carried is ignored");
        if (S4_PUTDOWN_MS >= 1500) CHECK_SIM(not_early, "still stopped 1 s after being put down: it has to rest for S4_PUTDOWN_MS first");
#if S4_LIFT_LATCH
        CHECK_SIM(latched_after, "S4_LIFT_LATCH = 1: still latched after resting long enough");
        CHECK_SIM(released, "s4_clear_lifted() releases it once the car is resting level");
#else
        CHECK_SIM(released, "released by itself after resting level for S4_PUTDOWN_MS, through s1_clear_fault()");
#endif
        CHECK_SIM(no_ghost, "FAULT_IMU_SENS is still CLEAR 1.2 s after the release (no ghost fault came back)");
        CHECK_SIM(csv_picked == 1 && csv_put_down == 1 && csv_over == 0, "logged once each: PICKED_UP and PUT_DOWN (no OVERTURNED)");
        CHECK_SIM(((s1_car_states_seen >> S4_LIFTED) & 1u) != 0, "car_state 6 (LIFTED) reached subsystem 1's telemetry");
        CHECK_SIM(wifi_bit(), "another subsystem's fault bit untouched");
        CHECK_SIM(!(s4_get_faults() & S4_FAULT_COMM), "no sensor fault");
        return finish();
    }
    if (!strcmp(mode, "overturn") || !strcmp(mode, "flip")) {
        /* overturn: tips onto its side 2.0-2.5 s, lies there until 4.5 s, is set upright 4.5-5.0 s.  flip: turns upside down and stays. */
        int flip = !strcmp(mode, "flip");
        WAIT_UNTIL(1.9);
        int quiet_before = (s4_get_lift() == S4_LIFT_NONE) && !imu_bit() && halt_calls == 0;
        WAIT_UNTIL(flip ? 3.4 : 3.6);
        int over = (s4_get_lift() == S4_LIFT_OVERTURNED), halted = (halt_first_t >= 2.0 && halt_first_t < 3.3), bit_up = imu_bit();
        int shown = (s4_get_motion() == S4_LIFTED), logged = (csv_over == 1 && csv_picked == 0);
        int held = 1, not_early = 1, released = 1;
        if (!flip) {
            WAIT_UNTIL(4.4);  held = (s4_get_lift() == S4_LIFT_OVERTURNED) && imu_bit();      /* lying on its side, quiet and still for 1.5 s */
            WAIT_UNTIL(7.0);  not_early = (s4_get_lift() != S4_LIFT_NONE) && imu_bit();       /* upright from 5 s, but "resting" only from about 6 s: the shaking figure needs about a second to settle after the swing */
            WAIT_UNTIL(9.0);  released = (s4_get_lift() == S4_LIFT_NONE) && !imu_bit();       /* resting from about 6 s + S4_PUTDOWN_MS = about 8 s */
        } else {
            WAIT_UNTIL(5.0);  held = (s4_get_lift() == S4_LIFT_OVERTURNED) && imu_bit();
        }
        printf("\n--- %s ---\n", flip ? "turned upside down at 2.0 s and left there" : "tipped onto its side at 2.0 s, set upright at 4.5 s");
        CHECK_SIM(quiet_before, "nothing fires while the car sits on the floor, and no halt was sent");
        CHECK_SIM(over, "the detector says OVERTURNED (not just PICKED_UP)");
        CHECK_SIM(halted, "the motors were halted soon after it tipped (motion_trigger_safety_halt called)");
        CHECK_SIM(bit_up, "FAULT_IMU_SENS is set through subsystem 1");
        CHECK_SIM(shown, "the motion state shown is LIFTED");
        CHECK_SIM(logged, "logged once as OVERTURNED");
        CHECK_SIM(held, flip ? "still stopped while it lies upside down" : "still stopped while it lies on its side, though quiet and still: it is not upright");
        if (!flip) {
            CHECK_SIM(not_early, "still stopped 2 s after it was set upright: it has to REST for S4_PUTDOWN_MS first, and the shaking figure needs about a second to settle");
            CHECK_SIM(released, "released after it was upright and resting for S4_PUTDOWN_MS");
            CHECK_SIM(csv_put_down == 1, "PUT_DOWN logged once");
        }
        CHECK_SIM(wifi_bit(), "another subsystem's fault bit untouched");
        return finish();
    }
    if (!strcmp(mode, "rough") || !strcmp(mode, "ramp")) {
        /* False-alarm tests: things a driving car meets that are NOT a pick-up.  Nothing may stop it. */
        int ramp = !strcmp(mode, "ramp"), bad = 0;
        double t;
        while ((t = now_s()) < 7.8) {
            if (imu_bit() || s4_get_lift() != S4_LIFT_NONE || halt_calls || s4_get_motion() == S4_LIFTED) bad = 1;
            usleep(5000);
        }
        printf("\n--- %s ---\n", ramp ? "a 30 degree slope, climbed in 1 s and held for 3 s" : "rough ground (0.12 g shaking) with a 1.8 g knock, a 0.9 g vertical bump and a 0.8 g sideways shove");
        CHECK_SIM(s4_is_calibrated() && s4_samples > 300, "calibrated and sampling");
        CHECK_SIM(!bad, ramp ? "the slope never stops the car: no fault, no LIFTED state" : "none of it stops the car: no fault, no LIFTED state");
        CHECK_SIM(halt_calls == 0, "the motors were never halted");
        CHECK_SIM(csv_picked == 0 && csv_over == 0 && csv_put_down == 0, "no PICKED_UP / OVERTURNED / PUT_DOWN logged");
        if (ramp) CHECK_SIM(s1_pitch_max >= 25, "the pitch really did reach about 30 degrees");
        CHECK_SIM(wifi_bit() && !imu_bit(), "the IMU bit stayed clear; another subsystem's bit untouched");
        return finish();
    }
    if (!strcmp(mode, "wrongmount")) {
        WAIT_UNTIL(4.8);        /* calibration has given up by 4.6 s; the scripted knock comes at 5 s */
        printf("\n--- board mounted the other way up, mapping NOT changed ---\n");
        CHECK_SIM(!s4_is_calibrated(), "calibration refuses to run (and says why)");
        CHECK_SIM(!(s4_get_faults() & S4_FAULT_COMM) && !imu_bit(), "the sensor itself is fine: no fault");
        CHECK_SIM(s4_get_lift() == S4_LIFT_NONE && halt_calls == 0, "an uncalibrated car does not stop itself: it cannot tell which way is up, so it reads as upside down but is NOT reported as overturned");
        return finish();
    }
    if (!strcmp(mode, "plugin")) {
        WAIT_UNTIL(2.0);
        int imu_before = imu_bit();
        /* The sensor appears at 3 s and calibration takes about 1 s. */
        WAIT_UNTIL(4.7);
        printf("\n--- sensor plugged in after 3 s (cable fixed while running) ---\n");
        CHECK_SIM(imu_before, "FAULT_IMU_SENS was set while the sensor was missing");
        CHECK_SIM(s4_is_calibrated(), "calibrated by itself once the sensor appeared");
        CHECK_SIM(!(s4_get_faults() & S4_FAULT_COMM) && !imu_bit(), "COMM condition over: FAULT_IMU_SENS cleared again through subsystem 1");
        CHECK_SIM(wifi_bit(), "another subsystem's fault bit untouched");
        CHECK_SIM(s4_samples > 5, "readings flowing again");
        return finish();
    }

    /* ---- the full scripted drive.  still 0-2 s, accelerate, a hard KNOCK at 5.0 (no longer a fault), hump 7.6-9.1, turn 10-11 ---- */
    int seen_stationary = 0, seen_moving = 0, seen_turning = 0, knock_fired = 0;
    int peak_turn = 0;
    double t;
    while ((t = now_s()) < 13.0) {
        S4_MOTION m = s4_get_motion();
        if (t > 1.5 && t < 2.0 && m == S4_STATIONARY) seen_stationary = 1;
        if (t > 4.0 && t < 4.9 && m == S4_MOVING) seen_moving = 1;
        if (t > 10.2 && t < 11.3) { if (m == S4_TURNING) seen_turning = 1; if (s4_get_turn_rate() > peak_turn) peak_turn = s4_get_turn_rate(); }
        if (t > 4.9 && (imu_bit() || s4_get_lift() != S4_LIFT_NONE || halt_calls || m == S4_LIFTED)) knock_fired = 1;   /* anything fired after the knock */
        usleep(5000);
        if (t > 12.9) {            /* report */
            S1_TELEMETRY_PKT tl; s1_get_telemetry(&tl);
            int n = 0, peak_mm = 0, len_mm = 0, max_mm = 0; sscanf(csv_hump_details, "n=%d,peak_mm=%d,len_mm=%d,max_mm=%d", &n, &peak_mm, &len_mm, &max_mm);
            int tol_mm = S4_USE_ENCODER_DISTANCE ? 6 : 12;
            printf("\n--- results (%s) ---\n", mode);
            printf("samples %u, S1 pitch updates %d (max %d, min %d deg), humps %d, run max %d mm, turn peak %d dps\n",
                   (unsigned)s4_samples, s1_pitch_updates, s1_pitch_max, s1_pitch_min, s4_get_hump_count(), s4_get_max_hump_mm(), peak_turn);
            CHECK_SIM(s4_is_calibrated(), "calibration succeeded (offsets removed, car still)");
            CHECK_SIM(s4_samples > 400, "sensor read about 50 times a second");
            CHECK_SIM(seen_stationary && seen_moving, "STATIONARY while standing still, MOVING while cruising");
            CHECK_SIM(s4_get_hump_count() == 1, "exactly one hump detected");
            CHECK_SIM(abs(s4_get_max_hump_mm() - 30) <= tol_mm, S4_USE_ENCODER_DISTANCE ? "peak height within +/-20% of the true 30 mm (distance from subsystem 2's get_distance_traveled_mm)" : "peak height within 12 mm of the true 30 mm (fixed-speed assumption)");
            CHECK_SIM(csv_hump == 1 && abs(peak_mm - 30) <= tol_mm && max_mm == s4_get_max_hump_mm(), "hump logged once through s1_csv_log_event(\"HUMP\"), in mm");
            printf("  -- subsystem 1: telemetry (IF-01) --\n");
            CHECK_SIM(s1_pitch_updates >= 50, "pitch pushed through s1_set_pitch() regularly");
            CHECK_SIM(s1_pitch_max >= 10 && s1_pitch_min <= -10, "nose up gives POSITIVE pitch, nose down NEGATIVE, in the telemetry");
            CHECK_SIM(tl.speed_mm_s == 123 && tl.obstacle_cm == 45 && tl.barcode_id == 7, "speed, obstacle and barcode set by others are NOT overwritten");
            CHECK_SIM((s1_hump_states_seen & 6u) == 6u, "hump_state reported both 1 (going up) and 2 (going down)");
            CHECK_SIM((s1_car_states_seen & ((1u<<S4_STATIONARY)|(1u<<S4_MOVING)|(1u<<S4_TURNING)|(1u<<S4_CLIMBING_HUMP)|(1u<<S4_DESCENDING_HUMP))) ==
                      ((1u<<S4_STATIONARY)|(1u<<S4_MOVING)|(1u<<S4_TURNING)|(1u<<S4_CLIMBING_HUMP)|(1u<<S4_DESCENDING_HUMP)), "car_state reported every motion state we went through (3-bit values)");
            printf("  -- the 1.8 g knock at 5.0 s: collision detection is GONE, so it must stop nothing --\n");
            CHECK_SIM(!knock_fired, "after the knock: no fault, no lift state, no LIFTED motion state, no halt");
            CHECK_SIM(halt_calls == 0, "the motors were never halted during the whole drive");
            CHECK_SIM(csv_picked == 0 && csv_over == 0 && csv_put_down == 0 && csv_fault == 0, "no PICKED_UP / OVERTURNED / PUT_DOWN / IMU_FAULT event was logged");
            CHECK_SIM(!((s1_car_states_seen >> S4_LIFTED) & 1u), "car_state never showed LIFTED");
            CHECK_SIM(!imu_bit() && wifi_bit(), "FAULT_IMU_SENS stayed clear all run; another subsystem's fault bit (WIFI) never disturbed");
            printf("  -- turning --\n");
            CHECK_SIM(seen_turning && peak_turn > 40 && peak_turn < 80, "TURNING seen; turn rate about +60 dps (left is positive)");
            CHECK_SIM(!(s4_get_faults() & S4_FAULT_COMM), "no sensor fault");
            s4_reset_run(); usleep(150000);
            CHECK_SIM(s4_get_hump_count() == 0 && s4_get_max_hump_mm() == 0, "s4_reset_run() clears the highest hump and the count");
            s4_publish_summary(); usleep(100000);
            CHECK_SIM(csv_summary == 1, "run summary logged");
            return finish();
        }
    }
    return finish();
}
