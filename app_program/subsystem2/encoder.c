/*
 *  encoder.c  --  wheel encoder counting for Subsystem 2.  See encoder.h.
 *
 *  Read it in this order:
 *    1. the counters         (what the rest of the system sees)
 *    2. encoder_service()    (turns pin levels into +1 / -1)
 *    3. encoder_isr()        (what runs on every edge)
 *    4. encoder_init()       (switches it all on)
 */

#include "encoder.h"
#include <tk/syslib.h>              /* EnableInt, ClearInt */

/* Every hardware register is touched through this one macro, so the PC test
 * (testing/subsystem2/encoder_host_test) can swap in a fake register file.
 * On the Pico it is a plain memory access. */
#ifndef ENC_REG
#define ENC_REG(addr)   (*(volatile UW *)(addr))
#endif

/* ======================================================================
 *  1. The counters
 *
 *  Only the interrupt adds to them (and encoder_reset() zeroes them).  A
 *  32-bit read is a single instruction, so a reader on the other core always
 *  sees a whole value.
 * ====================================================================== */
static volatile int32_t enc_left;
static volatile int32_t enc_right;
static volatile UW      enc_irq;            /* interrupts so far (for the wiring check) */
static UW               enc_prev_l;         /* last state of each wheel: A*2 + B */
static UW               enc_prev_r;

int32_t encoder_get_left(void)
{
    return enc_left;
}

int32_t encoder_get_right(void)
{
    return enc_right;
}

UW encoder_irq_count(void)
{
    return enc_irq;
}

void encoder_reset(void)
{
    UINT intsts;

    DI(intsts);                             /* keep the interrupt out while we write */
    enc_left  = 0;
    enc_right = 0;
    EI(intsts);
}

float get_distance_traveled_mm(void)
{
    int32_t ticks_l, ticks_r;
    UINT intsts;

    /* 1. Atomically sample both encoders at the exact same microsecond */
    DI(intsts);
    ticks_l = enc_left;
    ticks_r = enc_right;
    EI(intsts);

    /* 2. Convert ticks to millimeters */
    float dist_l = (float)ticks_l * MM_PER_TICK;
    float dist_r = (float)ticks_r * MM_PER_TICK;

    /* 3. Return the average (center-of-robot displacement) */
    return (dist_l + dist_r) / 2.0f;
}

/* ======================================================================
 *  2. Pin levels -> counts
 *
 *  gpio_in holds the level of all 30 pins, bit n = pin n.  Build each wheel's
 *  2-bit state from its A and B pins, look up the step in the table, and add
 *  it (times the wheel's sign) to the counter.
 * ====================================================================== */
void encoder_service(UW gpio_in)
{
    UW state_l = (((gpio_in >> LEFT_ENC_A)  & 1U) << 1) | ((gpio_in >> LEFT_ENC_B)  & 1U);
    UW state_r = (((gpio_in >> RIGHT_ENC_A) & 1U) << 1) | ((gpio_in >> RIGHT_ENC_B) & 1U);

    enc_left  += ENC_LEFT_SIGN  * (int32_t)enc_qdec[(enc_prev_l << 2) | state_l];
    enc_right += ENC_RIGHT_SIGN * (int32_t)enc_qdec[(enc_prev_r << 2) | state_r];
    enc_prev_l = state_l;
    enc_prev_r = state_r;
}

/* The four pins' edge bits, grouped by register: bank = pin / 8.  Our pins
 * (2, 3, 14, 15) sit in two registers, so two of the four masks are non-zero.
 * The compiler works this out at build time. */
static void enc_bank_masks(UW mask[4])
{
    mask[0] = 0U; mask[1] = 0U; mask[2] = 0U; mask[3] = 0U;
    mask[LEFT_ENC_A  / 8U] |= ENC_EDGES(LEFT_ENC_A);
    mask[LEFT_ENC_B  / 8U] |= ENC_EDGES(LEFT_ENC_B);
    mask[RIGHT_ENC_A / 8U] |= ENC_EDGES(RIGHT_ENC_A);
    mask[RIGHT_ENC_B / 8U] |= ENC_EDGES(RIGHT_ENC_B);
}

/* ======================================================================
 *  3. The interrupt: runs on every edge of any of the four pins
 *
 *  Order matters.  First CLEAR our edge flags, THEN read the pins.  If an edge
 *  arrives between the two, it sets its flag again and we are called once
 *  more; the extra call just finds nothing new.  Reading first and clearing
 *  second could throw an edge away.
 * ====================================================================== */
void encoder_isr(UINT intno)
{
    UW mask[4];
    UW in, bank;

    /* Writing 1 to a bit of INTR clears that edge flag (0 bits are ignored). */
    enc_bank_masks(mask);
    for (bank = 0; bank < 4U; bank++) {
        if (mask[bank] != 0U) {
            ENC_REG(ENC_INTR_REG(bank)) = mask[bank];
        }
    }

    in = ENC_REG(ENC_SIO_GPIO_IN);
    enc_irq++;
    encoder_service(in);

    ClearInt(intno);
}

/* ======================================================================
 *  4. Switch it on
 *
 *  GPIO interrupts of the RP2040 are routed to ONE core at a time, chosen by
 *  which set of "enable" registers (PROC0 or PROC1) we write.  The kernel's
 *  EnableInt() enables the interrupt in the NVIC of the core that CALLS it.
 *  So we ask which core we are on (SIO CPUID) and use that core's registers:
 *  then it works wherever usermain() runs.
 * ====================================================================== */
ER encoder_init(void)
{
    static const UW pins[4] = { LEFT_ENC_A, LEFT_ENC_B, RIGHT_ENC_A, RIGHT_ENC_B };
    T_DINT  dint;
    ER      err;
    UW      mask[4];
    UW      core, in, i;

    core = ENC_REG(ENC_SIO_CPUID) & 1U;
    enc_bank_masks(mask);

#if ENC_PULL_UP
    for (i = 0; i < 4U; i++) {
        UW pad = ENC_REG(ENC_PAD(pins[i]));
        ENC_REG(ENC_PAD(pins[i])) = (pad & ~ENC_PAD_PDE) | ENC_PAD_PUE;
    }
#else
    (void)pins;
#endif
    for (i = 0; i < 4U; i++) {
        if (mask[i] != 0U) {
            ENC_REG(ENC_INTR_REG(i)) = mask[i];                 /* forget old edges */
        }
    }

    /* Start from the pins' present levels, so the first edge is judged
     * against the truth and not against zero. */
    in = ENC_REG(ENC_SIO_GPIO_IN);
    enc_prev_l = (((in >> LEFT_ENC_A)  & 1U) << 1) | ((in >> LEFT_ENC_B)  & 1U);
    enc_prev_r = (((in >> RIGHT_ENC_A) & 1U) << 1) | ((in >> RIGHT_ENC_B) & 1U);
    encoder_reset();
    enc_irq = 0U;

    /* Tell the kernel which function to run for interrupt 13 (GPIO). */
    dint.intatr = TA_HLNG;
    dint.inthdr = encoder_isr;
    err = tk_def_int(ENC_INTNO, &dint);
    if (err != E_OK) {
        return err;
    }

    /* Let the four pins' EDGE events (both rising and falling) raise it. */
    for (i = 0; i < 4U; i++) {
        if (mask[i] != 0U) {
            ENC_REG(ENC_INTE_REG(core, i)) |= mask[i];
        }
    }

    ClearInt(ENC_INTNO);
    EnableInt(ENC_INTNO, ENC_INTPRI);
    return E_OK;
}
