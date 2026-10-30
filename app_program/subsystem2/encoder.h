/*
 *  encoder.h  --  wheel encoder counting for Subsystem 2 (quadrature, x4).
 *
 *  WHAT IT DOES
 *    Each wheel has an encoder with two channels, A and B.  When the wheel
 *    turns, A and B change in a fixed order, and the ORDER tells the
 *    direction:
 *
 *        forward :  00 -> 01 -> 11 -> 10 -> 00 ...      (state = A*2 + B)
 *        backward:  00 -> 10 -> 11 -> 01 -> 00 ...
 *
 *    A GPIO interrupt fires on EVERY edge of all four pins.  The interrupt
 *    reads the pins, compares them with the previous state, and adds +1 or
 *    -1 to that wheel's counter.  Counting every edge of both channels is
 *    called "x4 decoding": one wheel turn gives 4 x (pulses per turn on
 *    channel A) ticks.
 *
 *  WHY AN INTERRUPT
 *    A geared motor can produce thousands of edges a second.  The motor task
 *    only runs every 20 ms, and a 20 ms look at the pins would miss most of
 *    them.
 *
 *  WHO CAN USE IT
 *    Only the getters below are for other subsystems (Subsystem 4 reads them
 *    to measure hump height).  Both counters go UP when the car drives
 *    forward; if one wheel counts the wrong way, flip its sign below.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "../app.h"
#include <stdint.h>
#include "subsystem2.h"         /* LEFT_ENC_A/B and RIGHT_ENC_A/B pin numbers */

/* ---- Settings (change these, nothing else) ------------------------------ */

/* +1 or -1.  If a wheel's counter goes DOWN when the car drives forward,
 * change its sign.  Check each wheel by turning it forward by hand. */
#ifndef ENC_LEFT_SIGN
#define ENC_LEFT_SIGN       (-1)
#endif
#ifndef ENC_RIGHT_SIGN
#define ENC_RIGHT_SIGN      (-1)
#endif

/* 1 = switch on the pin's internal pull-up (and switch off the pull-down that
 * the RP2040 has after reset).  Harmless if the encoder board has its own
 * pull-ups.  Needed if the encoder outputs are open-collector. */
#ifndef ENC_PULL_UP
#define ENC_PULL_UP         1
#endif

/* Print the counters on the serial monitor this often (ms).  0 = never. */
#define ENC_PRINT_MS        500

/* Interrupt priority (same as the I2C driver; the handler is very short). */
#define ENC_INTPRI          2

/* ---- RP2040 facts the code relies on ------------------------------------
 * Each value was checked against the Pico SDK headers (io_bank0.h, sio.h,
 * pads_bank0.h, intctrl.h). */
#define ENC_INTNO           13U             /* IO_IRQ_BANK0: all GPIO pin interrupts */

#define ENC_IO_BANK0        0x40014000UL
#define ENC_PADS_BANK0      0x4001C000UL
#define ENC_SIO             0xD0000000UL
#define ENC_SIO_CPUID       (ENC_SIO + 0x000UL)     /* 0 on core 0, 1 on core 1 */
#define ENC_SIO_GPIO_IN     (ENC_SIO + 0x004UL)     /* all 30 pin levels at once */

/* Every pin has 4 interrupt bits in a nibble: level low, level high, edge low,
 * edge high.  Eight pins share one 32-bit register, so the register is pin/8
 * and the nibble is pin%8.  We use the two EDGE bits (0xC). */
#define ENC_INTR_REG(bank)        (ENC_IO_BANK0 + 0x0F0UL + 4UL * (bank))                          /* raw flags: write 1 to clear */
#define ENC_INTE_REG(core, bank)  (ENC_IO_BANK0 + ((core) ? 0x130UL : 0x100UL) + 4UL * (bank))     /* enable: core 0 or core 1  */
#define ENC_EDGES(pin)            (0xCUL << (4U * ((pin) % 8U)))

#define ENC_PAD(pin)        (ENC_PADS_BANK0 + 0x04UL + 4UL * (pin))
#define ENC_PAD_PUE         0x08UL          /* pull-up enable   */
#define ENC_PAD_PDE         0x04UL          /* pull-down enable */

/* ---- The decoding rule --------------------------------------------------
 * Index = (previous state * 4) + new state, where state = A*2 + B.
 * Forward steps (0->1->3->2->0) give +1, backward steps give -1.  "No change"
 * and an impossible jump (both channels changed at once, which means an edge
 * was missed or the signal bounced) give 0: they are ignored. */
static const int8_t enc_qdec[16] = {
    /* prev 00 -> new 00  01  10  11 */   0, +1, -1,  0,
    /* prev 01 -> new 00  01  10  11 */  -1,  0,  0, +1,
    /* prev 10 -> new 00  01  10  11 */  +1,  0,  0, -1,
    /* prev 11 -> new 00  01  10  11 */   0, -1, +1,  0
};

/* ---- Functions ----------------------------------------------------------- */

/* Set up the pins and the interrupt.  Call once, from motor_init(), AFTER the
 * four pins have been set to input.  Returns E_OK or a kernel error. */
IMPORT ER encoder_init(void);

IMPORT int32_t encoder_get_left(void);
IMPORT int32_t encoder_get_right(void);
IMPORT void    encoder_reset(void);             /* both counters back to 0 */
IMPORT UW      encoder_irq_count(void);         /* interrupts so far: 0 means the wiring or setup is wrong */

/* Odometry Hardware Constants */
#define WHEEL_DIAMETER_MM       66.0f   /* Replace with your exact wheel diameter */
#define ENCODER_TPR             360.0f  /* Replace with your exact motor TPR */
#define PI_APPROX               3.14159265f
#define MM_PER_TICK             ((PI_APPROX * WHEEL_DIAMETER_MM) / ENCODER_TPR)

IMPORT float get_distance_traveled_mm(void);

/* The interrupt handler and its core.  Public ONLY so the PC test can call
 * them; nothing else should. */
IMPORT void encoder_isr(UINT intno);
IMPORT void encoder_service(UW gpio_in);        /* gpio_in = the 30 pin levels */

#endif /* ENCODER_H */
