/**
 * @file    subsystem5.h
 * @brief   Subsystem 5: servo-mounted ultrasonic obstacle scanner.
 */
#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include <tk/tkernel.h>

/* Set up the servo and ultrasonic sensor (called once at startup). */
ER subsystem5_init(void);

/* Start the scanning task (called after every subsystem's init). */
ER subsystem5_start(void);

#endif /* SUBSYSTEM5_H */
