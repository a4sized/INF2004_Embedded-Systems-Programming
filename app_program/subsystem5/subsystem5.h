/**
 * @file    subsystem5.h
 * @brief   Subsystem 5 ultrasonic sensor lifecycle.
 */
#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include <tk/tkernel.h>
#include "drv_ultrasonic.h"

ER subsystem5_init(void);
ER subsystem5_start(void);

#endif /* SUBSYSTEM5_H */
