/** @file drv_ir.h
 * @brief Digital input owned exclusively by the IR bench task.
 */
#ifndef DRV_IR_H
#define DRV_IR_H

#include <stdbool.h>

#define DRV_IR_PIN 6u

/** @brief Configure GP6 as an input; report BSP configuration success. */
bool drv_ir_init (void);

/** @brief Read the electrical level, without assuming black polarity. */
bool drv_ir_is_high (void);

#endif

/* End of file. */

