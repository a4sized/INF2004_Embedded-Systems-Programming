/*
 *  lsm303d.h  --  driver for the LSM303D accelerometer + magnetometer
 *                 (the chip on the "GY-511" board in our kit).
 *
 *  Datasheet: ST LSM303D, DocID023312 Rev 2.  Table numbers below refer to it.
 *  The chip has ONE I2C address for both sensors.  Only imu_task may call
 *  these functions: it is the single owner of the I2C bus.
 */

#ifndef LSM303D_H
#define LSM303D_H

#include "../app.h"

/* Open the I2C device, check the chip is an LSM303D and configure it.
 * Returns E_OK, or a negative error: E_IO = nothing answered,
 * E_NOSPT = something answered but it is not an LSM303D. */
IMPORT ER lsm303d_init(void);

/* Acceleration in g, in SENSOR axes (X, Y, Z). */
IMPORT ER lsm303d_read_accel(float g[3]);

/* Magnetic field in gauss, in SENSOR axes (X, Y, Z). */
IMPORT ER lsm303d_read_mag(float gauss[3]);

#endif /* LSM303D_H */
