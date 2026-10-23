/*
 *  lsm303d.c  --  LSM303D driver.  Everything here comes from the datasheet.
 *
 *  Talking to the chip means reading and writing its registers over I2C.
 *  Two kernel calls do all the work:
 *
 *    WRITE a register:  tk_swri_dev(dd, address, bytes, size, &actual)
 *         A plain write.  The slave address goes in the "start" argument and
 *         the bytes are {register, value}.  (The same way the kernel's own
 *         i2c_write_reg() does it.)
 *
 *    READ registers:    tk_swri_dev(dd, TDN_I2C_EXEC, &T_I2C_EXEC, ...)
 *         "Execute" = write the register number, then read n bytes back
 *         using a repeated start.  It insists on BOTH parts (send size and
 *         receive size above 0), so it can not be used for plain writes.
 */

#include "lsm303d.h"
#include "s4_config.h"
#include "../../device/include/dev_i2c.h"   /* T_I2C_EXEC, TDN_I2C_EXEC */

/* ---- I2C address (section 6.1.1) ---------------------------------------
 * The address is 0011101b = 0x1D when the SA0 pin is high (the board pulls
 * it high), and 0011110b = 0x1E when SA0 is tied to ground. */
#define LSM303D_ADDR        0x1D

/* ---- Registers (Table 16) ---------------------------------------------- */
#define REG_OUT_X_L_M       0x08    /* magnetometer X,Y,Z: 6 bytes, low byte first */
#define REG_WHO_AM_I        0x0F    /* identification: reads 0x49 (Table 19)       */
#define REG_CTRL1           0x20
#define REG_CTRL2           0x21
#define REG_CTRL5           0x24
#define REG_CTRL6           0x25
#define REG_CTRL7           0x26
#define REG_OUT_X_L_A       0x28    /* accelerometer X,Y,Z: 6 bytes, low byte first */

#define WHO_AM_I_VALUE      0x49

/* Setting the top bit of the register number makes the chip step to the
 * next register after each byte, so 6 bytes come out in one transfer
 * (section 6.1.1: "the MSb enables address auto-increment"). */
#define AUTO_INCREMENT      0x80

/* ---- Configuration values ----------------------------------------------
 * CTRL1 = 0x6F : 0110 1 111
 *                |||| | +++-- AZEN, AYEN, AXEN: X, Y and Z enabled
 *                |||| +------ BDU = 1: output registers are not updated until
 *                ||||         both bytes of a value have been read
 *                ++++-------- AODR = 0110: 100 Hz output rate    (Table 36)
 * CTRL2 = 0x08 : AFS = 001: acceleration range +/-4 g             (Table 40)
 * CTRL5 = 0x70 : M_RES = 11 (high resolution), M_ODR = 100 (50 Hz) (Table 47)
 * CTRL6 = 0x20 : MFS = 01: magnetic range +/-4 gauss               (Table 50)
 * CTRL7 = 0x00 : MD = 00: magnetometer in continuous conversion    (Table 54)
 *                (the magnetometer is OFF until this is written) */
#define CTRL1_VALUE         0x6F
#define CTRL2_VALUE         0x08
#define CTRL5_VALUE         0x70
#define CTRL6_VALUE         0x20
#define CTRL7_VALUE         0x00

/* ---- Scale factors (Table 3) ------------------------------------------- */
#define ACCEL_G_PER_LSB     0.000122f   /* +/-4 g     : 0.122 mg per count  */
#define MAG_GAUSS_PER_LSB   0.00016f    /* +/-4 gauss : 0.160 mgauss/count  */

static ID dd = 0;       /* the open I2C device (0 = not open yet) */

/* Write one register. */
static ER write_reg(UB reg, UB value)
{
    UB  buf[2];
    SZ  actual = 0;

    buf[0] = reg;
    buf[1] = value;
    return tk_swri_dev(dd, LSM303D_ADDR, buf, 2, &actual);
}

/* Read n registers, starting at reg. */
static ER read_regs(UB reg, UB *dest, SZ n)
{
    T_I2C_EXEC  ex;
    UB          first = reg;
    SZ          actual = 0;

    ex.sadr     = LSM303D_ADDR;
    ex.snd_size = 1;            /* send one byte: the register number  */
    ex.snd_data = &first;
    ex.rcv_size = n;            /* then read n bytes back              */
    ex.rcv_data = dest;
    return tk_swri_dev(dd, TDN_I2C_EXEC, &ex, sizeof(ex), &actual);
}

/* Turn 2 bytes (low byte first) into a signed 16-bit number. */
static INT to_int16(const UB *p)
{
    return (INT)(H)((UH)p[0] | ((UH)p[1] << 8));
}

ER lsm303d_init(void)
{
    UB  id = 0;
    ER  err;

    if (dd <= 0) {
        dd = tk_opn_dev((UB*)S4_I2C_NAME, TD_UPDATE);
        if (dd < E_OK) {
            err = dd;
            dd = 0;
            return err;
        }
    }

    /* Is it really an LSM303D?  Nothing answering gives E_IO. */
    err = read_regs(REG_WHO_AM_I, &id, 1);
    if (err != E_OK) {
        return err;
    }
    if (id != WHO_AM_I_VALUE) {
        return E_NOSPT;
    }

    /* Configure.  Stop at the first write that fails. */
    err = write_reg(REG_CTRL1, CTRL1_VALUE);
    if (err == E_OK) err = write_reg(REG_CTRL2, CTRL2_VALUE);
    if (err == E_OK) err = write_reg(REG_CTRL5, CTRL5_VALUE);
    if (err == E_OK) err = write_reg(REG_CTRL6, CTRL6_VALUE);
    if (err == E_OK) err = write_reg(REG_CTRL7, CTRL7_VALUE);
    return err;
}

ER lsm303d_read_accel(float g[3])
{
    UB      raw[6];
    ER      err;
    UINT    axis;

    err = read_regs(REG_OUT_X_L_A | AUTO_INCREMENT, raw, 6);
    if (err != E_OK) {
        return err;
    }
    for (axis = 0; axis < 3; axis++) {
        g[axis] = (float)to_int16(&raw[2 * axis]) * ACCEL_G_PER_LSB;
    }
    return E_OK;
}

ER lsm303d_read_mag(float gauss[3])
{
    UB      raw[6];
    ER      err;
    UINT    axis;

    err = read_regs(REG_OUT_X_L_M | AUTO_INCREMENT, raw, 6);
    if (err != E_OK) {
        return err;
    }
    for (axis = 0; axis < 3; axis++) {
        gauss[axis] = (float)to_int16(&raw[2 * axis]) * MAG_GAUSS_PER_LSB;
    }
    return E_OK;
}
