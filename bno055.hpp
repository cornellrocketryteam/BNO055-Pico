#ifndef BNO055_
#define BNO055_

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define BNO055_ADDR (0x28)

#define BNO055_CHIP_ID (0xA0)

#define BNO055_GYRO_X_LSB (0x14)
#define BNO055_MAG_X_LSB (0x0E)
#define BNO055_ACCEL_X_LSB (0x08)

#define BNO055_OPR_MODE (0x3D)
#define BNO055_UNIT_SEL (0x3B)

class BNO055 {
public:
    BNO055(i2c_inst_t *i2c_type);
    bool begin();
    bool read_gyro(double *x, double *y, double *z);
    bool read_mag(double *x, double *y, double *z);
    bool read_accel(double *x, double *y, double *z);

    uint8_t get_id();

private:
    bool read_data(double *x, double *y, double *z, int sensor);

    i2c_inst_t *i2c;
};

#endif