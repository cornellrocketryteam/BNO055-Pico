#include "bno055.hpp"
#include <cstdio>


BNO055::BNO055(i2c_inst_t *i2c_type) {
    i2c = i2c_type;
}

bool BNO055::begin() {
    uint8_t config[2];
    uint8_t id = get_id();

    if (id != 0xA0) {
        fprintf(stderr, "Error: Got chip id value of %d\n", id);
        return false;
    }

    config[0] = BNO055_UNIT_SEL;
    config[1] = 0b0001000;
    i2c_write_blocking(i2c, BNO055_ADDR, config, 2, true);
    sleep_ms(30);

    config[0] = BNO055_OPR_MODE;
    config[1] = 0b00000111;
    i2c_write_blocking(i2c, BNO055_ADDR, config, 2, true);
    sleep_ms(100);

    return true;
}

bool BNO055::read_gyro(float *x, float *y, float *z) {
    return read_data(x, y, z, 0);
}

bool BNO055::read_mag(float *x, float *y, float *z) {
    return read_data(x, y, z, 1);
}

bool BNO055::read_accel(float *x, float *y, float *z) {
    return read_data(x, y, z, 2);
}

bool BNO055::read_data(float *x, float *y, float *z, int sensor) {
    int16_t data_x, data_y, data_z;
    uint8_t data_b[6];

    uint8_t reg[1];
    if (sensor == 0) {
        reg[0] = BNO055_GYRO_X_LSB;
    }
    else if (sensor == 1) {
        reg[0] = BNO055_MAG_X_LSB;
    }
    else {
        reg[0] = BNO055_ACCEL_X_LSB;
    }

    i2c_write_blocking(i2c, BNO055_ADDR, reg, 1, true);
    i2c_read_blocking(i2c, BNO055_ADDR, data_b, 6, false);

    data_x = (data_b[1] << 8) | data_b[0];
    data_y = (data_b[3] << 8) | data_b[2];
    data_z = (data_b[5] << 8) | data_b[4];

    *x = (float)data_x / 100.0;
    *y = (float)data_y / 100.0;
    *z = (float)data_z / 100.0;

    return true;
}

uint8_t BNO055::get_id() {
    uint8_t reg[1] = { BNO055_CHIP_ID };
    uint8_t val[1];

    i2c_write_blocking(i2c, BNO055_ADDR, reg, 1, true);
    i2c_read_blocking(i2c, BNO055_ADDR, val, 1, false);

    return val[0];
}