#include "bno055.hpp"
#include <cstdio>

BNO055::BNO055(i2c_inst_t *i2c_type) {
    i2c = i2c_type;
}

bool BNO055::begin(int g_range, OpMode op_mode) {
    uint8_t id = get_id();
    if (id != BNO055_CHIP_ID) {
#ifdef VERBOSE
        fprintf(stderr, "Error: IMU got chip ID value of %d\n", id);
#endif
        return false;
    }

    if (!set_op_mode(op_mode)) {
#ifdef VERBOSE
        fprintf(stderr, "Error: IMU could not set operating mode");
#endif
        return false;
    }

    uint8_t config[2];

    config[0] = BNO055_ACC_CONFIG;
    switch (g_range) {
    case 2:
        config[1] = 0b00000000;
        break;
    case 4:
        config[1] = 0b00000001;
        break;
    case 8:
        config[1] = 0b00000010;
        break;
    case 16:
        config[1] = 0b00000011;
        break;
    default:
#ifdef VERBOSE
        fprintf(stderr, "Error: Invalid G range for IMU\n");
#endif
        return false;
    }

    if (i2c_write_timeout_us(i2c, BNO055_ADDR, config, 2, true, 2 * BYTE_TIMEOUT_US) < 1) {
        return false;
    }
    sleep_ms(30);

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

bool BNO055::read_orientation(float *x, float *y, float *z) {
    return read_data(x, y, z, 3);
}

bool BNO055::read_gravity(float *x, float *y, float *z) {
    return read_data(x, y, z, 4);
}

bool BNO055::read_data(float *x, float *y, float *z, int sensor) {
    int16_t data_x, data_y, data_z;
    uint8_t data_b[6];

    uint8_t reg;
    switch (sensor) {
    default:
    case 0:
        reg = BNO055_GYRO_X_LSB;
        break;
    case 1:
        reg = BNO055_MAG_X_LSB;
        break;
    case 2:
        reg = BNO055_ACCEL_X_LSB;
        break;
    case 3:
        reg = BNO055_EUL_DATA_X_LSB;
        break;
    case 4:
        reg = BNO055_GRV_DATA_X_LSB;
        break;
    }

    if (i2c_write_timeout_us(i2c, BNO055_ADDR, &reg, 1, true, BYTE_TIMEOUT_US) < 1) {
        return false;
    }
    if (i2c_read_timeout_us(i2c, BNO055_ADDR, data_b, 6, false, 6 * BYTE_TIMEOUT_US) < 1) {
        return false;
    }

    data_x = (data_b[1] << 8) | data_b[0];
    data_y = (data_b[3] << 8) | data_b[2];
    data_z = (data_b[5] << 8) | data_b[4];

    *x = (float)data_x / 100.0;
    *y = (float)data_y / 100.0;
    *z = (float)data_z / 100.0;

    return true;
}

bool BNO055::set_op_mode(OpMode op_mode) {
    uint8_t config[2];
    config[0] = BNO055_OPR_MODE;
    config[1] = static_cast<uint8_t>(op_mode);
    if (i2c_write_timeout_us(i2c, BNO055_ADDR, config, 2, true, 2 * BYTE_TIMEOUT_US) < 1) {
        return false;
    }
    sleep_ms(100); // TODO: Look at this

    return true;
}

uint8_t BNO055::get_id() {
    uint8_t reg = BNO055_CHIP_ID;
    uint8_t val;

    i2c_write_timeout_us(i2c, BNO055_ADDR, &reg, 1, true, BYTE_TIMEOUT_US);
    i2c_read_timeout_us(i2c, BNO055_ADDR, &val, 1, false, BYTE_TIMEOUT_US);

    return val;
}