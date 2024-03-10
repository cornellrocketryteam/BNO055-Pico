#ifndef BNO055_HPP
#define BNO055_HPP

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define BNO055_ADDR (0x28)

#define BNO055_CHIP_ID (0xA0)

#define BNO055_ACCEL_X_LSB (0x08)
#define BNO055_GYRO_X_LSB (0x14)
#define BNO055_MAG_X_LSB (0x0E)
#define BNO055_EUL_DATA_X_LSB (0x1A)
#define BNO055_GRV_DATA_X_LSB (0x2E)

#define BNO055_ACC_CONFIG (0x08)
#define BNO055_OPR_MODE (0x3D)
#define BNO055_UNIT_SEL (0x3B)

/**
 * Operating modes of the sensor, as defined in
 * section 3.3 of the BNO055 datasheet.
 */
enum class OpMode : uint8_t {
    ACC_ONLY = 0b00000001,
    MAG_ONLY = 0b00000010,
    GYRO_ONLY = 0b00000011,
    ACC_MAG = 0b00000100,
    ACC_GYRO = 0b00000101,
    MAG_GYRO = 0b00000110,
    AMG = 0b00000111,
    IMU = 0b00001000,
    COMPASS = 0b00001001,
    M4G = 0b00001010,
    NDOF_FMC_OFF = 0b00001011,
    NDOF = 0b00001100
};

/**
 * Representation of the BNO055 sensor.
 */
class BNO055 {
public:
    /**
     * Initializes a BNO055 object on an I2C bus.
     * @param i2c_type The I2C bus that this sensor is on
     */
    BNO055(i2c_inst_t *i2c_type);

    /**
     * Attempts to establish a connection with the sensor and sets configuration options.
     * @return True on successful connection, false otherwise
     */
    bool begin(int g_range = 2, OpMode op_mode = OpMode::IMU);

     /**
     * Reads 3-axis angular rate values, in degrees per second.
     * @param x The resulting x-axis velocity
     * @param y The resulting y-axis velocity
     * @param z The resulting z-axis velocity
     * @return True on successful read, false otherwise
     */
    bool read_gyro(float *x, float *y, float *z);

    /**
     * Reads 3-axis mag acceleration values in meters per second squared.
     * @param x The resulting x-axis acceleration
     * @param y The resulting y-axis acceleration
     * @param z The resulting z-axis acceleration
     * @return True on successful read, false otherwise
     */
    bool read_mag(float *x, float *y, float *z);

    /**
     * Reads 3-axis gyro acceleration values in meters per second squared.
     * @param x The resulting x-axis acceleration
     * @param y The resulting y-axis acceleration
     * @param z The resulting z-axis acceleration
     * @return True on successful read, false otherwise
     */
    bool read_accel(float *x, float *y, float *z);

    /**
     * Reads 3-axis euler angle values in degrees.
     * @param x The resulting x-axis euler angle
     * @param y The resulting y-axis euler angle
     * @param z The resulting z-axis euler angle
     * @return True on successful read, false otherwise
     */
    bool read_orientation(float *x, float *y, float *z);

    /**
     * Reads 3-axis gravity vector values in meters per second squared.
     * @param x The resulting x-axis gravity vector
     * @param y The resulting y-axis gravity vector
     * @param z The resulting z-axis gravity vector
     * @return True on successful read, false otherwise
     */
    bool read_gravity(float *x, float *y, float *z);

private:
    /**
     * Reads the BNO055's electronic ID.
     * @return The resulting electronic ID
     */
    uint8_t get_id();

    /**
     * Reads 3-axis data values in meters per second squared.
     * @param x The resulting x-axis acceleration
     * @param y The resulting y-axis acceleration
     * @param z The resulting z-axis acceleration
     * @param sensor The sensor to query. 0-2 for gyro, mag, accel, respectively
     * @return True on successful read, false otherwise
     */
    bool read_data(float *x, float *y, float *z, int sensor);

    /**
     * Return value for I2C reads and writes.
     */
    int ret;

    /**
     * The I2C bus.
     */
    i2c_inst_t *i2c;
};

#endif // BNO055_HPP