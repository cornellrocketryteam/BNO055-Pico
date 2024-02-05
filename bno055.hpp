#ifndef BNO055_HPP
#define BNO055_HPP

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define BNO055_ADDR (0x28)

#define BNO055_CHIP_ID (0xA0)

#define BNO055_GYRO_X_LSB (0x14)
#define BNO055_MAG_X_LSB (0x0E)
#define BNO055_ACCEL_X_LSB (0x08)

#define BNO055_OPR_MODE (0x3D)
#define BNO055_UNIT_SEL (0x3B)

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
    bool begin();

     /**
     * Reads 3-axis gyro acceleration values in meters per second squared.
     * @param x The resulting x-axis acceleration
     * @param y The resulting y-axis acceleration
     * @param z The resulting z-axis acceleration
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