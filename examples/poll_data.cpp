#include "../bno055.hpp"
#include "tusb.h"

#define I2C_PORT i2c0
#define I2C_SDA 12
#define I2C_SCL 13

BNO055 imu(I2C_PORT);

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    while (!tud_cdc_connected()) {
        sleep_ms(500);
    }
    printf("Connected\n");

    if (imu.begin(2, OpMode::IMU)) {
        printf("Init successful\n");
    } else {
        printf("Init failed\n");
        return 1;
    }

    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float orientation_x, orientation_y, orientation_z;
    float grav_x, grav_y, grav_z;

    while (true) {

        imu.read_gyro(&gyro_x, &gyro_y, &gyro_z);
        imu.read_accel(&accel_x, &accel_y, &accel_z);
        imu.read_orientation(&orientation_x, &orientation_y, &orientation_z);
        imu.read_gravity(&grav_x, &grav_y, &grav_z);

        printf("Gyro X: %.3f\n", gyro_x);
        printf("Gyro Y: %.3f\n", gyro_y);
        printf("Gyro Z: %.3f\n\n", gyro_z);

        printf("Accel X: %6.2f\n", accel_x);
        printf("Accel Y: %6.2f\n", accel_y);
        printf("Accel Z: %6.2f\n\n", accel_z);

        printf("Orientation X: %6.2f\n", orientation_x);
        printf("Orientation Y: %6.2f\n", orientation_y);
        printf("Orientation Z: %6.2f\n\n", orientation_z);

        printf("Gravity X: %6.2f\n", grav_x);
        printf("Gravity Y: %6.2f\n", grav_y);
        printf("Gravity Z: %6.2f\n\n", grav_z);

        printf("----------------------------------------\n\n");

        sleep_ms(20);
    }
}