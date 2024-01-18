#include "../bno055.hpp"
#include "tusb.h"

#define I2C_PORT i2c0
#define I2C_SDA 12
#define I2C_SCL 13

BNO055 imu(I2C_PORT);

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    while (!tud_cdc_connected()) {
        sleep_ms(500);
    }
    printf("Connected\n");

    if (imu.begin()) {
        printf("Init successful\n");
    } else {
        printf("Init failed\n");
        return 1;
    }

    double gyro_x, gyro_y, gyro_z;
    double mag_x, mag_y, mag_z;
    double accel_x, accel_y, accel_z;

    while (true) {

        imu.read_gyro(&gyro_x, &gyro_y, &gyro_z);
        imu.read_mag(&mag_x, &mag_y, &mag_z);
        imu.read_accel(&accel_x, &accel_y, &accel_z);
        
        printf("Gyro X: %.3f\n", gyro_x);
        printf("Gyro Y: %.3f\n", gyro_y);
        printf("Gyro Z: %.3f\n\n", gyro_z);

        printf("Mag X: %.3f\n", mag_x);
        printf("Mag Y: %.3f\n", mag_y);
        printf("Mag Z: %.3f\n\n", mag_z);

        printf("Accel X: %6.2f\n", accel_x);
        printf("Accel Y: %6.2f\n", accel_y);
        printf("Accel Z: %6.2f\n\n", accel_z);

        sleep_ms(200);
    }

}