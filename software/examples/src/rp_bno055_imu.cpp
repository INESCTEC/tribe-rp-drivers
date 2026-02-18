#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "rp_agrolib_bno055.h"

#define I2C_ID i2c0
#define I2C_ADDR 0x28
#define SDA_PIN 16
#define SCL_PIN 17
#define BAUDRATE 400000


int main() {
    stdio_init_all();

    BNO055 imu(I2C_ID, SDA_PIN, SCL_PIN, BAUDRATE, I2C_ADDR, true);

    // setup sensor with power and operation modes
    imu.setup(BNO055_POWER_MODE_NORMAL, BNO055_OPERATION_MODE_NDOF);

    struct bno055_accel_double_t d_accel_xyz;
    struct bno055_mag_double_t d_mag_xyz;
    struct bno055_gyro_double_t d_gyro_xyz;
    struct bno055_euler_double_t d_euler_hpr;
    struct bno055_linear_accel_double_t d_lin_accel_xyz;
    struct bno055_gravity_double_t d_gravity_xyz;
    struct bno055_quaternion_double_t d_quat_wxyz;
    double temp_c;

    while (1) {
        d_accel_xyz = imu.getAccel();
        d_mag_xyz = imu.getMag();
        d_gyro_xyz = imu.getGyro();
        d_euler_hpr = imu.getEuler();
        d_lin_accel_xyz = imu.getLinearAccel();
        d_gravity_xyz = imu.getGravity();
        d_quat_wxyz = imu.getQuaternion();
        temp_c = imu.getTemperature();

        printf("Accel (xyz)     = %.3f,%.3f,%.3f\n", d_accel_xyz.x, d_accel_xyz.y, d_accel_xyz.z);
        printf("Mag (xyz)       = %.3f,%.3f,%.3f\n", d_mag_xyz.x, d_mag_xyz.y, d_mag_xyz.z);
        printf("Gyro (xyz)      = %.3f,%.3f,%.3f\n", d_gyro_xyz.x, d_gyro_xyz.y, d_gyro_xyz.z);
        printf("Euler (rph)     = %.3f,%.3f,%.3f\n", d_euler_hpr.r, d_euler_hpr.p, d_euler_hpr.h);
        printf("LinAccel (xyz)  = %.3f,%.3f,%.3f\n", d_lin_accel_xyz.x, d_lin_accel_xyz.y,
               d_lin_accel_xyz.z);
        printf("Gravity (xyz)   = %.3f,%.3f,%.3f\n", d_gravity_xyz.x, d_gravity_xyz.y,
               d_gravity_xyz.z);
        printf("Quat (wxyz)      = %.3f,%.3f,%.3f,%.3f\n", d_quat_wxyz.w, d_quat_wxyz.x,
               d_quat_wxyz.y, d_quat_wxyz.z);
        printf("Temp            = %.2f C\n\n", temp_c);
        sleep_ms(100);
    }

    imu.suspend();

    return 0;
}
