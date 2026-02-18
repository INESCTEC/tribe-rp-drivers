#pragma once
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "hardware/i2c.h"

#include "rp_agrolib_i2c.h"
extern "C" {
#include "bno055.h"
}

#define BNO055_MAX_I2C_BUFFER_SIZE 512


struct bno055_quaternion_double_t {
    double w;
    double x;
    double y;
    double z;
};

static i2c_inst_t *bno_i2c_inst_;

static s8 BNO055_I2C_BusWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static s8 BNO055_I2C_BusRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static void BNO055_Delay_ms(u32 msecs);
static void bno055_convert_double_quaternion_wxyz(struct bno055_quaternion_double_t *quat_wxyz);


class BNO055 {
private:
    i2c_inst_t *i2c_inst_;
    struct bno055_t dev_def_;
    u8 sda_pin_;
    u8 scl_pin_;
    int baudrate_;
    u8 i2c_addr_;
    bool use_internal_pullups_;

public:
    BNO055(i2c_inst_t *i2c_inst, const u8 &sda_pin, const u8 &scl_pin, const int &baudrate,
           const u8 &i2c_addr, const bool &use_internal_pullups);
    ~BNO055();

    bool setup(u8 power_mode, u8 operation_mode);
    bool suspend();

    // getters
    struct bno055_accel_double_t getAccel();
    struct bno055_mag_double_t getMag();
    struct bno055_gyro_double_t getGyro();
    struct bno055_euler_double_t getEuler();
    struct bno055_linear_accel_double_t getLinearAccel();
    struct bno055_gravity_double_t getGravity();
    struct bno055_quaternion_double_t getQuaternion();
    double getTemperature();
};
