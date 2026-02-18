#include "../include/rp_agrolib_bno055.h"

s8 BNO055_I2C_BusWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    s8 ret = BNO055_SUCCESS;
    u8 buf[BNO055_MAX_I2C_BUFFER_SIZE];

    if ((cnt + 1) > BNO055_MAX_I2C_BUFFER_SIZE) {
        ret = BNO055_ERROR;
        return ret;
    }

    // prepare buffer
    buf[0] = reg_addr;  // set register address
    for (u8 datapos = 0; datapos < cnt; ++datapos)
        buf[datapos + 1] = reg_data[datapos];  // copy data to buffer

    // try to write data to bno055
    if (i2c_write_blocking(bno_i2c_inst_, dev_addr, buf, cnt + 1, true) != (cnt + 1))
        ret = BNO055_ERROR;

    return ret;
}

s8 BNO055_I2C_BusRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    s8 ret = BNO055_SUCCESS;
    u8 buf[BNO055_MAX_I2C_BUFFER_SIZE];

    if (cnt > BNO055_MAX_I2C_BUFFER_SIZE) {
        ret = BNO055_ERROR;
        return ret;
    }

    // set register address to write to
    buf[0] = reg_addr;
    if (i2c_write_blocking(bno_i2c_inst_, dev_addr, buf, 1, true) !=
        1)  // if can't set register address
    {
        ret = BNO055_ERROR;
        return ret;
    }

    // try to read data from bno055
    if (i2c_read_blocking(bno_i2c_inst_, dev_addr, buf, cnt, false) != cnt) {
        ret = BNO055_ERROR;
        return ret;
    }

    // copy data from buffer
    for (u8 datapos = 0; datapos < cnt; ++datapos)
        reg_data[datapos] = buf[datapos];

    return ret;
}

void BNO055_Delay_ms(u32 msecs) {
    sleep_ms(msecs);
}

void bno055_convert_double_quaternion_wxyz(struct bno055_quaternion_double_t *quat_wxyz) {
    struct bno055_quaternion_t quat_raw;

    const double scale =
            1.0 /
            (1 << 14);  // in bno055 datasheet (sec. 3.6.5.5 Orientation): 1 Quaternion = 2^14 raw

    bno055_read_quaternion_wxyz(&quat_raw);

    quat_wxyz->w = (double) (quat_raw.w) * scale;
    quat_wxyz->x = (double) (quat_raw.x) * scale;
    quat_wxyz->y = (double) (quat_raw.y) * scale;
    quat_wxyz->z = (double) (quat_raw.z) * scale;
}


BNO055::BNO055(i2c_inst_t *i2c_inst, const u8 &sda_pin, const u8 &scl_pin, const int &baudrate,
               const u8 &i2c_addr, const bool &use_internal_pullups)
    : sda_pin_(sda_pin), scl_pin_(scl_pin), i2c_addr_(i2c_addr), baudrate_(baudrate),
      use_internal_pullups_(use_internal_pullups) {
    bno_i2c_inst_ = i2c_inst;
    i2c_inst_ = i2c_inst;
}

BNO055::~BNO055() {}

bool BNO055::setup(u8 power_mode, u8 operation_mode) {
    // set I2C comms
    if (!i2c_setup(this->i2c_inst_, this->sda_pin_, this->scl_pin_, this->baudrate_,
                   this->use_internal_pullups_)) {
        printf("Error: Could not setup I2C.\n");
        return false;
    }

    // set device definitions
    this->dev_def_.bus_read = BNO055_I2C_BusRead;
    this->dev_def_.bus_write = BNO055_I2C_BusWrite;
    this->dev_def_.delay_msec = BNO055_Delay_ms;
    this->dev_def_.dev_addr = this->i2c_addr_;

    // device power settings
    s8 ret = 0;
    ret += bno055_init(&this->dev_def_);
    ret += bno055_set_power_mode(power_mode);
    ret += bno055_set_operation_mode(operation_mode);
    if (ret > 0) {
        printf("Error: While defining power settings.\n");
        return false;
    }

    // device range settings
    ret = 0;
    ret += bno055_set_accel_range(BNO055_ACCEL_RANGE_4G);
    ret += bno055_set_gyro_range(BNO055_GYRO_RANGE_2000DPS);
    ret += bno055_set_mag_operation_mode(BNO055_MAG_OPERATION_MODE_HIGH_ACCURACY);
    if (ret > 0) {
        printf("Error: While defining range settings.\n");
        return false;
    }

    return true;
}

bool BNO055::suspend() {
    bool ret;
    ret = bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND) > 0 ? false : true;
    return ret;
}

struct bno055_accel_double_t BNO055::getAccel() {
    struct bno055_accel_double_t datum;
    bno055_convert_double_accel_xyz_msq(&datum);
    return datum;
}

struct bno055_mag_double_t BNO055::getMag() {
    struct bno055_mag_double_t datum;
    bno055_convert_double_mag_xyz_uT(&datum);
    return datum;
}

struct bno055_gyro_double_t BNO055::getGyro() {
    struct bno055_gyro_double_t datum;
    bno055_convert_double_gyro_xyz_rps(&datum);
    return datum;
}

struct bno055_euler_double_t BNO055::getEuler() {
    struct bno055_euler_double_t datum;
    bno055_convert_double_euler_hpr_deg(&datum);
    return datum;
}

struct bno055_linear_accel_double_t BNO055::getLinearAccel() {
    struct bno055_linear_accel_double_t datum;
    bno055_convert_double_linear_accel_xyz_msq(&datum);
    return datum;
}

struct bno055_gravity_double_t BNO055::getGravity() {
    struct bno055_gravity_double_t datum;
    bno055_convert_double_gravity_xyz_msq(&datum);
    return datum;
}

struct bno055_quaternion_double_t BNO055::getQuaternion() {
    struct bno055_quaternion_double_t datum;
    bno055_convert_double_quaternion_wxyz(&datum);
    return datum;
}

double BNO055::getTemperature() {
    double temp_c;
    bno055_convert_double_temp_celsius(&temp_c);
    return temp_c;
}
