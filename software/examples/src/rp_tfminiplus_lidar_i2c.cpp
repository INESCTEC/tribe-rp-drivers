#include <stdio.h>

#include "pico/stdlib.h"

#include "rp_agrolib_tfminiplus.h"

#define I2C_ID i2c1
#define BAUDRATE 400000
#define SDA_PIN 14
#define SCL_PIN 15
#define I2C_ADDR 0x10

int main() {
    stdio_init_all();

    i2c_inst_t *i2c_inst = I2C_ID;
    i2c_setup(i2c_inst, SDA_PIN, SCL_PIN, BAUDRATE, true);

    TFminiPlus TmP_lidar_i2c(i2c_inst, I2C_ADDR);

    sleep_ms(5);

    tfminiplus_data_t lidar_data;
    while (1) {
        lidar_data = TmP_lidar_i2c.getLidarData();
        printf("%d cm (%d)\n", lidar_data.distance_cm, lidar_data.intensity);
        sleep_ms(10);
    }

    return 0;
}
