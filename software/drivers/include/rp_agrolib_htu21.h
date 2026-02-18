#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include "rp_agrolib_i2c.h"

#define HTU21_ADDR 0x40
#define REG_TEMP_ADDR 0xF3
#define REG_HUMI_ADDR 0xF5
#define COEFF_TEMP                                                                                 \
    -0.15  // given in datasheet (%RH/ºC) and can be used inside the temperature range [0,80]ºC

class RPHTU21 {
public:
    double get_temperature(uint32_t timeout_us = 1000000, int *error = nullptr);
    double get_humidity(double temperature, uint32_t timeout_us = 1000000, int *error = nullptr);

    RPHTU21(i2c_inst_t *_i2c) : i2c(_i2c) {}

private:
    i2c_inst_t *i2c;
};