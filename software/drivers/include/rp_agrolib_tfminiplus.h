#pragma once

#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include "rp_agrolib_i2c.h"
#include "rp_agrolib_uart.h"

const uint8_t TFMINIPLUS_GET_DATA_I2C_CMD[5] = {0x5A, 0x05, 0x00, 0x01, 0x60};

typedef struct {
    uint16_t distance_cm;
    uint16_t intensity;
    double temperature;
} tfminiplus_data_t;

void get_tfmini_data_from_uart(uart_inst_t *uart, uint8_t *buf, tfminiplus_data_t *lidar_data);
void get_tfmini_data_from_i2c(i2c_inst_t *i2c, const uint8_t addr, uint8_t *buf,
                              tfminiplus_data_t *lidar_data);

class TFminiPlus {
public:
    TFminiPlus(i2c_inst_t *i2c, const uint8_t addr);
    TFminiPlus(uart_inst_t *uart);
    ~TFminiPlus() {}

    tfminiplus_data_t getLidarData();

private:
    i2c_inst_t *i2c_inst_;
    uart_inst_t *uart_inst_;
    bool is_i2c_;
    bool is_uart_;
    uint8_t buffer_i2c_[9];
    uint8_t buffer_uart_[9];
    uint8_t address_i2c_;
};
