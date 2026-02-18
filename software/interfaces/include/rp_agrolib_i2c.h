#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "hardware/i2c.h"

int reg_write(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf,
              const uint8_t nbytes);

int reg_write_raw(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf, int nbytes);

int reg_write_timeout(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *data,
                      const uint8_t nbytes, uint timeout_us);

int reg_read(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf,
             const uint8_t nbytes);

int reg_read2(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf,
              const uint8_t nbytes);

int reg_read_timeout(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *data,
                     const uint8_t nbytes, uint timeout_us);

bool reserved_addr(uint8_t addr);

void scan_bus(i2c_inst_t *i2c);

i2c_inst_t *init_i2c0(const uint8_t sda_pin, const uint8_t scl_pin, const uint32_t baudrate,
                      const bool internal_pullups);
i2c_inst_t *init_i2c1(const uint8_t sda_pin, const uint8_t scl_pin, const uint32_t baudrate,
                      const bool internal_pullups);
bool i2c_setup(i2c_inst_t *i2c_id, const uint8_t sda_pin, const uint8_t scl_pin,
               const uint32_t baudrate, const bool internal_pullups);