#include <stdio.h>
#include <unistd.h>

#include "pico/stdlib.h"

#include "hardware/spi.h"

#define SINGLE 0  // only output A
#define DOUBLE 1  // output A followed by B

class ADS8354 {
private:
    int SCLK_pin, SDO_pin, CS_pin, SDI_pin;
    int mode;

public:
    ADS8354(int CS_pin_, int SCLK_pin_, int SDI_pin_, int SDO_pin_, int mode_);
    void read_values(int16_t *valueA, int16_t *valueB);
    uint16_t read_registers();
    void write_configuration(int mode);
};
