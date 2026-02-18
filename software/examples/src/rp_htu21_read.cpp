#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "rp_agrolib_htu21.h"

#define BAUDRATE 400  // KHz
#define SDA_PIN 0
#define SCL_PIN 1

int main() {
    stdio_init_all();

    i2c_inst_t *i2c = i2c0;
    i2c_init(i2c, BAUDRATE * 1000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    sleep_ms(3000);

    scan_bus(i2c);

    RPHTU21 htu(i2c);

    while (true) {
        double t = htu.get_temperature();
        double h = htu.get_humidity(t);

        printf("Temperature: %f\n", t);
        printf("Humidity: %f\n", h);
        sleep_ms(1000);
    }
}