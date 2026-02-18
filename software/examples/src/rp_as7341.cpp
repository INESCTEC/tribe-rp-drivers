#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "rp_agrolib_as7341.h"


#define BAUDRATE 400  // KHz
#define SDA_PIN 16    // 14 to sda1
#define SCL_PIN 17    // 15 to scl1

#define AS7341_ADDR 0x39

#define I2C_TIMEOUT 500000

int main() {
    stdio_init_all();

    i2c_inst_t *i2c = i2c0;

    i2c_init(i2c, BAUDRATE * 1000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    AS7341 as7341(i2c, I2C_TIMEOUT);
    Channels channels;

    printf("AS7341 initialized\n");

    sleep_ms(5000);

    as7341.PowerOn();


    while (true) {
        printf("Read channels\n");
        as7341.ReadLight(&channels);
        printf("F1 - 415nm/Violet  %d\n", channels.F1);
        printf("F2 - 445nm/Indigo %d\n", channels.F2);
        printf("F3 - 480nm/Blue   %d\n", channels.F3);
        printf("F4 - 515nm/Cyan   %d\n", channels.F4);
        printf("F5 - 555nm/Green   %d\n", channels.F5);
        printf("F6 - 590nm/Yellow  %d\n", channels.F6);
        printf("F7 - 630nm/Orange  %d\n", channels.F7);
        printf("F8 - 680nm/Red     %d\n", channels.F8);
        printf("Clear              %d\n", channels.CLEAR);
        printf("Near-IR (NIR)      %d\n", channels.NIR);

        sleep_ms(1000);
    }
}