
#include <stdio.h>

#include "pico/stdlib.h"

#include "rp_agrolib_gp2y0e02b.h"

GP2Y0E02B_t tof;

void setup() {
    stdio_init_all();
    sleep_ms(2000);

    const uint8_t sda_pin = 14;
    const uint8_t scl_pin = 15;

    i2c_inst_t *i2c = init_i2c1(sda_pin, scl_pin, 100 * 1000, true);

    GP2Y0E02B_setup(&tof, i2c, GP2Y0E02B_ADDR_0X80);

    sleep_ms(500);
}


int main() {
    setup();

    while (1) {
        GP2Y0E02B_get_distance_cm(&tof);
        printf("distance: %f [cm]\n", tof.dist_cm);
        sleep_ms(1000);
    }

    return 0;
}