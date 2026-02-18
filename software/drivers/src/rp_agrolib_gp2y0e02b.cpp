#include "../include/rp_agrolib_gp2y0e02b.h"


void GP2Y0E02B_setup(GP2Y0E02B_t *GP2Y0E02B, i2c_inst_t *i2c, const uint8_t addr) {
    sleep_ms(50);

    GP2Y0E02B->addr = (addr >> 1);  // 7 lsb
    GP2Y0E02B->i2c = i2c;

    reg_read(GP2Y0E02B->i2c, GP2Y0E02B->addr, GP2Y0E02B_SHIFT_BIT_ADDR, &(GP2Y0E02B->shift), 1);

    // GP2Y0E02B_change_addr(GP2Y0E02B, GP2Y0E02B_ADDR_0X70); //ONLY RUN THIS ONCE, IT CHANGES THE
    // ADDRESS PERMANENTLY GP2Y0E02B->addr = (GP2Y0E02B_ADDR_0X70 >> 1);
}

// This function is extensive but these steps need to be performed in this order to effectively
// change the device's address
void GP2Y0E02B_change_addr(GP2Y0E02B_t *GP2Y0E02B, const uint8_t new_addr) {
    gpio_init(GP2Y0E02B_VPP_PIN);  // either have this pin pulled to 0 or disconnect vpp from the
                                   // sensor.
    gpio_set_dir(GP2Y0E02B_VPP_PIN, GPIO_OUT);
    gpio_put(GP2Y0E02B_VPP_PIN, 0);

    uint8_t data = 0xFF;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xEC, &data, 1);

    gpio_put(GP2Y0E02B_VPP_PIN, 1);  // VPP High

    data = 0x00;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xC8, &data, 1);


    data = 0x45;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xC9, &data, 1);


    data = new_addr >> 4;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xCD, &data, 1);


    data = 0x01;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xCA, &data, 1);
    sleep_us(500);

    data = 0x00;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xCA, &data, 1);
    gpio_put(GP2Y0E02B_VPP_PIN, 0);  // VPP Low


    data = 0x00;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xEF, &data, 1);


    data = 0x40;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xC8, &data, 1);


    data = 0x00;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xC8, &data, 1);


    data = 0x06;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xEE, &data, 1);


    data = 0xFF;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xEC, &data, 1);


    data = 0x03;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xEF, &data, 1);


    reg_read(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0x27, &data, 1);


    data = 0x00;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xEF, &data, 1);


    data = 0x7F;
    reg_write(GP2Y0E02B->i2c, GP2Y0E02B->addr, 0xEC, &data, 1);

    sleep_ms(50);
}

void GP2Y0E02B_get_distance_cm(GP2Y0E02B_t *GP2Y0E02B) {

    reg_read(GP2Y0E02B->i2c, GP2Y0E02B->addr, GP2Y0E02B_DATA_ADDR, GP2Y0E02B->dist_raw, 2);

    GP2Y0E02B->dist_cm = ((GP2Y0E02B->dist_raw[0] * 16.0 + GP2Y0E02B->dist_raw[1]) / 16.0) /
                         ((int) pow(2, GP2Y0E02B->shift));
}