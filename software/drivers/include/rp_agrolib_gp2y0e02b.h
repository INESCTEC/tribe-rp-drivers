#ifndef _GP2Y0E02B_H_
#define _GP2Y0E02B_H_

#include <stdio.h>

#include "pico/stdlib.h"

#include "math.h"
#include "rp_agrolib_i2c.h"

#define GP2Y0E02B_DATA_ADDR 0x5E
#define GP2Y0E02B_SHIFT_BIT_ADDR 0x35
#define GP2Y0E02B_VPP_PIN (uint8_t) 0  // change this value depending on the pin you're using

/****Valid sensor addresses****/
#define GP2Y0E02B_ADDR_0X00 0x00
#define GP2Y0E02B_ADDR_0X10 0x10
#define GP2Y0E02B_ADDR_0X20 0x20
#define GP2Y0E02B_ADDR_0X30 0x30
#define GP2Y0E02B_ADDR_0X40 0x40
#define GP2Y0E02B_ADDR_0X50 0x50
#define GP2Y0E02B_ADDR_0X60 0x60
#define GP2Y0E02B_ADDR_0X70 0x70
#define GP2Y0E02B_ADDR_0X80 0x80  // default
#define GP2Y0E02B_ADDR_0X90 0x90
#define GP2Y0E02B_ADDR_0XA0 0xA0
#define GP2Y0E02B_ADDR_0XB0 0xB0
#define GP2Y0E02B_ADDR_0XC0 0xC0
#define GP2Y0E02B_ADDR_0XD0 0xD0
#define GP2Y0E02B_ADDR_0XE0 0xE0
#define GP2Y0E02B_ADDR_0XF0 0xF0
/******************************/

typedef struct tof_sens_struct {
    uint8_t addr;

    uint8_t shift;
    uint8_t dist_raw[2];

    float dist_cm;

    i2c_inst_t *i2c;
} GP2Y0E02B_t;

void GP2Y0E02B_setup(GP2Y0E02B_t *GP2Y0E02B, i2c_inst_t *i2c, const uint8_t addr);
void GP2Y0E02B_change_addr(GP2Y0E02B_t *GP2Y0E02B, const uint8_t new_addr);
void GP2Y0E02B_get_distance_cm(GP2Y0E02B_t *GP2Y0E02B);


#endif