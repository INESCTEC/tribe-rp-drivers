#ifndef _RP_AGROLIB_USB_
#define _RP_AGROLIB_USB_

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

bool read_usb(uint8_t *buffer);
bool check_string(const uint8_t *buffer, const char *string);

#endif