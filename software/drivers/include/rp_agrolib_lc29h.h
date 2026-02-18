#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pico/stdlib.h"

#include "rp_agrolib_uart.h"

struct RtkState {
    double utc_;
    double latitude_;
    double longitude_;
    double altitude_;
    double heading_;
    uint8_t fix_status_;
    uint16_t num_satellites_;
    double speed_kph_;
};

class LC29HDriver {
public:
    LC29HDriver(uart_inst_t *uart_id);

    void writeCorrections(uint8_t *data, uint8_t size);
    void getNMEA(uint16_t size, struct RtkState *rtk_state, uint8_t *data);
    void getPQMT(uint16_t in_size, char *pqmt, uint8_t *data, uint16_t *out_size);
    void getPair(uint16_t in_size, char *pair, uint8_t *data, uint16_t *out_size);

private:
    uart_inst_t *uart_id_;

    void parseGGA(const char *data, struct RtkState *rtk_state);
    void parseVTG(const char *data, struct RtkState *rtk_state);
    void parsePQTMTAR(const char *data, struct RtkState *rtk_state);
    bool validateChecksum(const char *message);
    double convertToDegrees(double value);
};