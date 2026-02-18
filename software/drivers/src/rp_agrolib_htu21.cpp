#include "../include/rp_agrolib_htu21.h"

double RPHTU21::get_temperature(uint32_t timeout_us, int *error) {
    uint8_t data[2];

    printf("start reading\n");
    int status = reg_read_timeout(i2c, HTU21_ADDR, REG_TEMP_ADDR, data, 2, timeout_us);
    printf("end reading\n");

    if (error != nullptr) {
        *error = status;
    }

    if (status < 0) {
        return NAN;
    }

    unsigned int aux = (data[0] << 8 | data[1]) & 0xFFFC;
    double auxtemp = aux / 65536.0;
    double temperature = -46.85 + (175.72 * auxtemp);
    return temperature;
}

double RPHTU21::get_humidity(double temperature, uint32_t timeout_us, int *error) {
    uint8_t data[2];

    printf("start reading\n");
    int status = reg_read_timeout(i2c, HTU21_ADDR, REG_HUMI_ADDR, data, 2, timeout_us);
    printf("end reading\n");

    if (error != nullptr) {
        *error = status;
    }

    if (status < 0) {
        return NAN;
    }

    unsigned int aux = (data[0] << 8 | data[1]) & 0xFFFC;
    /*
    To accommodate/adapt any process variation (nominal capacitance value of the humidity die),
    tolerances of the sensor above 100%RH and below 0%RH must be considered So, 118%RH
    corresponds to 0xFF which is the maximum RH digital output that can be sent out and -6%RH
    corresponds to 0x00 which is the minimum RH digital output that can be sent out
    */
    double humidity = -6.0 + (125.0 * aux) / 65536.0;  // assign the value read inside the range
                                                       // [-6,119] using %RH as meter
    humidity +=
            (25.0 - temperature) * COEFF_TEMP;  // since the humidity value varies with air
                                                // temperature, is necessary apply a compensation

    if (humidity > 100.0)
        humidity = 100.0;
    else if (humidity < 0.0)
        humidity = 0.0;

    return humidity;
}
