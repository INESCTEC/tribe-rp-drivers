#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "rp_agrolib_hamamatsu_spectral.h"


#define ADS8354_SCLK 14
#define ADS8354_SDO 28
#define ADS8354_CS 13
#define ADS8354_SDI 15

#define C12880_EOS 22
#define C12880_TRG 23
#define C12880_START 24
#define C12880_CLK 25
#define C12880_VIDEO_3V3 29


// Calibration parameters (polynomial function: A0 + B1x + B2x^2 + ... + B5x^5)
double C12880_calibraion_parameters[6] = {306.1813561,          2.720457373,
                                          -0.001506879715,      -0.00000425187093,
                                          -0.00000000384217372, 0.0000000000224606721};


int main() {
    stdio_init_all();
    int16_t data_c12880[SPEC_CHANNELS];
    double c12880_wavelenghts[SPEC_CHANNELS];

    sleep_ms(5000);
    printf("start\n");
    ADS8354 ads8354(ADS8354_CS, ADS8354_SCLK, ADS8354_SDO, ADS8354_SDI, DOUBLE);

    //////////////////////////////////////////
    // EXTERN ADC
    //////////////////////////////////////////
    HAMAMATSU_SPECTRAL C12880(C12880_EOS, C12880_TRG, C12880_START, C12880_CLK, C12880_VIDEO_3V3,
                              C12880_calibraion_parameters, ADC_EXT);
    C12880.add_ads(&ads8354, CHANNEL_A);

    //////////////////////////////////////////
    // RP2040 ADC
    //////////////////////////////////////////
    // HAMAMATSU_SPECTRAL C12880(C12880_EOS,C12880_TRG,C12880_START, C12880_CLK, C12880_VIDEO_3V3,
    // parametersSensors, ADC_INT);

    C12880.get_wavelenghts(c12880_wavelenghts);
    for (int i = 0; i < SPEC_CHANNELS; i++) {
        printf("%f ", c12880_wavelenghts[i]);
    }
    printf("\n");

    while (true) {
        C12880.read(data_c12880, 1000);
        for (uint j = 0; j < SPEC_CHANNELS; j++) {
            printf("C12880[%d]=%d\n", j, data_c12880[j]);
        }
        sleep_ms(1000);
    }
}
