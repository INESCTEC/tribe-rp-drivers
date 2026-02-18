#include <stdio.h>
#include <unistd.h>

#include "pico/stdlib.h"

#include "hardware/adc.h"

#include "../include/rp_agrolib_ads8354.h"
#include "math.h"

#define SPEC_CHANNELS 288
#define ADC_INT 0
#define ADC_EXT 1

#define CHANNEL_A 0
#define CHANNEL_B 1


class HAMAMATSU_SPECTRAL {
private:
    // Calibration parameters (polynomial function: A0 + A1x + A2x^2 + ... + A7x^7)
    double parametersSensors[6];
    int adc_input;    // for internal adc
    int ads_channel;  // for external adc
    int EOS_pin, TRG_pin, STR_pin, CLK_pin, video_pin;
    int mode;  // indica se é usado o adc da rp2040 ou um adc externo

    void init_gpios(int mode);

    void startRead(const uint64_t delayTime);
    void integrate(const uint64_t delayTime, const uint32_t integration_time);
    void readData(int16_t *buff, uint64_t delayTime);
    void readSpectrometer(int16_t *buff, uint32_t itime);


public:
    ADS8354 *ads8354;
    HAMAMATSU_SPECTRAL(int EOS_pin_, int TRG_pin_, int STR_pin_, int CLK_pin_, int video_pin_,
                       double parameters_[6], int mode_);
    void add_ads(ADS8354 *ads, int channel);  // necessário chamar se o ADC externo for usado
    void read(int16_t *buff, uint32_t itime);
    void get_wavelenghts(double *wave_array);
};
