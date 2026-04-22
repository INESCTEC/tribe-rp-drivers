#pragma once

#include <stdio.h>
#include <unistd.h>

#include "pico/stdlib.h"

#include "hardware/adc.h"

#include "math.h"

#define SPEC_CHANNELS 288
#define ADC_INT 0
#define ADC_EXT 1

#define CHANNEL_A 0
#define CHANNEL_B 1

/**
 * @brief Interface for external ADC devices used with the Hamamatsu spectral sensor.
 */
class iExternalADC {
public:
    /**
     * @brief Reads a value from the external ADC.
     *
     * @return uint16_t The ADC reading.
     */
    virtual uint16_t read() = 0;
};

class HAMAMATSU_SPECTRAL {
private:
    // Calibration parameters (polynomial function: A0 + A1x + A2x^2 + ... + A7x^7)
    double parametersSensors[6];
    int adc_input;  // for internal adc
    // int ads_channel;  //for external adc
    int EOS_pin, TRG_pin, STR_pin, CLK_pin, video_pin;
    int mode;  // indica se é usado o adc da rp2040 ou um adc externo
    iExternalADC *_external_adc;

    void init_gpios(int mode);

    void startRead(const uint64_t delayTime);
    void integrate(const uint64_t delayTime, const uint32_t integration_time);
    void readData(uint16_t *buff, uint64_t delayTime);
    void readSpectrometer(uint16_t *buff, uint32_t itime);


public:
    HAMAMATSU_SPECTRAL(int EOS_pin_, int TRG_pin_, int STR_pin_, int CLK_pin_, int video_pin_,
                       double parameters_[6], int mode_);
    void add_external_adc(iExternalADC *external_adc);  // necessário se ADC externo for usado
    void read(uint16_t *buff, uint32_t itime);
    void get_wavelengths(double *wave_array);
};
