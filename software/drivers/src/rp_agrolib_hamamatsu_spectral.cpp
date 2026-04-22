
#include "../include/rp_agrolib_hamamatsu_spectral.h"

#define SPEC_CHANNELS 288


void HAMAMATSU_SPECTRAL::init_gpios(int mode) {
    /************************************************************************/
    /*                spectral sensor pin configurations                    */
    /************************************************************************/
    gpio_init(STR_pin);
    gpio_set_dir(STR_pin, GPIO_OUT);
    gpio_put(STR_pin, false);

    gpio_init(CLK_pin);
    gpio_set_dir(CLK_pin, GPIO_OUT);
    gpio_put(CLK_pin, false);

    gpio_init(EOS_pin);
    gpio_set_dir(EOS_pin, GPIO_IN);

    gpio_init(TRG_pin);
    gpio_set_dir(TRG_pin, GPIO_IN);


    if (mode == ADC_INT) {
        adc_init();
        adc_gpio_init(video_pin);
    }
}


void HAMAMATSU_SPECTRAL::readSpectrometer(uint16_t *buff, uint32_t itime) {

    const uint64_t delayTime = 1;  // delay time in us

    startRead(delayTime);
    integrate(delayTime, itime);
    readData(buff, delayTime);
}


void HAMAMATSU_SPECTRAL::startRead(const uint64_t delayTime) {
    // Start clock cycle and set start pulse to signal start
    gpio_put(CLK_pin, 0);
    sleep_us(delayTime);
    gpio_put(CLK_pin, 1);
    sleep_us(delayTime);
    gpio_put(CLK_pin, 0);
    gpio_put(STR_pin, 1);
    sleep_us(delayTime);
}

void HAMAMATSU_SPECTRAL::integrate(const uint64_t delayTime, const uint32_t integration_time) {
    for (uint32_t i = 0; i < (const uint32_t)(integration_time / 2); i++) {
        gpio_put(CLK_pin, 1);
        sleep_us(delayTime);
        gpio_put(CLK_pin, 0);
        sleep_us(delayTime);
    }

    // Set SPEC_ST to 0
    gpio_put(STR_pin, 0);

    // Sample for a period of time
    for (int i = 0; i < 85; i++) {
        gpio_put(CLK_pin, 1);
        sleep_us(delayTime);
        gpio_put(CLK_pin, 0);
        sleep_us(delayTime);
    }
}

void HAMAMATSU_SPECTRAL::readData(uint16_t *buff, uint64_t delayTime) {
    // One more clock pulse before the actual read
    gpio_put(CLK_pin, 1);
    sleep_us(delayTime);
    gpio_put(CLK_pin, 0);
    sleep_us(delayTime);

    int16_t valueA, valueB;

    // Read from SPEC_VIDEO
    for (int i = 0; i < SPEC_CHANNELS; i++) {

        if (mode == ADC_INT) {
            adc_select_input(adc_input);
            buff[i] = adc_read();
        } else if (mode == ADC_EXT) {
            buff[i] = _external_adc->read();
        }

        gpio_put(CLK_pin, 1);
        sleep_us(delayTime);
        gpio_put(CLK_pin, 0);
        sleep_us(delayTime);
    }

    // Set SPEC_ST to 1
    gpio_put(STR_pin, 1);

    // Sample for a small amount of time
    for (int i = 0; i < 7; i++) {
        gpio_put(CLK_pin, 1);
        sleep_us(delayTime);
        gpio_put(CLK_pin, 0);
        sleep_us(delayTime);
    }

    gpio_put(CLK_pin, 1);

    sleep_us(delayTime);
}

void HAMAMATSU_SPECTRAL::read(uint16_t *buff, uint32_t itime) {
    readSpectrometer(buff, 0);
    readSpectrometer(buff, itime);
}

void HAMAMATSU_SPECTRAL::add_external_adc(iExternalADC *external_adc) {
    _external_adc = external_adc;
}

void HAMAMATSU_SPECTRAL::get_wavelengths(double *wave_array) {
    for (int i = 1; i <= SPEC_CHANNELS; i++) {
        wave_array[i - 1] = parametersSensors[0] + parametersSensors[1] * i +
                            parametersSensors[2] * pow(i, 2) + parametersSensors[3] * pow(i, 3) +
                            parametersSensors[4] * pow(i, 4) + parametersSensors[5] * pow(i, 5);
    }
}


HAMAMATSU_SPECTRAL::HAMAMATSU_SPECTRAL(int EOS_pin_, int TRG_pin_, int STR_pin_, int CLK_pin_,
                                       int video_pin_, double parameters_[6], int mode_)
    : _external_adc(nullptr) {
    EOS_pin = EOS_pin_;
    TRG_pin = TRG_pin_;
    STR_pin = STR_pin_;
    CLK_pin = CLK_pin_;
    video_pin = video_pin_;
    mode = mode_;

    for (int i = 0; i < 6; i++) {
        parametersSensors[i] = parameters_[i];
    }

    if (video_pin >= 26 && video_pin <= 29) {
        adc_input = video_pin_ - 26;
    } else {
        printf("pin is not an ADC input");
    }
    init_gpios(mode);
}
