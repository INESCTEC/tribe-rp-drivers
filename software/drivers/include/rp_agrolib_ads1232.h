#pragma once
#include <stdio.h>

#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include <cstdlib>


// RP2040 GPIO to ADS1232
#define ADS1232_DOUT 20
#define ADS1232_SCLK 22
#define ADS1232_GAIN0 10
#define ADS1232_GAIN1 11
#define ADS1232_PDWN 13
#define ADS1232_A0 1
#define ADS1232_SPEED 12
#define ADS1232_TEMP 0

// PWM configuration
#define SYSTEM_FREQ 125000000
#define CLK_PRE 15
#define PHASE_CORRECT 0
#define FREQ 2000
#define DUTY_CYCLE 50

// modes of sensor operation
#define SING_MEASUREMENT 1
#define MULT_MEASUREMENT 2
#define MODE_CALIB 3


// Programmable Gain Amplifier (PGA) values
enum ADS1232_gain { GAIN1, GAIN2, GAIN64, GAIN128 };

// Data Acquisition rate values
enum ADS1232_sample_rate { RATE10, RATE80 };

class ADS1232 {
private:
    // Data members to store ADC readings
    volatile int32_t *raw_measurements;
    uint num_measurements;

    // sensor relevant variables
    ADS1232_sample_rate sample_rate;
    ADS1232_gain gain;

    // active ADS1232 input type bridge 0, bridge 1 or temperature (0, 1, 2)
    uint input_mode;

    // PWM Slice
    uint pwm_slice_num, pwm_channel;
    float NUM_CYCLES;

    // Data Acquisition relevant variables
    uint func_mode;
    volatile uint pulse_counter;
    bool data_ready = false;
    uint curr_measurement;

    void setup_communication();
    void init_gpio();


public:
    ADS1232(int num_measurements, ADS1232_sample_rate rate, ADS1232_gain gain, uint input_number);
    ~ADS1232();

    // static pointer to object
    static ADS1232 *instance;
    static void interrupHandler(uint gpio, uint32_t events);

    void begin();

    void set_power_on();
    void set_power_off();

    void set_sample_rate(ADS1232_sample_rate rate);
    void set_gain(ADS1232_gain gain);

    void set_bridge_0_mode();
    void set_bridge_1_mode();
    void set_temp_mode();

    void start_single_measurement();
    void start_multiple_measurement();
    void start_calibration();
    bool get_measurement(int32_t *val);
    bool get_measurements(int32_t *buffer);
    bool get_measurements_mean(double *val);
    bool get_measurements_mean(double *val, int32_t *buffer);

    void set_num_measurements(uint num_measurements);
    uint get_num_measurements();
};
