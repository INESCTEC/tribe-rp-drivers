#include "../include/rp_agrolib_ads1232.h"


// global declaration
ADS1232 *ADS1232::instance = nullptr;

void ADS1232::setup_communication() {
    // INTERRUPTS
    // gpio_set_irq_callback(&interrupHandler);
    // irq_set_enabled(IO_IRQ_BANK0, 1);

    this->pwm_slice_num = pwm_gpio_to_slice_num(ADS1232_SCLK);
    this->pwm_channel = pwm_gpio_to_channel(ADS1232_SCLK);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, CLK_PRE);

    pwm_init(this->pwm_slice_num, &config, 0);
    this->NUM_CYCLES = SYSTEM_FREQ / ((1 + PHASE_CORRECT) * (CLK_PRE) *FREQ) - 1;
    pwm_set_wrap(this->pwm_slice_num, this->NUM_CYCLES);
    pwm_set_chan_level(this->pwm_slice_num, this->pwm_channel,
                       this->NUM_CYCLES * (DUTY_CYCLE / 100.0));
    pwm_set_enabled(this->pwm_slice_num, 0);
}

void ADS1232::init_gpio() {
    // initiate all the gpios
    printf("init ADS1232 gpio");
    gpio_init(ADS1232_PDWN);
    gpio_set_dir(ADS1232_PDWN, 1);  // true for output
    gpio_put(ADS1232_PDWN, 0);

    gpio_set_function(ADS1232_SCLK, GPIO_FUNC_PWM);

    gpio_init(ADS1232_GAIN0);
    gpio_set_dir(ADS1232_GAIN0, 1);
    gpio_put(ADS1232_GAIN0, 0);

    gpio_init(ADS1232_GAIN1);
    gpio_set_dir(ADS1232_GAIN1, 1);
    gpio_put(ADS1232_GAIN1, 0);

    gpio_init(ADS1232_A0);
    gpio_set_dir(ADS1232_A0, 1);
    gpio_put(ADS1232_A0, 0);

    gpio_init(ADS1232_SPEED);
    gpio_set_dir(ADS1232_SPEED, 1);
    gpio_put(ADS1232_SPEED, 0);

    gpio_init(ADS1232_TEMP);
    gpio_set_dir(ADS1232_TEMP, 1);
    gpio_put(ADS1232_TEMP, 0);

    gpio_init(ADS1232_DOUT);
    gpio_set_dir(ADS1232_DOUT, 0);  // false for input
    gpio_set_inover(ADS1232_DOUT, GPIO_OVERRIDE_NORMAL);
}

void ADS1232::interrupHandler(uint gpio, uint32_t events) {
    if (gpio == ADS1232_SCLK && events == GPIO_IRQ_EDGE_FALL) {
        instance->pulse_counter--;

        if ((instance->func_mode == SING_MEASUREMENT || instance->func_mode == MULT_MEASUREMENT) &&
            instance->pulse_counter >= 1) {
            // read the data
            instance->raw_measurements[instance->curr_measurement] |=
                    (static_cast<uint32_t>(gpio_get(ADS1232_DOUT)))
                    << (instance->pulse_counter - 1);
        } else if (!instance->pulse_counter) {
            if (instance->func_mode != MODE_CALIB) {
                if (instance->raw_measurements[instance->curr_measurement] & 0x800000) {
                    instance->raw_measurements[instance->curr_measurement] |=
                            0xFF000000;  // sign extension
                } else {
                    instance->raw_measurements[instance->curr_measurement] &= 0x00FFFFFF;
                }
            }

            // deactivate PWM
            pwm_set_chan_level(instance->pwm_slice_num, instance->pwm_channel, 0);
            pwm_set_enabled(instance->pwm_slice_num, 0);

            // deactivate the irqSCKL
            gpio_set_irq_enabled(ADS1232_SCLK, GPIO_IRQ_EDGE_FALL, 0);
            // increment the measurement counter
            instance->curr_measurement++;

            if (instance->func_mode == MULT_MEASUREMENT &&
                instance->curr_measurement < instance->num_measurements) {
                // restart the communication with ADS1232
                instance->pulse_counter = 25;
                // activate irqDOUT
                gpio_set_irq_enabled_with_callback(ADS1232_DOUT, GPIO_IRQ_EDGE_FALL, true,
                                                   &interrupHandler);  // regist interrupt method

            } else if (instance->func_mode == SING_MEASUREMENT ||
                       (instance->func_mode == MULT_MEASUREMENT &&
                        instance->curr_measurement == instance->num_measurements)) {
                // no need to check in single if instance->num_measurements==1
                instance->data_ready = true;
            }
        }

    } else if (gpio == ADS1232_DOUT && events == GPIO_IRQ_EDGE_FALL) {
        // deactivate irqDOUT
        gpio_set_irq_enabled_with_callback(ADS1232_DOUT, GPIO_IRQ_EDGE_FALL, false,
                                           &interrupHandler);  // regist interrupt method

        // gpio_set_irq_enabled(ADS1232_DOUT, GPIO_IRQ_EDGE_FALL, 0);
        // activate irqSCKL
        gpio_set_irq_enabled_with_callback(ADS1232_SCLK, GPIO_IRQ_EDGE_FALL, true,
                                           &interrupHandler);  // regist interrupt method

        // gpio_set_irq_enabled(ADS1232_SCLK, GPIO_IRQ_EDGE_FALL, 1);
        // activate PWM
        pwm_set_chan_level(instance->pwm_slice_num, instance->pwm_channel,
                           instance->NUM_CYCLES * (DUTY_CYCLE / 100.0));
        pwm_set_enabled(instance->pwm_slice_num, 1);
    }
}

// Constructor
ADS1232::ADS1232(int num_measurements, ADS1232_sample_rate rate, ADS1232_gain gain,
                 uint input_number) {
    this->instance = this;

    this->num_measurements = num_measurements;
    this->raw_measurements = (volatile int32_t *) malloc(this->num_measurements * sizeof(int32_t));

    this->sample_rate = rate;
    this->gain = gain;

    // active bridge input of ADS1232
    if (input_number == 1) {
        this->input_mode = 1;
    } else {
        this->input_mode = 0;
    }
}

// Destructor
ADS1232::~ADS1232() {
    free((void *) raw_measurements);
}

void ADS1232::begin() {
    this->init_gpio();

    this->set_power_on();

    this->set_sample_rate(this->sample_rate);
    this->set_gain(gain);

    if (this->input_mode == 0) {
        this->set_bridge_0_mode();
    } else if (this->input_mode == 1) {
        this->set_bridge_1_mode();
    } else {
        this->set_temp_mode();
    }

    this->setup_communication();

    this->start_calibration();
}


void ADS1232::set_power_on() {
    gpio_put(ADS1232_PDWN, 1);
    // sleep
    sleep_ms(1);
    gpio_put(ADS1232_PDWN, 0);
    sleep_ms(1);
    gpio_put(ADS1232_PDWN, 1);
}

void ADS1232::set_power_off() {
    // testing needed
    gpio_put(ADS1232_PDWN, 0);
}

void ADS1232::set_sample_rate(ADS1232_sample_rate rate) {
    switch (rate) {
        case RATE10:
            gpio_put(ADS1232_SPEED, 0);
            break;
        case RATE80:
            gpio_put(ADS1232_SPEED, 1);
            break;
    }
}

void ADS1232::set_gain(ADS1232_gain gain) {
    switch (gain) {
        case GAIN1:
            gpio_put(ADS1232_GAIN0, 0);
            gpio_put(ADS1232_GAIN1, 0);
            break;
        case GAIN2:
            gpio_put(ADS1232_GAIN0, 1);
            gpio_put(ADS1232_GAIN1, 0);
            break;
        case GAIN64:
            gpio_put(ADS1232_GAIN0, 0);
            gpio_put(ADS1232_GAIN1, 1);
            break;
        case GAIN128:
            gpio_put(ADS1232_GAIN0, 1);
            gpio_put(ADS1232_GAIN1, 1);
            break;
    }
}

void ADS1232::set_bridge_0_mode() {
    gpio_put(ADS1232_TEMP, 0);
    gpio_put(ADS1232_A0, 0);
}

void ADS1232::set_bridge_1_mode() {
    gpio_put(ADS1232_TEMP, 0);
    gpio_put(ADS1232_A0, 1);
}

void ADS1232::set_temp_mode() {
    gpio_put(ADS1232_TEMP, 1);
    gpio_put(ADS1232_A0, 0);
}


void ADS1232::start_single_measurement() {
    this->pulse_counter = 25;
    this->func_mode = SING_MEASUREMENT;
    this->curr_measurement = 0;
    for (int i = 0; i < this->num_measurements; i++) {
        this->raw_measurements[i] = 0;
    }
    gpio_set_irq_enabled_with_callback(ADS1232_DOUT, GPIO_IRQ_EDGE_FALL, true,
                                       &interrupHandler);  // regist interrupt method

    // gpio_set_irq_enabled(ADS1232_DOUT, GPIO_IRQ_EDGE_FALL, 1);
}

void ADS1232::start_multiple_measurement() {
    this->pulse_counter = 25;
    this->func_mode = MULT_MEASUREMENT;
    this->curr_measurement = 0;
    for (int i = 0; i < this->num_measurements; i++) {
        this->raw_measurements[i] = 0;
    }
    gpio_set_irq_enabled_with_callback(ADS1232_DOUT, GPIO_IRQ_EDGE_FALL, true,
                                       &interrupHandler);  // regist interrupt method

    // gpio_set_irq_enabled(ADS1232_DOUT, GPIO_IRQ_EDGE_FALL, 1);
}

void ADS1232::start_calibration() {
    this->pulse_counter = 26;
    this->func_mode = MODE_CALIB;
    gpio_set_irq_enabled_with_callback(ADS1232_DOUT, GPIO_IRQ_EDGE_FALL, true,
                                       &interrupHandler);  // regist interrupt method

    // gpio_set_irq_enabled(ADS1232_DOUT, GPIO_IRQ_EDGE_FALL, 1);
}

bool ADS1232::get_measurement(int32_t *val) {
    // currently gets the measurement value without calibration
    if (this->data_ready) {
        this->data_ready = false;

        // get the most recent measurement independent of the requested measurement type
        *val = this->raw_measurements[0];
        return true;
    }
    return false;
}


bool ADS1232::get_measurements(int32_t *buffer) {
    // the user is responsible for ensuring that buffer has at least num_measurements space

    if (this->data_ready && this->func_mode == MULT_MEASUREMENT) {
        this->data_ready = false;

        for (int i = 0; i < this->num_measurements; i++) {
            *(buffer + i) = this->raw_measurements[i];
        }

        return true;
    }
    return false;
}

bool ADS1232::get_measurements_mean(double *val) {
    // calculate the mean of the measurements
    double mean = 0.0;

    if (this->data_ready && this->func_mode == MULT_MEASUREMENT) {
        this->data_ready = false;

        for (int i = 0; i < this->num_measurements; i++) {
            mean += static_cast<double>(this->raw_measurements[i]);
        }
        mean /= this->num_measurements;

        *val = mean;
        return true;
    }
    return false;
}


bool ADS1232::get_measurements_mean(double *val, int32_t *buffer) {
    // calculate the mean of the measurements
    double mean = 0.0;

    if (this->data_ready && this->func_mode == MULT_MEASUREMENT) {
        this->data_ready = false;

        for (int i = 0; i < this->num_measurements; i++) {
            mean += static_cast<double>(this->raw_measurements[i]);
        }
        mean /= this->num_measurements;

        *val = mean;

        // copy the information from the sensor buffer to the user buffer
        for (int i = 0; i < this->num_measurements; i++) {
            *(buffer + i) = this->raw_measurements[i];
        }
        return true;
    }
    return false;
}


void ADS1232::set_num_measurements(uint num_measurements) {
    this->num_measurements = num_measurements;

    // dealloc memory
    free((void *) raw_measurements);

    // alloc memory
    this->raw_measurements = (volatile int32_t *) malloc(this->num_measurements * sizeof(int32_t));
}

uint ADS1232::get_num_measurements() {
    return this->num_measurements;
}