
#include "../include/rp_agrolib_drv8825.h"

void gpio_output_init(int pin, bool state) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, state);
}


void set_microstepping(stepper_structure *stepper, int mode) {
    if (mode == 1) {
        gpio_put(stepper->M0, false);
        gpio_put(stepper->M1, false);
        gpio_put(stepper->M2, false);
    } else if (mode == 2) {
        gpio_put(stepper->M0, true);
        gpio_put(stepper->M1, false);
        gpio_put(stepper->M2, false);
    } else if (mode == 4) {
        gpio_put(stepper->M0, false);
        gpio_put(stepper->M1, true);
        gpio_put(stepper->M2, false);
    } else if (mode == 8) {
        gpio_put(stepper->M0, true);
        gpio_put(stepper->M1, true);
        gpio_put(stepper->M2, false);

    } else if (mode == 16) {
        gpio_put(stepper->M0, false);
        gpio_put(stepper->M1, false);
        gpio_put(stepper->M2, true);
    } else if (mode == 32) {
        gpio_put(stepper->M0, true);
        gpio_put(stepper->M1, false);
        gpio_put(stepper->M2, true);
    }
    stepper->microstep_mode = mode;
}

void stepper_sleep(stepper_structure *stepper, bool sleep) {
    gpio_put(stepper->Nsleep_pin, !sleep);
}

void stepper_set_speed(stepper_structure *stepper, float rpm) {

    stepper_sleep(stepper, false);
    float frequency = (rpm * 200) / 60;
    float count_top = SYSTEM_FREQ / ((1 + PHASE_CORRECT) * (CLK_PRE) *frequency) - 1;

    // Set PWM TOP
    pwm_set_wrap(pwm_gpio_to_slice_num(stepper->step_pin), count_top);
    // Set compare values for counter
    pwm_set_chan_level(pwm_gpio_to_slice_num(stepper->step_pin),
                       pwm_gpio_to_channel(stepper->step_pin), count_top / 2);  // 50% de duty cycle
    pwm_set_enabled(pwm_gpio_to_slice_num(stepper->step_pin), true);
}

void stop_motor(stepper_structure *stepper) {
    pwm_set_enabled(pwm_gpio_to_slice_num(stepper->step_pin), false);
}


void change_dir(stepper_structure *stepper, bool dir) {
    stop_motor(stepper);
    gpio_put(stepper->dir_pin, dir);
    sleep_ms(100);
    stepper->dir = dir;
}

void stepper_init(stepper_structure *stepper, int step_num) {

    if (step_num == 1) {
        stepper->Nsleep_pin = SLEEP1_PIN;
        stepper->dir_pin = DIR1_PIN;
        stepper->step_pin = STEP1_PIN;
        stepper->Nenable_pin = nENABLE1_PIN;
        stepper->Nfault = nFAULT1_PIN;
        stepper->M0 = M0_1_PIN;
        stepper->M1 = M1_1_PIN;
        stepper->M2 = M2_1_PIN;
    } else if (step_num == 2) {
        stepper->Nsleep_pin = SLEEP2_PIN;
        stepper->dir_pin = DIR2_PIN;
        stepper->step_pin = STEP2_PIN;
        stepper->Nenable_pin = nENABLE2_PIN;
        stepper->Nfault = nFAULT2_PIN;
        stepper->M0 = M0_2_PIN;
        stepper->M1 = M1_2_PIN;
        stepper->M2 = M2_2_PIN;
    } else {
        printf("motor not available\n");
    }

    // gpio
    gpio_output_init(stepper->Nenable_pin, false);
    gpio_output_init(stepper->Nsleep_pin, true);
    gpio_output_init(stepper->dir_pin, false);
    gpio_output_init(stepper->M0, false);
    gpio_output_init(stepper->M1, false);
    gpio_output_init(stepper->M2, false);
    gpio_init(stepper->Nfault);
    gpio_set_dir(stepper->Nfault, GPIO_IN);

    set_microstepping(stepper, 1);

    gpio_set_irq_enabled_with_callback(stepper->step_pin, GPIO_IRQ_EDGE_RISE, true, &count_steps);


    // pwm
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, CLK_PRE);  // Set the prescaler to adjust frequency
    // Initialize PWM with the specified configuration
    pwm_init(pwm_gpio_to_slice_num(stepper->step_pin), &config, true);
    // Tell GPIO  they are allocated to the PWM
    gpio_set_function(stepper->step_pin, GPIO_FUNC_PWM);
}
