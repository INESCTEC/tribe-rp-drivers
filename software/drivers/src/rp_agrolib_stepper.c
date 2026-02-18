#include "../include/rp_agrolib_stepper.h"


void pwm_stepper_init(stepper_structure *stepper) {
    // Find out which PWM slice is connected to GPIO
    uint slice_num_A = pwm_gpio_to_slice_num(stepper->motorA1);
    uint slice_num_B = pwm_gpio_to_slice_num(stepper->motorB1);

    // Set PWM configuration
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, CLK_PRE);  // Set the prescaler to adjust frequency

    // Initialize PWM with the specified configuration for windingA
    pwm_init(slice_num_A, &config, true);
    // Initialize PWM with the specified configuration for windingB
    pwm_init(slice_num_B, &config, true);


    // Tell GPIO  they are allocated to the PWM
    gpio_set_function(stepper->motorA1, GPIO_FUNC_PWM);
    gpio_set_function(stepper->motorA2, GPIO_FUNC_PWM);
    gpio_set_function(stepper->motorB1, GPIO_FUNC_PWM);
    gpio_set_function(stepper->motorB2, GPIO_FUNC_PWM);

    /*pwm_set_phase_correct(slice_num_A, true);
    pwm_set_phase_correct(slice_num_B, true);*/

    // interrupst to count steps
    gpio_set_irq_enabled_with_callback(stepper->motorA1, GPIO_IRQ_EDGE_RISE, true, &count_steps);
    gpio_set_irq_enabled_with_callback(stepper->motorA2, GPIO_IRQ_EDGE_RISE, true, &count_steps);
    gpio_set_irq_enabled_with_callback(stepper->motorB1, GPIO_IRQ_EDGE_RISE, true, &count_steps);
    gpio_set_irq_enabled_with_callback(stepper->motorB2, GPIO_IRQ_EDGE_RISE, true, &count_steps);
}

void motor_stop(stepper_structure *stepper) {

    pwm_set_output_polarity(pwm_gpio_to_slice_num(stepper->motorA1), false,
                            false);  // waves with opposite phases
    pwm_set_output_polarity(pwm_gpio_to_slice_num(stepper->motorB1), false, false);
    // Set PWM TOP
    pwm_set_wrap(pwm_gpio_to_slice_num(stepper->motorA1), 0);
    pwm_set_wrap(pwm_gpio_to_slice_num(stepper->motorB1), 0);
    // Set compare values for counter
    pwm_set_chan_level(pwm_gpio_to_slice_num(stepper->motorA1), PWM_CHAN_A, 0);
    pwm_set_chan_level(pwm_gpio_to_slice_num(stepper->motorA1), PWM_CHAN_B, 0);
    pwm_set_chan_level(pwm_gpio_to_slice_num(stepper->motorB1), PWM_CHAN_A, 0);
    pwm_set_chan_level(pwm_gpio_to_slice_num(stepper->motorB1), PWM_CHAN_B, 0);

    pwm_set_counter(pwm_gpio_to_slice_num(stepper->motorA1), 0);
    pwm_set_counter(pwm_gpio_to_slice_num(stepper->motorB1), 0);

    pwm_set_enabled(pwm_gpio_to_slice_num(stepper->motorA1), false);
    pwm_set_enabled(pwm_gpio_to_slice_num(stepper->motorB1), false);
    gpio_put(stepper->sleep_pin, false);
    stepper->last_top_value = 0;
    stepper->last_rpm = 0;
}

void motor_rotate(stepper_structure *stepper, bool dir, int rpm) {

    if (rpm <= 0) {
        motor_stop(stepper);
    } else {
        gpio_put(stepper->sleep_pin, true);

        float frequency = (rpm * stepper->steps) / 60;
        float count_top = SYSTEM_FREQ / ((1 + PHASE_CORRECT) * (CLK_PRE) *frequency) - 1;
        // Set PWM TOP
        pwm_set_wrap(pwm_gpio_to_slice_num(stepper->motorA1), count_top);
        pwm_set_wrap(pwm_gpio_to_slice_num(stepper->motorB1), count_top);
        // Set compare values for counter
        pwm_set_chan_level(pwm_gpio_to_slice_num(stepper->motorA1), PWM_CHAN_A,
                           count_top / 2);  // 50% de duty cycle
        pwm_set_chan_level(pwm_gpio_to_slice_num(stepper->motorA1), PWM_CHAN_B, count_top / 2);
        pwm_set_chan_level(pwm_gpio_to_slice_num(stepper->motorB1), PWM_CHAN_A, count_top / 2);
        pwm_set_chan_level(pwm_gpio_to_slice_num(stepper->motorB1), PWM_CHAN_B, count_top / 2);

        if (stepper->last_top_value == 0 || (stepper->dir != dir)) {
            if (dir) {
                pwm_set_counter(pwm_gpio_to_slice_num(stepper->motorA1), 0);
                pwm_set_counter(pwm_gpio_to_slice_num(stepper->motorB1),
                                count_top / 4);  // 90% phase shift //rotate backward
            } else {
                pwm_set_counter(pwm_gpio_to_slice_num(stepper->motorA1),
                                count_top / 4);  // 90% phase shift, rotate forward
                pwm_set_counter(pwm_gpio_to_slice_num(stepper->motorB1), 0);
            }
        } else {
            int last_countA, last_countB, new_countA, new_countB;
            pwm_set_enabled(pwm_gpio_to_slice_num(stepper->motorA1), false);
            pwm_set_enabled(pwm_gpio_to_slice_num(stepper->motorB1), false);
            last_countA = pwm_get_counter(pwm_gpio_to_slice_num(stepper->motorA1));
            last_countB = pwm_get_counter(pwm_gpio_to_slice_num(stepper->motorB1));
            new_countA = last_countA * count_top / stepper->last_top_value;
            new_countB = last_countB * count_top / stepper->last_top_value;
            // printf("count A %d, %d, count B %d, %d, COUNT %d %f\n", last_countA, new_countA,
            // last_countB, new_countB, stepper->last_top_value, count_top);
            pwm_set_counter(pwm_gpio_to_slice_num(stepper->motorA1), new_countA);
            pwm_set_counter(pwm_gpio_to_slice_num(stepper->motorB1), new_countB);
        }

        pwm_set_output_polarity(pwm_gpio_to_slice_num(stepper->motorA1), true,
                                false);  // opposite-phase waves
        pwm_set_output_polarity(pwm_gpio_to_slice_num(stepper->motorB1), true,
                                false);  // opposite-phase waves
        // Set the PWM running
        pwm_set_enabled(pwm_gpio_to_slice_num(stepper->motorA1), true);
        pwm_set_enabled(pwm_gpio_to_slice_num(stepper->motorB1), true);

        stepper->last_top_value = count_top;
        stepper->dir = dir;
        stepper->last_rpm = rpm;
    }
}


void accelerate(stepper_structure *stepper, bool dir, int rpm_final, int aceleration) {
    uint64_t time_wait_us = (1.0 / aceleration) * 1000 * 1000;  // s to ms
    uint64_t time_us = time_us_64();
    // printf("wait_us %llu\n", time_wait_us);
    // printf("time_us %llu\n", time_us);

    if (rpm_final != stepper->last_rpm) {
        // printf("first if %llu \n", time_us - stepper->last_time_us);
        if ((time_us - stepper->last_time_us) > time_wait_us) {
            printf("rpm %d\n", stepper->last_rpm);
            if (rpm_final > stepper->last_rpm) {  // accelerate
                if (stepper->last_rpm < stepper->min_rpm)
                    stepper->last_rpm = stepper->min_rpm;
                motor_rotate(stepper, dir, stepper->last_rpm + 1);
            } else {  // decelerate
                motor_rotate(stepper, dir, stepper->last_rpm - 1);
                if (stepper->last_rpm < stepper->min_rpm)
                    motor_stop(stepper);
            }
            stepper->last_time_us = time_us_64();
        }
    } else {
        stepper->accelerate_end_flag = true;
        printf("end\n");
    }
}
