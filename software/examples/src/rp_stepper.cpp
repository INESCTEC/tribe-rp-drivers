#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
extern "C" {
#include "rp_agrolib_stepper.h"
}

// Example that rotates a motor 1 second clockwise and 1 second anticlockwise
#define MOTOR_A1 20
#define MOTOR_A2 21
#define MOTOR_B1 22
#define MOTOR_B2 23
#define MOTOR_STEPS 200
#define SLEEP_PIN 27

stepper_structure stepper;


// Even if the callback is not used, it needs to be declared with an empty body
// Has to be modified to count the steps of a possible second motor
void count_steps(uint gpio, uint32_t events) {

    if (events == GPIO_IRQ_EDGE_RISE) {
        if (gpio == stepper.motorA1 || gpio == stepper.motorA2 || gpio == stepper.motorB1 ||
            gpio == stepper.motorB2) {
            if (stepper.dir == CLOCK_WISE) {
                stepper.steps++;
            } else if (stepper.dir == ANTICLOCK_WISE) {
                stepper.steps--;
            }
        }
    }
}


int main() {
    stdio_init_all();

    gpio_init(SLEEP_PIN);
    gpio_set_dir(SLEEP_PIN, GPIO_OUT);
    stepper.motorA1 = MOTOR_A1;
    stepper.motorA2 = MOTOR_A2;
    stepper.motorB1 = MOTOR_B1;
    stepper.motorB2 = MOTOR_B2;
    stepper.sleep_pin = SLEEP_PIN;
    stepper.last_top_value = 0;
    stepper.dir = CLOCK_WISE;
    stepper.min_rpm = MIN_RPM;
    stepper.steps = MOTOR_STEPS;


    pwm_stepper_init(&stepper);

    sleep_ms(3000);

    motor_rotate(&stepper, CLOCK_WISE, 70);
    sleep_ms(1000);
    motor_stop(&stepper);
    sleep_ms(1000);
    motor_rotate(&stepper, ANTICLOCK_WISE, 70);
    sleep_ms(1000);
    motor_stop(&stepper);
    sleep_ms(2000);


    while (true) {
        printf("end\n");
        sleep_ms(1000);
    }
}