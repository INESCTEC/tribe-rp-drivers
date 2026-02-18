#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
extern "C" {
#include "rp_agrolib_drv8825.h"
}

stepper_structure stepper2;

void count_steps(uint gpio, uint32_t events) {

    if (events == GPIO_IRQ_EDGE_RISE) {
        /*if(gpio == STEP1_PIN){
            if(stepper1.dir){
                stepper1.steps += 1/stepper1.microstep_mode;
            }
            else{
                stepper1.steps -= 1/stepper1.microstep_mode;
            }
        }*/
        if (gpio == STEP2_PIN) {
            if (stepper2.dir) {
                stepper2.steps += 1 / stepper2.microstep_mode;
            } else {
                stepper2.steps -= 1 / stepper2.microstep_mode;
            }
        }
    }
}

int main() {
    stdio_init_all();


    int rpm = 300;
    stepper_init(&stepper2, 2);
    stepper_set_speed(&stepper2, rpm);

    int c;
    while (1) {
        int c = getchar_timeout_us(READ_TIMEOUT);
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == 'w') {
                rpm += 50;
                stepper_set_speed(&stepper2, rpm);
            }
            if (c == 's') {
                rpm -= 50;
                stepper_set_speed(&stepper2, rpm);
            }
            if (c == 'd') {
                change_dir(&stepper2, false);
                stepper_set_speed(&stepper2, rpm);
                if (stepper2.dir == CW)
                    printf("CW\n");
                else if (stepper2.dir == CCW)
                    printf("CCW\n");
            }
            if (c == 'a') {
                change_dir(&stepper2, true);
                stepper_set_speed(&stepper2, rpm);
                if (stepper2.dir == CW)
                    printf("CW\n");
                else if (stepper2.dir == CCW)
                    printf("CCW\n");
            }
            if (c == 'i') {  // stop
                stepper_set_speed(&stepper2, 0);
            }
            if (c == 'o') {  // sleep
                stepper_sleep(&stepper2, true);
            }
            if (c == 'p') {  // nSleep
                stepper_sleep(&stepper2, false);
            }
            if (c == '1') {
                set_microstepping(&stepper2, 1);
            }
            if (c == '2') {
                set_microstepping(&stepper2, 2);
            }
            if (c == '3') {
                set_microstepping(&stepper2, 4);
            }
            if (c == '4') {
                set_microstepping(&stepper2, 8);
            }
            if (c == '5') {
                set_microstepping(&stepper2, 16);
            }
            if (c == '6') {
                set_microstepping(&stepper2, 32);
            }
        }
    }

    return 0;
}
