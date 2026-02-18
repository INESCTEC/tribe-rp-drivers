#include <stdio.h>

#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"


#define PWM_TOP 30000
#define CLK_PRE 15
#define PWM_COMPARE PWM_TOP / 2
#define CLOCK_WISE true
#define ANTICLOCK_WISE false
#define SYSTEM_FREQ 125000000
#define PHASE_CORRECT 0
#define MIN_RPM 40


typedef struct {
    int last_top_value;
    int last_rpm;
    uint64_t last_time_us;
    bool dir;
    int motorA1;
    int motorA2;
    int motorB1;
    int motorB2;
    int sleep_pin;
    bool accelerate_end_flag;
    bool min_rpm;
    int32_t steps;
} stepper_structure;

// callback that is called on the rising edge of the 4 signals that are send to the motor
void count_steps(uint gpio, uint32_t events);

void pwm_stepper_init(stepper_structure *stepper);

void motor_stop(stepper_structure *stepper);

void motor_rotate(stepper_structure *stepper, bool dir, int rpm);

void accelerate(stepper_structure *stepper, bool dir, int rpm_final, int aceleration);
