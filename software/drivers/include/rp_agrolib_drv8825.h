
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pico/stdlib.h"

#include "hardware/pwm.h"

#define CLK_PRE 15
#define SYSTEM_FREQ 125000000
#define PHASE_CORRECT 0

#define READ_TIMEOUT 2000


#define SLEEP1_PIN 13
#define DIR1_PIN 11
#define nENABLE1_PIN 12
#define STEP1_PIN 10
#define M0_1_PIN 27
#define M1_1_PIN 28
#define M2_1_PIN 0
#define nFAULT1_PIN 26


#define SLEEP2_PIN 23
#define DIR2_PIN 29
#define nENABLE2_PIN 22
#define STEP2_PIN 20
#define M0_2_PIN 16
#define M1_2_PIN 17
#define M2_2_PIN 1
#define nFAULT2_PIN 21

#define CW true
#define CCW false


typedef struct {
    int dir_pin;
    int Nsleep_pin;
    int step_pin;
    int Nenable_pin;
    int Nfault;
    int M0;
    int M1;
    int M2;
    int microstep_mode;
    float steps;
    bool dir;
} stepper_structure;


void gpio_output_init(int pin, bool state);
void set_microstepping(stepper_structure *stepper, int mode);
void stepper_sleep(stepper_structure *stepper, bool sleep);
void stepper_set_speed(stepper_structure *stepper, float rpm);
void stop_motor(stepper_structure *stepper);
void change_dir(stepper_structure *stepper, bool dir);
void stepper_pin_define();
void stepper_init(stepper_structure *stepper, int step_num);
void count_steps(uint gpio, uint32_t events);
