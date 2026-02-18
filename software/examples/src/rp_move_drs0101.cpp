#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "hardware/irq.h"

#include "rp_agrolib_drs0101.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define SERVO_ID 0x00

HerkulexServoBus herkulex_bus(UART_ID);
HerkulexServo my_servo(herkulex_bus, SERVO_ID);

void uart0_callback() {
    herkulex_bus.update();
}

int main() {
    stdio_init_all();

    int uart_irq =
            uart_setup(UART_ID, UART_RX_PIN, UART_TX_PIN, BAUD_RATE, DATA_BITS, STOP_BITS, PARITY);
    uart_enable_interrupt(UART_ID, uart_irq, uart0_callback);

    sleep_ms(500);

    my_servo.reboot();
    sleep_ms(500);
    my_servo.setTorqueOn();
    sleep_ms(500);

    while (1) {

        my_servo.setLedColor(HerkulexLed::Green);
        sleep_ms(1000);
        herkulex_bus.prepareIndividualMove();
        my_servo.setPosition(512, 100);
        herkulex_bus.executeMove();
        my_servo.setLedColor(HerkulexLed::Purple);
        sleep_ms(1000);
        herkulex_bus.prepareIndividualMove();
        my_servo.setPosition(112, 100);
        herkulex_bus.executeMove();
        my_servo.setLedColor(HerkulexLed::Blue);
        sleep_ms(1000);
    }

    return 0;
}