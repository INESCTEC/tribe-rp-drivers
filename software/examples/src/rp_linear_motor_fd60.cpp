#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "hardware/uart.h"

#include "mcp23017.h"
#include "rp_agrolib_i2c.h"


#define GPIO_INPUT 26

#define GPIO_HBRIDGE_EN 20
#define GPIO_HBRIDGE_PH 21
#define GPIO_MCP_PMODE 2

#define GPIO_I2C0_SDA 4
#define GPIO_I2C0_SCL 5
#define I2C0_BAUDRATE 100000


static const uint16_t MCP_ALL_PINS_OFF = 0x0000;
static const uint16_t MCP_ALL_PINS_OUTPUT = 0x0008;
static volatile uint16_t npulses = 0;  // maximum number of pulses is below 1500
static volatile uint8_t actuator_state = 0;


static void set_hbridge_actuator(bool enable, bool direction = false) {
    gpio_put(GPIO_HBRIDGE_EN, enable);
    gpio_put(GPIO_HBRIDGE_PH, direction);
}

void count_pulses(uint gpio, uint32_t events) {
    if (events == GPIO_IRQ_EDGE_RISE) {
        if (gpio == GPIO_INPUT && actuator_state == 1) {
            npulses++;
        } else if (gpio == GPIO_INPUT && actuator_state == 2) {
            npulses--;
        }
    }
}


int main() {
    stdio_init_all();
    sleep_ms(500);

    // setup gpios
    gpio_init(GPIO_HBRIDGE_EN);
    gpio_set_dir(GPIO_HBRIDGE_EN, GPIO_OUT);
    gpio_init(GPIO_HBRIDGE_PH);
    gpio_set_dir(GPIO_HBRIDGE_PH, GPIO_OUT);

    gpio_init(GPIO_INPUT);
    gpio_set_dir(GPIO_INPUT, GPIO_IN);
    gpio_set_irq_enabled_with_callback(GPIO_INPUT, GPIO_IRQ_EDGE_RISE, true, &count_pulses);

    // setup I2C
    i2c_inst_t *i2c_0 = init_i2c0(GPIO_I2C0_SDA, GPIO_I2C0_SCL, I2C0_BAUDRATE, false);

    // setup MCP
    Mcp23017 mcp(i2c_0, 0x20);  // setting MCP with A0,A1,A2 pins connected to GND
    mcp.setup(true, false);
    mcp.set_io_direction(MCP_ALL_PINS_OUTPUT);
    mcp.set_output_bit_for_pin(GPIO_MCP_PMODE,
                               false);  // setting MCP pin 3 to LOW (PMODE pin on H-Bridge)
    mcp.flush_output();

    int prev_input = 0;
    while (1) {
        // get instruction for linear motor actuation
        int input = getchar_timeout_us(100);

        if (input != prev_input) {
            if (prev_input == (int) 'd' && input == (int) 'f') {
                set_hbridge_actuator(false);  // brake
                sleep_ms(100);
                set_hbridge_actuator(true, true);  // forward
                actuator_state = 1;
            } else if (prev_input == (int) 'f' && input == (int) 'd') {
                set_hbridge_actuator(false);  // brake
                sleep_ms(100);
                set_hbridge_actuator(true, false);  // reverse
                actuator_state = 2;
            } else if (input == (int) 'f') {
                set_hbridge_actuator(true, true);  // forward
                actuator_state = 1;
            } else if (input == (int) 'd') {
                set_hbridge_actuator(true, false);  // reverse
                actuator_state = 2;
            } else if (input == (int) 'b') {
                set_hbridge_actuator(false);  // brake
                actuator_state = 0;
            }
            prev_input = input;
        }

        // show pulses counter
        uint32_t interrupts = save_and_disable_interrupts();
        printf("Number of pulses: %d\n", npulses);
        restore_interrupts(interrupts);

        sleep_ms(100);
    }

    return 0;
}
