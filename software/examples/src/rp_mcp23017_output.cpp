#include <stdio.h>

#include "pico/stdlib.h"

#include "hardware/i2c.h"

#include "mcp23017.h"

static const int MCP_ALL_PINS_OFF = 0x0000;
static const int MCP_ALL_PINS_OUTPUT = 0x0000;

#define I2C_GPIO_PIN_SDA 4
#define I2C_GPIO_PIN_SLC 5

#define EN_U8 22
#define PH_U8 23

Mcp23017 mcp1(i2c0, 0x20);  // MCP with A0,A1,A2 to GND

void setup_output(Mcp23017 mcp) {
    int result;

    result = mcp.setup(true, false);
    result = mcp.set_io_direction(MCP_ALL_PINS_OUTPUT);
}


i2c_inst_t *init_i2c1(const uint8_t sda_pin, const uint8_t scl_pin, const uint32_t baudrate,
                      const bool internal_pullups) {
    i2c_inst_t *i2c_1 = i2c0;

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    if (internal_pullups) {
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);
    }

    i2c_init(i2c_1, baudrate);

    return i2c_1;
}


int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("Starting up\n");

    gpio_init(EN_U8);
    gpio_init(PH_U8);
    gpio_set_dir(EN_U8, GPIO_OUT);
    gpio_set_dir(PH_U8, GPIO_OUT);


    i2c_inst_t *i2c_0 = init_i2c1(I2C_GPIO_PIN_SDA, I2C_GPIO_PIN_SLC, 100000, true);
    // gpio_set_function(I2C_GPIO_PIN_SDA, GPIO_FUNC_I2C);
    // gpio_set_function(I2C_GPIO_PIN_SLC, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_GPIO_PIN_SDA);
    // gpio_pull_up(I2C_GPIO_PIN_SLC);

    // i2c_init(i2c0, 100000);

    setup_output(mcp1);
    mcp1.set_all_output_bits(MCP_ALL_PINS_OFF);

    printf("Setting MCP(0x21) pin 2\n");
    mcp1.set_output_bit_for_pin(3, false);
    mcp1.flush_output();


    gpio_put(EN_U8, false);
    gpio_put(PH_U8, false);
    gpio_put(EN_U8, true);

    while (1) {
        sleep_ms(2000);

        gpio_put(PH_U8, true);
        sleep_ms(2000);
        gpio_put(EN_U8, false);
        sleep_ms(2000);
        gpio_put(EN_U8, true);
        gpio_put(PH_U8, false);
    }
    return 0;
}