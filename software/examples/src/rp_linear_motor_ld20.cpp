#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

#include "mcp23017.h"
#include "rp_agrolib_i2c.h"


#define ADC_INPUT 0
#define GPIO_ADC0 26

#define GPIO_HBRIDGE_EN_U8 20
#define GPIO_HBRIDGE_PH_U8 21
#define GPIO_MCP_PMODE 2

#define GPIO_I2C0_SDA 4
#define GPIO_I2C0_SCL 5
#define I2C0_BAUDRATE 100000


static const uint16_t MCP_ALL_PINS_OFF = 0x0000;
static const uint16_t MCP_ALL_PINS_OUTPUT = 0x0008;


int main() {
    stdio_init_all();
    sleep_ms(500);

    // setup ADC
    adc_init();
    adc_gpio_init(GPIO_ADC0);
    adc_select_input(ADC_INPUT);

    // setup gpios
    gpio_init(GPIO_HBRIDGE_EN_U8);
    gpio_set_dir(GPIO_HBRIDGE_EN_U8, GPIO_OUT);
    gpio_init(GPIO_HBRIDGE_PH_U8);
    gpio_set_dir(GPIO_HBRIDGE_PH_U8, GPIO_OUT);

    // setup I2C
    i2c_inst_t *i2c_0 = init_i2c0(GPIO_I2C0_SDA, GPIO_I2C0_SCL, I2C0_BAUDRATE, false);

    // setup MCP
    Mcp23017 mcp(i2c_0, 0x20);  // setting MCP with A0,A1,A2 pins connected to GND
    mcp.setup(true, false);
    mcp.set_io_direction(MCP_ALL_PINS_OUTPUT);
    mcp.set_output_bit_for_pin(GPIO_MCP_PMODE,
                               false);  // setting MCP pin 2 to LOW (PMODE pin on H-Bridge)
    mcp.flush_output();

    int prev_input = 0;
    while (1) {
        // get instruction for linear motor actuation
        int input = getchar_timeout_us(100);

        if (input != prev_input) {
            if (prev_input == (int) 'd' && input == (int) 'f') {
                gpio_put(GPIO_HBRIDGE_EN_U8, false);  // brake
                sleep_ms(100);
                gpio_put(GPIO_HBRIDGE_EN_U8, true);  // forward
                gpio_put(GPIO_HBRIDGE_PH_U8, true);
            } else if (prev_input == (int) 'f' && input == (int) 'd') {
                gpio_put(GPIO_HBRIDGE_EN_U8, false);  // brake
                sleep_ms(100);
                gpio_put(GPIO_HBRIDGE_EN_U8, true);  // reverse
                gpio_put(GPIO_HBRIDGE_PH_U8, false);
            } else if (input == (int) 'f') {
                gpio_put(GPIO_HBRIDGE_EN_U8, true);  // forward
                gpio_put(GPIO_HBRIDGE_PH_U8, true);
            } else if (input == (int) 'd') {
                gpio_put(GPIO_HBRIDGE_EN_U8, true);  // reverse
                gpio_put(GPIO_HBRIDGE_PH_U8, false);
            } else if (input == (int) 'b') {
                gpio_put(GPIO_HBRIDGE_EN_U8, false);  // brake
            }
            prev_input = input;
        }

        // acquire and convert ADC value
        const float conversion_factor =
                3.33f / (1 << 12);  // 12-bit ADC for a 3.33V max. input voltage
        uint16_t adc_res = adc_read();
        printf("ADC raw value: %u | 0x%04x, voltage: %.3f V\n", adc_res, adc_res,
               adc_res * conversion_factor);
        sleep_ms(100);
    }

    return 0;
}
