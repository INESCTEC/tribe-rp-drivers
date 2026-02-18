#include "hardware/irq.h"

#include "rp_agrolib_tps25750.h"
#include "rp_agrolib_uart.h"
#include "tps_config.h"

#ifdef LOW_REGION_BIN
#include "low_region_bin.h"
#elif LOW_REGION_NAB_42_05_04_012
#include "low_region_nab_42_05_04_012.h"
#endif

/* This example assume TPS25750 connected to I2C bus 1 and to GPIOs 14 and 15 */

#define BAUDRATE 100  // KHz

#define SDA_PIN 14
#define SCL_PIN 15

#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define UART_TX_PIN 0
#define UART_RX_PIN 1

int main() {
    stdio_init_all();

    gpio_init(SDA_PIN);
    gpio_init(SCL_PIN);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    i2c_inst_t *i2c = i2c1;
    uint baud = i2c_init(i2c, BAUDRATE * 1000);
    RPTPS25750 tps(i2c);

    int uart_irq =
            uart_setup(UART_ID, UART_RX_PIN, UART_TX_PIN, BAUD_RATE, DATA_BITS, STOP_BITS, PARITY);

    sleep_ms(2000);

    //------READ LOW REGION BINARY FILE FROM LOW_REGION_BIN.H
    uint8_t *low_region_bin_data = (uint8_t *) tps25750x_lowRegion_i2c_array;
    int low_region_bin_size = gSizeLowRegionArray;

    //------UPDATE PATCH BUNDLE STATE MACHINE
    bool flag = 0;
    uint8_t state = SM_START_UPDATE;
    while (true) {
        tps.state_machine_it(state, low_region_bin_data, low_region_bin_size, 1, UART_ID);
        sleep_ms(10);
    }
}