#include <stdio.h>

#include "pico/stdlib.h"

#include "rp_agrolib_tflidar.h"

#define UART_ID uart0
#define BAUDRATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define TX_PIN 0
#define RX_PIN 1

int main() {
    stdio_init_all();

    uart_inst_t *uart_inst = UART_ID;
    uart_setup(uart_inst, RX_PIN, TX_PIN, BAUDRATE, DATA_BITS, STOP_BITS, PARITY);

    TFLidar lidar(uart_inst);

    sleep_ms(5);
    tf_lidar_data_t lidar_data;
    while (1) {
        lidar_data = lidar.getLidarData();
        printf("%d cm (%d) at %.1fC\n", lidar_data.distance_cm, lidar_data.intensity, lidar_data.temperature);
        sleep_ms(5);
    }

    return 0;
}
