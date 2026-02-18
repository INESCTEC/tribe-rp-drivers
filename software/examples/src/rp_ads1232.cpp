#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "hardware/adc.h"

#include "rp_agrolib_ads1232.h"


int main() {
    stdio_init_all();

    uint input_number = 0;
    int num_measurements = 5;

    sleep_ms(5000);

    printf("Sensor ADS1232\n");
    ADS1232 sensor(num_measurements, RATE10, GAIN128, input_number);
    printf("Object created\n");

    printf("\nStarting sensor\n");
    sensor.begin();
    printf("Sensor started\n");

    printf("\nStarting Multiple Measurements\n");

    int32_t last_measurement;

    // start a measurement of the ADC
    sensor.start_single_measurement();

    while (true) {
        if (sensor.get_measurement(&last_measurement)) {
            // print the measurement
            printf("Measurement: %d\n", last_measurement);

            // start another measurement
            sensor.start_single_measurement();
        }
        sleep_ms(50);
    }
}