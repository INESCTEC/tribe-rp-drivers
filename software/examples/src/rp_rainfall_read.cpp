
#include <stdio.h>

#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include "hardware/i2c.h"

#include "rp_agrolib_rainfall_sensor.h"

#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

int main() {
    // Initialize standard I/O
    stdio_init_all();

    // Initialize I2C with default pins
    i2c_init(i2c_default, 400 * 1000);  // 100 kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

    // Add program information for picotool
    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));

    // Give serial time to initialize
    sleep_ms(2000);

    printf("DFRobot Rainfall Sensor Example for Pico\n");

    // Create an instance of the rainfall sensor
    DFRobot_RainfallSensor sensor(i2c0);

    // Initialize the sensor
    while (!sensor.begin()) {
        printf("Failed to initialize the rainfall sensor. Check connection!\n");
        sleep_ms(1000);
    }

    printf("Rainfall sensor initialized successfully!\n");

    // Print sensor information
    printf("PID: 0x%lX\n", sensor.getPID());
    printf("VID: 0x%X\n", sensor.getVID());
    printf("Firmware Version: %s\n", sensor.getFirmwareVersion());

    // Set rainfall accumulated value (mm per tip)
    // Default is 0.2794mm per tip, change if your sensor has a different value
    sensor.setRainAccumulatedValue(0.2794);

    while (1) {
        // Get total cumulative rainfall since sensor started working
        float total_rainfall = sensor.getRainfall();

        // Get rainfall in last hour
        float last_hour_rainfall = sensor.getRainfall(1);

        // Get rainfall in last 24 hours
        float last_24_hours_rainfall = sensor.getRainfall(24);

        // Get raw data (number of tipping bucket counts)
        uint32_t raw_counts = sensor.getRawData();

        // Get sensor working time in hours
        float working_time = sensor.getSensorWorkingTime();

        // Print all readings
        printf("\n------ Rainfall Sensor Readings ------\n");
        printf("Total rainfall: %.4f mm\n", total_rainfall);
        printf("Rainfall in last hour: %.4f mm\n", last_hour_rainfall);
        printf("Rainfall in last 24 hours: %.4f mm\n", last_24_hours_rainfall);
        printf("Raw tip counts: %u\n", raw_counts);
        printf("Sensor working time: %.2f hours\n", working_time);
        printf("-------------------------------------\n");

        sleep_ms(5000);  // Wait 5 seconds before next reading
    }

    return 0;
}
