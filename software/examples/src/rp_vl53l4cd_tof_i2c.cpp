#include <stdio.h>

#include "pico/stdlib.h"

#include "hardware/i2c.h"

#include "rp_agrolib_i2c.h"
extern "C" {
#include "VL53L4CD_api.h"
}

#define I2C_ID i2c0
#define I2C_DEV_ADDR 0x52  // default I2C address
#define SDA_PIN 16
#define SCL_PIN 17
#define BAUDRATE 400000


int main() {
    stdio_init_all();

    // Setup I2C
    vl_i2c_inst_ = I2C_ID;
    i2c_setup(vl_i2c_inst_, SDA_PIN, SCL_PIN, BAUDRATE, true);
    sleep_ms(3000);
    printf("i2c ready!\n");

    /*********************************/
    /*   VL53L4CD ranging variables  */
    /*********************************/

    Dev_t dev = I2C_DEV_ADDR;
    uint8_t status, loop, isReady;
    uint16_t sensor_id;
    VL53L4CD_ResultsData_t results;

    /*********************************/
    /*   Power on sensor and init    */
    /*********************************/

    /* (Optional) Check if there is a VL53L4CD sensor connected */
    status = VL53L4CD_GetSensorId(dev, &sensor_id);
    if (status || (sensor_id != 0xEBAA)) {
        printf("VL53L4CD not detected at requested address with status %u\n", status);
        while (1)
            tight_loop_contents();
    } else
        printf("VL53L4CD (0x%X) detected at requested address !\n", sensor_id);

    /* (Mandatory) Init VL53L4CD sensor */
    status = VL53L4CD_SensorInit(dev);
    if (status) {
        printf("VL53L4CD ULD Loading failed with status %u\n", status);
        while (1)
            tight_loop_contents();
    } else
        printf("VL53L4CD ULD ready !\n");

    /*********************************/
    /*     Sensor configuration      */
    /*********************************/

    /* Program the TimingBudget:
     * - higher times results in better accuracy
     * - lower times results in faster ranging
     */

    status = VL53L4CD_SetRangeTiming(dev, 100, 0);  // freq. = 10Hz
    if (status) {
        printf("VL53L4CD_SetRangeTiming failed with status %u\n", status);
        while (1)
            tight_loop_contents();
    } else
        printf("VL53L4CD_SetRangeTiming returned with success !\n");

    /*********************************/
    /*         Ranging loop          */
    /*********************************/

    status = VL53L4CD_StartRanging(dev);
    printf("Start ranging loop ...\n");

    loop = 0;
    while (loop < 1000) {
        /* Use polling function to know when a new measurement is ready.
         * Another way can be to wait for HW interrupt raised on PIN 7
         * (GPIO 1) when a new measurement is ready */

        status = VL53L4CD_CheckForDataReady(dev, &isReady);

        if (isReady) {
            /* (Mandatory) Clear HW interrupt to restart measurements */
            VL53L4CD_ClearInterrupt(dev);

            /* Read measured distance. RangeStatus = 0 means valid data */
            VL53L4CD_GetResult(dev, &results);
            printf("iter. %d: Status = %6u, Distance = %6u, Signal = %6u\n", loop,
                   results.range_status, results.distance_mm, results.signal_per_spad_kcps);
            loop++;
        }

        /* Wait a few ms to avoid too high polling */
        VL53L4CD_WaitMs(dev, 5);
    }

    status = VL53L4CD_StopRanging(dev);
    printf("End of ULD demo\n");
    return status;
}
