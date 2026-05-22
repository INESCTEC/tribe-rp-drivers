#include <stdio.h>
#include "../include/rp_agrolib_vl6180x.h"


#define I2C_ID i2c1
#define I2C_DEV_ADDR 0x29  // default I2C address
#define SDA_PIN 14
#define SCL_PIN 15
#define BAUDRATE 400000


int main() {
    stdio_init_all();

    // Setup I2C
   i2c_inst_t* i2c_1 = init_i2c1(SDA_PIN, SCL_PIN, 400 * 1000, true);
  if(!i2c_setup(i2c_1, SDA_PIN, SCL_PIN, BAUDRATE, true)){
    return 0;
  } else {
    sleep_ms(100); // Short delay to ensure I2C is ready
    printf("I2C setup successful.\n");
  }
   picovl6180x_t tof_sensor;
  printf("Initializing VL6180X sensor...\n");

  picovl6180x_status_t status = picovl6180x_hw_begin_default(
        &tof_sensor, 
        I2C_ID, 
        PICOVL6180X_DEFAULT_ADDR, 
        SDA_PIN, 
        SCL_PIN, 
        BAUDRATE, 
        true 
    );
   picovl6180x_set_snr_check(&tof_sensor, true);
   picovl6180x_set_crosstalk_compensation(&tof_sensor, 56); 

   picovl6180x_set_scaling(&tof_sensor, 1); // Set 1x scaling for max range ~1000mm
   if(status != PICOVL6180X_OK){
    printf("Failed to initialize VL6180X sensor. Status code: %d\n", status);
    return 0;
   } else {
    sleep_ms(1000); // Short delay to ensure sensor is ready
    printf("VL6180X sensor initialized successfully.\n");
    
    
   }

    while (true) {
        int scaling = picovl6180x_get_scaling(&tof_sensor);
        /*uint16_t distance_mm;
        picovl6180x_range_error_t range_err;
        uint8_t range_status_raw;

        picovl6180x_status_t rc = picovl6180x_read_range(&tof_sensor, &distance_mm, &range_err, &range_status_raw);
        if (rc == PICOVL6180X_OK) {
            printf("Distance: %d mm, Range Error: %d, Status Raw: 0x%02X\n",
                   distance_mm, range_err, range_status_raw);
        } else {
            printf("Error reading range data. Status code: %d\n", rc);
        }

        sleep_ms(300); // Read every 300 milliseconds
        */
        //Average over 10 readings
        uint32_t total_distance = 0;
        int valid_readings = 0; 
        const int num_readings = 25;
        float total_signal = 0;

        for (int i = 0; i < num_readings; i++) {
        uint16_t distance_mm;
        float sinal;
        picovl6180x_range_error_t range_err;

            if (picovl6180x_read_range(&tof_sensor, &distance_mm, &range_err, NULL) == PICOVL6180X_OK) {
                
                if (range_err == PICOVL6180X_RANGE_ERROR_NONE) {
                    total_distance += distance_mm;
                    
                }
            }
            if (picovl6180x_read_return_rate(&tof_sensor, &sinal) == PICOVL6180X_OK) {
                    total_signal += sinal;
                    valid_readings++;
                }
                 
            else {
                printf("Warning, reading with erros. Error: %s\n", picovl6180x_range_error_str(range_err));
            }
        }       
            if (valid_readings > 0) {
               int16_t average_distance = total_distance / valid_readings;
               float average_signal = total_signal / valid_readings;
                printf("Average Signal: %.2f Mcps\n", average_signal);
                printf("Average (%d samples): %d mm\n", valid_readings, average_distance);
            } else {
                printf("Error: No valid readings in 10 attempts (excessive light or object absent).\n");
            }        

        
    }

    return 0;
}
