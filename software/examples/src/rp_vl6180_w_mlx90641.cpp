#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "../include/rp_agrolib_vl6180x.h"
#include "../include/rp_agrolib_mlx90641_melexis.h" 


#define I2C_PORT i2c1
#define I2C_SDA_PIN 14  
#define I2C_SCL_PIN 15
#define MLX_I2C_ADDR 0x33
#define VL_I2C_ADDR 0x29

#define TOF_INTERVAL_US 20000 // 50 Hz

#define REFRESH_RATE 0x03 // 0x04 = 8 Hz
#define EMISSIVITY 0.98         
#define NUM_FRAMES_AVG 20

typedef enum {
    STATE_IDLE,
    STATE_APPROACH,
    STATE_CAPTURE
} SystemState;

SystemState current_state = STATE_IDLE;


char serial_buf[256];
int buf_index = 0;

picovl6180x_t tof_sensor;
uint16_t eeMLX90641[832];         //Storage for raw EEPROM data
paramsMLX90641 mlx90641_params;

void setup_single_shot(picovl6180x_t *dev) {

    // 10ms convergence time 
    write_reg8(dev, PICOVL6180X_REG_SYSRANGE__MAX_CONVERGENCE_TIME, 0x0a);
    picovl6180x_set_snr_check(dev, true);
    int8_t offset_mm = -6; 
    write_reg8(&tof_sensor, PICOVL6180X_REG_SYSRANGE__PART_TO_PART_OFFSET, uint8_t(offset_mm));
    picovl6180x_set_crosstalk_compensation(dev, 79, 20); 

    picovl6180x_set_scaling(dev, 1); // Set 1x scaling for range ~100mm
}

void process_tof() {
    write_reg8(&tof_sensor, PICOVL6180X_REG_SYSRANGE__START, 0x01);
    uint8_t int_status = 0;
    int timeout = 0;
    uint8_t result_status = 0;
    uint8_t distance_mm = 0;
        while ((int_status & 0x04) == 0 && timeout < 1000) {
            read_reg8(&tof_sensor, PICOVL6180X_REG_RESULT__INTERRUPT_STATUS_GPIO, &int_status);
            sleep_us(500);
            timeout ++;
        }
    
    read_reg8(&tof_sensor, PICOVL6180X_REG_RESULT__RANGE_STATUS, &result_status);

    if (result_status >> 4 != 0) {
        printf("$$tof,-1.00##\n");
    } else {
        read_reg8(&tof_sensor, PICOVL6180X_REG_RESULT__RANGE_VAL, &distance_mm);
        float corrected_distance = (1.02f * (float)distance_mm) + 4.90f;
        printf("$$tof,%.2f##\n", corrected_distance);
    }
    write_reg8(&tof_sensor, PICOVL6180X_REG_SYSTEM__INTERRUPT_CLEAR, 0x07);
}
void setup_mlx90641() {


    MLX90641_SetRefreshRate(MLX_I2C_ADDR, REFRESH_RATE);
    MLX90641_DumpEE(MLX_I2C_ADDR, eeMLX90641);
    MLX90641_ExtractParameters(eeMLX90641, &mlx90641_params);
}
void process_thermal() {

    uint16_t frameData[242]; // Buffer for raw foton data (192 pixels + metadata)
    float mlx90641To[192];   // Final output array for temperatures in Celsius

    MLX90641_GetFrameData(MLX_I2C_ADDR, frameData);
    float Ta = MLX90641_GetTa(frameData, &mlx90641_params);
    float Vdd = MLX90641_GetVdd(frameData, &mlx90641_params);
    float Tr = Ta;
    MLX90641_CalculateTo(frameData, &mlx90641_params, EMISSIVITY, Tr, mlx90641To);
    MLX90641_BadPixelsCorrection(mlx90641_params.brokenPixels, mlx90641To, &mlx90641_params);

    printf("$$thermal,%.2f,", Ta);
    for (int i = 0; i < 192; i++) {
        printf("%.2f", mlx90641To[i]);
        if (i < 191) printf(",");
    }
    printf("##\n");
}

void process_ros() {
    if (strstr(serial_buf, "$$cmd,start_tof##") != NULL) {
        current_state = STATE_APPROACH;
    }
    else if (strstr(serial_buf, "$$req,thermal##") != NULL) {
        current_state = STATE_CAPTURE;
    }
    else if (strstr(serial_buf, "$$cmd,stop##") != NULL) {
        current_state = STATE_IDLE; //EMERGNCY STOP OR RESET
    }
    
    
    memset(serial_buf, 0, sizeof(serial_buf));
    buf_index = 0;
}


int main() {
    stdio_init_all();

    picovl6180x_status_t status = picovl6180x_hw_begin_default(
        &tof_sensor, 
        i2c1, 
        VL_I2C_ADDR, 
        I2C_SDA_PIN, 
        I2C_SCL_PIN, 
        400*1000, 
        true 
    );

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

    sleep_ms(580); // Power on reset delay
    
    setup_single_shot(&tof_sensor);
    setup_mlx90641();

    uint64_t tof_last_time = time_us_64();

    while (true) {
        uint64_t time = time_us_64();

        if (current_state == STATE_APPROACH) {
            if (time - tof_last_time >= TOF_INTERVAL_US) {
                process_tof();
                tof_last_time = time;
            }
        } 
        else if (current_state == STATE_CAPTURE) {
            process_thermal();
            current_state = STATE_IDLE; 
        }

        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            serial_buf[buf_index++] = (char)c;
            
            
            if (c == '#' && buf_index > 1 && serial_buf[buf_index-2] == '#') {
                serial_buf[buf_index] = '\0';
                process_ros();
            } 
    
            else if (buf_index >= 255) {
                memset(serial_buf, 0, sizeof(serial_buf));
                buf_index = 0;
            }
        }
    }
    return 0;
}