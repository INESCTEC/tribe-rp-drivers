#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/time.h"


#include "../include/rp_agrolib_mlx90641_melexis.h" 

#define I2C_PORT i2c1
#define BAUDRATE 400000
#define SDA_PIN 14
#define SCL_PIN 15
#define MLX_I2C_ADDR 0x33

#define REFRESH_RATE 0x03       // 0x04 = 8 Hz
#define EMISSIVITY 0.98         
#define NUM_FRAMES_AVG 20



int main() {
    stdio_init_all();
    
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(1000);
    
    
    //I2C Initialization
    i2c_init(I2C_PORT, BAUDRATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    int refresh_rate_status = MLX90641_SetRefreshRate(MLX_I2C_ADDR, REFRESH_RATE);
    if (refresh_rate_status != 0) {
        printf("Erro ao configurar taxa de atualização: %d\n", refresh_rate_status);
    }
    printf("Taxa de atualizacao configurada para 0x%02X\n", REFRESH_RATE);
    
    sleep_ms(580); // Power on reset delay

    // Memory allocation for EEPROM data and parameters
    uint16_t eeMLX90641[832];         //Storage for raw EEPROM data
    paramsMLX90641 mlx90641_params;

    int status = MLX90641_DumpEE(MLX_I2C_ADDR, eeMLX90641);
    if (status != 0) {
        printf("Fatal error, EEPROM data corrupted or hamming code error: %d\n", status);
        return -1;
    }

    status = MLX90641_ExtractParameters(eeMLX90641, &mlx90641_params);
    if (status != 0) {
        printf("AVISO/ERRO: Problema ao extrair parametros (Erro: %d)\n", status);
        return -1;
    }

    printf("Setup Concluido. A iniciar streaming...\n\n");

    
    uint16_t frameData[242]; // Buffer for raw foton data (192 pixels + metadata)
    float mlx90641To[192];   // Final output array for temperatures in Celsius
    float frameAccum[192] = {0};
    float Ta_accum = 0;
    int frameCount = 0;

    while (1) {
        status = MLX90641_GetFrameData(MLX_I2C_ADDR, frameData);
        
        
        if (status < 0) {
            printf("Erro de leitura do frame I2C\n");
            continue;
        }

    
        float Ta = MLX90641_GetTa(frameData, &mlx90641_params);
        float Vdd = MLX90641_GetVdd(frameData, &mlx90641_params);
        printf("Ta: %.2f ºC, Vdd: %.2f V\n", Ta, Vdd);
        // assumimos que é igual à Ta (menos ~8C em céu limpo, ou igual dentro da estufa)
        float Tr = Ta; 

        MLX90641_CalculateTo(frameData, &mlx90641_params, EMISSIVITY, Tr, mlx90641To);

        MLX90641_BadPixelsCorrection(mlx90641_params.brokenPixels, mlx90641To, &mlx90641_params);

        printf("Emissivity: %.2f \n", EMISSIVITY);
        Ta_accum += Ta;
        for (int i = 0; i < 192; i++) {
                frameAccum[i] += mlx90641To[i];
            }
        frameCount++;

        if (frameCount >= NUM_FRAMES_AVG) {
            printf("%.2f,", Ta_accum / NUM_FRAMES_AVG);
            for (int i = 0; i < 192; i++) {
                printf("%.2f", frameAccum[i] / NUM_FRAMES_AVG);
                if (i < 191) printf(",");
                else printf(" Fim de Frame\n");
            }
        // reset
        for (int i = 0; i < 192; i++) frameAccum[i] = 0;
        Ta_accum = 0;
        frameCount = 0;
        }
        /*printf("%.2f,", Ta);
            for (int i = 0; i < 192; i++) {
                printf("%.2f", mlx90641To[i]);
            if (i < 191) {
                printf(",");
            }else{
                printf(" Fim de Frame\n");
            }
    }*/
         
        
}
    return 0;
}