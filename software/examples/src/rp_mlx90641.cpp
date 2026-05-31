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

#define REFRESH_RATE 0x04       // 0x04 = 8 Hz
#define EMISSIVITY 0.98         


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

    MLX90641_SetRefreshRate(MLX_I2C_ADDR, REFRESH_RATE);
    //printf("Taxa de atualizacao configurada para 0x%02X\n", REFRESH_RATE);
    
    sleep_ms(330); // Power on reset delay

    // Memory allocation for EEPROM data and parameters
    uint16_t eeMLX90641[832];         //Storage for raw EEPROM data
    paramsMLX90641 mlx90641_params;

    int status = MLX90641_DumpEE(MLX_I2C_ADDR, eeMLX90641);
    if (status != 0) {
        printf("Fatal error, EEPROM data corrupted or hamming code error: %d\n", status);
        while (1) sleep_ms(1000);
    }

    status = MLX90641_ExtractParameters(eeMLX90641, &mlx90641_params);
    if (status != 0) {
        printf("AVISO/ERRO: Problema ao extrair parametros (Erro: %d)\n", status);
    }

    //printf("Setup Concluido. A iniciar streaming de dados termicos...\n\n");

    
    uint16_t frameData[242]; // Buffer for raw foton data (192 pixels + metadata)
    float mlx90641To[192];   // Final output array for temperatures in Celsius

    while (1) {
        status = MLX90641_GetFrameData(MLX_I2C_ADDR, frameData);
        
        if (status < 0) {
            printf("Erro de leitura do frame I2C\n");
            continue;
        }

    
        float Ta = MLX90641_GetTa(frameData, &mlx90641_params);
        
        // Define a Temperatura Refletida (Tr). Sem papel de alumínio, assumimos que é igual à Ta (menos ~8C em céu limpo, ou igual dentro da estufa)
        float Tr = Ta; 

        MLX90641_CalculateTo(frameData, &mlx90641_params, EMISSIVITY, Tr, mlx90641To);

        MLX90641_BadPixelsCorrection(mlx90641_params.brokenPixels, mlx90641To, &mlx90641_params);

        
        //printf("Emissivity: %.2f \n", EMISSIVITY);
        printf("%.2f,", Ta);
        // Imprime a matriz no formato esperado (ex: Valores separados por vírgula)
        // O Processing costuma ler linhas limpas. Ajusta o formato printf conforme a tua app antiga.
        for (int i = 0; i < 192; i++) {
            printf("%.2f", mlx90641To[i]);
            if (i < 191) {
                printf(","); // Adiciona vírgula entre os valores
            }else{
                printf("\n"); // Quebra de linha ao final do frame}
        }

        // Pausa para estabilizar (se necessário, pois o GetFrameData já impõe o tempo do refresh rate)
        sleep_ms(10); 
    }
}
    return 0;
}