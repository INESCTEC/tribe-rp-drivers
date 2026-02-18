
#include "../include/rp_agrolib_ads8354.h"


void ADS8354::read_values(int16_t *valueA, int16_t *valueB) {
    uint16_t read_value[3] = {0x00}, in_buf[3];
    gpio_put(CS_pin, 0);
    if (mode == SINGLE) {
        spi_write16_read16_blocking(spi1, read_value, in_buf, 2);
    } else if (mode == DOUBLE) {
        spi_write16_read16_blocking(spi1, read_value, in_buf, 3);
    }

    *valueA = (int16_t) in_buf[1];
    *valueB = (int16_t) in_buf[2];
    /*for(int i = 0; i<3; i++){
        printf("%d\n",(int16_t)in_buf[i]);
    }*/
    gpio_put(CS_pin, 1);
}

uint16_t ADS8354::read_registers() {
    uint16_t read_reg[3] = {0x3000};
    int16_t valueA, valueB;
    gpio_put(CS_pin, 0);
    spi_write16_blocking(spi1, read_reg, 3);
    gpio_put(CS_pin, 1);
    sleep_ms(1);

    // leitura dos regs
    uint16_t read_value[3] = {0x00}, in_buf[3];
    gpio_put(CS_pin, 0);
    spi_write16_read16_blocking(spi1, read_value, in_buf, 3);

    gpio_put(CS_pin, 1);
    printf("%d\n", in_buf[0]);

    return in_buf[0];
}

void ADS8354::write_configuration(int mode) {
    uint16_t out_buf[3] = {0x8400};
    gpio_put(CS_pin, 0);
    spi_write16_blocking(spi1, out_buf, 3);
    gpio_put(CS_pin, 1);
    sleep_ms(1);
}


ADS8354::ADS8354(int CS_pin_, int SCLK_pin_, int SDI_pin_, int SDO_pin_, int mode_) {
    CS_pin = CS_pin_;
    SCLK_pin = SCLK_pin_;
    SDI_pin = SDI_pin_;
    SDO_pin = SDO_pin_;
    mode = mode_;


    // SPI
    spi_init(spi1, 26 * 1000000);
    spi_set_format(spi1, 16, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(SDO_pin, GPIO_FUNC_SPI);
    gpio_set_function(SCLK_pin, GPIO_FUNC_SPI);
    gpio_set_function(SDI_pin, GPIO_FUNC_SPI);

    // CS é controlado manualmente porque na biblioteca da rp2040 ao fim de 16 bits este é posto
    // momentanenamente a HIGH (para este adc precisamos de enviar 48 clocks seguidos)
    gpio_init(CS_pin);
    gpio_set_dir(CS_pin, GPIO_OUT);
    gpio_put(CS_pin, 1);

    if (mode == DOUBLE) {
        write_configuration(mode);
        int reg = read_registers();
    }
}
