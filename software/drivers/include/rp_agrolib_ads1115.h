/*
 This code was converted to C from the following repository https://github.com/addicore/ADS1115.git
 (available as submodule of rp2040_agrolib)
*/

#ifndef ADS1X15LIB_H
#define ADS1X15LIB_H

#include <stdio.h>

#include "pico/stdlib.h"

#include "hardware/i2c.h"

#define ADSX_I2C_DELAY 50000

typedef enum {
    ADSX_ADDRESS_GND = 0x48,
    ADSX_ADDRESS_VDD = 0x49,
    ADSX_ADDRESS_SDA = 0x4A,
    ADSX_ADDRESS_SCLK = 0x4B
} ADSXAddressI2C_e;

#define ADSX_REG_POINTER_MASK (0x03)
#define ADSX_REG_POINTER_CONVERT (0x00)
#define ADSX_REG_POINTER_CONFIG (0x01)
#define ADSX_REG_POINTER_LOWTHRESH (0x02)
#define ADSX_REG_POINTER_HITHRESH (0x03)

#define ADSX_REG_CONFIG_OS_MASK (0x8000)
#define ADSX_REG_CONFIG_OS_SINGLE (0x8000)
#define ADSX_REG_CONFIG_OS_BUSY (0x0000)
#define ADSX_REG_CONFIG_OS_NOTBUSY (0x8000)

typedef enum {
    ADSXRegConfigMuxDiff_0_1 = 0x0000,
    ADSXRegConfigMuxDiff_0_3 = 0x1000,
    ADSXRegConfigMuxDiff_1_3 = 0x2000,
    ADSXRegConfigMuxDiff_2_3 = 0x3000,
    ADSXRegConfigMuxSingle_0 = 0x4000,
    ADSXRegConfigMuxSingle_1 = 0x5000,
    ADSXRegConfigMuxSingle_2 = 0x6000,
    ADSXRegConfigMuxSingle_3 = 0x7000
} ADSXRegConfig_e;

typedef enum { ADSX_AIN0 = 0, ADSX_AIN1 = 1, ADSX_AIN2 = 2, ADSX_AIN3 = 3 } ADSX_AINX_e;

#define ADSX_REG_CONFIG_PGA_MASK (0x0E00)
#define ADSX_REG_CONFIG_PGA_6_144V (0x0000)
#define ADSX_REG_CONFIG_PGA_4_096V (0x0200)
#define ADSX_REG_CONFIG_PGA_2_048V (0x0400)
#define ADSX_REG_CONFIG_PGA_1_024V (0x0600)
#define ADSX_REG_CONFIG_PGA_0_512V (0x0800)
#define ADSX_REG_CONFIG_PGA_0_256V (0x0A00)
#define ADSX_REG_CONFIG_MODE_MASK (0x0100)
#define ADSX_REG_CONFIG_MODE_CONTIN (0x0000)
#define ADSX_REG_CONFIG_MODE_SINGLE (0x0100)

typedef enum {
    ADSContinuousMode = 1,
    ADSSingleShotMode = 0,
} ADSXConfigMode_e;

#define ADSX_REG_CONFIG_RATE_MASK (0x00E0)
#define ADSX_REG_CONFIG_CMODE_MASK (0x0010)
#define ADSX_REG_CONFIG_CMODE_TRAD (0x0000)
#define ADSX_REG_CONFIG_CMODE_WINDOW (0x0010)
#define ADSX_REG_CONFIG_CPOL_MASK (0x0008)
#define ADSX_REG_CONFIG_CPOL_ACTVLOW (0x0000)
#define ADSX_REG_CONFIG_CPOL_ACTVHI (0x0008)

#define ADSX_REG_CONFIG_CLAT_MASK (0x0004)
#define ADSX_REG_CONFIG_CLAT_NONLAT (0x0000)
#define ADSX_REG_CONFIG_CLAT_LATCH (0x0004)

#define ADSX_REG_CONFIG_CQUE_MASK (0x0003)
#define ADSX_REG_CONFIG_CQUE_1CONV (0x0000)
#define ADSX_REG_CONFIG_CQUE_2CONV (0x0001)
#define ADSX_REG_CONFIG_CQUE_4CONV (0x0002)
#define ADSX_REG_CONFIG_CQUE_NONE (0x0003)

typedef enum {
    ADSXGain_TWOTHIRDS = ADSX_REG_CONFIG_PGA_6_144V,
    ADSXGain_ONE = ADSX_REG_CONFIG_PGA_4_096V,
    ADSXGain_TWO = ADSX_REG_CONFIG_PGA_2_048V,
    ADSXGain_FOUR = ADSX_REG_CONFIG_PGA_1_024V,
    ADSXGain_EIGHT = ADSX_REG_CONFIG_PGA_0_512V,
    ADSXGain_SIXTEEN = ADSX_REG_CONFIG_PGA_0_256V
} ADSXGain_e;

#define RATE_ADS1015_128SPS (0x0000)
#define RATE_ADS1015_250SPS (0x0020)
#define RATE_ADS1015_490SPS (0x0040)
#define RATE_ADS1015_920SPS (0x0060)
#define RATE_ADS1015_1600SPS (0x0080)
#define RATE_ADS1015_2400SPS (0x00A0)
#define RATE_ADS1015_3300SPS (0x00C0)

#define RATE_ADS1115_8SPS (0x0000)
#define RATE_ADS1115_16SPS (0x0020)
#define RATE_ADS1115_32SPS (0x0040)
#define RATE_ADS1115_64SPS (0x0060)
#define RATE_ADS1115_128SPS (0x0080)
#define RATE_ADS1115_250SPS (0x00A0)
#define RATE_ADS1115_475SPS (0x00C0)
#define RATE_ADS1115_860SPS (0x00E0)

typedef struct {
    uint8_t _BitShift;
    ADSXGain_e _ADCGain;
    uint16_t _DataRate;
    i2c_inst_t *_i2c;
    uint8_t _SDataPin;
    uint8_t _SClkPin;
    uint16_t _CLKSpeed;
    ADSXAddressI2C_e _AddresI2C;
    uint8_t _dataBuffer[3];
} PICO_ADS1X15;

bool PICO_ADS1X15_beginADSX(PICO_ADS1X15 *ads1x15);
void PICO_ADS1X15_deinitI2C();

void PICO_ADS1X15_setGain(PICO_ADS1X15 *ads1x15, ADSXGain_e gain);
ADSXGain_e PICO_ADS1X15_getGain();
void PICO_ADS1X15_setDataRate(PICO_ADS1X15 *ads1x15, uint16_t rate);
uint16_t PICO_ADS1X15_getDataRate();
uint16_t PICO_ADS1X15_readRegister(PICO_ADS1X15 *ads1x15, uint8_t registerRead);


int16_t PICO_ADS1X15_readADC_SingleEnded(PICO_ADS1X15 *ads1x15, ADSX_AINX_e channel);
void PICO_ADS1X15_startADCReading(PICO_ADS1X15 *ads1x15, ADSXRegConfig_e mux,
                                  ADSXConfigMode_e ConfigMode);
void PICO_ADS1X15_startComparator_SingleEnded(PICO_ADS1X15 *ads1x15, ADSX_AINX_e channel,
                                              int16_t threshold);

int16_t PICO_ADS1X15_readADC_Diff01();
int16_t PICO_ADS1X15_readADC_Diff03();
int16_t PICO_ADS1X15_readADC_Diff13();
int16_t PICO_ADS1X15_readADC_Diff23();

int16_t PICO_ADS1X15_getLastConversionResults();
float PICO_ADS1X15_computeVolts(PICO_ADS1X15 *ads1x15, int16_t counts);
bool PICO_ADS1X15_conversionComplete();

#endif  // ADS1X15LIB_H
