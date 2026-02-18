/*
 * File: ads1x15.c
 * Description:
 * ADC ADS1015 ADS1115 library c file for RPI Rp2040 PICO C++ SDK
 * Description: See URL for full details.
 * URL: https://github.com/gavinlyonsrepo/ADS1x15_PICO
 */

#include "../include/rp_agrolib_ads1115.h"


// Constructor ADS1115 sub class
void PICO_ADS1115_init(PICO_ADS1X15 *ads1115) {
    ads1115->_BitShift = 0;
    ads1115->_ADCGain = ADSXGain_TWOTHIRDS;
    ads1115->_DataRate = RATE_ADS1115_128SPS;
}


// Desc :: Sets up the I2C interface
// Param1 : enum ADSX_AddressI2C_e : I2C address 8 bit address
// Param2 : i2c_inst_t* : I2C instance of port, IC20 or I2C1
// Param3 : uint16_t : I2C Bus Clock speed in KHz. see 8.5.1.3 datasheet
// Param4 : uint8_t : I2C Data pin
// Param5 : uint8_t : I2C Clock pin
// Returns : bool :true if successful, otherwise false
bool PICO_ADS1X15_beginADSX(PICO_ADS1X15 *ads1x15) {
    int ReturnCode = 0;
    uint8_t rxData = 0;

    // check connection?
    ReturnCode = i2c_read_timeout_us(ads1x15->_i2c, ads1x15->_AddresI2C, &rxData, 1, false,
                                     ADSX_I2C_DELAY);
    if (ReturnCode < 1) {  // no bytes read back from device or error issued
#ifdef ADS_SERIAL_DEBUG
        printf("1201  PICO_ADS1X15::begin: \r\n");
        printf("Check Connection, Return code :: %d ,RX data :: %u \r\n", ReturnCode, rxdata);
#endif
        return false;
    }
    return true;
}

// Switch off the I2C
void PICO_ADS1X15_deinitI2C(PICO_ADS1X15 *ads1x15) {
    gpio_set_function(ads1x15->_SDataPin, GPIO_FUNC_NULL);
    gpio_set_function(ads1x15->_SClkPin, GPIO_FUNC_NULL);
    i2c_deinit(ads1x15->_i2c);
}

// Desc :: Sets the gain and input voltage range
// Param 1 :: enum ADSXGain_e , Gain setting to use
void PICO_ADS1X15_setGain(PICO_ADS1X15 *ads1x15, ADSXGain_e gain) {
    ads1x15->_ADCGain = gain;
}

// Desc :: Gets the gain and input voltage range
// Return 1 :: enum ADSXGain_e , Gain setting in use
ADSXGain_e PICO_ADS1X15_getGain(PICO_ADS1X15 *ads1x15) {
    return ads1x15->_ADCGain;
}

// Desc :: Sets the data rate
// Param 1 :: uint16_t : Data rate to use
void PICO_ADS1X15_setDataRate(PICO_ADS1X15 *ads1x15, uint16_t rate) {
    ads1x15->_DataRate = rate;
}

// Desc :: Gets the current data rate
// returns :: uint16_t : Data rate in use
uint16_t PICO_ADS1X15_getDataRate(PICO_ADS1X15 *ads1x15) {
    return ads1x15->_DataRate;
}

// Desc :: Gets a single-ended ADC reading from the specified channel
// Param1 :: enum ADSX_AINX_e : Adc channel to read 0-3
// Return :: ADC reading
int16_t PICO_ADS1X15_readADC_SingleEnded(PICO_ADS1X15 *ads1x15, ADSX_AINX_e channel) {
    if (channel > 3) {
        return 0;
    }

    switch (channel) {
        case 0:
            PICO_ADS1X15_startADCReading(ads1x15, ADSXRegConfigMuxSingle_0, ADSSingleShotMode);
            break;
        case 1:
            PICO_ADS1X15_startADCReading(ads1x15, ADSXRegConfigMuxSingle_1, ADSSingleShotMode);
            break;
        case 2:
            PICO_ADS1X15_startADCReading(ads1x15, ADSXRegConfigMuxSingle_2, ADSSingleShotMode);
            break;
        case 3:
            PICO_ADS1X15_startADCReading(ads1x15, ADSXRegConfigMuxSingle_3, ADSSingleShotMode);
            break;
    }

    // Wait for the conversion to complete
    while (!PICO_ADS1X15_conversionComplete(ads1x15))
        ;
    // Read the conversion results
    return PICO_ADS1X15_getLastConversionResults(ads1x15);
}

// Desc :: Reads the conversion results difference between the P (AIN0) and N (AIN1) input.
// Returns :: int16_t : The ADC reading a signed value since the difference
// can be either positive or negative.
int16_t PICO_ADS1X15_readADC_Diff01(PICO_ADS1X15 *ads1x15) {
    PICO_ADS1X15_startADCReading(ads1x15, ADSXRegConfigMuxDiff_0_1, ADSSingleShotMode);
    while (!PICO_ADS1X15_conversionComplete(ads1x15))
        ;  // Wait for the conversion to complete

    return PICO_ADS1X15_getLastConversionResults(ads1x15);  // Read the conversion results
}

// Desc :: Reads the conversion results difference between the P (AIN0) and N (AIN3) input.
// Returns :: int16_t : The ADC reading a signed value since the difference
// can be either positive or negative.
int16_t PICO_ADS1X15_readADC_Diff03(PICO_ADS1X15 *ads1x15) {
    PICO_ADS1X15_startADCReading(ads1x15, ADSXRegConfigMuxDiff_0_3, ADSSingleShotMode);

    // Wait for the conversion to complete
    while (!PICO_ADS1X15_conversionComplete(ads1x15))
        ;

    // Read the conversion results
    return PICO_ADS1X15_getLastConversionResults(ads1x15);
}

// Desc :: Reads the conversion results difference between the P (AIN1) and N (AIN3) input.
// Returns :: int16_t : The ADC reading a signed value since the difference
// can be either positive or negative.
int16_t PICO_ADS1X15_readADC_Diff13(PICO_ADS1X15 *ads1x15) {
    PICO_ADS1X15_startADCReading(ads1x15, ADSXRegConfigMuxDiff_1_3, ADSSingleShotMode);

    // Wait for the conversion to complete
    while (!PICO_ADS1X15_conversionComplete(ads1x15))
        ;

    // Read the conversion results
    return PICO_ADS1X15_getLastConversionResults(ads1x15);
}

// Desc :: Reads the conversion results difference between the P (AIN2) and N (AIN3) input.
// Returns :: int16_t : The ADC reading a signed value since the difference
// can be either positive or negative.
int16_t PICO_ADS1X15_readADC_Diff23(PICO_ADS1X15 *ads1x15) {
    PICO_ADS1X15_startADCReading(ads1x15, ADSXRegConfigMuxDiff_2_3, ADSSingleShotMode);

    // Wait for the conversion to complete
    while (!PICO_ADS1X15_conversionComplete(ads1x15))
        ;
    // Read the conversion results
    return PICO_ADS1X15_getLastConversionResults(ads1x15);
}

// Desc :: Sets up the comparator to operate in basic mode
// Param 1 :: enum ADSX_AINX_e : ADC channel to use 0-3
// Param 2 :: int16_t : threshold comparator
// Notes :: the ALERT/RDY pin to assert (go from high to low) when the ADC
// value exceeds the specified threshold.
// This will also set the ADC in continuous conversion mode.
void PICO_ADS1X15_startComparator_SingleEnded(PICO_ADS1X15 *ads1x15, ADSX_AINX_e channel,
                                              int16_t threshold) {
    uint16_t configuration = ADSX_REG_CONFIG_CQUE_1CONV |    // Comparator enabled and asserts on 1
                                                             // match
                             ADSX_REG_CONFIG_CLAT_LATCH |    // Latching mode
                             ADSX_REG_CONFIG_CPOL_ACTVLOW |  // Alert/Rdy active low   (default val)
                             ADSX_REG_CONFIG_CMODE_TRAD |    // Traditional comparator (default val)
                             ADSX_REG_CONFIG_MODE_CONTIN |   // Continuous conversion mode
                             ADSX_REG_CONFIG_MODE_CONTIN;    // Continuous conversion mode

    configuration |= ads1x15->_ADCGain;
    configuration |= ads1x15->_DataRate;
    switch (channel) {
        case 0:
            configuration |= ADSXRegConfigMuxSingle_0;
            break;
        case 1:
            configuration |= ADSXRegConfigMuxSingle_1;
            break;
        case 2:
            configuration |= ADSXRegConfigMuxSingle_2;
            break;
        case 3:
            configuration |= ADSXRegConfigMuxSingle_3;
            break;
    }

    // Set the high threshold register
    // Shift 12-bit results left 4 bits for the ADS1015
    PICO_ADS1X15_writeRegister(ads1x15, ADSX_REG_POINTER_HITHRESH, threshold << ads1x15->_BitShift);
    // Write config register to the ADC
    PICO_ADS1X15_writeRegister(ads1x15, ADSX_REG_POINTER_CONFIG, configuration);
}

// Desc :: In order to clear the comparator, we need to read the conversion results.
// This function reads the last conversion results without changing the config value.
// Returns :: the last ADC reading
int16_t PICO_ADS1X15_getLastConversionResults(PICO_ADS1X15 *ads1x15) {
    // Read the conversion results
    uint16_t converisonResults =
            PICO_ADS1X15_readRegister(ads1x15, ADSX_REG_POINTER_CONVERT) >> ads1x15->_BitShift;
    if (ads1x15->_BitShift == 0) {
        return (int16_t) converisonResults;
    } else {
        // Shift 12-bit results right 4 bits for the ADS1015,
        // making sure we keep the sign bit intact
        if (converisonResults > 0x07FF) {
            // negative number - extend the sign to 16th bit
            converisonResults |= 0xF000;
        }
        return (int16_t) converisonResults;
    }
}

// Desc :: Returns true if conversion is complete, false otherwise
// Param1 :: The ADC reading in raw counts
// Returns :: the ADC reading in voltage.
// Notes :: see data sheet Table 3
float PICO_ADS1X15_computeVolts(PICO_ADS1X15 *ads1x15, int16_t counts) {
    float FullScaleRange;
    switch (ads1x15->_ADCGain) {
        case ADSXGain_TWOTHIRDS:
            FullScaleRange = 6.144f;
            break;
        case ADSXGain_ONE:
            FullScaleRange = 4.096f;
            break;
        case ADSXGain_TWO:
            FullScaleRange = 2.048f;
            break;
        case ADSXGain_FOUR:
            FullScaleRange = 1.024f;
            break;
        case ADSXGain_EIGHT:
            FullScaleRange = 0.512f;
            break;
        case ADSXGain_SIXTEEN:
            FullScaleRange = 0.256f;
            break;
        default:
            FullScaleRange = 0.0f;
    }
    return counts * (FullScaleRange / (32768 >> ads1x15->_BitShift));
}

// Desc :: Non-blocking start conversion function
// Param1 :: enum ADSXRegConfig_e :: Mux field value
// Param2 :: enum :: Config mode , Continuous conversion or single shot
// Notes :: Call getLastConversionResults() once conversionComplete() returns true.
// In continuous mode, getLastConversionResults() will always return the
// latest result. ALERT/RDY pin is set to RDY mode, and a 8us pulse is generated every
// time new data is ready.
void PICO_ADS1X15_startADCReading(PICO_ADS1X15 *ads1x15, ADSXRegConfig_e mux,
                                  ADSXConfigMode_e ConfigMode) {
    uint16_t configuration = ADSX_REG_CONFIG_CQUE_1CONV |    // Set CQUE to any value other than
                                                             // None so we can use it in RDY mode
                             ADSX_REG_CONFIG_CLAT_NONLAT |   // Non-latching (default val)
                             ADSX_REG_CONFIG_CPOL_ACTVLOW |  // Alert/Rdy active low   (default val)
                             ADSX_REG_CONFIG_CMODE_TRAD;     // Traditional comparator (default val)

    if (ConfigMode == ADSContinuousMode) {
        configuration |= ADSX_REG_CONFIG_MODE_CONTIN;
    } else {
        configuration |= ADSX_REG_CONFIG_MODE_SINGLE;
    }

    configuration |= ads1x15->_ADCGain;
    configuration |= ads1x15->_DataRate;
    configuration |= mux;  // Set channel

    // Set start single-conversion bit
    configuration |= ADSX_REG_CONFIG_OS_SINGLE;
    // Write config register to the ADC
    PICO_ADS1X15_writeRegister(ads1x15, ADSX_REG_POINTER_CONFIG, configuration);
    // Set ALERT/RDY to RDY mode.
    PICO_ADS1X15_writeRegister(ads1x15, ADSX_REG_POINTER_HITHRESH, 0x8000);
    PICO_ADS1X15_writeRegister(ads1x15, ADSX_REG_POINTER_LOWTHRESH, 0x0000);
}

// Desc :: Checks whether conversion is complete
// Returns :: bool : true if conversion is complete, false otherwise
bool PICO_ADS1X15_conversionComplete(PICO_ADS1X15 *ads1x15) {
    return (PICO_ADS1X15_readRegister(ads1x15, ADSX_REG_POINTER_CONFIG) & 0x8000) != 0;
}

// Desc :: Write 16-bits to the specified destination register
// Param 1 :: uint8_t : register address to write
// Param 2 :: uint16_t : Value to write to register
void PICO_ADS1X15_writeRegister(PICO_ADS1X15 *ads1x15, uint8_t reg, uint16_t value) {
    ads1x15->_dataBuffer[0] = reg;
    ads1x15->_dataBuffer[1] = value >> 8;
    ads1x15->_dataBuffer[2] = value & 0xFF;
    int ReturnCode = 0;

    ReturnCode = i2c_write_timeout_us(ads1x15->_i2c, ads1x15->_AddresI2C, ads1x15->_dataBuffer, 3,
                                      false, ADSX_I2C_DELAY);
    if (ReturnCode < 1) {
#ifdef ADS_SERIAL_DEBUG
        printf("1203 data: \r\n");
        printf("I2C error :: writeRegister \r\n");
        printf("Tranmission code : %d \r\n", ReturnCode);
        busy_wait_ms(100);
#endif
    }
}

// Desc :: Read 16-bits from the specified destination register
// Param 1 :: uint8_t : register address to read from
// Return :: uint16_t : Value to read to register
uint16_t PICO_ADS1X15_readRegister(PICO_ADS1X15 *ads1x15, uint8_t registerRead) {
    ads1x15->_dataBuffer[0] = registerRead;

    int ReturnCode = 0;
    ReturnCode = i2c_write_timeout_us(ads1x15->_i2c, ads1x15->_AddresI2C, ads1x15->_dataBuffer, 1,
                                      false, ADSX_I2C_DELAY);
    if (ReturnCode < 1) {
#ifdef ADS_SERIAL_DEBUG
        printf("1201 error I2C readRegister A: \r\n");
        printf("Tranmission code : %d \r\n", ReturnCode);
        busy_wait_ms(100);
#endif
    }
    ReturnCode = 0;

    ReturnCode = i2c_read_timeout_us(ads1x15->_i2c, ads1x15->_AddresI2C, ads1x15->_dataBuffer, 2,
                                     false, ADSX_I2C_DELAY);
    if (ReturnCode < 1) {  // no bytes read back from device or error issued
#ifdef ADS_SERIAL_DEBUG
        printf("1202 I2C Error readRegister B: \r\n");
        printf("Tranmission Code :: %d\r\n", ReturnCode);
        busy_wait_ms(100);
#endif
    }

    return ((ads1x15->_dataBuffer[0] << 8) | ads1x15->_dataBuffer[1]);
}
