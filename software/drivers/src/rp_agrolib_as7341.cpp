#include "../include/rp_agrolib_as7341.h"


void AS7341::PowerOn() {
    // Sets the Atime for integration time from 0 to 255 in register (0x81), integration time =
    // (ATIME + 1) * (ASTEP + 1) * 2.78µS
    setATIME(uint8_t(0x64));  // original value
    // setATIME(uint8_t (0x09));
    // Sets the Astep for integration time from 0 to 65535 in register (0xCA[7:0]) and (0xCB[15:8]),
    // integration time = (ATIME + 1) * (ASTEP + 1) * 2.78µS
    setASTEP(uint8_t(0xE7), uint8_t(0x03));
    // Sets the Spectral Gain in CFG1 Register (0xAA) in [4:0] bit
    setGAIN(uint8_t(0x08));
    // Setting the PON bit in Enable register 0x80
    PON();
}

bool AS7341::SetIntTime(long int int_time) {
    uint8_t atime, astep_low, astep_high;
    uint16_t astep;
    bool result;

    if (int_time > 3 && int_time < 46974771) {
        if (int_time >= 3 && int_time < 717) {  // change ASTEP, astep = 0
            atime = int_time / 2.8;
            setASTEP(uint8_t(0x00), uint8_t(0x00));
            setATIME(atime);
            // printf("atime: %d\n", atime);

        } else {  // change astep, fix atime at max
            astep = int_time / (2.8 * 256) - 1;
            astep_low = astep & 0x00FF;
            astep_high = astep >> 8;
            setASTEP(astep_low, astep_high);
            setATIME(uint8_t(0xFF));
            // printf("astep: %d\n", astep);
            result = true;
        }
    } else {
        printf("[AS7341]:: Invalid integration time");
        result = false;
    }
    return result;
}

uint32_t AS7341::GetIntTime() {
    uint8_t atime, astep_low, astep_high;
    uint32_t int_time;
    atime = readRegister(uint8_t(0x81));
    astep_low = readRegister(uint8_t(0xCA));
    astep_high = readRegister(uint8_t(0xCB));
    uint16_t astep = ((uint16_t) astep_high << 8) | astep_low;
    int_time = (astep + 1) * (atime + 1) * 2.8;
    // printf("intTime:  %x %x %x\n", atime, astep_low, astep_high);
    // printf("t: %d\n", int_time);
    return int_time;
}

void AS7341::ReadLight(Channels *channels) {
    bool isEnabled = true;
    bool isDataReady = false;

    SpEn(false);

    // Setting the PON bit in Enable register 0x80
    PON();

    // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10 to CFG6)
    SmuxConfigRAM();

    // Write new configuration to all the 20 registers
    F1F4_Clear_NIR();

    // Start SMUX command: Enable the SMUXEN bit (bit 4) in register ENABLE
    SMUXEN();

    // Checking on the enabled SMUXEN bit whether back to zero- Poll the SMUXEN bit -> if it is 0
    // SMUX command is started
    while (isEnabled) {
        isEnabled = getSmuxEnabled();
    }

    // Enable SP_EN bit
    SpEn(true);

    // Reading and Polling the the AVALID bit in Status 2 Register 0xA3
    while (!(isDataReady)) {
        isDataReady = getIsDataReady();
    }

    channels->F1 = readTwoRegister1(0x95);
    channels->F2 = readTwoRegister1(0x97);
    channels->F3 = readTwoRegister1(0x99);
    channels->F4 = readTwoRegister1(0x9B);


    isEnabled = true;
    isDataReady = false;

    // Disable SP_EN bit
    SpEn(false);

    // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10 to CFG6)
    SmuxConfigRAM();

    // Write new configuration to all the 20 registers for reading channels from F5-F8, Clear and
    // NIR
    F5F8_Clear_NIR();

    // Start SMUX command: Enable the SMUXEN bit (bit 4) in register ENABLE
    SMUXEN();

    // Checking on the enabled SMUXEN bit whether back to zero- Poll the SMUXEN bit -> if it is 0
    // SMUX command is started
    while (isEnabled) {
        isEnabled = getSmuxEnabled();
    }

    // Enable SP_EN bit
    SpEn(true);

    // Reading and Polling the the AVALID bit in Status 2 Register 0xA3
    while (!(isDataReady)) {
        isDataReady = getIsDataReady();
    }

    channels->F5 = readTwoRegister1(0x95);
    channels->F6 = readTwoRegister1(0x97);
    channels->F7 = readTwoRegister1(0x99);
    channels->F8 = readTwoRegister1(0x9B);
    channels->CLEAR = readTwoRegister1(0x9D);
    channels->NIR = readTwoRegister1(0x9F);
}

void AS7341::FlickerDetection(bool enable) {
    if (enable) {
        bool isEnabled = true;
        bool isFdmeasReady = false;
        writeRegister(uint8_t(0x80), uint8_t(0x00));

        // Setting the PON bit in Enable register 0x80
        PON();


        // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10 to CFG6)
        SmuxConfigRAM();

        // Write new configuration to all the 20 registers for detecting Flicker
        FDConfig();

        // Start SMUX command: Enable the SMUXEN bit (bit 4) in register ENABLE
        SMUXEN();

        // Checking on the enabled SMUXEN bit whether back to zero- Poll the SMUXEN bit -> if it is
        // 0 SMUX command is started
        while (isEnabled) {
            isEnabled = getSmuxEnabled();
        }

        // Enable SP_EN bit
        SpEn(true);

        /*----- Functions for setting Flicker Sample, Flicker time, Flicker Gain (not implemented
         * for default flicker detection)------*/
        //            writeRegister(uint8_t(0xD7), uint8_t(0x21)); //33 default value, function for
        //            setting for Fd_sample and Fd_compare_value
        //
        //            writeRegister(uint8_t(0xD8), uint8_t(0x68)); //104 default value, function for
        //            setting for Fd_time lower bit(7:0)
        //
        //            writeRegister(uint8_t(0xDA), uint8_t(0x49)); //73 default value, function for
        //            setting for fd_gain and fd_time higher bit(10:8)


        // Function to set the Flicker detection via enabling the fden bit in 0x80 register
        writeRegister(uint8_t(0x80), uint8_t(0x41));

    } else {
        // Setting back the PON bit in the ENABLE Register
        writeRegister(uint8_t(0x80), uint8_t(0x01));
    }
    sleep_ms(500);
}

bool AS7341::detect_overflow(Channels *channels) {
    bool flag = false;
    if (channels->F1 == 65535)
        flag = true;
    else if (channels->F2 == 65535)
        flag = true;
    else if (channels->F3 == 65535)
        flag = true;
    else if (channels->F4 == 65535)
        flag = true;
    else if (channels->F5 == 65535)
        flag = true;
    else if (channels->F6 == 65535)
        flag = true;
    else if (channels->F7 == 65535)
        flag = true;
    else if (channels->F8 == 65535)
        flag = true;
    else if (channels->NIR == 65535)
        flag = true;

    // if(flag == true) printf("overflow\n");
    return flag;
}


uint16_t AS7341::determine_max_value(Channels *channels) {
    uint32_t max;
    max = channels->F1;
    if (channels->F2 > max)
        max = channels->F2;
    if (channels->F3 > max)
        max = channels->F3;
    if (channels->F4 > max)
        max = channels->F4;
    if (channels->F5 > max)
        max = channels->F5;
    if (channels->F6 > max)
        max = channels->F6;
    if (channels->F7 > max)
        max = channels->F7;
    if (channels->F8 > max)
        max = channels->F8;
    if (channels->NIR > max)
        max = channels->NIR;
    // printf("max: %d", max);

    return max;
}

void AS7341::autoTimeReadLight(
        Channels *channels) {  // varies between 0.18s (where the ADC reaches full scale) and 1s

    int min = 187084;
    int max = 1000000;
    int med = (min + max) / 2;
    uint16_t max_value = 0;
    setGAIN(GAIN_1);
    for (int i = 0; i < 5; i++) {
        med = (min + max) / 2;
        SetIntTime(med);
        ReadLight(channels);
        max_value = determine_max_value(channels);
        if (detect_overflow(channels)) {
            max = med;
        } else if (max_value > 0.8 * 65535) {
            break;
        } else
            min = med;
    }
}

bool AS7341::SatStatus() {
    uint8_t status = readRegister(uint8_t(0x94));
    return (bool) ((status >> 7) & 1);  // Spectral and Flicker Detect saturation. If ASIEN is set,
                                        // indicates Spectral saturation. Check STATUS2 register to
                                        // distinguish between analog or digital saturation.
}

uint8_t AS7341::GainStatus() {
    uint8_t status = readRegister(uint8_t(0x94));
    return (status & 15);  // Gain Status. Indicates the gain applied for the spectral data latched
                           // to this ASTATUS read. The gain from this status read is required to
                           // calculate spectral results if AGC is enabled.
}

// NOT WORKING
void AS7341::AutoGain() {
    // enables the AGC AutoGainControl feature
    // 0xB1:2
    uint8_t regVal = readRegister(uint8_t(0xB1));
    regVal = regVal & 0xFB;
    regVal = regVal | 0x04;
    writeRegister(uint8_t(0xB1), uint8_t(regVal));
}

void AS7341::FlickerRead(FD_STATUS *fd_status) {
    // reading the flicker status in FD_STATUS register 0xDB
    uint8_t flicker_value = readRegister(uint8_t(0xDB));

    fd_status->FD_100HZ = ((flicker_value >> 0) & 1) ==
                          1;  // Indicates if an ambient light source is flickering at 100Hz.
    fd_status->FD_120HZ = ((flicker_value >> 1) & 1) ==
                          1;  // Indicates if an ambient light source is flickering at 120Hz.
    fd_status->FD_100HZ_VALID = ((flicker_value >> 2) & 1) ==
                                1;  // Indicates that the 100Hz flicker detection calculation is
                                    // valid. Write 1 to this bit to clear this field.
    fd_status->FD_120HZ_VALID = ((flicker_value >> 3) & 1) ==
                                1;  // Indicates that the 120Hz flicker detection calculation is
                                    // valid. Write 1 to this bit to clear this field.
    fd_status->FD_SAT =
            ((flicker_value >> 4) & 1) ==
            1;  // Indicates that saturation occurred during the last flicker detection measurement,
                // and the result may not be valid. Write 1 to this bit to clear this field.
    fd_status->FD_VALID =
            ((flicker_value >> 5) & 1) == 1;  // Indicates that flicker detection measurement is
                                              // complete. Write 1 to this bit to clear this field.
}

/*
void AS7341::ErrorStatus(Error * err) {
  uint8_t status = readRegister(uint8_t(0xA7));
  err->INT_BUSY = ((status >> 0) & 1) == 1; //Indicates that the device is initializing. This bit
will remain 1 for about 300μs after power on. Do not interact with the device until initialization
is complete. err->SAI_ACTIVE = ((status >> 1) & 1) == 1; //Sleep after Interrupt Active. Indicates
that the device is in SLEEP due to an interrupt. To exit SLEEP mode, clear this bit. err->SP_TRIG =
((status >> 2) & 1) == 1; //Spectral Trigger Error. Indicates that there is a timing error. The
WTIME is too short for the selected ATIME. err->FD_TRIG = ((status >> 4) & 1) == 1; //Flicker Detect
Trigger Error. Indicates that there is a timing error that prevents flicker detect from working
correctly. err->OVTEMP = ((status >> 5) & 1) == 1; //Over Temperature Detected. Indicates the device
temperature is too high. Write 1 to clear this bit err->FIFO_OV = ((status >> 7) & 1) == 1;
//Indicates that the FIFO buffer overflowed and information has been lost. Bit is automatically
cleared when the FIFO buffer is read
}
*/

/* ----- Read/Write to i2c register ----- */

// <summary>
// Read a single i2c register
// <summary>
// param name = "addr">Register address of the the register to be read
// param name = "_i2cAddr">Device address 0x39

uint8_t AS7341::readRegister(uint8_t addr) {
    uint8_t data[2];
    int n_bytes_read = reg_read_timeout(i2c, _i2cAddr, addr, data, 1, timeout_us);
    ;
    if (n_bytes_read == PICO_ERROR_GENERIC) {
        return (0xFF);
    } else
        return (data[0]);
}

void AS7341::readID() {
    uint8_t regVal = readRegister(uint8_t(0x92));
    printf("%x\n", regVal >> 2);
}

// <summary>
// Read two consecutive i2c registers
// <summary>
// param name = "addr">First register address of two consecutive registers to be read
// param name = "_i2cAddr">Device address 0x39

uint16_t AS7341::readTwoRegister1(uint8_t addr) {
    uint8_t readingL;
    uint16_t reading, readingH;
    uint8_t data[2];

    int n_bytes_read = reg_read_timeout(i2c, _i2cAddr, addr, data, 2, timeout_us);
    if (n_bytes_read == PICO_ERROR_GENERIC) {
        return (0xFFFF);
    } else {
        readingL = data[0];
        readingH = data[1];
        readingH = readingH << 8;
        reading = (readingH | readingL);
        return (reading);
    }
}


// <summary>
// Write a value to a single i2c register
// <summary>
// param name = "addr">Register address of the the register to the value to be written
// param name = "val">The value written to the Register
// param name = "_i2cAddr">Device address 0x39

void AS7341::writeRegister(uint8_t addr, uint8_t val) {
    int n_bytes_written = reg_write_timeout(i2c, _i2cAddr, addr, &val, 1, timeout_us);
    /*if(n_bytes_written == PICO_ERROR_GENERIC){
          printf("Error writing to I2C addr %x in register %x\n", _i2cAddr, addr);
    }*/
    // printf("Written: %d\n",n_bytes_written);
}

/*----- Register configuration  -----*/

// <summary>
// Setting the PON (Power on) bit on the chip (bit0 at register ENABLE 0x80)
// Attention: This function clears only the PON bit in ENABLE register and keeps the other bits
// <summary>

void AS7341::PON() {

    uint8_t regVal = readRegister(uint8_t(0x80));
    uint8_t temp = regVal;
    regVal = regVal & 0xFE;
    regVal = regVal | 0x01;
    writeRegister(uint8_t(0x80), uint8_t(regVal));
}

// <summary>
// Write SMUX configration from RAM to set SMUX chain in CFG6 register 0xAF
// <summary>

void AS7341::SmuxConfigRAM() {

    writeRegister(uint8_t(0xAF), uint8_t(0x10));
}


// <summary>
// Setting the SP_EN (spectral measurement enabled) bit on the chip (bit 1 in register ENABLE)
// <summary>
// <param name="isEnable">Enabling (true) or disabling (false) the SP_EN bit</param>

void AS7341::SpEn(bool isEnable) {

    uint8_t regVal = readRegister(uint8_t(0x80));
    uint8_t temp = regVal;
    regVal = regVal & 0xFD;
    if (isEnable == true) {
        regVal = regVal | 0x02;
    } else {
        regVal = temp & 0xFD;
    }
    writeRegister(uint8_t(0x80), uint8_t(regVal));
}


// <summary>
// Starting the SMUX command via enabling the SMUXEN bit (bit 4) in register ENABLE 0x80
// The SMUXEN bit gets cleared automatically as soon as SMUX operation is finished
// <summary>

void AS7341::SMUXEN() {

    uint8_t regVal = readRegister(uint8_t(0x80));
    uint8_t temp = regVal;
    regVal = regVal & 0xEF;
    regVal = regVal | 0x10;
    writeRegister(uint8_t(0x80), uint8_t(regVal));
}


// <summary>
// Reading and Polling the the SMUX Enable bit in Enable Register 0x80
// The SMUXEN bit gets cleared automatically as soon as SMUX operation is finished
// <summary>

bool AS7341::getSmuxEnabled() {

    bool isEnabled = false;
    uint8_t regVal = readRegister(uint8_t(0x80));

    if ((regVal & 0x10) == 0x10) {
        return isEnabled = true;
    }

    else {
        return isEnabled = false;
    }
}


// <summary>
// Reading and Polling the the AVALID bit in Status 2 Register 0xA3,if the spectral measurement is
// ready or busy. True indicates that a cycle is completed since the last readout of the Raw Data
// register <summary>

bool AS7341::getIsDataReady() {
    bool isDataReady = false;
    uint8_t regVal = readRegister(uint8_t(0xA3));

    if ((regVal & 0x40) == 0x40) {

        return isDataReady = true;
    }

    else {
        return isDataReady = false;
    }
}


//<summary>
// Reading and polling of Flicker measurement ready bit (bit [5] on FD_Status register
// True indicates that the Flicker Detection measurement was finished
//<summary>

bool AS7341::getFdMeasReady() {
    bool isFdmeasReady = false;
    uint8_t regVal = readRegister(uint8_t(0xDB));

    if ((regVal & 0x20) == 0x20) {

        return isFdmeasReady = true;
    }

    else {
        return isFdmeasReady = false;
    }
}

/*----- SMUX Configuration for F1,F2,F3,F4,CLEAR,NIR -----*/

//<summary>
// Mapping the individual Photo diodes to dedicated ADCs using SMUX Configuration for
// F1-F4,Clear,NIR
//<summary>

void AS7341::F1F4_Clear_NIR() {
    // SMUX Config for F1,F2,F3,F4,NIR,Clear
    writeRegister(uint8_t(0x00), uint8_t(0x30));  // F3 left set to ADC2
    writeRegister(uint8_t(0x01), uint8_t(0x01));  // F1 left set to ADC0
    writeRegister(uint8_t(0x02), uint8_t(0x00));  // Reserved or disabled
    writeRegister(uint8_t(0x03), uint8_t(0x00));  // F8 left disabled
    writeRegister(uint8_t(0x04), uint8_t(0x00));  // F6 left disabled
    writeRegister(uint8_t(0x05),
                  uint8_t(0x42));  // F4 left connected to ADC3/f2 left connected to ADC1
    writeRegister(uint8_t(0x06), uint8_t(0x00));  // F5 left disbled
    writeRegister(uint8_t(0x07), uint8_t(0x00));  // F7 left disbled
    writeRegister(uint8_t(0x08), uint8_t(0x50));  // CLEAR connected to ADC4
    writeRegister(uint8_t(0x09), uint8_t(0x00));  // F5 right disabled
    writeRegister(uint8_t(0x0A), uint8_t(0x00));  // F7 right disabled
    writeRegister(uint8_t(0x0B), uint8_t(0x00));  // Reserved or disabled
    writeRegister(uint8_t(0x0C), uint8_t(0x20));  // F2 right connected to ADC1
    writeRegister(uint8_t(0x0D), uint8_t(0x04));  // F4 right connected to ADC3
    writeRegister(uint8_t(0x0E), uint8_t(0x00));  // F6/F7 right disabled
    writeRegister(uint8_t(0x0F), uint8_t(0x30));  // F3 right connected to AD2
    writeRegister(uint8_t(0x10), uint8_t(0x01));  // F1 right connected to AD0
    writeRegister(uint8_t(0x11), uint8_t(0x50));  // CLEAR right connected to AD4
    writeRegister(uint8_t(0x12), uint8_t(0x00));  // Reserved or disabled
    writeRegister(uint8_t(0x13), uint8_t(0x06));  // NIR connected to ADC5
}

/*----- SMUX Configuration for F5,F6,F7,F8,CLEAR,NIR -----*/

//<summary>
// Mapping the individual Photo diodes to dedicated ADCs using SMUX Configuration for
// F5-F8,Clear,NIR
//<summary>

void AS7341::F5F8_Clear_NIR() {
    // SMUX Config for F5,F6,F7,F8,NIR,Clear
    writeRegister(uint8_t(0x00), uint8_t(0x00));  // F3 left disable
    writeRegister(uint8_t(0x01), uint8_t(0x00));  // F1 left disable
    writeRegister(uint8_t(0x02), uint8_t(0x00));  // reserved/disable
    writeRegister(uint8_t(0x03), uint8_t(0x40));  // F8 left connected to ADC3
    writeRegister(uint8_t(0x04), uint8_t(0x02));  // F6 left connected to ADC1
    writeRegister(uint8_t(0x05), uint8_t(0x00));  // F4/ F2 disabled
    writeRegister(uint8_t(0x06), uint8_t(0x10));  // F5 left connected to ADC0
    writeRegister(uint8_t(0x07), uint8_t(0x03));  // F7 left connected to ADC2
    writeRegister(uint8_t(0x08), uint8_t(0x50));  // CLEAR Connected to ADC4
    writeRegister(uint8_t(0x09), uint8_t(0x10));  // F5 right connected to ADC0
    writeRegister(uint8_t(0x0A), uint8_t(0x03));  // F7 right connected to ADC2
    writeRegister(uint8_t(0x0B), uint8_t(0x00));  // Reserved or disabled
    writeRegister(uint8_t(0x0C), uint8_t(0x00));  // F2 right disabled
    writeRegister(uint8_t(0x0D), uint8_t(0x00));  // F4 right disabled
    writeRegister(uint8_t(0x0E), uint8_t(0x24));  // F7 connected to ADC2/ F6 connected to ADC1
    writeRegister(uint8_t(0x0F), uint8_t(0x00));  // F3 right disabled
    writeRegister(uint8_t(0x10), uint8_t(0x00));  // F1 right disabled
    writeRegister(uint8_t(0x11), uint8_t(0x50));  // CLEAR right connected to AD4
    writeRegister(uint8_t(0x12), uint8_t(0x00));  // Reserved or disabled
    writeRegister(uint8_t(0x13), uint8_t(0x06));  // NIR connected to ADC5
}


/*----- //SMUX Configuration for Flicker detection - register (0x13)left set to ADC6 for flicker
 * detection-----*/

//<summary>
// Mapping the individual Photo diodes to dedicated ADCs using SMUX Configuration for Flicker
// detection
//<summary>

void AS7341::FDConfig() {
    // SMUX Config for Flicker- register (0x13)left set to ADC6 for flicker detection
    writeRegister(uint8_t(0x00), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x01), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x02), uint8_t(0x00));  // reserved/disabled
    writeRegister(uint8_t(0x03), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x04), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x05), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x06), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x07), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x08), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x09), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x0A), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x0B), uint8_t(0x00));  // Reserved or disabled
    writeRegister(uint8_t(0x0C), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x0D), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x0E), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x0F), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x10), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x11), uint8_t(0x00));  // disabled
    writeRegister(uint8_t(0x12), uint8_t(0x00));  // Reserved or disabled
    writeRegister(uint8_t(0x13), uint8_t(0x60));  // Flicker connected to ADC5 to left of 0x13
}

/*----- Set integration time = (ATIME + 1) * (ASTEP + 1) * 2.78µS -----*/


//<summary>
// Sets the ATIME for integration time from 0 to 255, integration time = (ATIME + 1) * (ASTEP + 1)
// * 2.78µS
//<summary>
// param name = "value"> integer value from 0 to 255 written to ATIME register 0x81

void AS7341::setATIME(uint8_t value) {

    writeRegister(uint8_t(0x81), value);
}

//<summary>
// Sets the ASTEP for integration time from 0 to 65535, integration time = (ATIME + 1) * (ASTEP + 1)
// * 2.78µS
//<summary>
// param name = "value1,"> Defines the lower byte[7:0] of the base step time written to ASTEP
// register 0xCA param name = "value2,"> Defines the higher byte[15:8] of the base step time written
// to ASTEP register 0xCB

void AS7341::setASTEP(uint8_t value1, uint8_t value2) {

    // astep[7:0]
    writeRegister(uint8_t(0xCA), value1);

    // astep[15:8]
    writeRegister(uint8_t(0xCB), value2);
}


//<summary>
// Sets the Spectral Gain in CFG1 Register (0xAA) in [4:0] bit
//<summary>
// param name = "value"> integer value from 0 to 10 written to AGAIN register 0xAA
/*
VALUE GAIN
0 0.5x
1 1x
2 2x
3 4x
4 8x
5 16x
6 32x
7 64x
8 128x
9 256x  --> default
10 512x
*/
void AS7341::setGAIN(uint8_t value) {
    writeRegister(uint8_t(0xAA), value);
}

/*
void AS7341::ReadStatus(STATUS * sts){
  uint8_t status = readRegister(uint8_t(0x93));
  sts->ASAT = ((status >> 7) & 1) == 1; //Spectral and  Flicker Detect saturation. If ASIEN is set,
indicates Spectral saturation. Check STATUS2 register to distinguish between analog or digital
saturation. sts->AINT = ((status >> 3) & 1) == 1;  //Spectral Channel Interrupt. If SP_IEN is set,
indicates that a spectral event that met the programmed thresholds and persistence (APERS) occurred.
  sts->FINT = ((status >> 2) & 1) == 1; //FIFO Buffer Interrupt. If FIEN is set, indicates that the
FIFO_LVL fulfills the threshold condition. If cleared by writing 1, the interrupt will be asserted
again as more data is collected. To fully clear this interrupt, all data must be read from the FIFO
buffer. sts->C_INT = ((status >> 1) & 1) == 1; //Calibration Interrupt. uint8_t status2 =
readRegister(uint8_t(0xA3)); sts->AVALID = ((status2 >> 6) & 1) == 1; //Indicates that the spectral
measurement has been completed sts->ASAT_A = ((status2 >> 3) & 1) == 1; //Analog saturation.
Indicates that the intensity of ambient light has exceeded the maximum integration level for the
spectral analog circuit. sts->ASAT_D = ((status2 >> 4) & 1) == 1; //Digital saturation. Indicates
that the maximum counter value has been reached. Maximum counter value depends on integration time
set in the ATIME register. sts->FDSAT_A = ((status2 >> 1) & 1) == 1; //Flicker detect analog
saturation. Indicates that the intensity of ambient light has exceeded the maximum integration level
for the analog circuit for flicker detection sts->FDSAT_D = ((status2 >> 0) & 1) == 1; //Flicker
detect digital saturation. Indicates that the maximum counter value has been reached during flicker
detection.
}
*/

//<summary>
// Reads Revision identification number in 0x91
//<summary>

void AS7341::readREVID() {
    readRegister(uint8_t(0x91));
}


//<summary>
// Reads Auxiliary identification number in 0x90
//<summary>
void AS7341::readAUXID() {
    readRegister(uint8_t(0x90));
}
