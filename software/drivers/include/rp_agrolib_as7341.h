/*
    Author: DarkeTechCorp
    Modified by Francisco Terra and Pedro Moura
    25/07/2022
*/

// #pragma once
// #ifndef AS7341_H
// #define AS7341_H

#include <inttypes.h>

#include "rp_agrolib_i2c.h"

#define _i2cAddr (0x39)  // I2C device address - 0x39

#define GAIN_05 0x00
#define GAIN_1 0x01
#define GAIN_2 0x02
#define GAIN_4 0x03
#define GAIN_8 0x04
#define GAIN_16 0x05
#define GAIN_32 0x06
#define GAIN_64 0x07
#define GAIN_128 0x08
#define GAIN_256 0x09
#define GAIN_512 0x0A

#define CHANNEL1_FACTOR 0.00215  // Irradiance Flux density μW/cm2
#define CHANNEL2_FACTOR 0.0193   //
#define CHANNEL3_FACTOR 0.02536  //
#define CHANNEL4_FACTOR 0.2193   //
#define CHANNEL5_FACTOR 0.2193   //
#define CHANNEL6_FACTOR 0.01898  //
#define CHANNEL7_FACTOR 0.02215  //
#define CHANNEL8_FACTOR 0.02164  //


struct Channels {
    int F1 = 0;
    int F2 = 0;
    int F3 = 0;
    int F4 = 0;
    int F5 = 0;
    int F6 = 0;
    int F7 = 0;
    int F8 = 0;
    int CLEAR = 0;
    int NIR = 0;
};

struct FD_STATUS {
    bool FD_VALID = false;
    bool FD_SAT = false;
    bool FD_120HZ_VALID = false;
    bool FD_100HZ_VALID = false;
    bool FD_120HZ = false;
    bool FD_100HZ = false;
};


class AS7341 {
public:
    void PowerOn();
    void ReadLight(Channels *channels);
    void FlickerDetection(bool enable);
    bool SatStatus();
    uint8_t GainStatus();
    void FlickerRead(FD_STATUS *fd_status);
    void setGAIN(uint8_t value);
    void ReadStatus();
    void AutoGain();
    void SpEn(bool isEnable);
    void readID();
    uint32_t GetIntTime();
    bool SetIntTime(long int int_time);
    void autoTimeReadLight(Channels *channels);

    AS7341(i2c_inst_t *_i2c, uint32_t _timeout_us) : i2c(_i2c), timeout_us(_timeout_us) {}


private:
    uint8_t readRegister(uint8_t addr);
    uint16_t readTwoRegister1(uint8_t addr);
    void writeRegister(uint8_t addr, uint8_t val);
    void PON();
    void SmuxConfigRAM();
    // void SpEn(bool isEnable);
    void SMUXEN();
    bool getSmuxEnabled();
    bool getIsDataReady();
    bool getFdMeasReady();
    void F1F4_Clear_NIR();
    void F5F8_Clear_NIR();
    void FDConfig();
    void setATIME(uint8_t value);
    void setASTEP(uint8_t value1, uint8_t value2);
    void readREVID();
    void readAUXID();
    bool detect_overflow(Channels *channels);
    uint16_t determine_max_value(Channels *channels);
    i2c_inst_t *i2c;
    uint32_t timeout_us;
};

// #endif
