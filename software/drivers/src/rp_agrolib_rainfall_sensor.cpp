#include "../include/rp_agrolib_rainfall_sensor.h"

#include <stdio.h>
#include <string.h>

DFRobot_RainfallSensor::DFRobot_RainfallSensor(i2c_inst_t *i2c, uint32_t timeout_us, uint8_t addr)
    : _i2c(i2c), _addr(addr), _timeout_us(timeout_us), _pid(0), _vid(0) {
    memset(_firmware, 0, sizeof(_firmware));
}

bool DFRobot_RainfallSensor::begin() {
    // Read PID and VID to check if sensor is connected and valid
    uint8_t buffer[4];

    // Start with register address
    uint8_t reg = REG_PID;
    i2c_write_timeout_us(_i2c, _addr, &reg, 1, true, _timeout_us);

    // Read 4 bytes from that register
    if (i2c_read_timeout_us(_i2c, _addr, buffer, 4, false, _timeout_us) != 4) {
        return false;
    }

    // Calculate PID and VID according to original library
    _pid = buffer[0] | ((uint16_t) buffer[1] << 8) | (((uint32_t) (buffer[3] & 0xC0)) << 10);
    _vid = buffer[2] | (uint16_t) ((buffer[3] & 0x3F) << 8);

    // Get firmware version if device is valid
    if ((_vid == EXPECTED_VID) && (_pid == EXPECTED_PID)) {
        uint16_t version = readRegister16(REG_VERSION);
        snprintf(_firmware, sizeof(_firmware), "%d.%d.%d.%d", (version >> 12),
                 ((version >> 8) & 0x0F), ((version >> 4) & 0x0F), (version & 0x0F));
        return true;
    }

    return false;
}

const char *DFRobot_RainfallSensor::getFirmwareVersion() {
    return _firmware;
}

float DFRobot_RainfallSensor::getRainfall() {
    uint32_t rainfall = readRegister32(REG_CUMULATIVE_RAINFALL);
    return rainfall / 10000.0f;
}

float DFRobot_RainfallSensor::getRainfall(uint8_t hours) {
    // Validate hours (1-24)
    if (hours < 1 || hours > 24) {
        hours = 1;  // Default to 1 hour if out of range
    }

    // Set the time to calculate cumulative rainfall
    writeRegister8(REG_RAIN_HOUR, hours);

    // Read the rainfall for the specified time
    uint32_t rainfall = readRegister32(REG_TIME_RAINFALL);
    return rainfall / 10000.0f;
}

uint32_t DFRobot_RainfallSensor::getRawData() {
    return readRegister32(REG_RAW_DATA);
}

float DFRobot_RainfallSensor::getSensorWorkingTime() {
    uint16_t workingTime = readRegister16(REG_SYS_TIME);
    return workingTime / 60.0f;
}

bool DFRobot_RainfallSensor::setRainAccumulatedValue(float value) {
    uint16_t data = (uint16_t) (value * 10000);
    return writeRegister16(REG_BASE_RAINFALL, data);
}

uint32_t DFRobot_RainfallSensor::readRegister32(uint8_t reg) {
    uint8_t buffer[4];

    i2c_write_timeout_us(_i2c, _addr, &reg, 1, true, _timeout_us);

    if (i2c_read_timeout_us(_i2c, _addr, buffer, 4, false, _timeout_us) != 4) {
        return 0;  // Return 0 on error
    }

    return buffer[0] | ((uint32_t) buffer[1] << 8) | ((uint32_t) buffer[2] << 16) |
           ((uint32_t) buffer[3] << 24);
}

uint16_t DFRobot_RainfallSensor::readRegister16(uint8_t reg) {
    uint8_t buffer[2];

    i2c_write_timeout_us(_i2c, _addr, &reg, 1, true, _timeout_us);

    if (i2c_read_timeout_us(_i2c, _addr, buffer, 2, false, _timeout_us) != 2) {
        return 0;  // Return 0 on error
    }

    return buffer[0] | ((uint16_t) buffer[1] << 8);
}

bool DFRobot_RainfallSensor::writeRegister8(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};

    int result = i2c_write_timeout_us(_i2c, _addr, buffer, 2, false, _timeout_us);
    sleep_ms(100);  // Delay for sensor to process the command

    return result == 2;
}

bool DFRobot_RainfallSensor::writeRegister16(uint8_t reg, uint16_t value) {
    uint8_t buffer[3] = {
            reg,
            (uint8_t) (value & 0xFF),        // Low byte
            (uint8_t) ((value >> 8) & 0xFF)  // High byte
    };

    int result = i2c_write_timeout_us(_i2c, _addr, buffer, 3, false, _timeout_us);
    sleep_ms(100);  // Delay for sensor to process the command

    return result == 3;
}
