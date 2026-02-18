#ifndef DFROBOT_RAINFALL_SENSOR_H
#define DFROBOT_RAINFALL_SENSOR_H

#include "pico/stdlib.h"

#include "hardware/i2c.h"

class DFRobot_RainfallSensor {
public:
    // Register definitions
    static const uint8_t REG_PID = 0x00;
    static const uint8_t REG_VID = 0x02;
    static const uint8_t REG_VERSION = 0x0A;
    static const uint8_t REG_TIME_RAINFALL = 0x0C;
    static const uint8_t REG_CUMULATIVE_RAINFALL = 0x10;
    static const uint8_t REG_RAW_DATA = 0x14;
    static const uint8_t REG_SYS_TIME = 0x18;
    static const uint8_t REG_RAIN_HOUR = 0x26;
    static const uint8_t REG_BASE_RAINFALL = 0x28;

    // Expected PID and VID values
    static const uint32_t EXPECTED_PID = 0x100C0;
    static const uint16_t EXPECTED_VID = 0x3343;

    /**
     * @brief Constructor for the rainfall sensor class
     *
     * @param i2c The I2C instance to use (i2c0 or i2c1)
     * @param addr The I2C address of the sensor (default 0x1D)
     */
    DFRobot_RainfallSensor(i2c_inst_t *i2c = i2c0, uint32_t timeout_us = 500000,
                           uint8_t addr = 0x1D);

    /**
     * @brief Initialize the sensor
     *
     * @return true if sensor is detected with correct PID/VID, false otherwise
     */
    bool begin();

    /**
     * @brief Get the firmware version
     *
     * @return String representation of the firmware version
     */
    const char *getFirmwareVersion();

    /**
     * @brief Get the total cumulative rainfall since sensor started
     *
     * @return Rainfall in millimeters
     */
    float getRainfall();

    /**
     * @brief Get the cumulative rainfall within the specified time
     *
     * @param hours Specified time (valid range is 1-24 hours)
     * @return Rainfall in millimeters
     */
    float getRainfall(uint8_t hours);

    /**
     * @brief Get the raw data (number of tipping bucket counts)
     *
     * @return Number of tipping bucket counts
     */
    uint32_t getRawData();

    /**
     * @brief Get the sensor working time
     *
     * @return Working time in hours
     */
    float getSensorWorkingTime();

    /**
     * @brief Set the rainfall accumulated value (mm per tip)
     *
     * @param value Rainfall per tip in millimeters (default 0.2794)
     * @return true if successful, false otherwise
     */
    bool setRainAccumulatedValue(float value = 0.2794);

    /**
     * @brief Get the PID of the sensor
     */
    uint32_t getPID() { return _pid; }

    /**
     * @brief Get the VID of the sensor
     */
    uint16_t getVID() { return _vid; }

private:
    i2c_inst_t *_i2c;      // I2C instance
    uint8_t _addr;         // I2C address
    uint32_t _timeout_us;  // I2C timeout
    uint32_t _pid;         // Product ID
    uint16_t _vid;         // Vendor ID
    char _firmware[16];    // Firmware version string

    /**
     * @brief Read a 32-bit register
     *
     * @param reg Register address
     * @return 32-bit value
     */
    uint32_t readRegister32(uint8_t reg);

    /**
     * @brief Read a 16-bit register
     *
     * @param reg Register address
     * @return 16-bit value
     */
    uint16_t readRegister16(uint8_t reg);

    /**
     * @brief Write an 8-bit value to a register
     *
     * @param reg Register address
     * @param value Value to write
     * @return true if successful, false otherwise
     */
    bool writeRegister8(uint8_t reg, uint8_t value);

    /**
     * @brief Write a 16-bit value to a register
     *
     * @param reg Register address
     * @param value Value to write
     * @return true if successful, false otherwise
     */
    bool writeRegister16(uint8_t reg, uint16_t value);
};

#endif  // DFROBOT_RAINFALL_SENSOR_H
