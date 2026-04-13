// ad5680.h
#ifndef AD5680_H
#define AD5680_H

#include "pico/stdlib.h"

#include "hardware/spi.h"

#include <cstdint>

/**
 * @brief Driver for AD5680 18-bit DAC
 *
 * Features:
 * - 18-bit resolution with 12-bit accuracy
 * - SPI interface up to 30 MHz
 * - Rail-to-rail output (0V to VDD)
 * - Power-on reset to zero or midscale
 */
class AD5680 {
public:
    /**
     * @brief Power-on reset mode
     */
    enum class ResetMode {
        ZERO_SCALE,  // AD5680-1: Powers up to 0V
        MIDSCALE     // AD5680-2: Powers up to midscale
    };

    /**
     * @brief Constructor
     * @param spi_inst SPI instance (spi0 or spi1)
     * @param cs_pin Chip select (SYNC) pin
     * @param sck_pin SPI clock pin
     * @param mosi_pin SPI MOSI (DIN) pin
     * @param baudrate SPI baudrate (max 30 MHz)
     * @param reset_mode Power-on reset configuration
     */
    AD5680(spi_inst_t *spi_inst, uint cs_pin, uint sck_pin, uint mosi_pin,
           uint32_t baudrate = 10000000,  // 10 MHz default
           ResetMode reset_mode = ResetMode::ZERO_SCALE);

    /**
     * @brief Initialize the DAC
     * @return true if successful
     */
    bool init();

    /**
     * @brief Set DAC output using 18-bit code
     * @param code 18-bit DAC code (0 to 262143)
     * @return true if successful
     */
    bool setCode(uint32_t code);

    /**
     * @brief Set DAC output voltage
     * @param voltage Desired output voltage
     * @param vref Reference voltage (VDD)
     * @return true if successful
     */
    bool setVoltage(float voltage, float vref);

    /**
     * @brief Set DAC output as percentage of full scale
     * @param percent Percentage (0.0 to 100.0)
     * @return true if successful
     */
    bool setPercent(float percent);

    /**
     * @brief Get current DAC code
     * @return Current 18-bit code
     */
    uint32_t getCode() const { return current_code_; }

    /**
     * @brief Reset DAC to power-on value
     */
    void reset();

    /**
     * @brief Set to zero scale (0V)
     */
    void setZeroScale() { setCode(0); }

    /**
     * @brief Set to full scale (VDD)
     */
    void setFullScale() { setCode(MAX_CODE); }

    /**
     * @brief Set to midscale (VDD/2)
     */
    void setMidscale() { setCode(MIDSCALE_CODE); }

    // Constants
    static constexpr uint32_t MAX_CODE = 0x3FFFF;       // 262143 (18-bit)
    static constexpr uint32_t MIDSCALE_CODE = 0x20000;  // 131072
    static constexpr uint32_t MAX_BAUDRATE = 30000000;  // 30 MHz

private:
    /**
     * @brief Write 24-bit data to DAC
     * @param data 24-bit word to write
     * @return true if successful
     */
    bool writeData(uint32_t data);

    /**
     * @brief Build 24-bit data word from 18-bit code
     * @param code 18-bit DAC code
     * @return 24-bit formatted word
     */
    uint32_t buildDataWord(uint32_t code);

    void pulseSync();

    spi_inst_t *spi_;
    uint cs_pin_;
    uint sck_pin_;
    uint mosi_pin_;
    uint32_t baudrate_;
    ResetMode reset_mode_;
    uint32_t current_code_;
    bool initialized_;
};

#endif  // AD5680_H