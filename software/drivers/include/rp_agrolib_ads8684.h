/**
 * @file rp_agrolib_ads8684.h
 * @brief Driver for TI ADS8684 16-bit 4-channel SAR ADC
 * @author CRIIS - INESC TEC
 *
 * Based on TI ADS868x datasheet SBAS582C (July 2014, Revised April 2015)
 *
 * Features:
 * - 4 single-ended analog inputs
 * - 16-bit resolution, no missing codes
 * - 500 kSPS maximum throughput rate
 * - Programmable bipolar input ranges: ±10.24V, ±5.12V, ±2.56V
 * - Programmable unipolar input ranges: 0-10.24V, 0-5.12V
 * - Internal 4.096V reference
 * - SPI interface (up to 17 MHz SCLK)
 *
 * TODO: NOT TESTED YET
 */

#ifndef RP_AGROLIB_ADS8684_H
#define RP_AGROLIB_ADS8684_H

#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"

#include <cstdint>

class ADS8684 {
public:
    // Register addresses (SBAS582C Table 10, Page 40)
    enum Register : uint8_t {
        REG_AUTO_SEQ_EN = 0x01,  // Auto sequence enable
        REG_CHANNEL_PD = 0x02,   // Channel power down/up
        REG_FEATURE_SEL = 0x03,  // Feature selection
        // 0x04 is reserved
        REG_RANGE_CH0 = 0x05,  // Channel 0 input range
        REG_RANGE_CH1 = 0x06,  // Channel 1 input range
        REG_RANGE_CH2 = 0x07,  // Channel 2 input range
        REG_RANGE_CH3 = 0x08,  // Channel 3 input range
        // 0x09 is reserved
        // REG_ALARM           = 0x0A,  // Alarm flag readback (READ ONLY)
        // // 0x0B-0x0C reserved
        // REG_ALARM_H0_CH0    = 0x0D,  // CH0 high threshold trip point 0
        // REG_ALARM_H1_CH0    = 0x0E,  // CH0 high threshold trip point 1
        // REG_ALARM_H0_CH1    = 0x0F,  // CH1 high threshold trip point 0
        // REG_ALARM_H1_CH1    = 0x10,  // CH1 high threshold trip point 1
        // REG_ALARM_H0_CH2    = 0x11,  // CH2 high threshold trip point 0
        // REG_ALARM_H1_CH2    = 0x12,  // CH2 high threshold trip point 1
        // REG_ALARM_H0_CH3    = 0x13,  // CH3 high threshold trip point 0
        // REG_ALARM_H1_CH3    = 0x14,  // CH3 high threshold trip point 1
        // REG_ALARM_L0_CH0    = 0x15,  // CH0 low threshold trip point 0
        // REG_ALARM_L1_CH0    = 0x16,  // CH0 low threshold trip point 1
        // REG_ALARM_L0_CH1    = 0x17,  // CH1 low threshold trip point 0
        // REG_ALARM_L1_CH1    = 0x18,  // CH1 low threshold trip point 1
        // REG_ALARM_L0_CH2    = 0x19,  // CH2 low threshold trip point 0
        // REG_ALARM_L1_CH2    = 0x1A,  // CH2 low threshold trip point 1
        // REG_ALARM_L0_CH3    = 0x1B,  // CH3 low threshold trip point 0
        // REG_ALARM_L1_CH3    = 0x1C   // CH3 low threshold trip point 1
        REG_CMD_READ_BACK = 0x3F  // Command readback (READ ONLY)
    };

    // Command definitions (SBAS582C Table 6, Page 45)
    enum Command : uint32_t {
        CMD_NO_OP = 0x0000,     // Continue previous operation
        CMD_STDBY = 0x8200,     // Device standby
        CMD_PWR_DN = 0x8300,    // Device power-down
        CMD_RST = 0x8500,       // Program register reset
        CMD_AUTO_RST = 0xA000,  // Auto mode with reset
        CMD_MAN_CH_0 = 0xC000,  // Manual channel 0 select
        CMD_MAN_CH_1 = 0xC400,  // Manual channel 1 select
        CMD_MAN_CH_2 = 0xC800,  // Manual channel 2 select
        CMD_MAN_CH_3 = 0xCC00,  // Manual channel 3 select
        CMD_MAN_AUX = 0xE000,   // Manual auxiliary channel select
    };

    // Input range configurations (SBAS582C Table 8, Page 38)
    // With internal 4.096V reference
    enum InputRange : uint8_t {
        RANGE_PM_10P24V = 0x00,   // ±10.24V (±2.5 × VREF, bipolar)
        RANGE_PM_5P12V = 0x01,    // ±5.12V (±1.25 × VREF, bipolar)
        RANGE_PM_2P56V = 0x02,    // ±2.56V (±0.625 × VREF, bipolar)
        RANGE_0_TO_5P12V = 0x05,  // 0 to 5.12V (1.25 × VREF, unipolar)
        RANGE_0_TO_10P24V = 0x06  // 0 to 10.24V (2.5 × VREF, unipolar)
    };

    // Channel numbers
    enum Channel : uint8_t { CH0 = 0, CH1 = 1, CH2 = 2, CH3 = 3 };

    /**
     * @brief Constructor
     * @param spi_inst SPI instance (spi0 or spi1)
     * @param cs_pin Chip select GPIO pin (active low)
     * @param sclk_pin SPI clock GPIO pin
     * @param sdi_pin SPI MOSI GPIO pin (connects to ADS8684 SDI)
     * @param sdo_pin SPI MISO GPIO pin (connects to ADS8684 SDO)
     * @param spi_baudrate SPI clock frequency (default 10 MHz, max 17 MHz)
     */
    ADS8684(spi_inst_t *spi_inst, uint cs_pin, uint sclk_pin, uint sdi_pin, uint sdo_pin,
            uint32_t spi_baudrate = 10000000);

    /**
     * @brief Initialize the ADC
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Reset the device to default state
     */
    void reset();

    /**
     * @brief Read a single channel
     * @param channel Channel number (0-3)
     * @return 16-bit ADC value
     */
    uint16_t readChannel(Channel channel);

    /**
     * @brief Read all 4 channels sequentially
     * @param data Array to store 4 channel readings
     */
    void readAllChannels(uint16_t data[4]);

    /**
     * @brief Configure input range for a channel
     * @param channel Channel number (0-3)
     * @param range Input range setting
     * @return true if successful
     */
    bool setInputRange(Channel channel, InputRange range);

    /**
     * @brief Get current input range for a channel
     * @param channel Channel number (0-3)
     * @return Current input range setting
     */
    InputRange getInputRange(Channel channel);

    /**
     * @brief Convert raw ADC value to voltage
     * @param raw_value 16-bit ADC reading
     * @param range Input range used for the measurement
     * @return Voltage in volts
     */
    float rawToVoltage(uint16_t raw_value, InputRange range);

    /**
     * @brief Write to a register
     * @param reg Register address
     * @param value 8-bit value to write
     * @return true if successful
     */
    bool writeRegister(Register reg, uint8_t value);

    /**
     * @brief Read from a register
     * @param reg Register address
     * @return 8-bit register value
     */
    uint8_t readRegister(Register reg);

    /**
     * @brief Power down the device
     */
    void powerDown();

    /**
     * @brief Wake from standby/power-down mode
     */
    void wakeUp();

    /**
     * @brief Enable/disable a channel
     * @param channel Channel to control
     * @param enable true to enable, false to power down
     * @return true if successful
     */
    bool setChannelEnabled(Channel channel, bool enable);

private:
    spi_inst_t *_spi;
    uint _cs_pin;
    uint _sclk_pin;
    uint _sdi_pin;  // RP2040 MOSI -> ADS8684 SDI
    uint _sdo_pin;  // RP2040 MISO -> ADS8684 SDO
    uint32_t _baudrate;

    InputRange _channel_ranges[4];  // Track current range for each channel

    /**
     * @brief Send a command and receive response
     * @param command 16-bit command word
     * @return 16-bit response (contains previous conversion result)
     */
    uint16_t sendCommand(uint16_t command);

    /**
     * @brief SPI transfer helper
     * @param tx_data Transmit buffer (4 bytes)
     * @param rx_data Receive buffer (4 bytes)
     */
    void spiTransfer(const uint8_t *tx_data, uint8_t *rx_data);

    /**
     * @brief Assert chip select (active low)
     */
    inline void csLow() { gpio_put(_cs_pin, 0); }

    /**
     * @brief Deassert chip select
     */
    inline void csHigh() { gpio_put(_cs_pin, 1); }
};

#endif  // RP_AGROLIB_ADS8684_H