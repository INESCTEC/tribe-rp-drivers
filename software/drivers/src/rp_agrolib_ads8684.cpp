/**
 * @file rp_agrolib_ads8684.cpp
 * @brief Implementation of ADS8684 driver
 *
 * Based on TI ADS868x datasheet SBAS582C (July 2014, Revised April 2015)
 */

#include "rp_agrolib_ads8684.h"

#include "hardware/gpio.h"

#include <cstring>

ADS8684::ADS8684(spi_inst_t *spi_inst, uint cs_pin, uint sclk_pin, uint sdi_pin, uint sdo_pin,
                 uint32_t spi_baudrate)
    : _spi(spi_inst), _cs_pin(cs_pin), _sclk_pin(sclk_pin), _sdi_pin(sdi_pin), _sdo_pin(sdo_pin),
      _baudrate(spi_baudrate) {

    // Initialize all channels to ±10.24V range by default
    for (int i = 0; i < 4; i++) {
        _channel_ranges[i] = RANGE_PM_10P24V;
    }
}

bool ADS8684::begin() {
    // Initialize CS pin
    gpio_init(_cs_pin);
    gpio_set_dir(_cs_pin, GPIO_OUT);
    gpio_put(_cs_pin, 1);  // CS idle high

    // Initialize SPI
    spi_init(_spi, _baudrate);

    // Configure SPI pins
    gpio_set_function(_sclk_pin, GPIO_FUNC_SPI);
    gpio_set_function(_sdi_pin, GPIO_FUNC_SPI);
    gpio_set_function(_sdo_pin, GPIO_FUNC_SPI);

    // SPI Mode 1: CPOL=0, CPHA=1 (per SBAS582C page 8)
    spi_set_format(_spi, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    // Small delay for device power-up
    sleep_ms(10);

    // Reset the device
    reset();
    sleep_ms(5);

    // Configure all channels to default range
    for (int i = 0; i < 4; i++) {
        setInputRange(static_cast<Channel>(i), RANGE_PM_10P24V);
    }

    return true;
}

void ADS8684::reset() {
    sendCommand(CMD_RST);
    sleep_ms(5);  // Wait for reset to complete

    // Re-initialize channel ranges to default
    for (int i = 0; i < 4; i++) {
        _channel_ranges[i] = RANGE_PM_10P24V;
    }
}

uint16_t ADS8684::readChannel(Channel channel) {
    // Select the channel for next conversion
    uint32_t cmd;
    switch (channel) {
        case CH0:
            cmd = CMD_MAN_CH_0;
            break;
        case CH1:
            cmd = CMD_MAN_CH_1;
            break;
        case CH2:
            cmd = CMD_MAN_CH_2;
            break;
        case CH3:
            cmd = CMD_MAN_CH_3;
            break;
        default:
            return 0;
    }

    // First command: select channel (returns previous conversion)
    uint16_t response = sendCommand(cmd);

    // Second command: NO-OP to get the result of selected channel
    response = sendCommand(CMD_NO_OP);

    // Extract 16-bit result from response (bits 15-0)
    return response;
}

void ADS8684::readAllChannels(uint16_t data[4]) {
    for (int i = 0; i < 4; i++) {
        data[i] = readChannel(static_cast<Channel>(i));
    }
}

bool ADS8684::setInputRange(Channel channel, InputRange range) {
    if (channel > CH3)
        return false;

    // Determine correct register address (0x05-0x08)
    Register reg = static_cast<Register>(REG_RANGE_CH0 + channel);

    // Write range value to register
    if (!writeRegister(reg, static_cast<uint8_t>(range))) {
        return false;
    }

    // Update cached range
    _channel_ranges[channel] = range;

    sleep_us(10);  // Small delay for setting to take effect
    return true;
}

ADS8684::InputRange ADS8684::getInputRange(Channel channel) {
    if (channel > CH3)
        return RANGE_PM_10P24V;
    return _channel_ranges[channel];
}

float ADS8684::rawToVoltage(uint16_t raw_value, InputRange range) {
    // ADS8684 is 16-bit, two's complement for bipolar ranges
    // With internal 4.096V reference (SBAS582C Table 8, Page 38)

    int16_t signed_value = static_cast<int16_t>(raw_value);
    float voltage = 0.0f;

    switch (range) {
        case RANGE_PM_10P24V:
            // ±10.24V range (±2.5 × VREF): LSB = 20.48V / 65536 = 312.5 µV
            voltage = (signed_value * 20.48f) / 65536.0f;
            break;

        case RANGE_PM_5P12V:
            // ±5.12V range (±1.25 × VREF): LSB = 10.24V / 65536 = 156.25 µV
            voltage = (signed_value * 10.24f) / 65536.0f;
            break;

        case RANGE_PM_2P56V:
            // ±2.56V range (±0.625 × VREF): LSB = 5.12V / 65536 = 78.125 µV
            voltage = (signed_value * 5.12f) / 65536.0f;
            break;

        case RANGE_0_TO_5P12V:
            // 0 to 5.12V range (1.25 × VREF, unipolar)
            voltage = (raw_value * 5.12f) / 65536.0f;
            break;

        case RANGE_0_TO_10P24V:
            // 0 to 10.24V range (2.5 × VREF, unipolar)
            voltage = (raw_value * 10.24f) / 65536.0f;
            break;

        default:
            voltage = 0.0f;
    }

    return voltage;
}

bool ADS8684::writeRegister(Register reg, uint8_t value) {
    // Register write format:
    // Bits 15-9: Register address (7 bits)
    // Bit 8: Write flag (1 for write)
    // Bits 7-0: Data value (8 bits)

    uint16_t cmd_reg = reg | 0x01;  // Set write flag

    uint16_t cmd = 0x0000;
    cmd |= (static_cast<uint32_t>(cmd_reg) << 8);  // Address at bits 15-9
    cmd |= (static_cast<uint32_t>(value) << 0);    // Data at bits 7-0

    sendCommand(cmd);
    return true;
}

uint8_t ADS8684::readRegister(Register reg) {
    // Register read format (SBAS582C Section 10.5.2.4, Page 41):
    // Bits 31-25: 0100100 (0x48)
    // Bits 24-17: Register address (8 bits)
    // Bits 16-0:  Don't care

    uint16_t cmd_reg = reg && 0xFE;  // Clear write flag for read

    uint16_t cmd = 0x0000;
    cmd |= (static_cast<uint16_t>(cmd_reg) << 8);  // Address at bits 15-9

    // First command: request register read
    uint16_t response = sendCommand(cmd);

    // Extract 8-bit register value from bits 15-8
    return static_cast<uint8_t>((response >> 8) & 0xFF);
}

void ADS8684::powerDown() {
    sendCommand(CMD_PWR_DN);
}

void ADS8684::wakeUp() {
    sendCommand(CMD_STDBY);
    sleep_us(100);  // Wake-up time
}

bool ADS8684::setChannelEnabled(Channel channel, bool enable) {
    if (channel > CH3)
        return false;

    // Read current channel power down register (0x02)
    uint8_t channel_pd = readRegister(REG_CHANNEL_PD);

    // Upper nibble controls channel enable (1=enabled, 0=powered down)
    // Bits 7-4: CH3, CH2, CH1, CH0
    if (enable) {
        channel_pd |= (1 << (4 + channel));  // Set bit to enable
    } else {
        channel_pd &= ~(1 << (4 + channel));  // Clear bit to power down
    }

    // Write back
    return writeRegister(REG_CHANNEL_PD, channel_pd);
}

uint16_t ADS8684::sendCommand(uint16_t command) {
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[2];

    // Pack 32-bit command into bytes (MSB first)
    tx_buffer[0] = (command >> 8) & 0xFF;
    tx_buffer[1] = command & 0xFF;

    // Perform SPI transfer
    spiTransfer(tx_buffer, rx_buffer);

    // Unpack 16-bit response (MSB first)
    uint16_t response = 0;
    response |= (static_cast<uint16_t>(rx_buffer[0]) << 8);
    response |= static_cast<uint16_t>(rx_buffer[1]);

    return response;
}

void ADS8684::spiTransfer(const uint8_t *tx_data, uint8_t *rx_data) {
    csLow();
    sleep_us(1);  // CS setup time (min 10 ns per SBAS582C)

    // Transfer 2 bytes
    spi_write_read_blocking(_spi, tx_data, rx_data, 2);

    sleep_us(1);  // CS hold time (min 10 ns per SBAS582C)
    csHigh();

    sleep_us(1);  // Inter-command delay
}