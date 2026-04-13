// ad5680.cpp
#include "rp_agrolib_ad5680.h"

#include "hardware/gpio.h"

#include <algorithm>

AD5680::AD5680(spi_inst_t *spi_inst, uint cs_pin, uint sck_pin, uint mosi_pin, uint32_t baudrate,
               ResetMode reset_mode)
    : spi_(spi_inst), cs_pin_(cs_pin), sck_pin_(sck_pin), mosi_pin_(mosi_pin),
      baudrate_(std::min(baudrate, MAX_BAUDRATE)), reset_mode_(reset_mode),
      current_code_(reset_mode == ResetMode::ZERO_SCALE ? 0 : MIDSCALE_CODE), initialized_(false) {}

bool AD5680::init() {
    if (initialized_) {
        return true;
    }

    // Initialize SPI at specified baudrate
    uint actual_baudrate = spi_init(spi_, baudrate_);

    // Set SPI format: 8 data bits, CPOL=0, CPHA=1 (Mode 1)
    // Data valid on falling edge of SCLK
    spi_set_format(spi_, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    // Initialize GPIO pins
    gpio_set_function(sck_pin_, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin_, GPIO_FUNC_SPI);

    // Initialize CS (SYNC) as GPIO output
    gpio_init(cs_pin_);
    gpio_set_dir(cs_pin_, GPIO_OUT);
    gpio_put(cs_pin_, 0);  // Idle low for lower power

    // Wait for power-on reset to complete (>250us per datasheet)
    sleep_us(300);

    initialized_ = true;

    return actual_baudrate > 0;
}

void AD5680::pulseSync() {
    // Pulse SYNC low to high to low
    gpio_put(cs_pin_, 0);
    sleep_us(1);  // t4 = 13ns min
    gpio_put(cs_pin_, 1);
    sleep_us(1);  // t8 = 33ns min
    gpio_put(cs_pin_, 0);
}

uint32_t AD5680::buildDataWord(uint32_t code) {
    // Limit code to 18 bits
    code &= MAX_CODE;

    // Build 24-bit word:
    // Bits 23-22: Don't care
    // Bits 21-20: Reserved (set to 0)
    // Bits 19-2:  18-bit DAC data (D17 to D0)
    // Bits 1-0:   Don't care

    // Shift code left by 2 bits (bits 19-2)
    uint32_t data = (code << 2);

    return data;
}

bool AD5680::writeData(uint32_t data) {
    if (!initialized_) {
        return false;
    }

    // Prepare 3 bytes to send (24 bits)
    uint8_t tx_data[3];
    tx_data[0] = (data >> 16) & 0xFF;  // MSB
    tx_data[1] = (data >> 8) & 0xFF;   // Middle byte
    tx_data[2] = data & 0xFF;          // LSB


    pulseSync();

    // Small delay to meet setup time (t4 = 13ns min)
    sleep_us(1);

    // Write 3 bytes (24 bits)
    spi_write_blocking(spi_, tx_data, 3);

    // Wait for update time (tUPDATE = 250us min per datasheet)
    sleep_us(250);

    return true;
}

bool AD5680::setCode(uint32_t code) {
    if (!initialized_) {
        return false;
    }

    // Limit to 18-bit range
    if (code > MAX_CODE) {
        code = MAX_CODE;
    }

    uint32_t data = buildDataWord(code);

    if (writeData(data)) {
        current_code_ = code;
        return true;
    }

    return false;
}

bool AD5680::setVoltage(float voltage, float vref) {
    if (!initialized_ || vref <= 0.0f) {
        return false;
    }

    // Clamp voltage to valid range
    if (voltage < 0.0f)
        voltage = 0.0f;
    if (voltage > vref)
        voltage = vref;

    // Calculate code: VOUT = VREF * (D / 262144)
    // D = (VOUT / VREF) * 262144
    uint32_t code = static_cast<uint32_t>((voltage / vref) * (MAX_CODE + 1));

    // Ensure we don't exceed max code due to rounding
    if (code > MAX_CODE) {
        code = MAX_CODE;
    }

    return setCode(code);
}

bool AD5680::setPercent(float percent) {
    if (!initialized_) {
        return false;
    }

    // Clamp to valid range
    if (percent < 0.0f)
        percent = 0.0f;
    if (percent > 100.0f)
        percent = 100.0f;

    uint32_t code = static_cast<uint32_t>((percent / 100.0f) * (MAX_CODE + 1));

    if (code > MAX_CODE) {
        code = MAX_CODE;
    }

    return setCode(code);
}

void AD5680::reset() {
    if (reset_mode_ == ResetMode::ZERO_SCALE) {
        setZeroScale();
    } else {
        setMidscale();
    }
}