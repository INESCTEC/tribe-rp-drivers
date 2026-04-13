/**
 * @file rp_agrolib_ad7980.cpp
 * @brief Implementation of AD7980 driver
 */

#include "rp_agrolib_ad7980.h"

#include "hardware/gpio.h"

#include <cstring>

// Global pointer for interrupt handler
static AD7980 *g_ad7980_instance = nullptr;

// GPIO interrupt callback wrapper
static void gpio_callback_wrapper(uint gpio, uint32_t events) {
    if (g_ad7980_instance && events & GPIO_IRQ_EDGE_FALL) {
        g_ad7980_instance->handleBusyInterrupt();
    }
}

AD7980::AD7980(spi_inst_t *spi_inst, uint cnv_pin, uint sclk_pin, uint sdo_pin, int sdi_pin,
               int busy_pin, uint32_t spi_baudrate)
    : _spi(spi_inst), _cnv_pin(cnv_pin), _sclk_pin(sclk_pin), _sdo_pin(sdo_pin), _sdi_pin(sdi_pin),
      _busy_pin(busy_pin), _baudrate(spi_baudrate), _mode(MODE_POLLING), _ref_voltage(REF_2V5),
      _callback(nullptr), _conversion_in_progress(false), _last_result(0) {}

bool AD7980::begin(Mode mode) {
    _mode = mode;

    // Initialize CNV pin (conversion start)
    gpio_init(_cnv_pin);
    gpio_set_dir(_cnv_pin, GPIO_OUT);
    gpio_put(_cnv_pin, 0);  // CNV idle low

    // Initialize BUSY pin if provided
    if (_busy_pin >= 0) {
        gpio_init(_busy_pin);
        gpio_set_dir(_busy_pin, GPIO_IN);
        gpio_pull_up(_busy_pin);  // BUSY is active low
    }

    // Initialize SPI
    spi_init(_spi, _baudrate);

    // Configure SPI pins
    gpio_set_function(_sclk_pin, GPIO_FUNC_SPI);  // SCLK
    gpio_set_function(_sdo_pin, GPIO_FUNC_SPI);   // SDO (MISO)

    // SDI (MOSI) is optional for AD7980
    if (_sdi_pin >= 0) {
        gpio_set_function(_sdi_pin, GPIO_FUNC_SPI);
    }

    // SPI Mode 0: CPOL=0, CPHA=0
    spi_set_format(_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Setup interrupts if in interrupt mode
    if (_mode == MODE_INTERRUPT && _busy_pin >= 0) {
        setupBusyInterrupt();
    }

    // Perform dummy conversion to initialize
    sleep_us(100);
    readSingle();

    return true;
}

void AD7980::pulseCNV() {
    // CNV rising edge initiates conversion
    // Minimum CNV high time: 10 ns (easily met by RP2040)
    cnvHigh();
    sleep_us(1);  // Keep CNV high for >10ns
    cnvLow();
}

uint16_t AD7980::readSPI() {
    uint8_t rx_buffer[2] = {0, 0};
    uint8_t tx_buffer[2] = {0, 0};  // Dummy data for clock generation

    // Read 16 bits (2 bytes)
    // TODO: Change to spi_read_blocking if available
    spi_write_read_blocking(_spi, tx_buffer, rx_buffer, 2);

    // Combine bytes into 16-bit result (MSB first)
    uint16_t result = (static_cast<uint16_t>(rx_buffer[0]) << 8) | rx_buffer[1];

    return result;
}

void AD7980::waitConversionComplete() {
    if (_busy_pin >= 0) {
        // Wait for BUSY to go low (conversion complete)
        // BUSY goes high after CNV rising edge
        // BUSY goes low when conversion is complete
        // Maximum conversion time: 710 ns at 1 MSPS

        uint32_t timeout = 1000;  // 1000 iterations timeout
        while (gpio_get(_busy_pin) == 0 && timeout > 0) {
            timeout--;
            sleep_us(1);
        }

        // Now wait for BUSY to go low (conversion complete)
        timeout = 1000;
        while (gpio_get(_busy_pin) == 1 && timeout > 0) {
            timeout--;
            sleep_us(1);
        }
    } else {
        // Without BUSY pin, use fixed delay
        // Conversion time is ~710 ns, add margin
        sleep_us(1);
    }
}

uint16_t AD7980::readSingle() {
    // Start conversion
    pulseCNV();

    // Wait for conversion to complete
    waitConversionComplete();

    // Read result via SPI
    uint16_t result = readSPI();

    _last_result = result;
    return result;
}

bool AD7980::startConversion() {
    if (_conversion_in_progress) {
        return false;  // Conversion already in progress
    }

    _conversion_in_progress = true;
    pulseCNV();

    return true;
}

bool AD7980::isConversionReady() {
    if (_busy_pin >= 0) {
        // BUSY is active high during conversion, goes low when complete
        return (gpio_get(_busy_pin) == 0);
    }

    // Without BUSY pin, assume conversion is complete after minimum time
    return !_conversion_in_progress;
}

uint16_t AD7980::readResult() {
    if (_conversion_in_progress) {
        waitConversionComplete();
    }

    uint16_t result = readSPI();
    _last_result = result;
    _conversion_in_progress = false;

    return result;
}

void AD7980::setCallback(ConversionCallback callback) {
    _callback = callback;
}

void AD7980::setupBusyInterrupt() {
    if (_busy_pin < 0) {
        return;  // No BUSY pin configured
    }

    // Store global instance pointer for interrupt handler
    g_ad7980_instance = this;

    // Configure interrupt on falling edge (conversion complete)
    gpio_set_irq_enabled_with_callback(_busy_pin, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_wrapper);
}

void AD7980::enableInterrupts(bool enable) {
    if (_busy_pin < 0) {
        return;
    }

    gpio_set_irq_enabled(_busy_pin, GPIO_IRQ_EDGE_FALL, enable);
}

void AD7980::handleBusyInterrupt() {
    // Called when BUSY goes low (conversion complete)
    if (_conversion_in_progress) {
        // Read the conversion result
        _last_result = readSPI();
        _conversion_in_progress = false;

        // Call user callback if registered
        if (_callback != nullptr) {
            _callback(_last_result);
        }
    }
}

float AD7980::rawToVoltage(uint16_t raw_value, ReferenceVoltage ref_voltage) {
    float vref = (ref_voltage == REF_2V5) ? 2.5f : 5.0f;

    // AD7980 is straight binary output
    // Output code = (VIN / VREF) × 65536
    // Therefore: VIN = (code / 65536) × VREF
    float voltage = (static_cast<float>(raw_value) / 65536.0f) * vref;

    return voltage;
}

void AD7980::setReferenceVoltage(ReferenceVoltage ref) {
    _ref_voltage = ref;
}

AD7980::ReferenceVoltage AD7980::getReferenceVoltage() const {
    return _ref_voltage;
}

uint16_t AD7980::readAveraged(uint16_t num_samples) {
    if (num_samples == 0) {
        return 0;
    }

    uint32_t accumulator = 0;

    for (uint16_t i = 0; i < num_samples; i++) {
        accumulator += readSingle();
    }

    return static_cast<uint16_t>(accumulator / num_samples);
}

void AD7980::powerDown() {
    // AD7980 automatically enters power-down when CNV is held low
    // Already in power-down state when not converting
    cnvLow();
}

void AD7980::wakeUp() {
    // Wake up by performing a conversion
    readSingle();
}