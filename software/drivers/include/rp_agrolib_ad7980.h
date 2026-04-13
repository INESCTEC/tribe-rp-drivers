/**
 * @file rp_agrolib_ad7980.h
 * @brief Driver for Analog Devices AD7980 16-bit 1 MSPS SAR ADC
 * @author CRIIS - INESC TEC
 *
 * Features:
 * - 16-bit resolution, no missing codes
 * - 1 MSPS sampling rate
 * - Single pseudo-differential input
 * - 3-wire SPI interface (CNV, SCK, SDO)
 * - BUSY signal for interrupt-driven conversion
 * - 2.5V or 5V reference options
 * - Low power: 4.9 mW at 1 MSPS
 *
 * TODO: WIP - NOT TESTED YET
 */

#ifndef RP_AGROLIB_AD7980_H
#define RP_AGROLIB_AD7980_H

#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"

#include <cstdint>

class AD7980 {
public:
    /**
     * @brief Conversion complete callback function type
     * @param conversion_result 16-bit ADC reading
     */
    typedef void (*ConversionCallback)(uint16_t conversion_result);

    /**
     * @brief Operating modes
     */
    enum Mode {
        MODE_POLLING,   // Software polling (blocking)
        MODE_INTERRUPT  // Interrupt-driven (non-blocking)
    };

    /**
     * @brief Reference voltage options
     */
    enum ReferenceVoltage {
        REF_2V5 = 0,  // 2.5V reference
        REF_5V0 = 1   // 5.0V reference
    };

    /**
     * @brief Constructor
     * @param spi_inst SPI instance (spi0 or spi1)
     * @param cnv_pin CNV (Convert) GPIO pin - initiates conversion
     * @param sclk_pin SPI clock GPIO pin
     * @param sdo_pin SPI MISO GPIO pin (connects to AD7980 SDO - Serial Data Out)
     * @param sdi_pin SPI MOSI GPIO pin (connects to AD7980 SDI - Serial Data In, optional)
     * @param busy_pin BUSY GPIO pin - indicates conversion in progress (optional, -1 to disable)
     * @param spi_baudrate SPI clock frequency (default 20 MHz, max 48 MHz)
     */
    AD7980(spi_inst_t *spi_inst, uint cnv_pin, uint sclk_pin, uint sdo_pin, int sdi_pin = -1,
           int busy_pin = -1, uint32_t spi_baudrate = 20000000);

    /**
     * @brief Initialize the ADC
     * @param mode Operating mode (polling or interrupt)
     * @return true if initialization successful
     */
    bool begin(Mode mode = MODE_POLLING);

    /**
     * @brief Perform a single conversion (blocking)
     * @return 16-bit ADC reading
     */
    uint16_t readSingle();

    /**
     * @brief Start a conversion (non-blocking, requires BUSY pin)
     * @return true if conversion started successfully
     */
    bool startConversion();

    /**
     * @brief Check if conversion is complete
     * @return true if conversion is ready to be read
     */
    bool isConversionReady();

    /**
     * @brief Read the result of a completed conversion
     * @return 16-bit ADC reading
     */
    uint16_t readResult();

    /**
     * @brief Set callback for interrupt-driven conversions
     * @param callback Function to call when conversion completes
     */
    void setCallback(ConversionCallback callback);

    /**
     * @brief Enable interrupt-driven conversions
     * @param enable true to enable, false to disable
     */
    void enableInterrupts(bool enable);

    /**
     * @brief Convert raw ADC value to voltage
     * @param raw_value 16-bit ADC reading
     * @param ref_voltage Reference voltage used (REF_2V5 or REF_5V0)
     * @return Voltage in volts
     */
    float rawToVoltage(uint16_t raw_value, ReferenceVoltage ref_voltage);

    /**
     * @brief Set reference voltage for voltage calculations
     * @param ref Reference voltage setting
     */
    void setReferenceVoltage(ReferenceVoltage ref);

    /**
     * @brief Get current reference voltage setting
     * @return Current reference voltage
     */
    ReferenceVoltage getReferenceVoltage() const;

    /**
     * @brief Perform multiple conversions and return average
     * @param num_samples Number of samples to average
     * @return Averaged 16-bit result
     */
    uint16_t readAveraged(uint16_t num_samples);

    /**
     * @brief Get maximum sampling rate in Hz
     * @return Maximum sampling rate
     */
    uint32_t getMaxSampleRate() const { return 1000000; }  // 1 MSPS

    /**
     * @brief Power down the ADC (stops conversions)
     */
    void powerDown();

    /**
     * @brief Wake up the ADC from power-down
     */
    void wakeUp();

    /**
     * @brief Get BUSY pin GPIO number
     * @return BUSY pin number, or -1 if not configured
     */
    int getBusyPin() const { return _busy_pin; }

    /**
     * @brief GPIO interrupt handler (internal use)
     */
    void handleBusyInterrupt();

private:
    spi_inst_t *_spi;
    uint _cnv_pin;
    uint _sclk_pin;
    uint _sdo_pin;  // RP2040 MISO -> AD7980 SDO (Serial Data Out)
    int _sdi_pin;   // RP2040 MOSI -> AD7980 SDI (optional, -1 if unused)
    int _busy_pin;  // BUSY signal pin (-1 if unused)
    uint32_t _baudrate;

    Mode _mode;
    ReferenceVoltage _ref_voltage;
    ConversionCallback _callback;
    volatile bool _conversion_in_progress;
    volatile uint16_t _last_result;

    /**
     * @brief Initiate conversion by pulsing CNV
     */
    void pulseCNV();

    /**
     * @brief Read data from SPI
     * @return 16-bit conversion result
     */
    uint16_t readSPI();

    /**
     * @brief Wait for conversion to complete (polling BUSY)
     */
    void waitConversionComplete();

    /**
     * @brief Setup BUSY pin interrupt
     */
    void setupBusyInterrupt();

    /**
     * @brief CNV high time (minimum 10 ns)
     */
    inline void cnvHigh() { gpio_put(_cnv_pin, 1); }

    /**
     * @brief CNV low time (minimum 10 ns)
     */
    inline void cnvLow() { gpio_put(_cnv_pin, 0); }
};

#endif  // RP_AGROLIB_AD7980_H