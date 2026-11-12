/**
 * @file rp_agrolib_ads868x.h
 * @brief Driver for the TI ADS868x family of 16-bit SAR ADCs (ADS8684, ADS8688, ADS8684A, ADS8688A)
 * @author CRIIS - INESC TEC
 *
 * Formerly rp_agrolib_ads8684 (ADS8684-only); renamed when it became multi-variant.
 *
 * References (page numbers are the printed page numbers of each datasheet):
 * - SBAS680  (July 2015)                    — ADS8684A / ADS8688A
 * - SBAS582C (July 2014, rev. April 2015)  — ADS8684 / ADS8688
 *
 * Both datasheets describe the same SPI protocol, command set, register map and timing. The A
 * variants add four extra input ranges (SBAS680 Table 3 p27, Table 15 p58) and the ALARM feature
 * (SBAS680 §8.3.11 p36, registers 10h–3Ch). Unless stated otherwise, the references below are to
 * SBAS680.
 *
 * SPI: mode 1 (CPOL = 0, CPHA = 1), MSB first, CS driven by GPIO, SCLK ≤ 17 MHz (p11). The device
 * latches SDI and shifts SDO on SCLK falling edges (p37); the conversion MSB is output on the 16th
 * falling edge, so it is sampled by the host on the 17th (Fig. 1 p11).
 *
 * IMPLEMENTED
 * - Manual channel scan (MAN_Ch_n) and AUX channel (MAN_AUX), raw codes and volts
 * - All input ranges of each variant (5 for ADS8684/88, 9 for ADS8684A/88A), validated per variant
 * - Auto-scan (AUTO_RST + AUTO_SEQ_EN + per-channel power-down), channel taken from the SDO tag
 * - STDBY and PWR_DN (software, and hardware through the optional RST/PD pin), SW and HW reset
 * - ALARM (A variants only): thresholds in codes or volts, flag readout, optional IRQ callback
 * - Register readback verification on every write (echo), self-test, configuration check with
 *   automatic re-application after an ADC brown-out/reset, per-sample channel tag check (SDO 001)
 * - Error, re-configuration and saturation counters; rate-limited logging on the read path
 *
 * NOT IMPLEMENTED
 * - Daisy-chain operation (N × 16-bit frames, DEV[1:0] in register 03h)
 * - SDO formats 010 and 011 (device address / range in the output stream)
 * - REFSEL control: only the VREF value used for conversions is configurable
 * - Concurrent use from both RP2040 cores without an external lock
 * - Bench validation of ADS8684, ADS8688 and ADS8688A (only the ADS8684A was available)
 *
 * [VERIFICAR] The channel tag that SDO format 001 outputs for the AUX channel is not specified
 * (Table 14 p57 lists CH0–CH7 only); AUX reads skip the tag check.
 * [VERIFICAR] The PGA/LPF settling time after a range change is not specified in SBAS680; the
 * driver discards the first conversion of a channel after setInputRange().
 */

#ifndef RP_AGROLIB_ADS868X_H
#define RP_AGROLIB_ADS868X_H

#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"

#include <cstdarg>
#include <cstdint>

class ADS868x {
public:
    /// Device variant. ADS8684/ADS8684A have 4 channels, ADS8688/ADS8688A have 8 (p1).
    enum class Variant : uint8_t { ADS8684, ADS8688, ADS8684A, ADS8688A };

    /// Analog input channel. CH4..CH7 exist only on the 8-channel parts (Table 14 p57).
    enum Channel : uint8_t { CH0 = 0, CH1, CH2, CH3, CH4, CH5, CH6, CH7 };

    /**
     * Range_CHn[3:0] codes (Table 15 p58; SBAS582C Table 3 p25). VREF = 4.096 V gives the volts in
     * the names (Table 4 p35). Codes marked (A) exist only on the ADS8684A/ADS8688A.
     */
    enum InputRange : uint8_t {
        RANGE_PM_10P24V = 0x0,    ///< ±2.5 × VREF
        RANGE_PM_5P12V = 0x1,     ///< ±1.25 × VREF
        RANGE_PM_2P56V = 0x2,     ///< ±0.625 × VREF
        RANGE_PM_1P28V = 0x3,     ///< ±0.3125 × VREF (A)
        RANGE_PM_0P64V = 0xB,     ///< ±0.15625 × VREF (A)
        RANGE_0_TO_10P24V = 0x5,  ///< 0 to 2.5 × VREF
        RANGE_0_TO_5P12V = 0x6,   ///< 0 to 1.25 × VREF
        RANGE_0_TO_2P56V = 0x7,   ///< 0 to 0.625 × VREF (A)
        RANGE_0_TO_1P28V = 0xF    ///< 0 to 0.3125 × VREF (A)
    };

    /// Program register addresses (Table 9 p53-54). Shaded (8-channel only) entries are marked (8).
    enum Register : uint8_t {
        REG_AUTO_SEQ_EN = 0x01,       ///< Auto-scan sequence enable, default FFh (Table 10 p55)
        REG_CH_PWR_DN = 0x02,         ///< Channel power down, 1 = powered down (Table 11 p56)
        REG_FEATURE_SELECT = 0x03,    ///< DEV[7:6], ALARM_EN[4] (A), SDO[2:0] (Table 12 p57)
        REG_RANGE_CH0 = 0x05,         ///< Range_CH0..CH7 at 05h..0Ch (Table 15 p58)
        REG_RANGE_CH7 = 0x0C,         ///< (8)
        REG_ALARM_OVERVIEW = 0x10,    ///< Tripped flag per channel, read-only (A) (Table 16 p59)
        REG_ALARM_CH0_3_TRIP = 0x11,  ///< Tripped low/high CH0–3, read-only (A) (Table 17 p60)
        REG_ALARM_CH0_3_ACT = 0x12,   ///< Active low/high CH0–3, read-only (A) (Table 18 p60)
        REG_ALARM_CH4_7_TRIP = 0x13,  ///< (8) Tripped low/high CH4–7, read-only (A) (Table 19 p60)
        REG_ALARM_CH4_7_ACT = 0x14,   ///< (8) Active low/high CH4–7, read-only (A) (Table 20 p61)
        REG_CH0_HYST = 0x15,          ///< CHn_HYST at 15h + 5n (A) (Table 21 p63)
        REG_CH0_HT_MSB = 0x16,        ///< CHn_HT[15:8] at 16h + 5n, default FFh (A) (Table 22 p63)
        REG_CH0_HT_LSB = 0x17,        ///< CHn_HT[7:0] at 17h + 5n, default FFh (A) (Table 23 p64)
        REG_CH0_LT_MSB = 0x18,        ///< CHn_LT[15:8] at 18h + 5n, default 00h (A) (Table 24 p64)
        REG_CH0_LT_LSB = 0x19,        ///< CHn_LT[7:0] at 19h + 5n, default 00h (A) (Table 25 p65)
        REG_CH7_LT_LSB = 0x3C,        ///< (8) Last alarm threshold register (A)
        REG_CMD_READBACK = 0x3F  ///< Last executed command word [15:8], read-only (Table 26 p65)
    };

    /// Status of the last operation. See lastError() / errorToString().
    enum class Error : uint8_t {
        NONE = 0,
        NOT_INITIALIZED,         ///< begin() not called or failed
        INVALID_ARG,             ///< Channel/register/value out of range for this variant
        UNSUPPORTED_BY_VARIANT,  ///< Range or feature exists only on the A variants
        VERIFY_FAILED,           ///< Register echo/readback mismatch, or configuration lost
        CHANNEL_MISMATCH,        ///< SDO channel tag differs from the requested channel
        INVALID_STATE,           ///< Operation not allowed in the current mode (STDBY/PWR_DN/...)
        COUNT_                   ///< Number of error codes (internal)
    };

    /// Alarm flags decoded from registers 10h–14h. Bit n = channel n.
    struct AlarmFlags {
        uint8_t overview_tripped;  ///< 10h (Table 16 p59)
        uint8_t tripped_low;       ///< 11h/13h (Tables 17, 19)
        uint8_t tripped_high;      ///< 11h/13h
        uint8_t active_low;        ///< 12h/14h (Tables 18, 20)
        uint8_t active_high;       ///< 12h/14h
    };

    /// Conversion code limits used by isSaturated() (16-bit straight binary, p35).
    static constexpr uint16_t CODE_MIN = 0x0000;
    static constexpr uint16_t CODE_MAX = 0xFFFF;
    /// Sentinel returned by readChannel() on error (see lastError()).
    static constexpr uint16_t READ_ERROR = 0xFFFF;

    /**
     * @param spi        SPI instance (spi0 / spi1)
     * @param cs_pin     Chip-select GPIO (driven by the driver, active low)
     * @param sclk_pin   SCLK GPIO
     * @param sdi_pin    GPIO wired to the ADC SDI (RP2040 TX / MOSI)
     * @param sdo_pin    GPIO wired to the ADC SDO (RP2040 RX / MISO)
     * @param baudrate   Requested SCLK in Hz; ≤ 17 MHz (p11). The real value is printed by begin().
     * @param variant    Device variant; the default is the most restrictive one
     * @param rst_pd_pin GPIO wired to RST/PD (pin 2), or -1 if not connected
     * @param alarm_pin  GPIO wired to ALARM (pin 35, A variants), or -1 if not connected
     * @param vref       Reference voltage in volts (internal reference: 4.096 V, p9)
     */
    ADS868x(spi_inst_t *spi, uint cs_pin, uint sclk_pin, uint sdi_pin, uint sdo_pin,
            uint32_t baudrate = 1000000, Variant variant = Variant::ADS8684, int rst_pd_pin = -1,
            int alarm_pin = -1, float vref = 4.096f);

    /**
     * Initializes the GPIOs and (optionally) the SPI peripheral, waits for the internal reference,
     * resets the device, selects the SDO format and runs selfTest(). Prints the cause and returns
     * false on any failure.
     * @param init_spi false if the SPI bus is shared and already initialized by another driver.
     */
    bool begin(bool init_spi = true);

    /// Software reset (RST, 8500h, p49): registers and cache back to defaults, SDO format
    /// re-applied.
    bool reset();
    /// RST/PD low ≥ 1 µs (PWR_DN with register reset, Table 5 p38), 15 ms (p45), cached config
    /// re-applied. Requires rst_pd_pin.
    bool hardwareReset();
    /// Register write/readback test (Range_CH0; CH0 hysteresis on A variants). Restores the config.
    bool selfTest();
    /// Reads back 01h, 02h, 03h and the range registers. On mismatch re-applies the cached
    /// configuration, increments reconfigCount() and returns false (VERIFY_FAILED).
    bool checkConfig();

    uint32_t errorCount() const { return _error_count; }
    uint32_t reconfigCount() const { return _reconfig_count; }
    uint32_t saturationCount(Channel ch) const;
    /// Clears the counters and re-arms the one-shot logging of read errors.
    void clearErrors();
    static bool isSaturated(uint16_t raw) { return raw == CODE_MIN || raw == CODE_MAX; }

    /// Writes and verifies Range_CHn. The next conversion of the channel is discarded.
    bool setInputRange(Channel ch, InputRange range);
    /// Cached range (no SPI access).
    InputRange getInputRange(Channel ch) const;
    /// Range read from the device.
    bool readInputRange(Channel ch, InputRange &range);

    /// Manual-mode conversion of one channel (MAN_Ch_n + NO_OP, p47).
    bool read(Channel ch, uint16_t &raw);
    bool readVoltage(Channel ch, float &volts);
    /// Converts numChannels() channels in a pipeline; data must hold numChannels() values.
    bool readAllChannels(uint16_t *data);
    /// AUX input conversion (MAN_AUX, E000h); range 0..VREF (p9, §8.3.9 p32).
    bool readAux(uint16_t &raw);
    float rawToVoltage(uint16_t raw, InputRange range) const;
    float auxToVoltage(uint16_t raw) const;
    /// Compatibility wrapper: returns READ_ERROR (0xFFFF) on failure, see lastError().
    uint16_t readChannel(Channel ch);

    /**
     * Starts AUTO_RST (A000h, p46) over the channels in channel_mask (bit n = channel n). Forces
     * SDO format 001 so that each sample carries its channel. Any later register access leaves
     * auto mode; call startAutoScan() again.
     * @param power_down_unused powers down the channels outside the sequence (register 02h).
     */
    bool startAutoScan(uint8_t channel_mask, bool power_down_unused = false);
    /// Reads the next auto-scan sample and the channel it belongs to.
    bool readAutoScan(uint16_t &raw, Channel &ch);

    /// STDBY (8200h, p43). Registers are kept.
    bool standby();
    /// PWR_DN by software (8300h, p45; registers kept) or by holding RST/PD low (registers reset,
    /// re-applied by wakeUp()).
    bool powerDown(bool hardware = false);
    /// Leaves STDBY/PWR_DN and waits 20 µs (p44) or 15 ms (p45) before the next conversion.
    bool wakeUp();
    /// Channel power down bit in 02h (1 = powered down, Table 11 p56).
    bool setChannelPowerDown(Channel ch, bool power_down);

    // ---- ALARM (A variants only, UNSUPPORTED_BY_VARIANT otherwise) ----
    /// ALARM_EN, register 03h bit 4 (Table 12 p57).
    bool enableAlarm(bool enable);
    /// Thresholds in output codes; they depend on the channel range (p36). Hysteresis in LSB.
    bool setAlarmThresholds(Channel ch, uint16_t low, uint16_t high, uint8_t hysteresis);
    /// Thresholds in volts; recomputed when the channel range changes.
    bool setAlarmThresholdsVolts(Channel ch, float low_v, float high_v, float hysteresis_v);
    /// Reads 10h–14h. Tripped flags stay set until read (p59).
    bool readAlarmFlags(AlarmFlags &flags);
    /// Calls cb(ctx) from the GPIO IRQ on the ALARM rising edge (active high, p36). The callback
    /// runs in interrupt context: set a flag there and access the ADC from the main loop.
    bool setAlarmCallback(void (*cb)(void *), void *ctx);

    // ---- Low level ----
    /// Writes a program register and verifies the echo (p51). Updates the internal cache.
    bool writeRegister(uint8_t address, uint8_t value);
    /// Reads a program register (Table 8 p52).
    bool readRegister(uint8_t address, uint8_t &value);
    /// true: SDO format 001 (40 SCLK, channel tag checked on every sample); false: 000 (32 SCLK).
    bool setChannelCheck(bool enable);
    bool channelCheck() const { return _channel_check; }
    /// Changes SCLK; returns the real baud rate.
    uint32_t setBaudrate(uint32_t hz);
    uint32_t baudrate() const { return _baud_actual; }

    Error lastError() const { return _last_error; }
    static const char *errorToString(Error e);
    Variant variant() const { return _variant; }
    uint8_t numChannels() const { return _num_channels; }
    bool isRangeSupported(InputRange range) const;
    bool isAVariant() const;

    /// Called by the GPIO IRQ dispatcher (internal).
    void handleAlarmIrq_();

private:
    // Command register words (Table 6 p50).
    static constexpr uint16_t CMD_NO_OP = 0x0000;
    static constexpr uint16_t CMD_STDBY = 0x8200;
    static constexpr uint16_t CMD_PWR_DN = 0x8300;
    static constexpr uint16_t CMD_RST = 0x8500;
    static constexpr uint16_t CMD_AUTO_RST = 0xA000;
    static constexpr uint16_t CMD_MAN_CH0 = 0xC000;  ///< MAN_Ch_n = C000h + n × 0400h
    static constexpr uint16_t CMD_MAN_AUX = 0xE000;

    // Feature Select (Table 12 p57).
    static constexpr uint8_t FEAT_ALARM_EN = 0x10;
    static constexpr uint8_t FEAT_SDO_MASK = 0x07;
    static constexpr uint8_t SDO_FMT_000 = 0x00;  ///< Conversion only (Table 13 p57)
    static constexpr uint8_t SDO_FMT_001 = 0x01;  ///< Conversion + 4-bit channel tag

    // Timing.
    static constexpr uint32_t T_REF_STARTUP_MS = 15;  ///< Reference turn-on, 22 µF (p9)
    static constexpr uint32_t T_PWR_DN_EXIT_MS = 15;  ///< PWR_DN exit (p45)
    static constexpr uint32_t T_STDBY_EXIT_US = 20;   ///< Minimum CS high after STDBY exit (p44)
    static constexpr uint32_t T_RST_PD_LOW_US =
            1;  ///< ≥ 400 ns → PWR_DN + register reset (Table 5)

    static constexpr uint8_t AUX_INDEX = 8;  ///< Internal index of the AUX channel
    static constexpr uint8_t NO_CHANNEL = 0xFF;

    enum class Mode : uint8_t { IDLE, MAN, AUTO, STDBY, PWR_DN_SW, PWR_DN_HW };

    spi_inst_t *_spi;
    uint _cs_pin, _sclk_pin, _sdi_pin, _sdo_pin;
    int _rst_pd_pin, _alarm_pin;
    uint32_t _baud_requested, _baud_actual;
    Variant _variant;
    uint8_t _num_channels;
    uint8_t _ch_mask;
    float _vref;

    bool _initialized;
    Mode _mode;
    uint8_t _cur_ch;      ///< Channel selected by the last MAN command
    bool _channel_check;  ///< User preference (setChannelCheck)
    bool _sdo_tag;        ///< Format currently programmed (001 = true)

    // Register cache (what the device should contain).
    uint8_t _feature_shadow;  ///< 03h
    uint8_t _auto_seq;        ///< 01h
    uint8_t _ch_pd;           ///< 02h
    uint8_t _range[8];
    bool _settle_pending[8];
    uint8_t _hyst[8];
    uint16_t _thr_high[8], _thr_low[8];
    bool _thr_in_volts[8];
    float _thr_low_v[8], _thr_high_v[8], _hyst_v[8];

    // Diagnostics.
    Error _last_error;
    uint32_t _error_count, _reconfig_count;
    uint32_t _saturation[8];
    uint32_t _reported_mask;  ///< One bit per Error already printed on the read path
    bool _in_read_path;       ///< Rate-limit fail() while repairing from a read

    void (*_alarm_cb)(void *);
    void *_alarm_ctx;

    // Error helpers. fail() always prints; failRead() prints once per error type.
    bool fail(Error e, const char *fmt, ...);
    bool failRead(Error e, const char *fmt, ...);
    bool report(Error e, bool rate_limited, const char *fmt, va_list args);
    bool ok();

    // SPI.
    void prepareBus();
    void transfer(const uint8_t *tx, uint8_t *rx, size_t len);
    uint16_t convFrame(uint16_t cmd, uint8_t *tag);
    bool regWriteRaw(uint8_t address, uint8_t value, uint8_t *echo);
    uint8_t regReadRaw(uint8_t address);
    bool regWriteVerified(uint8_t address, uint8_t value);
    void afterRegisterAccess();

    // Helpers.
    bool validChannel(uint8_t ch) const { return ch < _num_channels; }
    uint8_t writableMask(uint8_t address) const;
    bool addressAllowed(uint8_t address, bool for_write, Error &why) const;
    void updateCache(uint8_t address, uint8_t value);
    void resetCache();
    bool applyConfig();
    bool verifyAndRepair(bool from_read_path);
    bool writeThresholds(uint8_t ch);
    void thresholdsFromVolts(uint8_t ch);
    static float rangeFactor(uint8_t code, bool &bipolar);
    bool checkTag(uint8_t expected, uint8_t tag);
    void countSaturation(uint8_t ch, uint16_t raw);
    bool ensureConversionAllowed();
};


#endif  // RP_AGROLIB_ADS868X_H
