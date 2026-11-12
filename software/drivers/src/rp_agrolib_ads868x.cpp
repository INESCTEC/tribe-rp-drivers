/**
 * @file rp_agrolib_ads868x.cpp
 * @brief Driver for the TI ADS868x family (ADS8684, ADS8688, ADS8684A, ADS8688A)
 * @author CRIIS - INESC TEC
 *
 * Page/table references are to SBAS680 (ADS8684A/ADS8688A) unless stated otherwise; SBAS582C
 * (ADS8684/ADS8688) describes the same protocol. See the header for the feature list.
 */

#include "rp_agrolib_ads868x.h"

#include "hardware/irq.h"

#include <cstdarg>
#include <cstdio>

namespace {

/// Owner of each GPIO used as ALARM input, for the shared GPIO IRQ dispatcher.
ADS868x *g_alarm_owner[NUM_BANK0_GPIOS] = {nullptr};

void alarm_irq_dispatch() {
    for (uint pin = 0; pin < NUM_BANK0_GPIOS; ++pin) {
        ADS868x *owner = g_alarm_owner[pin];
        if (owner != nullptr && (gpio_get_irq_event_mask(pin) & GPIO_IRQ_EDGE_RISE) != 0u) {
            gpio_acknowledge_irq(pin, GPIO_IRQ_EDGE_RISE);
            owner->handleAlarmIrq_();
        }
    }
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Construction / diagnostics
// ---------------------------------------------------------------------------------------------

ADS868x::ADS868x(spi_inst_t *spi, uint cs_pin, uint sclk_pin, uint sdi_pin, uint sdo_pin,
                 uint32_t baudrate, Variant variant, int rst_pd_pin, int alarm_pin, float vref)
    : _spi(spi), _cs_pin(cs_pin), _sclk_pin(sclk_pin), _sdi_pin(sdi_pin), _sdo_pin(sdo_pin),
      _rst_pd_pin(rst_pd_pin), _alarm_pin(alarm_pin), _baud_requested(baudrate), _baud_actual(0),
      _variant(variant), _num_channels(4), _ch_mask(0x0F), _vref(vref), _initialized(false),
      _mode(Mode::IDLE), _cur_ch(NO_CHANNEL), _channel_check(true), _sdo_tag(false),
      _feature_shadow(0), _auto_seq(0), _ch_pd(0), _last_error(Error::NONE), _error_count(0),
      _reconfig_count(0), _reported_mask(0), _in_read_path(false), _alarm_cb(nullptr),
      _alarm_ctx(nullptr) {
    if (variant == Variant::ADS8688 || variant == Variant::ADS8688A) {
        _num_channels = 8;
        _ch_mask = 0xFF;
    }
    for (uint8_t i = 0; i < 8; ++i) {
        _saturation[i] = 0;
        _settle_pending[i] = false;
        _thr_low_v[i] = 0.0f;
        _thr_high_v[i] = 0.0f;
        _hyst_v[i] = 0.0f;
    }
    resetCache();
}

bool ADS868x::isAVariant() const {
    return _variant == Variant::ADS8684A || _variant == Variant::ADS8688A;
}

bool ADS868x::isRangeSupported(InputRange range) const {
    switch (range) {
        // Common to both datasheets (SBAS582C Table 3 p25, SBAS680 Table 15 p58).
        case RANGE_PM_10P24V:
        case RANGE_PM_5P12V:
        case RANGE_PM_2P56V:
        case RANGE_0_TO_10P24V:
        case RANGE_0_TO_5P12V:
            return true;
        // A variants only (SBAS680 Table 15 p58).
        case RANGE_PM_1P28V:
        case RANGE_PM_0P64V:
        case RANGE_0_TO_2P56V:
        case RANGE_0_TO_1P28V:
            return isAVariant();
        default:
            return false;
    }
}

const char *ADS868x::errorToString(Error e) {
    switch (e) {
        case Error::NONE:
            return "NONE";
        case Error::NOT_INITIALIZED:
            return "NOT_INITIALIZED";
        case Error::INVALID_ARG:
            return "INVALID_ARG";
        case Error::UNSUPPORTED_BY_VARIANT:
            return "UNSUPPORTED_BY_VARIANT";
        case Error::VERIFY_FAILED:
            return "VERIFY_FAILED";
        case Error::CHANNEL_MISMATCH:
            return "CHANNEL_MISMATCH";
        case Error::INVALID_STATE:
            return "INVALID_STATE";
        default:
            return "UNKNOWN";
    }
}

bool ADS868x::ok() {
    _last_error = Error::NONE;
    return true;
}

bool ADS868x::report(Error e, bool rate_limited, const char *fmt, va_list args) {
    _last_error = e;
    ++_error_count;
    if (rate_limited) {
        // Read path: only the first failure of each type is printed, because readChannel() runs
        // once per pixel and a USB-CDC printf may block (design decision D18).
        const uint32_t bit = 1u << static_cast<uint8_t>(e);
        if ((_reported_mask & bit) != 0u) {
            return false;
        }
        _reported_mask |= bit;
        printf("[ADS868x] %s (further errors of this type are only counted): ", errorToString(e));
    } else {
        printf("[ADS868x] %s: ", errorToString(e));
    }
    vprintf(fmt, args);
    printf("\n");
    return false;
}

bool ADS868x::fail(Error e, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    // Configuration errors always print, except while repairing from the read path.
    report(e, _in_read_path, fmt, args);
    va_end(args);
    return false;
}

bool ADS868x::failRead(Error e, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    report(e, true, fmt, args);
    va_end(args);
    return false;
}

void ADS868x::clearErrors() {
    _error_count = 0;
    _reconfig_count = 0;
    _reported_mask = 0;
    _last_error = Error::NONE;
    for (uint32_t &s : _saturation) {
        s = 0;
    }
}

uint32_t ADS868x::saturationCount(Channel ch) const {
    return (ch < 8) ? _saturation[ch] : 0;
}

void ADS868x::countSaturation(uint8_t ch, uint16_t raw) {
    if (ch < 8 && isSaturated(raw)) {
        ++_saturation[ch];
    }
}

// ---------------------------------------------------------------------------------------------
// SPI layer
// ---------------------------------------------------------------------------------------------

void ADS868x::prepareBus() {
    // The bus may be shared (SlurrySense 3.0: spi1 also serves AD7980/AD5680) and another driver
    // may have changed the mode or the clock: re-apply both on every transaction (D17).
    // SPI mode 1: CPOL = 0, CPHA = 1 (device shifts on SCLK falling edges, p37).
    spi_set_format(_spi, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    if (spi_get_baudrate(_spi) != _baud_actual) {
        _baud_actual = spi_set_baudrate(_spi, _baud_requested);
    }
}

void ADS868x::transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
    prepareBus();
    gpio_put(_cs_pin, 0);  // tSU_CSCK ≥ 30 ns (p11) is met by the GPIO/SPI start latency
    spi_write_read_blocking(_spi, tx, rx, len);
    gpio_put(_cs_pin, 1);  // tPH_CS ≥ 30 ns (p11)
}

uint16_t ADS868x::convFrame(uint16_t cmd, uint8_t *tag) {
    // Full conversion frame (p47 requires a complete frame for the next conversion to be valid):
    //  SDO 000: 32 SCLK; bits 17-32 = conversion result (Table 13 p57).
    //  SDO 001: 40 SCLK; bits 33-36 = channel address (Table 14 p57).
    uint8_t tx[5] = {static_cast<uint8_t>(cmd >> 8), static_cast<uint8_t>(cmd & 0xFF), 0, 0, 0};
    uint8_t rx[5] = {0, 0, 0, 0, 0};
    const size_t len = _sdo_tag ? 5 : 4;
    transfer(tx, rx, len);
    if (tag != nullptr) {
        *tag = _sdo_tag ? static_cast<uint8_t>(rx[4] >> 4) : NO_CHANNEL;
    }
    return static_cast<uint16_t>((rx[2] << 8) | rx[3]);
}

bool ADS868x::regWriteRaw(uint8_t address, uint8_t value, uint8_t *echo) {
    // Write cycle (Table 7 p51): ADDR[6:0] in bits 15-9, WR = 1 in bit 8, DIN[7:0] in bits 7-0;
    // the device echoes the written data on the next 8 SCLK (bits 17-24). Minimum 24 SCLK.
    const uint8_t tx[3] = {static_cast<uint8_t>((address << 1) | 0x01), value, 0};
    uint8_t rx[3] = {0, 0, 0};
    transfer(tx, rx, sizeof(tx));
    if (echo != nullptr) {
        *echo = rx[2];
    }
    afterRegisterAccess();
    return true;
}

uint8_t ADS868x::regReadRaw(uint8_t address) {
    // Read cycle (Table 8 p52): ADDR[6:0] in bits 15-9, WR = 0; DOUT[7:0] on bits 17-24.
    const uint8_t tx[3] = {static_cast<uint8_t>(address << 1), 0, 0};
    uint8_t rx[3] = {0, 0, 0};
    transfer(tx, rx, sizeof(tx));
    afterRegisterAccess();
    return rx[2];
}

void ADS868x::afterRegisterAccess() {
    // After a program-register access the device returns to IDLE and waits for a new command
    // (p42), so the next conversion needs a fresh MAN_Ch_n / AUTO_RST. STDBY and PWR_DN are kept:
    // register access is allowed there and does not leave the low-power mode (p43, p45).
    if (_mode == Mode::MAN || _mode == Mode::AUTO) {
        _mode = Mode::IDLE;
        _cur_ch = NO_CHANNEL;
    }
}

uint8_t ADS868x::writableMask(uint8_t address) const {
    if (address == REG_AUTO_SEQ_EN || address == REG_CH_PWR_DN) {
        // 4-channel parts: bits 7:4 are shaded, writes ignored and reads return 1 (p53 note 2).
        return _ch_mask;
    }
    if (address == REG_FEATURE_SELECT) {
        // DEV[7:6], ALARM_EN[4] (A only; SBAS582C Table 12 has bit 4 read-only), SDO[2:0].
        return isAVariant() ? 0xD7 : 0xC7;
    }
    if (address >= REG_RANGE_CH0 && address <= REG_RANGE_CH7) {
        return 0x0F;  // Range_CHn[3:0]; bits 7:4 read-only 0 (Table 15 p58)
    }
    return 0xFF;
}

bool ADS868x::addressAllowed(uint8_t address, bool for_write, Error &why) const {
    why = Error::INVALID_ARG;
    if (address == REG_AUTO_SEQ_EN || address == REG_CH_PWR_DN || address == REG_FEATURE_SELECT) {
        return true;
    }
    if (address >= REG_RANGE_CH0 && address <= REG_RANGE_CH7) {
        return validChannel(address - REG_RANGE_CH0);
    }
    if (address >= REG_ALARM_OVERVIEW && address <= REG_CH7_LT_LSB) {
        if (!isAVariant()) {
            why = Error::UNSUPPORTED_BY_VARIANT;  // SBAS582C has no alarm registers
            return false;
        }
        if (address <= REG_ALARM_CH4_7_ACT) {
            if (for_write) {
                return false;  // read-only flags (Table 9 p53)
            }
            return address <= REG_ALARM_CH0_3_ACT || _num_channels == 8;
        }
        return validChannel((address - REG_CH0_HYST) / 5);
    }
    if (address == REG_CMD_READBACK) {
        return !for_write;  // read-only (Table 26 p65)
    }
    return false;
}

void ADS868x::updateCache(uint8_t address, uint8_t value) {
    const uint8_t v = value & writableMask(address);
    if (address == REG_AUTO_SEQ_EN) {
        _auto_seq = v;
    } else if (address == REG_CH_PWR_DN) {
        _ch_pd = v;
    } else if (address == REG_FEATURE_SELECT) {
        _feature_shadow = v;
        _sdo_tag = (v & FEAT_SDO_MASK) == SDO_FMT_001;
    } else if (address >= REG_RANGE_CH0 && address <= REG_RANGE_CH7) {
        const uint8_t ch = address - REG_RANGE_CH0;
        if (_range[ch] != v) {
            _settle_pending[ch] = true;  // [VERIFICAR] settling after range change not specified
        }
        _range[ch] = v;
    } else if (address >= REG_CH0_HYST && address <= REG_CH7_LT_LSB) {
        const uint8_t idx = address - REG_CH0_HYST;
        const uint8_t ch = idx / 5;
        switch (idx % 5) {
            case 0:
                _hyst[ch] = v;
                break;
            case 1:
                _thr_high[ch] = static_cast<uint16_t>((_thr_high[ch] & 0x00FF) | (v << 8));
                break;
            case 2:
                _thr_high[ch] = static_cast<uint16_t>((_thr_high[ch] & 0xFF00) | v);
                break;
            case 3:
                _thr_low[ch] = static_cast<uint16_t>((_thr_low[ch] & 0x00FF) | (v << 8));
                break;
            default:
                _thr_low[ch] = static_cast<uint16_t>((_thr_low[ch] & 0xFF00) | v);
                break;
        }
    }
}

bool ADS868x::regWriteVerified(uint8_t address, uint8_t value) {
    uint8_t echo = 0;
    regWriteRaw(address, value, &echo);
    const uint8_t mask = writableMask(address);
    if ((echo & mask) != (value & mask)) {
        return fail(Error::VERIFY_FAILED, "write %02Xh <- %02Xh, echo %02Xh (p51)", address, value,
                    echo);
    }
    updateCache(address, value);
    return ok();
}

bool ADS868x::writeRegister(uint8_t address, uint8_t value) {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "writeRegister(%02Xh) before begin()", address);
    }
    Error why = Error::NONE;
    if (!addressAllowed(address, true, why)) {
        return fail(why, "register %02Xh is not writable on this variant", address);
    }
    if (address == REG_FEATURE_SELECT) {
        const uint8_t sdo = value & FEAT_SDO_MASK;
        if (sdo != SDO_FMT_000 && sdo != SDO_FMT_001) {
            return fail(Error::INVALID_ARG, "SDO format %u not supported (only 000/001)", sdo);
        }
    }
    return regWriteVerified(address, value);
}

bool ADS868x::readRegister(uint8_t address, uint8_t &value) {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "readRegister(%02Xh) before begin()", address);
    }
    Error why = Error::NONE;
    if (!addressAllowed(address, false, why)) {
        return fail(why, "register %02Xh is not readable on this variant", address);
    }
    value = regReadRaw(address);
    return ok();
}

// ---------------------------------------------------------------------------------------------
// Configuration cache
// ---------------------------------------------------------------------------------------------

void ADS868x::resetCache() {
    // Register defaults (Table 9 p53-54).
    _feature_shadow = 0x00;
    _sdo_tag = false;
    _auto_seq = _ch_mask;  // FFh, shaded bits ignored
    _ch_pd = 0x00;
    for (uint8_t i = 0; i < 8; ++i) {
        _range[i] = RANGE_PM_10P24V;  // 00h
        _hyst[i] = 0x00;
        _thr_high[i] = 0xFFFF;
        _thr_low[i] = 0x0000;
        _thr_in_volts[i] = false;
    }
}

bool ADS868x::applyConfig() {
    // Writes the whole cache to the device (used after reset / brown-out / HW power-down).
    const uint8_t feature = _feature_shadow;
    if (!regWriteVerified(REG_FEATURE_SELECT, feature) ||
        !regWriteVerified(REG_AUTO_SEQ_EN, _auto_seq) || !regWriteVerified(REG_CH_PWR_DN, _ch_pd)) {
        return false;
    }
    for (uint8_t ch = 0; ch < _num_channels; ++ch) {
        if (!regWriteVerified(REG_RANGE_CH0 + ch, _range[ch])) {
            return false;
        }
        if (isAVariant() && !writeThresholds(ch)) {
            return false;
        }
    }
    return ok();
}

bool ADS868x::verifyAndRepair(bool from_read_path) {
    bool same = true;
    uint8_t bad_addr = 0;
    uint8_t bad_val = 0;
    auto check = [&](uint8_t address, uint8_t expected) {
        const uint8_t v = regReadRaw(address) & writableMask(address);
        if (same && v != (expected & writableMask(address))) {
            same = false;
            bad_addr = address;
            bad_val = v;
        }
    };
    check(REG_FEATURE_SELECT, _feature_shadow);
    check(REG_AUTO_SEQ_EN, _auto_seq);
    check(REG_CH_PWR_DN, _ch_pd);
    for (uint8_t ch = 0; ch < _num_channels; ++ch) {
        check(REG_RANGE_CH0 + ch, _range[ch]);
    }
    if (same) {
        return ok();
    }
    // Configuration lost (ADC brown-out/reset, D15): re-apply it and report this operation.
    ++_reconfig_count;
    const bool saved_flag = _in_read_path;
    _in_read_path = from_read_path;
    const bool reapplied = applyConfig();
    _in_read_path = saved_flag;
    if (from_read_path) {
        return failRead(Error::VERIFY_FAILED,
                        "configuration lost (%02Xh = %02Xh), re-applied %s, reconfig #%lu",
                        bad_addr, bad_val, reapplied ? "OK" : "FAILED",
                        static_cast<unsigned long>(_reconfig_count));
    }
    return fail(Error::VERIFY_FAILED, "configuration lost (%02Xh = %02Xh), re-applied %s", bad_addr,
                bad_val, reapplied ? "OK" : "FAILED");
}

bool ADS868x::checkConfig() {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "checkConfig() before begin()");
    }
    return verifyAndRepair(false);
}

// ---------------------------------------------------------------------------------------------
// Initialization, reset, self-test
// ---------------------------------------------------------------------------------------------

bool ADS868x::begin(bool init_spi) {
    _initialized = false;

    gpio_init(_cs_pin);
    gpio_set_dir(_cs_pin, GPIO_OUT);
    gpio_put(_cs_pin, 1);
    if (_rst_pd_pin >= 0) {
        gpio_init(static_cast<uint>(_rst_pd_pin));
        gpio_set_dir(static_cast<uint>(_rst_pd_pin), GPIO_OUT);
        gpio_put(static_cast<uint>(_rst_pd_pin), 1);
    }
    if (_alarm_pin >= 0) {
        gpio_init(static_cast<uint>(_alarm_pin));
        gpio_set_dir(static_cast<uint>(_alarm_pin), GPIO_IN);
    }

    if (_baud_requested == 0 || _baud_requested > 17000000u) {
        return fail(Error::INVALID_ARG, "baud %lu Hz outside 1..17 MHz (fSCLK, p11)",
                    static_cast<unsigned long>(_baud_requested));
    }
    if (init_spi) {
        _baud_actual = spi_init(_spi, _baud_requested);
    } else {
        _baud_actual = spi_set_baudrate(_spi, _baud_requested);
    }
    gpio_set_function(_sclk_pin, GPIO_FUNC_SPI);
    gpio_set_function(_sdi_pin, GPIO_FUNC_SPI);
    gpio_set_function(_sdo_pin, GPIO_FUNC_SPI);
    prepareBus();
    printf("[ADS868x] SPI%u: requested %lu Hz, actual %lu Hz, %u channels, %s variant\n",
           spi_get_index(_spi), static_cast<unsigned long>(_baud_requested),
           static_cast<unsigned long>(_baud_actual), _num_channels, isAVariant() ? "A" : "non-A");

    // Internal reference turn-on time with 22 µF on REFCAP (p9).
    sleep_ms(T_REF_STARTUP_MS);

    const bool want_tag = _channel_check;
    resetCache();
    if (_rst_pd_pin >= 0) {
        // Hardware reset through PWR_DN (registers reset), D16.
        gpio_put(static_cast<uint>(_rst_pd_pin), 0);
        sleep_us(T_RST_PD_LOW_US);
        gpio_put(static_cast<uint>(_rst_pd_pin), 1);
        sleep_ms(T_PWR_DN_EXIT_MS);
    } else {
        convFrame(CMD_RST, nullptr);  // RST, 8500h (p49)
    }
    _mode = Mode::IDLE;
    _cur_ch = NO_CHANNEL;
    _sdo_tag = false;  // device is back to SDO 000 after reset (Table 12 p57)

    _feature_shadow = want_tag ? SDO_FMT_001 : SDO_FMT_000;
    _initialized = true;  // bus and device ready; cleared again on failure
    if (!applyConfig()) {
        _initialized = false;
        printf("[ADS868x] begin(): configuration write failed - check wiring/power\n");
        return false;
    }
    if (!selfTest()) {
        _initialized = false;
        printf("[ADS868x] begin(): self-test failed - check SDO/SDI/SCLK/CS wiring\n");
        return false;
    }

    // Select CH0 and wait as after PWR_DN (p45): covers the case where the ADC was left in
    // power-down by a previous run and its reference must start again.
    convFrame(CMD_MAN_CH0, nullptr);
    sleep_ms(T_PWR_DN_EXIT_MS);
    _mode = Mode::MAN;
    _cur_ch = 0;
    return ok();
}

bool ADS868x::reset() {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "reset() before begin()");
    }
    if (_mode == Mode::PWR_DN_HW) {
        return fail(Error::INVALID_STATE, "reset() during hardware power-down; call wakeUp()");
    }
    convFrame(CMD_RST, nullptr);  // RST, 8500h (p49)
    resetCache();
    _sdo_tag = false;
    _mode = Mode::IDLE;
    _cur_ch = NO_CHANNEL;
    _feature_shadow = _channel_check ? SDO_FMT_001 : SDO_FMT_000;
    if (!applyConfig()) {
        printf("[ADS868x] reset(): configuration write failed\n");
        return false;
    }
    return ok();
}

bool ADS868x::hardwareReset() {
    if (_rst_pd_pin < 0) {
        return fail(Error::INVALID_ARG, "hardwareReset() needs rst_pd_pin");
    }
    // RST/PD low ≥ 400 ns: PWR_DN with register reset (Table 5 p38); 15 ms to power up (p45).
    gpio_put(static_cast<uint>(_rst_pd_pin), 0);
    sleep_us(T_RST_PD_LOW_US);
    gpio_put(static_cast<uint>(_rst_pd_pin), 1);
    sleep_ms(T_PWR_DN_EXIT_MS);
    _sdo_tag = false;
    _mode = Mode::IDLE;
    _cur_ch = NO_CHANNEL;
    // A register write is required after RST/PD before the next command (p38).
    if (!applyConfig()) {
        printf("[ADS868x] hardwareReset(): configuration write failed\n");
        return false;
    }
    return ok();
}

bool ADS868x::selfTest() {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "selfTest() before begin()");
    }
    if (_mode == Mode::PWR_DN_HW) {
        return fail(Error::INVALID_STATE, "selfTest() during hardware power-down");
    }
    // Range_CH0 accepts 0010 and 0101 on every variant (Table 15 p58; SBAS582C Table 3).
    const uint8_t saved_range = _range[0];
    const uint8_t patterns[2] = {RANGE_PM_2P56V, RANGE_0_TO_10P24V};
    for (uint8_t p : patterns) {
        if (!regWriteVerified(REG_RANGE_CH0, p)) {
            return false;
        }
        const uint8_t v = regReadRaw(REG_RANGE_CH0) & 0x0F;
        if (v != p) {
            regWriteVerified(REG_RANGE_CH0, saved_range);
            return fail(Error::VERIFY_FAILED, "self-test: Range_CH0 wrote %Xh, read %Xh", p, v);
        }
    }
    if (!regWriteVerified(REG_RANGE_CH0, saved_range)) {
        return false;
    }
    _settle_pending[0] = true;

    if (isAVariant()) {
        // CH0 hysteresis (15h) has 8 writable bits (Table 21 p63): alternating patterns.
        const uint8_t saved_hyst = _hyst[0];
        const uint8_t hp[2] = {0xA5, 0x5A};
        for (uint8_t p : hp) {
            if (!regWriteVerified(REG_CH0_HYST, p)) {
                return false;
            }
            const uint8_t v = regReadRaw(REG_CH0_HYST);
            if (v != p) {
                regWriteVerified(REG_CH0_HYST, saved_hyst);
                return fail(Error::VERIFY_FAILED, "self-test: CH0_HYST wrote %02Xh, read %02Xh", p,
                            v);
            }
        }
        if (!regWriteVerified(REG_CH0_HYST, saved_hyst)) {
            return false;
        }
    }
    return ok();
}

// ---------------------------------------------------------------------------------------------
// Ranges and conversion
// ---------------------------------------------------------------------------------------------

float ADS868x::rangeFactor(uint8_t code, bool &bipolar) {
    // Full-scale as a multiple of VREF (Table 15 p58, Table 4 p35).
    bipolar = true;
    switch (code) {
        case RANGE_PM_10P24V:
            return 2.5f;
        case RANGE_PM_5P12V:
            return 1.25f;
        case RANGE_PM_2P56V:
            return 0.625f;
        case RANGE_PM_1P28V:
            return 0.3125f;
        case RANGE_PM_0P64V:
            return 0.15625f;
        default:
            break;
    }
    bipolar = false;
    switch (code) {
        case RANGE_0_TO_10P24V:
            return 2.5f;
        case RANGE_0_TO_5P12V:
            return 1.25f;
        case RANGE_0_TO_2P56V:
            return 0.625f;
        case RANGE_0_TO_1P28V:
            return 0.3125f;
        default:
            return 0.0f;
    }
}

float ADS868x::rawToVoltage(uint16_t raw, InputRange range) const {
    // Straight binary output (p35, Fig. 85): bipolar 8000h = 0 V, LSB = FSR / 65536 (Table 4).
    bool bipolar = true;
    const float k = rangeFactor(range, bipolar);
    if (bipolar) {
        const float lsb = (2.0f * k * _vref) / 65536.0f;
        return (static_cast<int32_t>(raw) - 32768) * lsb;
    }
    const float lsb = (k * _vref) / 65536.0f;
    return static_cast<float>(raw) * lsb;
}

float ADS868x::auxToVoltage(uint16_t raw) const {
    // AUX input range 0..VREF (p9, §8.3.9 p32).
    return static_cast<float>(raw) * _vref / 65536.0f;
}

bool ADS868x::setInputRange(Channel ch, InputRange range) {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "setInputRange() before begin()");
    }
    if (!validChannel(ch)) {
        return fail(Error::INVALID_ARG, "channel %u does not exist on this variant", ch);
    }
    bool bipolar = true;
    if (rangeFactor(range, bipolar) == 0.0f) {
        return fail(Error::INVALID_ARG, "range code %Xh is not defined (Table 15 p58)", range);
    }
    if (!isRangeSupported(range)) {
        return fail(Error::UNSUPPORTED_BY_VARIANT, "range code %Xh exists only on A variants",
                    range);
    }
    if (_mode == Mode::PWR_DN_HW) {
        return fail(Error::INVALID_STATE, "setInputRange() during hardware power-down");
    }
    const uint8_t address = REG_RANGE_CH0 + ch;
    if (!regWriteVerified(address, range)) {
        return false;
    }
    const uint8_t v = regReadRaw(address) & 0x0F;
    if (v != range) {
        return fail(Error::VERIFY_FAILED, "Range_CH%u wrote %Xh, read %Xh", ch, range, v);
    }
    if (isAVariant() && _thr_in_volts[ch]) {
        // Alarm thresholds compare output codes (p36): recompute them for the new range (D21).
        thresholdsFromVolts(ch);
        if (!writeThresholds(ch)) {
            return false;
        }
    }
    return ok();
}

ADS868x::InputRange ADS868x::getInputRange(Channel ch) const {
    return static_cast<InputRange>((ch < 8) ? _range[ch] : 0);
}

bool ADS868x::readInputRange(Channel ch, InputRange &range) {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "readInputRange() before begin()");
    }
    if (!validChannel(ch)) {
        return fail(Error::INVALID_ARG, "channel %u does not exist on this variant", ch);
    }
    range = static_cast<InputRange>(regReadRaw(REG_RANGE_CH0 + ch) & 0x0F);
    return ok();
}

bool ADS868x::ensureConversionAllowed() {
    if (!_initialized) {
        return failRead(Error::NOT_INITIALIZED, "conversion before begin()");
    }
    if (_mode == Mode::STDBY || _mode == Mode::PWR_DN_SW || _mode == Mode::PWR_DN_HW) {
        // Frames in STDBY/PWR_DN return invalid data (p43, p45): call wakeUp() first (D19).
        return failRead(Error::INVALID_STATE, "conversion in STDBY/PWR_DN, call wakeUp()");
    }
    return true;
}

bool ADS868x::checkTag(uint8_t expected, uint8_t tag) {
    if (!_sdo_tag || !_channel_check || expected == AUX_INDEX || tag == expected) {
        return true;
    }
    failRead(Error::CHANNEL_MISMATCH, "expected CH%u, SDO tag %u (Table 14 p57)", expected, tag);
    // A reset ADC is back to SDO 000 and outputs tag 0000: verify and repair the configuration.
    verifyAndRepair(true);
    _last_error = Error::CHANNEL_MISMATCH;
    return false;
}

bool ADS868x::read(Channel ch, uint16_t &raw) {
    if (!ensureConversionAllowed()) {
        return false;
    }
    if (!validChannel(ch)) {
        return failRead(Error::INVALID_ARG, "channel %u does not exist on this variant", ch);
    }
    // The channel converted in frame N is the one selected by the command of frame N-1 (p47).
    // With NO_OP the device keeps converting the same channel (p42), so a repeated read of the
    // same channel needs a single frame.
    if (_mode != Mode::MAN || _cur_ch != ch) {
        convFrame(static_cast<uint16_t>(CMD_MAN_CH0 + (ch << 10)), nullptr);  // MAN_Ch_n
        _mode = Mode::MAN;
        _cur_ch = ch;
    }
    if (_settle_pending[ch]) {
        convFrame(CMD_NO_OP, nullptr);  // discard the 1st conversion after a range change
        _settle_pending[ch] = false;
    }
    uint8_t tag = NO_CHANNEL;
    raw = convFrame(CMD_NO_OP, &tag);
    if (!checkTag(ch, tag)) {
        return false;
    }
    countSaturation(ch, raw);
    return ok();
}

bool ADS868x::readVoltage(Channel ch, float &volts) {
    uint16_t raw = 0;
    if (!read(ch, raw)) {
        return false;
    }
    volts = rawToVoltage(raw, static_cast<InputRange>(_range[ch]));
    return true;
}

uint16_t ADS868x::readChannel(Channel ch) {
    uint16_t raw = 0;
    return read(ch, raw) ? raw : READ_ERROR;
}

bool ADS868x::readAllChannels(uint16_t *data) {
    if (data == nullptr) {
        return failRead(Error::INVALID_ARG, "readAllChannels(nullptr)");
    }
    if (!ensureConversionAllowed()) {
        return false;
    }
    bool settle = false;
    for (uint8_t ch = 0; ch < _num_channels; ++ch) {
        settle = settle || _settle_pending[ch];
    }
    if (settle) {
        for (uint8_t ch = 0; ch < _num_channels; ++ch) {
            if (!read(static_cast<Channel>(ch), data[ch])) {
                return false;
            }
        }
        return ok();
    }
    // Pipeline (p47): MAN_Ch_0, MAN_Ch_1 → data CH0, ..., NO_OP → data CH(n-1).
    convFrame(CMD_MAN_CH0, nullptr);
    for (uint8_t i = 1; i <= _num_channels; ++i) {
        const uint16_t cmd =
                (i < _num_channels) ? static_cast<uint16_t>(CMD_MAN_CH0 + (i << 10)) : CMD_NO_OP;
        uint8_t tag = NO_CHANNEL;
        const uint16_t raw = convFrame(cmd, &tag);
        const uint8_t ch = i - 1;
        _mode = Mode::MAN;
        _cur_ch = (i < _num_channels) ? i : ch;
        if (!checkTag(ch, tag)) {
            return false;
        }
        data[ch] = raw;
        countSaturation(ch, raw);
    }
    return ok();
}

bool ADS868x::readAux(uint16_t &raw) {
    if (!ensureConversionAllowed()) {
        return false;
    }
    if (_mode != Mode::MAN || _cur_ch != AUX_INDEX) {
        convFrame(CMD_MAN_AUX, nullptr);  // MAN_AUX, E000h (Table 6 p50)
        _mode = Mode::MAN;
        _cur_ch = AUX_INDEX;
    }
    // [VERIFICAR] AUX channel tag in SDO 001 is not specified (Table 14 p57): not checked.
    raw = convFrame(CMD_NO_OP, nullptr);
    return ok();
}

// ---------------------------------------------------------------------------------------------
// Auto-scan
// ---------------------------------------------------------------------------------------------

bool ADS868x::startAutoScan(uint8_t channel_mask, bool power_down_unused) {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "startAutoScan() before begin()");
    }
    if (_mode == Mode::STDBY || _mode == Mode::PWR_DN_SW || _mode == Mode::PWR_DN_HW) {
        return fail(Error::INVALID_STATE, "startAutoScan() in STDBY/PWR_DN, call wakeUp()");
    }
    const uint8_t mask = channel_mask & _ch_mask;
    if (mask == 0 || mask != channel_mask) {
        return fail(Error::INVALID_ARG, "channel mask %02Xh invalid for %u channels", channel_mask,
                    _num_channels);
    }
    // Channels in the sequence must be powered up (Table 11 p56), D20.
    const uint8_t pd = power_down_unused ? static_cast<uint8_t>(~mask & _ch_mask)
                                         : static_cast<uint8_t>(_ch_pd & ~mask);
    if (!regWriteVerified(REG_AUTO_SEQ_EN, mask) || !regWriteVerified(REG_CH_PWR_DN, pd)) {
        return false;
    }
    if (!_sdo_tag) {
        // Auto-scan always uses SDO 001 so that each sample carries its channel (D20).
        const uint8_t feature =
                static_cast<uint8_t>((_feature_shadow & ~FEAT_SDO_MASK) | SDO_FMT_001);
        if (!regWriteVerified(REG_FEATURE_SELECT, feature)) {
            return false;
        }
    }
    convFrame(CMD_AUTO_RST, nullptr);  // AUTO_RST, A000h (p46); 1st channel sampled next frame
    _mode = Mode::AUTO;
    _cur_ch = NO_CHANNEL;
    return ok();
}

bool ADS868x::readAutoScan(uint16_t &raw, Channel &ch) {
    if (!_initialized) {
        return failRead(Error::NOT_INITIALIZED, "readAutoScan() before begin()");
    }
    if (_mode != Mode::AUTO) {
        return failRead(Error::INVALID_STATE, "not in auto-scan; call startAutoScan()");
    }
    uint8_t tag = NO_CHANNEL;
    raw = convFrame(CMD_NO_OP, &tag);  // NO_OP keeps AUTO_RST running (p46)
    if (tag >= _num_channels || ((_auto_seq >> tag) & 1u) == 0u) {
        failRead(Error::CHANNEL_MISMATCH, "auto-scan tag %u not in sequence %02Xh", tag, _auto_seq);
        verifyAndRepair(true);  // leaves auto mode
        _last_error = Error::CHANNEL_MISMATCH;
        return false;
    }
    ch = static_cast<Channel>(tag);
    countSaturation(tag, raw);
    return ok();
}

// ---------------------------------------------------------------------------------------------
// Power modes
// ---------------------------------------------------------------------------------------------

bool ADS868x::standby() {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "standby() before begin()");
    }
    if (_mode == Mode::PWR_DN_SW || _mode == Mode::PWR_DN_HW) {
        return fail(Error::INVALID_STATE, "standby() from PWR_DN, call wakeUp() first");
    }
    convFrame(CMD_STDBY, nullptr);  // STDBY, 8200h; entered on CS rising edge (p43)
    _mode = Mode::STDBY;
    _cur_ch = NO_CHANNEL;
    return ok();
}

bool ADS868x::powerDown(bool hardware) {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "powerDown() before begin()");
    }
    if (hardware) {
        if (_rst_pd_pin < 0) {
            return fail(Error::INVALID_ARG, "powerDown(true) needs rst_pd_pin");
        }
        // RST/PD held low ≥ 400 ns: PWR_DN, registers reset on wake-up (Table 5 p38, p45).
        gpio_put(static_cast<uint>(_rst_pd_pin), 0);
        _mode = Mode::PWR_DN_HW;
    } else {
        convFrame(CMD_PWR_DN, nullptr);  // PWR_DN, 8300h; registers kept (p45)
        _mode = Mode::PWR_DN_SW;
    }
    _cur_ch = NO_CHANNEL;
    return ok();
}

bool ADS868x::wakeUp() {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "wakeUp() before begin()");
    }
    switch (_mode) {
        case Mode::PWR_DN_HW: {
            gpio_put(static_cast<uint>(_rst_pd_pin), 1);
            sleep_ms(T_PWR_DN_EXIT_MS);  // p45
            _sdo_tag = false;            // registers are back to defaults
            _mode = Mode::IDLE;
            if (!applyConfig()) {
                printf("[ADS868x] wakeUp(): configuration write failed\n");
                return false;
            }
            return ok();
        }
        case Mode::PWR_DN_SW:
        case Mode::STDBY: {
            // A valid MAN/AUTO command leaves the mode (p44, p45). The device returns to manual
            // mode on CH0; auto-scan must be restarted with startAutoScan().
            const bool from_pd = (_mode == Mode::PWR_DN_SW);
            convFrame(CMD_MAN_CH0, nullptr);
            if (from_pd) {
                sleep_ms(T_PWR_DN_EXIT_MS);  // 15 ms before the 1st accurate conversion (p45)
            } else {
                sleep_us(T_STDBY_EXIT_US);  // CS high ≥ 20 µs after STDBY exit (p44)
            }
            _mode = Mode::MAN;
            _cur_ch = 0;
            return ok();
        }
        default:
            return ok();
    }
}

bool ADS868x::setChannelPowerDown(Channel ch, bool power_down) {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "setChannelPowerDown() before begin()");
    }
    if (!validChannel(ch)) {
        return fail(Error::INVALID_ARG, "channel %u does not exist on this variant", ch);
    }
    if (power_down && _mode == Mode::AUTO && ((_auto_seq >> ch) & 1u) != 0u) {
        return fail(Error::INVALID_ARG, "CH%u is in the auto-scan sequence (Table 11 p56)", ch);
    }
    const uint8_t bit = static_cast<uint8_t>(1u << ch);
    const uint8_t v =
            power_down ? static_cast<uint8_t>(_ch_pd | bit) : static_cast<uint8_t>(_ch_pd & ~bit);
    return regWriteVerified(REG_CH_PWR_DN, v);
}

// ---------------------------------------------------------------------------------------------
// Low level options
// ---------------------------------------------------------------------------------------------

bool ADS868x::setChannelCheck(bool enable) {
    if (!_initialized) {
        _channel_check = enable;  // applied by begin()
        return ok();
    }
    const uint8_t feature = static_cast<uint8_t>((_feature_shadow & ~FEAT_SDO_MASK) |
                                                 (enable ? SDO_FMT_001 : SDO_FMT_000));
    if (!regWriteVerified(REG_FEATURE_SELECT, feature)) {
        return false;
    }
    _channel_check = enable;
    return ok();
}

uint32_t ADS868x::setBaudrate(uint32_t hz) {
    if (hz == 0 || hz > 17000000u) {
        fail(Error::INVALID_ARG, "baud %lu Hz outside 1..17 MHz (fSCLK, p11)",
             static_cast<unsigned long>(hz));
        return _baud_actual;
    }
    _baud_requested = hz;
    _baud_actual = spi_set_baudrate(_spi, hz);
    ok();
    return _baud_actual;
}

// ---------------------------------------------------------------------------------------------
// ALARM (A variants)
// ---------------------------------------------------------------------------------------------

bool ADS868x::enableAlarm(bool enable) {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "enableAlarm() before begin()");
    }
    if (!isAVariant()) {
        return fail(Error::UNSUPPORTED_BY_VARIANT, "ALARM exists only on A variants");
    }
    const uint8_t feature = enable ? static_cast<uint8_t>(_feature_shadow | FEAT_ALARM_EN)
                                   : static_cast<uint8_t>(_feature_shadow & ~FEAT_ALARM_EN);
    return regWriteVerified(REG_FEATURE_SELECT, feature);
}

bool ADS868x::writeThresholds(uint8_t ch) {
    // CHn_HYST, HT MSB/LSB, LT MSB/LSB at 15h + 5n .. 19h + 5n (Tables 21-25 p63-65).
    const uint8_t base = static_cast<uint8_t>(REG_CH0_HYST + 5 * ch);
    return regWriteVerified(base, _hyst[ch]) &&
           regWriteVerified(base + 1, static_cast<uint8_t>(_thr_high[ch] >> 8)) &&
           regWriteVerified(base + 2, static_cast<uint8_t>(_thr_high[ch] & 0xFF)) &&
           regWriteVerified(base + 3, static_cast<uint8_t>(_thr_low[ch] >> 8)) &&
           regWriteVerified(base + 4, static_cast<uint8_t>(_thr_low[ch] & 0xFF));
}

bool ADS868x::setAlarmThresholds(Channel ch, uint16_t low, uint16_t high, uint8_t hysteresis) {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "setAlarmThresholds() before begin()");
    }
    if (!isAVariant()) {
        return fail(Error::UNSUPPORTED_BY_VARIANT, "ALARM exists only on A variants");
    }
    if (!validChannel(ch) || low > high) {
        return fail(Error::INVALID_ARG, "CH%u thresholds low %u high %u", ch, low, high);
    }
    _thr_low[ch] = low;
    _thr_high[ch] = high;
    _hyst[ch] = hysteresis;
    _thr_in_volts[ch] = false;
    return writeThresholds(ch) && ok();
}

void ADS868x::thresholdsFromVolts(uint8_t ch) {
    bool bipolar = true;
    const float k = rangeFactor(_range[ch], bipolar);
    const float lsb = ((bipolar ? 2.0f : 1.0f) * k * _vref) / 65536.0f;
    const float offset = bipolar ? 32768.0f : 0.0f;
    auto to_code = [&](float v) {
        float c = v / lsb + offset + 0.5f;
        if (c < 0.0f) {
            c = 0.0f;
        }
        if (c > 65535.0f) {
            c = 65535.0f;
        }
        return static_cast<uint16_t>(c);
    };
    _thr_low[ch] = to_code(_thr_low_v[ch]);
    _thr_high[ch] = to_code(_thr_high_v[ch]);
    float h = _hyst_v[ch] / lsb + 0.5f;
    if (h < 0.0f) {
        h = 0.0f;
    }
    if (h > 255.0f) {
        h = 255.0f;
    }
    _hyst[ch] = static_cast<uint8_t>(h);
}

bool ADS868x::setAlarmThresholdsVolts(Channel ch, float low_v, float high_v, float hysteresis_v) {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "setAlarmThresholdsVolts() before begin()");
    }
    if (!isAVariant()) {
        return fail(Error::UNSUPPORTED_BY_VARIANT, "ALARM exists only on A variants");
    }
    if (!validChannel(ch) || low_v > high_v || hysteresis_v < 0.0f) {
        return fail(Error::INVALID_ARG, "CH%u thresholds low %.4f V high %.4f V", ch,
                    static_cast<double>(low_v), static_cast<double>(high_v));
    }
    _thr_low_v[ch] = low_v;
    _thr_high_v[ch] = high_v;
    _hyst_v[ch] = hysteresis_v;
    _thr_in_volts[ch] = true;
    thresholdsFromVolts(ch);
    return writeThresholds(ch) && ok();
}

bool ADS868x::readAlarmFlags(AlarmFlags &flags) {
    if (!_initialized) {
        return fail(Error::NOT_INITIALIZED, "readAlarmFlags() before begin()");
    }
    if (!isAVariant()) {
        return fail(Error::UNSUPPORTED_BY_VARIANT, "ALARM exists only on A variants");
    }
    flags = AlarmFlags{0, 0, 0, 0, 0};
    flags.overview_tripped = regReadRaw(REG_ALARM_OVERVIEW) & _ch_mask;  // Table 16 p59
    // 11h-14h: bit (7 - 2c) = CH(b+c) low, bit (6 - 2c) = CH(b+c) high (Tables 17-20 p60-61).
    auto decode = [](uint8_t reg, uint8_t base, uint8_t &low, uint8_t &high) {
        for (uint8_t c = 0; c < 4; ++c) {
            if ((reg >> (7 - 2 * c)) & 1u) {
                low = static_cast<uint8_t>(low | (1u << (base + c)));
            }
            if ((reg >> (6 - 2 * c)) & 1u) {
                high = static_cast<uint8_t>(high | (1u << (base + c)));
            }
        }
    };
    decode(regReadRaw(REG_ALARM_CH0_3_TRIP), 0, flags.tripped_low, flags.tripped_high);
    decode(regReadRaw(REG_ALARM_CH0_3_ACT), 0, flags.active_low, flags.active_high);
    if (_num_channels == 8) {
        decode(regReadRaw(REG_ALARM_CH4_7_TRIP), 4, flags.tripped_low, flags.tripped_high);
        decode(regReadRaw(REG_ALARM_CH4_7_ACT), 4, flags.active_low, flags.active_high);
    }
    return ok();
}

bool ADS868x::setAlarmCallback(void (*cb)(void *), void *ctx) {
    if (!isAVariant()) {
        return fail(Error::UNSUPPORTED_BY_VARIANT, "ALARM exists only on A variants");
    }
    if (_alarm_pin < 0 || _alarm_pin >= static_cast<int>(NUM_BANK0_GPIOS)) {
        return fail(Error::INVALID_ARG, "setAlarmCallback() needs alarm_pin");
    }
    const uint pin = static_cast<uint>(_alarm_pin);
    _alarm_cb = cb;
    _alarm_ctx = ctx;
    if (cb != nullptr) {
        g_alarm_owner[pin] = this;
        gpio_add_raw_irq_handler(pin, &alarm_irq_dispatch);
        gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE, true);  // ALARM is active high (p36)
        irq_set_enabled(IO_IRQ_BANK0, true);
    } else {
        gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE, false);
        g_alarm_owner[pin] = nullptr;
    }
    return ok();
}

void ADS868x::handleAlarmIrq_() {
    if (_alarm_cb != nullptr) {
        _alarm_cb(_alarm_ctx);
    }
}
