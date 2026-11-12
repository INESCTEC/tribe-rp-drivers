/**
 * @file rp_ads868x.cpp
 * @brief Bench test program for the ADS868x driver (ADS8684/ADS8688/ADS8684A/ADS8688A)
 *
 * USB-CDC menu: read one channel, read all, change range, continuous CSV, statistics over N
 * samples, register self-test, SPI baud sweep, AUX input. After each command the driver error,
 * re-configuration and saturation counters are printed.
 *
 * Pins, SPI port, baud rate, variant and the chip selects of other devices sharing the bus come
 * from CMake cache variables (examples/CMakeLists.txt), never from literals in the logic.
 * Defaults = SlurrySense 3.0 board (spi1, SCK GP14, ADC SDI <- GP11, ADC SDO -> GP8, CS GP16).
 *
 * Bench procedure: docs/ads8684a_test_protocol.md (agiot4spectral repository).
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"

#include "hardware/spi.h"

#include "rp_agrolib_ads868x.h"

// ---- Configuration (overridden by CMake cache, see examples/CMakeLists.txt) ----
#ifndef ADS868X_SPI_PORT
#define ADS868X_SPI_PORT 1
#endif
#ifndef ADS868X_PIN_SCK
#define ADS868X_PIN_SCK 14
#endif
#ifndef ADS868X_PIN_SDI
#define ADS868X_PIN_SDI 11  // RP2040 TX -> ADC SDI
#endif
#ifndef ADS868X_PIN_SDO
#define ADS868X_PIN_SDO 8  // ADC SDO -> RP2040 RX
#endif
#ifndef ADS868X_PIN_CS
#define ADS868X_PIN_CS 16
#endif
#ifndef ADS868X_PIN_RST_PD
#define ADS868X_PIN_RST_PD -1
#endif
#ifndef ADS868X_PIN_ALARM
#define ADS868X_PIN_ALARM -1
#endif
#ifndef ADS868X_BAUD
#define ADS868X_BAUD 1000000
#endif
#ifndef ADS868X_VARIANT
#define ADS868X_VARIANT 2  // 0 ADS8684, 1 ADS8688, 2 ADS8684A, 3 ADS8688A
#endif
#ifndef ADS868X_VREF
#define ADS868X_VREF 4.096f
#endif
#ifndef ADS868X_OTHER_CS
#define ADS868X_OTHER_CS 13, 4, 17  // other chip selects on the same SPI bus, held high
#endif

#define ADS868X_SPI_INST (ADS868X_SPI_PORT == 0 ? spi0 : spi1)

static const int k_other_cs[] = {ADS868X_OTHER_CS};
static const uint32_t k_sweep_baud[] = {1000000,  2000000,  4000000, 8000000,
                                        10000000, 12500000, 15625000};
static const uint32_t k_sweep_reads = 1000;

static const ADS868x::InputRange k_ranges[] = {
        ADS868x::RANGE_PM_10P24V,  ADS868x::RANGE_PM_5P12V,   ADS868x::RANGE_PM_2P56V,
        ADS868x::RANGE_PM_1P28V,   ADS868x::RANGE_PM_0P64V,   ADS868x::RANGE_0_TO_10P24V,
        ADS868x::RANGE_0_TO_5P12V, ADS868x::RANGE_0_TO_2P56V, ADS868x::RANGE_0_TO_1P28V};
static const char *const k_range_names[] = {"+-10.24 V", "+-5.12 V", "+-2.56 V",
                                            "+-1.28 V",  "+-0.64 V", "0-10.24 V",
                                            "0-5.12 V",  "0-2.56 V", "0-1.28 V"};
static const size_t k_num_ranges = sizeof(k_ranges) / sizeof(k_ranges[0]);

static const char *range_name(ADS868x::InputRange r) {
    for (size_t i = 0; i < k_num_ranges; ++i) {
        if (k_ranges[i] == r) {
            return k_range_names[i];
        }
    }
    return "?";
}

// ---- USB-CDC line input ----
static void read_line(char *buf, size_t len) {
    size_t n = 0;
    while (true) {
        const int c = getchar_timeout_us(100000);
        if (c == PICO_ERROR_TIMEOUT) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (n == 0) {
                continue;
            }
            break;
        }
        if ((c == 8 || c == 127) && n > 0) {
            --n;
            printf("\b \b");
            continue;
        }
        if (n + 1 < len && c >= 32 && c < 127) {
            buf[n++] = static_cast<char>(c);
            putchar(c);
        }
    }
    buf[n] = '\0';
    printf("\n");
}

static long ask_long(const char *prompt, long min, long max) {
    char buf[24];
    while (true) {
        printf("%s [%ld..%ld]: ", prompt, min, max);
        read_line(buf, sizeof(buf));
        char *end = nullptr;
        const long v = strtol(buf, &end, 10);
        if (end != buf && *end == '\0' && v >= min && v <= max) {
            return v;
        }
        printf("invalid value\n");
    }
}

static bool key_pressed() {
    return getchar_timeout_us(0) != PICO_ERROR_TIMEOUT;
}

// ---- Commands ----
static void print_counters(const ADS868x &adc) {
    printf("counters: errors=%lu reconfig=%lu saturation=[",
           static_cast<unsigned long>(adc.errorCount()),
           static_cast<unsigned long>(adc.reconfigCount()));
    for (uint8_t ch = 0; ch < adc.numChannels(); ++ch) {
        printf("%s%lu", ch ? " " : "",
               static_cast<unsigned long>(adc.saturationCount(static_cast<ADS868x::Channel>(ch))));
    }
    printf("] last=%s\n", ADS868x::errorToString(adc.lastError()));
}

static ADS868x::Channel ask_channel(const ADS868x &adc) {
    return static_cast<ADS868x::Channel>(ask_long("channel", 0, adc.numChannels() - 1));
}

static void cmd_read_one(ADS868x &adc) {
    const ADS868x::Channel ch = ask_channel(adc);
    uint16_t raw = 0;
    if (adc.read(ch, raw)) {
        printf("CH%u raw=%u (0x%04X) volts=%.6f range=%s\n", ch, raw, raw,
               static_cast<double>(adc.rawToVoltage(raw, adc.getInputRange(ch))),
               range_name(adc.getInputRange(ch)));
    }
}

static void cmd_read_all(ADS868x &adc) {
    uint16_t data[8] = {0};
    if (!adc.readAllChannels(data)) {
        return;
    }
    for (uint8_t ch = 0; ch < adc.numChannels(); ++ch) {
        const ADS868x::InputRange r = adc.getInputRange(static_cast<ADS868x::Channel>(ch));
        printf("CH%u raw=%5u volts=%+.6f range=%s\n", ch, data[ch],
               static_cast<double>(adc.rawToVoltage(data[ch], r)), range_name(r));
    }
}

static void cmd_set_range(ADS868x &adc) {
    const ADS868x::Channel ch = ask_channel(adc);
    for (size_t i = 0; i < k_num_ranges; ++i) {
        printf("  %u) %-10s code %Xh%s\n", static_cast<unsigned>(i), k_range_names[i], k_ranges[i],
               adc.isRangeSupported(k_ranges[i]) ? "" : "  (not on this variant)");
    }
    const long idx = ask_long("range", 0, static_cast<long>(k_num_ranges) - 1);
    if (adc.setInputRange(ch, k_ranges[idx])) {
        ADS868x::InputRange back = ADS868x::RANGE_PM_10P24V;
        adc.readInputRange(ch, back);
        printf("CH%u range = %s (readback code %Xh)\n", ch, range_name(back), back);
    }
}

static void cmd_continuous(ADS868x &adc) {
    const long sel = ask_long("channel (-1 = all)", -1, adc.numChannels() - 1);
    const long period_us = ask_long("period in us (0 = as fast as possible)", 0, 10000000);
    printf("Press any key to stop.\n");
    printf("timestamp_us,canal,raw,volts\n");
    while (!key_pressed()) {
        const uint8_t first = (sel < 0) ? 0 : static_cast<uint8_t>(sel);
        const uint8_t last = (sel < 0) ? adc.numChannels() - 1 : static_cast<uint8_t>(sel);
        for (uint8_t ch = first; ch <= last; ++ch) {
            uint16_t raw = 0;
            const uint64_t t = time_us_64();
            const ADS868x::Channel c = static_cast<ADS868x::Channel>(ch);
            if (adc.read(c, raw)) {
                printf("%llu,%u,%u,%.6f\n", static_cast<unsigned long long>(t), ch, raw,
                       static_cast<double>(adc.rawToVoltage(raw, adc.getInputRange(c))));
            } else {
                printf("%llu,%u,ERR,%s\n", static_cast<unsigned long long>(t), ch,
                       ADS868x::errorToString(adc.lastError()));
            }
        }
        if (period_us > 0) {
            sleep_us(static_cast<uint64_t>(period_us));
        }
    }
}

static void cmd_stats(ADS868x &adc) {
    const ADS868x::Channel ch = ask_channel(adc);
    const long n = ask_long("number of samples", 2, 1000000);
    const ADS868x::InputRange r = adc.getInputRange(ch);
    // Welford running mean/variance (no buffer needed for large N).
    double mean = 0.0;
    double m2 = 0.0;
    uint16_t vmin = 0xFFFF;
    uint16_t vmax = 0;
    long ok_count = 0;
    long failures = 0;
    const uint64_t t0 = time_us_64();
    for (long i = 0; i < n; ++i) {
        uint16_t raw = 0;
        if (!adc.read(ch, raw)) {
            ++failures;
            continue;
        }
        ++ok_count;
        const double delta = raw - mean;
        mean += delta / ok_count;
        m2 += delta * (raw - mean);
        vmin = raw < vmin ? raw : vmin;
        vmax = raw > vmax ? raw : vmax;
    }
    const uint64_t dt = time_us_64() - t0;
    if (ok_count < 2) {
        printf("not enough valid samples (%ld failures)\n", failures);
        return;
    }
    const double std_code = sqrt(m2 / (ok_count - 1));
    const double lsb_v = static_cast<double>(adc.rawToVoltage(1, r) - adc.rawToVoltage(0, r));
    const double mean_v =
            static_cast<double>(adc.rawToVoltage(0, r)) + mean * lsb_v;  // linear in the code
    printf("CH%u range=%s N=%ld failures=%ld\n", ch, range_name(r), ok_count, failures);
    printf("  mean = %.3f code = %+.6f V\n", mean, mean_v);
    printf("  std  = %.3f LSB rms = %.6f V\n", std_code, std_code * lsb_v);
    printf("  min  = %u (%+.6f V)  max = %u (%+.6f V)  p-p = %u LSB\n", vmin,
           static_cast<double>(adc.rawToVoltage(vmin, r)), vmax,
           static_cast<double>(adc.rawToVoltage(vmax, r)), static_cast<unsigned>(vmax - vmin));
    printf("  elapsed = %llu us -> effective rate = %.1f samples/s (SPI %lu Hz)\n",
           static_cast<unsigned long long>(dt), 1e6 * static_cast<double>(n) / dt,
           static_cast<unsigned long>(adc.baudrate()));
}

static void dump_registers(ADS868x &adc) {
    uint8_t v = 0;
    const uint8_t regs[] = {ADS868x::REG_AUTO_SEQ_EN, ADS868x::REG_CH_PWR_DN,
                            ADS868x::REG_FEATURE_SELECT};
    for (uint8_t a : regs) {
        if (adc.readRegister(a, v)) {
            printf("  reg %02Xh = %02Xh\n", a, v);
        }
    }
    for (uint8_t ch = 0; ch < adc.numChannels(); ++ch) {
        if (adc.readRegister(ADS868x::REG_RANGE_CH0 + ch, v)) {
            printf("  reg %02Xh = %02Xh (Range_CH%u)\n", ADS868x::REG_RANGE_CH0 + ch, v, ch);
        }
    }
    if (adc.readRegister(ADS868x::REG_CMD_READBACK, v)) {
        printf("  reg 3Fh = %02Xh (command readback)\n", v);
    }
}

static void cmd_self_test(ADS868x &adc) {
    const bool st = adc.selfTest();
    printf("selfTest: %s\n", st ? "PASS" : "FAIL");
    const bool cc = adc.checkConfig();
    printf("checkConfig: %s\n", cc ? "PASS" : "FAIL (configuration re-applied)");
    dump_registers(adc);
}

static void cmd_baud_sweep(ADS868x &adc) {
    const uint32_t original = adc.baudrate();
    printf("baud_req,baud_real,selftest,reads_ok,reads_fail,rate_sps\n");
    for (uint32_t baud : k_sweep_baud) {
        const uint32_t real = adc.setBaudrate(baud);
        adc.clearErrors();
        const bool st = adc.selfTest();
        uint32_t ok_count = 0;
        uint32_t fail_count = 0;
        const uint64_t t0 = time_us_64();
        for (uint32_t i = 0; i < k_sweep_reads; ++i) {
            uint16_t raw = 0;
            if (adc.read(ADS868x::CH0, raw)) {
                ++ok_count;
            } else {
                ++fail_count;
            }
        }
        const uint64_t dt = time_us_64() - t0;
        printf("%lu,%lu,%s,%lu,%lu,%.1f\n", static_cast<unsigned long>(baud),
               static_cast<unsigned long>(real), st ? "PASS" : "FAIL",
               static_cast<unsigned long>(ok_count), static_cast<unsigned long>(fail_count),
               1e6 * k_sweep_reads / static_cast<double>(dt));
    }
    const uint32_t back = adc.setBaudrate(original);
    printf("baud restored to %lu Hz\n", static_cast<unsigned long>(back));
}

static void cmd_aux(ADS868x &adc) {
    const long n = ask_long("number of samples", 1, 100);
    for (long i = 0; i < n; ++i) {
        uint16_t raw = 0;
        if (adc.readAux(raw)) {
            printf("AUX raw=%u volts=%.6f\n", raw, static_cast<double>(adc.auxToVoltage(raw)));
        }
    }
}

static void print_menu(const ADS868x &adc) {
    printf("\n=== ADS868x bench test (%u channels, SPI %lu Hz) ===\n", adc.numChannels(),
           static_cast<unsigned long>(adc.baudrate()));
    printf(" 1) read one channel\n");
    printf(" 2) read all channels\n");
    printf(" 3) change range of a channel\n");
    printf(" 4) continuous CSV (timestamp_us,canal,raw,volts)\n");
    printf(" 5) statistics over N samples (+ effective sample rate)\n");
    printf(" 6) register self-test + register dump\n");
    printf(" 7) SPI baud sweep\n");
    printf(" 8) AUX input\n");
    printf(" 9) reset counters\n");
    printf("> ");
}

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(500);
    printf("\nADS868x test: SPI%d SCK=%d SDI=%d SDO=%d CS=%d RST/PD=%d ALARM=%d variant=%d\n",
           ADS868X_SPI_PORT, ADS868X_PIN_SCK, ADS868X_PIN_SDI, ADS868X_PIN_SDO, ADS868X_PIN_CS,
           ADS868X_PIN_RST_PD, ADS868X_PIN_ALARM, ADS868X_VARIANT);

    // Hold the other devices of the shared bus deselected (D17).
    for (int cs : k_other_cs) {
        if (cs >= 0) {
            gpio_init(static_cast<uint>(cs));
            gpio_set_dir(static_cast<uint>(cs), GPIO_OUT);
            gpio_put(static_cast<uint>(cs), 1);
            printf("other CS GP%d held high\n", cs);
        }
    }

    static ADS868x adc(ADS868X_SPI_INST, ADS868X_PIN_CS, ADS868X_PIN_SCK, ADS868X_PIN_SDI,
                       ADS868X_PIN_SDO, ADS868X_BAUD,
                       static_cast<ADS868x::Variant>(ADS868X_VARIANT), ADS868X_PIN_RST_PD,
                       ADS868X_PIN_ALARM, ADS868X_VREF);

    while (!adc.begin()) {
        printf("begin() FAILED (%s) - retrying in 2 s\n", ADS868x::errorToString(adc.lastError()));
        sleep_ms(2000);
    }
    printf("begin() OK\n");

    char buf[8];
    while (true) {
        print_menu(adc);
        read_line(buf, sizeof(buf));
        switch (buf[0]) {
            case '1':
                cmd_read_one(adc);
                break;
            case '2':
                cmd_read_all(adc);
                break;
            case '3':
                cmd_set_range(adc);
                break;
            case '4':
                cmd_continuous(adc);
                break;
            case '5':
                cmd_stats(adc);
                break;
            case '6':
                cmd_self_test(adc);
                break;
            case '7':
                cmd_baud_sweep(adc);
                break;
            case '8':
                cmd_aux(adc);
                break;
            case '9':
                adc.clearErrors();
                break;
            default:
                printf("unknown option\n");
                break;
        }
        print_counters(adc);
    }
}
