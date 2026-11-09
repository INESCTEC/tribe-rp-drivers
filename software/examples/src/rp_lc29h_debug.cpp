/*
 * rp_lc29h_debug: debug firmware for an LC29H GNSS module (see docs/lc29h.md). Not for production.
 *
 * Shows on USB what the module sends and what the driver makes of it, without CAN or a robot:
 *   epoch mode (default): the driver (begin(): it parses every 10 ms itself) and its debug lines:
 *       "E,..." one per closed epoch, "DU/DS/DB/DC/DL,..." every 5 s (LC29HDriver::format*DebugLine*()), and
 *       "S,..." every 1 s: getMostRecentEpoch() and getState() checked against the epochs popped (check_newest())
 *   sniffer mode (LC29H_DEBUG_SNIFFER): no driver; every raw module line with the RP2040 time of its '\n':
 *       "N,<t_us>,<line>", and every 5 s "D,<t_us>,<bytes>,<lines>,<uart_overrun>,<uart_framing_break>,
 *       <ring_overflow>,<ring_max>,<line_too_long>,<rtcm_frames>,<rtcm_bytes>"
 * Both modes take corrections from USB framed as the phone app does ("$=" + 2-byte little-endian length +
 * payload) and pass them to the module, so the PC (scripts/lc29h/gnss_bench.py) can play the phone.
 * Epoch mode also takes "$#<h>" (h = one hex digit 1..f, GNSS_SENTENCE_* bits) to change the epoch sentence set.
 *
 * Board start-up: lc29h_debug_board_start() only starts USB stdio. A board that needs more before USB
 * works (e.g. USB-C power delivery) builds this file from its own repository with a file that defines it.
 */
#include <stdio.h>
#include <string.h>

#include "hardware/uart.h"
#include "pico/stdlib.h"

#ifndef LC29H_DEBUG_SNIFFER
#include "rp_agrolib_lc29h.h"
#endif


#ifndef UART_ID
#define UART_ID uart0
#endif
#ifndef UART_TX_PIN
#define UART_TX_PIN 0
#endif
#ifndef UART_RX_PIN
#define UART_RX_PIN 1
#endif
#ifndef UART_BAUDRATE
#define UART_BAUDRATE 460800
#endif

#define REPORT_PERIOD_MS 5000
#define CHECK_PERIOD_MS 100   // getMostRecentEpoch() / getState() checked this often
#define S_PERIOD_MS 1000      // S line
#define RTCM_MAX 4096  // largest correction frame payload accepted

// ---------------- board start-up (replaceable) ----------------
// Weak: a board's own definition, linked with this file, replaces it
__attribute__((weak)) void lc29h_debug_board_start() {
    stdio_init_all();
    for (int i = 0; i < 50 && !stdio_usb_connected(); i++)
        sleep_ms(100);
}

// ---------------- sniffer mode: own UART interrupt, no driver ----------------
#ifdef LC29H_DEBUG_SNIFFER
#define RING_SIZE 16384  // bytes from the module waiting to go out on USB
#define LINE_MAX 256

static uint8_t ring_[RING_SIZE];
static uint32_t ring_t_[RING_SIZE];  // time_us_32() of each byte (kept only to stamp line ends)
static volatile uint32_t ring_w_ = 0, ring_r_ = 0;
static volatile uint32_t bytes_ = 0, overrun_ = 0, framing_break_ = 0, ring_overflow_ = 0, ring_max_ = 0;
static uint32_t rtcm_frames_ = 0, rtcm_bytes_ = 0;

static void uart_irq() {
    while (uart_is_readable(UART_ID)) {
        uint8_t c = uart_getc(UART_ID);
        bytes_++;
        uint32_t used = ring_w_ - ring_r_;
        if (used >= RING_SIZE) {
            ring_overflow_++;  // USB too slow: byte dropped here (never expected)
            continue;
        }
        ring_[ring_w_ % RING_SIZE] = c;
        ring_t_[ring_w_ % RING_SIZE] = time_us_32();
        ring_w_ = ring_w_ + 1;
        if (used + 1 > ring_max_)
            ring_max_ = used + 1;
    }
    uint32_t rsr = uart_get_hw(UART_ID)->rsr;
    if (rsr) {
        if (rsr & UART_UARTRSR_OE_BITS)
            overrun_++;
        if (rsr & (UART_UARTRSR_FE_BITS | UART_UARTRSR_BE_BITS))
            framing_break_++;
        uart_get_hw(UART_ID)->rsr = 0;
    }
}

static void mode_start() {
    uart_init(UART_ID, UART_BAUDRATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
    int irq = uart_get_index(UART_ID) == 0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(irq, uart_irq);
    irq_set_enabled(irq, true);
    uart_set_irqs_enabled(UART_ID, true, false);
    printf("I,rp_lc29h_debug,sniffer,uart%u %d baud,fifo on\n", uart_get_index(UART_ID), UART_BAUDRATE);
}

static void send_corrections(uint8_t *data, uint16_t len) {
    uart_write_blocking(UART_ID, data, len);
    rtcm_frames_++;
    rtcm_bytes_ += len;
}

static void set_epoch_sentences(uint8_t) {}

static void mode_poll() {
    static char line[LINE_MAX];
    static uint32_t n = 0, lines = 0, too_long = 0;
    static uint32_t last_report = 0;
    while (ring_r_ != ring_w_) {
        uint32_t i = ring_r_ % RING_SIZE;
        uint8_t c = ring_[i];
        uint32_t t = ring_t_[i];
        ring_r_ = ring_r_ + 1;
        if (c == '\r')
            continue;
        if (c == '\n') {
            if (n > 0) {
                line[n] = '\0';
                printf("N,%lu,%s\n", (unsigned long) t, line);
                lines++;
            }
            n = 0;
            continue;
        }
        if (n < LINE_MAX - 1)
            line[n++] = (c >= 0x20 && c < 0x7f) ? (char) c : '?';  // one text line per NMEA line
        else {
            too_long++;
            n = 0;
        }
    }
    uint32_t now = time_us_32();
    if (now - last_report >= REPORT_PERIOD_MS * 1000u) {
        last_report = now;
        printf("D,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n", (unsigned long) now, (unsigned long) bytes_,
               (unsigned long) lines, (unsigned long) overrun_, (unsigned long) framing_break_,
               (unsigned long) ring_overflow_, (unsigned long) ring_max_, (unsigned long) too_long,
               (unsigned long) rtcm_frames_, (unsigned long) rtcm_bytes_);
        ring_max_ = 0;
    }
}

// ---------------- epoch mode: the driver ----------------
#else
static LC29HDriver gnss(UART_ID);

static void mode_start() {
    if (!gnss.begin(UART_RX_PIN, UART_TX_PIN, UART_BAUDRATE)) {
        printf("I,rp_lc29h_debug,begin() failed: UART interrupt already in use, or no DMA channel or timer free\n");
    }
    printf("I,rp_lc29h_debug,epochs,uart%u %d baud,sentences %x\n", uart_get_index(UART_ID), UART_BAUDRATE,
           gnss.epochSentences());
}

static void send_corrections(uint8_t *data, uint16_t len) {
    gnss.writeCorrections(data, len);
}

static void set_epoch_sentences(uint8_t sentences) {
    gnss.setEpochSentences(sentences);
    printf("I,rp_lc29h_debug,sentences %x\n", gnss.epochSentences());
}

// ---- getMostRecentEpoch() / getState() on the board ----
// The newest epoch must be the last one popped, field by field, whether converted now or copied from the driver's
// converted copy (second call). getState() must hold that epoch's valid fields and keep the others as they were.
// Skipped (not counted as checks) when a newer epoch closes during the check.
static GnssEpoch last_popped_;
static bool have_popped_ = false;
static RtkState prev_state_ = {};
static uint32_t checks_ = 0, skipped_ = 0, epoch_mismatch_ = 0, state_mismatch_ = 0;

static bool same_epoch(const GnssEpoch &a, const GnssEpoch &b) {
    return a.utc_ms_ == b.utc_ms_ && a.latitude_ == b.latitude_ && a.longitude_ == b.longitude_ &&
           a.altitude_ == b.altitude_ && a.fix_quality_ == b.fix_quality_ && a.num_satellites_ == b.num_satellites_ &&
           a.heading_ == b.heading_ && a.heading_quality_ == b.heading_quality_ && a.heading_acc_ == b.heading_acc_ &&
           a.baseline_m_ == b.baseline_m_ && a.epe_north_ == b.epe_north_ && a.epe_east_ == b.epe_east_ &&
           a.epe_down_ == b.epe_down_ && a.speed_kph_ == b.speed_kph_ && a.flags_ == b.flags_ &&
           a.sentences_ == b.sentences_ && a.seq_ == b.seq_ && a.close_reason_ == b.close_reason_;
}

static bool state_as_expected(const RtkState &st, const RtkState &prev, const GnssEpoch &e) {
    RtkState x = prev;
    if (e.flags_ & GNSS_VALID_TIME) {
        uint32_t s = e.utc_ms_ / 1000;
        x.utc_ = (s / 3600) * 10000.0 + (s / 60 % 60) * 100.0 + s % 60 + (e.utc_ms_ % 1000) / 1000.0;
    }
    if (e.flags_ & GNSS_VALID_POSITION) {
        x.latitude_ = e.latitude_;
        x.longitude_ = e.longitude_;
    }
    if (e.flags_ & GNSS_VALID_ALTITUDE)
        x.altitude_ = e.altitude_;
    if (e.sentences_ & GNSS_SENTENCE_GGA) {
        x.fix_status_ = e.fix_quality_;
        x.num_satellites_ = e.num_satellites_;
    }
    if (e.flags_ & GNSS_VALID_HEADING)
        x.heading_ = e.heading_;
    if (e.flags_ & GNSS_VALID_SPEED)
        x.speed_kph_ = e.speed_kph_;
    double dutc = st.utc_ - x.utc_;
    return dutc < 1e-6 && dutc > -1e-6 && st.latitude_ == x.latitude_ && st.longitude_ == x.longitude_ &&
           st.altitude_ == x.altitude_ && st.heading_ == x.heading_ && st.fix_status_ == x.fix_status_ &&
           st.num_satellites_ == x.num_satellites_ && st.speed_kph_ == x.speed_kph_;
}

static void check_newest() {
    GnssEpoch a, b, c;
    RtkState st;
    if (!have_popped_ || !gnss.getMostRecentEpoch(&a)) {
        return;
    }
    gnss.getMostRecentEpoch(&b);  // same epoch: the driver's converted copy
    gnss.getState(&st);
    gnss.getMostRecentEpoch(&c);  // still the same epoch after getState()?
    if (a.seq_ != last_popped_.seq_ || b.seq_ != a.seq_ || c.seq_ != a.seq_) {
        skipped_++;  // a newer epoch closed meanwhile (the main loop pops it next)
        prev_state_ = st;
        return;
    }
    checks_++;
    if (!same_epoch(a, last_popped_) || !same_epoch(b, last_popped_))
        epoch_mismatch_++;
    if (!state_as_expected(st, prev_state_, a))
        state_mismatch_++;
    prev_state_ = st;
}

static void mode_poll() {
    static char buf[512];
    static uint32_t last_report = 0, last_check = 0, last_s = 0;

    // Every closed epoch waiting in the driver's queue
    GnssEpoch e;
    while (gnss.popOldestEpoch(&e)) {
        last_popped_ = e;
        have_popped_ = true;
        uint32_t take_ms = to_ms_since_boot(get_absolute_time());
        gnss.formatEpochDebugLine(&e, buf, sizeof(buf));
        // No CAN here: "done" = printed, right after the take
        printf("%s,%lu,0\n", buf, (unsigned long) (take_ms - e.closed_ms_));
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_check >= CHECK_PERIOD_MS) {
        last_check = now;
        check_newest();
    }
    if (now - last_s >= S_PERIOD_MS) {
        // S,t,seq,flags,fix,lat,lon,alt,speed_kph,heading (getState()),checks,skipped,epoch_mismatch,state_mismatch
        last_s = now;
        printf("S,%lu,%u,%x,%u,%.8f,%.8f,%.2f,%.3f,%.2f,%lu,%lu,%lu,%lu\n", (unsigned long) (now % 10000000u),
               last_popped_.seq_, last_popped_.flags_, prev_state_.fix_status_, prev_state_.latitude_,
               prev_state_.longitude_, prev_state_.altitude_, prev_state_.speed_kph_, prev_state_.heading_,
               (unsigned long) checks_, (unsigned long) skipped_, (unsigned long) epoch_mismatch_,
               (unsigned long) state_mismatch_);
    }
    if (now - last_report >= REPORT_PERIOD_MS) {
        last_report = now;
        gnss.formatStatsDebugLines(now, buf, sizeof(buf));
        printf("%s", buf);
    }
}
#endif

// ---------------- USB -> module (correction frames, commands) ----------------
static void poll_usb_input() {
    static uint8_t rtcm[RTCM_MAX];
    static int state = 0;  // 0: wait '$', 1: got '$', 2: length low, 3: length high, 4: payload, 5: '#' digit
    static uint32_t len = 0, pos = 0;
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        switch (state) {
            case 0:
                state = (c == '$') ? 1 : 0;
                break;
            case 1:  // "$!RTK1", "$$..." and anything else from the phone: ignored
                state = (c == '=') ? 2 : (c == '#') ? 5 : 0;
                break;
            case 2:
                len = (uint32_t) c;
                state = 3;
                break;
            case 3:
                len |= (uint32_t) c << 8;
                pos = 0;
                state = (len > 0 && len <= RTCM_MAX) ? 4 : 0;
                break;
            case 4:
                rtcm[pos++] = (uint8_t) c;
                if (pos >= len) {
                    send_corrections(rtcm, (uint16_t) len);
                    state = 0;
                }
                break;
            case 5:
                if (c >= '1' && c <= '9')
                    set_epoch_sentences((uint8_t) (c - '0'));
                else if (c >= 'a' && c <= 'f')
                    set_epoch_sentences((uint8_t) (c - 'a' + 10));
                state = 0;
                break;
        }
    }
}

int main() {
    lc29h_debug_board_start();
    mode_start();
    while (1) {
        poll_usb_input();
        mode_poll();
    }
}
