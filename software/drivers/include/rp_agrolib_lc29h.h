#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pico/critical_section.h"
#include "pico/stdlib.h"

#include "rp_agrolib_uart.h"

// Ring sizes. They change the size of LC29HDriver, so override them on the library target
// (target_compile_definitions(rp_agrolib_lc29h PUBLIC ...)), never only in the firmware
#ifndef LC29H_RX_RING_SIZE
#define LC29H_RX_RING_SIZE 1000  // bytes of module output kept for the parser (~0.3-0.4 s at 10 Hz)
#endif
#ifndef LC29H_TX_RING_SIZE
#define LC29H_TX_RING_SIZE 8192  // bytes of corrections waiting for the UART (~8 s at ~1 KB/s)
#endif

// Longest sentence kept (characters, with the terminating '\0'); longer lines are dropped (LC29HStats::long_lines)
#define LC29H_LINE_MAX 128

// Default for begin()'s parse period: much shorter than the epoch timeout (it is checked at each parse) and
// than what the RX ring holds (~0.3 s), long enough not to add interrupts for nothing
#define LC29H_PARSE_PERIOD_MS 10

// LC29H_DEBUG_TIMING (debug only, off by default): adds timing fields to GnssEpoch and LC29HStats for
// bench debugging. It changes the struct layout, so link the rp_agrolib_lc29h_debug library (which
// defines it for the library and the firmware alike) instead of defining it only in the firmware

// GnssEpoch::flags_: which fields hold a value (a field whose bit is 0 must not be used)
#define GNSS_VALID_POSITION (1 << 0)     // GGA latitude/longitude present and fix quality >= 1
#define GNSS_VALID_HEADING (1 << 1)      // PQTMTAR heading present and heading quality >= 1
#define GNSS_VALID_HEADING_ACC (1 << 2)  // PQTMTAR heading accuracy present
#define GNSS_VALID_EPE (1 << 3)          // PQTMEPE north/east/down present
#define GNSS_VALID_TIME (1 << 4)         // GGA UTC time present
#define GNSS_VALID_CLIPPED (1 << 5)      // set by the encoder: a value did not fit and was clipped

// GnssEpoch::sentences_ (which sentences of the epoch arrived) and setEpochSentences()
#define GNSS_SENTENCE_GGA (1 << 0)
#define GNSS_SENTENCE_PQTMTAR (1 << 1)
#define GNSS_SENTENCE_PQTMEPE (1 << 2)
#define GNSS_SENTENCE_ALL (GNSS_SENTENCE_GGA | GNSS_SENTENCE_PQTMTAR | GNSS_SENTENCE_PQTMEPE)
#define GNSS_SENTENCE_DEFAULT (GNSS_SENTENCE_GGA | GNSS_SENTENCE_PQTMTAR)  // the sentences used before PQTMEPE

// Default for setEpochTimeoutMs(). Measured on the LC29H EA at 10 Hz: PQTMTAR arrives ~50 ms after GGA,
// so this must be above that and below the 100 ms fix period
#define GNSS_EPOCH_TIMEOUT_MS 90

// Closed epochs kept for takeEpoch(); when full, the oldest is dropped (LC29HStats::overwritten)
#define GNSS_EPOCH_QUEUE_MAX 8

/// One GNSS solution: the GGA, PQTMTAR and PQTMEPE sentences the module prints for the same fix
struct GnssEpoch {
    uint32_t utc_ms_;          // UTC time of the fix, ms since 00:00
    double latitude_;          // deg, south negative
    double longitude_;         // deg, west negative
    uint8_t fix_quality_;      // GGA quality: 0 invalid, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float, 6 dead reckoning
    uint8_t num_satellites_;   // satellites used in the position
    double heading_;           // deg, 0-360
    uint8_t heading_quality_;  // PQTMTAR quality, same values as fix_quality_
    double heading_acc_;       // deg
    double baseline_m_;        // distance between the two antennas reported by the module
    double epe_north_;         // m
    double epe_east_;          // m
    double epe_down_;          // m
    uint8_t flags_;            // GNSS_VALID_*
    uint8_t sentences_;        // GNSS_SENTENCE_*
    uint8_t seq_;              // closing order: +1 per epoch closed (wraps), the first closed is 1
    uint8_t close_reason_;     // GNSS_CLOSE_*
#ifdef LC29H_DEBUG_TIMING
    // Debug only (caller's ms clock, resolution = the parse period)
    uint32_t opened_ms_;      // first sentence of the epoch parsed
    uint32_t closed_ms_;
    uint16_t tar_after_ms_;   // PQTMTAR / PQTMEPE parsed this long after the epoch opened (0xFFFF: not received)
    uint16_t epe_after_ms_;
#endif
};

// GnssEpoch::close_reason_
#define GNSS_CLOSE_COMPLETE 1  // every sentence of the set arrived (setEpochSentences())
#define GNSS_CLOSE_NEXT_GGA 2  // the next fix's GGA arrived first
#define GNSS_CLOSE_REPEAT 3    // a second copy of a sentence arrived (e.g. the GGA of this fix was lost)
#define GNSS_CLOSE_TIMEOUT 4   // the epoch timeout passed

/// Counters, cumulative since start (only incremented: cheap enough to keep always)
struct LC29HStats {
    uint32_t lines;               // complete lines assembled
    uint32_t long_lines;          // lines longer than the line buffer, dropped
    uint32_t rx_bytes;            // bytes moved from the UART to the RX ring
    uint32_t rx_dropped;          // bytes dropped: RX ring full (the parser was not called often enough)
    uint32_t uart_overrun;        // bytes lost in the UART: its RX FIFO was full
    uint32_t uart_framing_break;  // framing or break errors
    uint32_t rtcm_bytes;          // correction bytes queued by writeCorrections()
    uint32_t rtcm_dropped;        // correction frames dropped: no room in the TX ring
    uint32_t ok_gga, ok_tar, ok_epe;  // valid checksum, by sentence
    uint32_t ok_other;                // other lines: VTG (checked), and the sentences the driver does not use,
                                      // skipped after their header without checking the checksum
    uint32_t bad_gga, bad_tar, bad_epe, bad_other;  // checksum failed (or no checksum), by sentence prefix
    uint32_t closed[5];                             // epochs closed, by GNSS_CLOSE_* (index 0 unused)
    uint32_t overwritten;  // closed epochs dropped for a newer one before takeEpoch() read them (queue full)
#ifdef LC29H_DEBUG_TIMING
    // Debug only; maxima since the previous formatStatsDebugLines()
    uint32_t irq_max_us;           // longest UART interrupt (RX + TX)
    uint32_t rx_ring_max_pending;  // most unparsed bytes in the RX ring at a parse call
    uint32_t parse_max_us;         // longest parse call
    uint32_t parse_gap_max_us;     // longest time between two parse calls
#endif
};

struct RtkState {
    double utc_;
    double latitude_;
    double longitude_;
    double altitude_;
    double heading_;
    uint8_t fix_status_;
    uint16_t num_satellites_;
    double speed_kph_;
};

class LC29HDriver {
public:
    LC29HDriver(uart_inst_t *uart_id);

    /// Sets up the UART (8N1, RX/TX FIFOs on), the RX ring, the UART interrupt and a timer that parses the
    /// received sentences every parse_period_ms (in the timer interrupt). From here on the driver owns this
    /// UART's interrupt: the firmware must not call uart_enable_interrupt() on it. parse_period_ms = 0:
    /// no timer, the firmware calls process() itself. Returns false if the UART's interrupt already has a
    /// handler (another driver or the firmware) or no timer is free.
    bool begin(int rx_pin, int tx_pin, uint baudrate, uint16_t parse_period_ms = LC29H_PARSE_PERIOD_MS);

    /// After begin(): queues a correction frame for the module and returns at once; the UART interrupt
    /// sends it. Drops the whole frame (stats().rtcm_dropped) if the TX ring has no room. Nothing else
    /// may write to this UART's TX, or its bytes get mixed into a frame.
    /// Without begin(): writes the frame and waits until it is sent, as before.
    void writeCorrections(uint8_t *data, uint16_t size);

    /// Copies the oldest closed epoch not yet taken into *epoch. Returns false if there is none. Call it in a
    /// loop until it returns false: several epochs may be waiting. Safe from anywhere (it protects the queue
    /// from the parse timer, on either core). The parse only stores the sentences as text; they are converted
    /// to numbers here, in the caller's context.
    bool takeEpoch(struct GnssEpoch *epoch);

    /// Copies the newest GGA, PQTMTAR and VTG values into *rtk_state. Each field holds the newest sentence of
    /// its type, so position and heading may belong to different fixes: use epochs when that matters. An empty
    /// field keeps the value of the previous call. The sentences are converted here, in the caller's context:
    /// call it from one place only (not from two cores at once).
    void getState(struct RtkState *rtk_state);

    /// Sentences that make up an epoch (GNSS_SENTENCE_*). GGA is always included (it opens the epoch and
    /// carries the time). An epoch is complete when all of them arrived; sentences outside the set are
    /// ignored (counted in stats()). The module must be configured to send them: a sentence in the set
    /// that never arrives makes every epoch wait the epoch timeout. Default GNSS_SENTENCE_DEFAULT.
    void setEpochSentences(uint8_t sentences);
    uint8_t epochSentences() const { return epoch_sentences_; }

    /// How long an open epoch waits for the rest of its sentences before it closes anyway
    /// (GNSS_CLOSE_TIMEOUT). Must be above the delay between GGA and the last sentence of the epoch and
    /// below the fix period. Default GNSS_EPOCH_TIMEOUT_MS (90 ms, for 10 Hz).
    void setEpochTimeoutMs(uint16_t ms);

    /// Manual mode only (begin() with parse_period_ms = 0): parses the sentences received since the previous
    /// call and closes the epochs that are complete. Call it every ~10 ms; now_ms is the caller's millisecond
    /// clock. Returns how many epochs were closed. With the driver's timer (the default) never call it.
    uint8_t process(uint32_t now_ms);

    /// Deprecated, and only for firmware that does not call begin() and keeps its own RX ring: data is that
    /// ring and size its write index. Parses only the sentences that arrived since the previous call, so
    /// rtk_state holds the newest ones. A size smaller than the previous one means the ring wrapped; the
    /// ring length is unknown here, so the sentence across the wrap is skipped (one update, ~100 ms).
    [[deprecated("use begin() + takeEpoch() / getState()")]] void getNMEA(uint16_t size, struct RtkState *rtk_state,
                                                                           uint8_t *data);
    void getPQMT(uint16_t in_size, char *pqmt, uint8_t *data, uint16_t *out_size);
    void getPair(uint16_t in_size, char *pair, uint8_t *data, uint16_t *out_size);

    /// Counters (32-bit reads are atomic on the RP2040).
    const LC29HStats &stats() const { return stats_; }

#ifdef LC29H_DEBUG_TIMING
    // Debug only: the lines read by scripts/lc29h/gnss_bench_analyze.py. The driver only formats them into
    // buf (at most size - 1 characters, always terminated); where they go (USB, UART, nowhere) is the
    // caller's choice. Times are the caller's ms clock modulo 10^7.

    /// "E,seq,utc_ms,sentences,flags,close_reason,opened_ms,tar_after_ms,epe_after_ms,close_after_open_ms"
    /// (hex sentences/flags, 65535 = not received, no line ending). The caller appends its own
    /// ",take_after_close_ms,done_after_take_ms" (done = -1: not sent) and the '\n'.
    int formatEpochDebugLine(const struct GnssEpoch *epoch, char *buf, int size) const;

    /// Counter lines, each ending in '\n' (counters cumulative, maxima since the previous call, then reset):
    ///   DU,t,rx_bytes,uart_overrun,uart_framing_break,rx_ring_max_pending,irq_max_us
    ///   DS,t,lines,ok_gga,ok_tar,ok_epe,ok_other,long_lines
    ///   DB,t,bad_gga,bad_tar,bad_epe,bad_other
    ///   DC,t,complete,next_gga,repeat,timeout,overwritten
    ///   DL,t,rx_dropped,rtcm_bytes,rtcm_dropped,parse_max_us,parse_gap_max_us
    int formatStatsDebugLines(uint32_t now_ms, char *buf, int size);
#endif

private:
    uart_inst_t *uart_id_;
    bool begun_ = false;
    LC29HStats stats_ = {};

    // Shared between the parse (timer interrupt) and the firmware: the epoch queue, state_ and the settings
    critical_section_t lock_;
    struct repeating_timer parse_timer_;
    static bool parseTimerCb(struct repeating_timer *t);
#ifdef LC29H_DEBUG_TIMING
    uint32_t last_parse_us_ = 0;
#endif

    // UART interrupt: the SDK handler takes no argument, so one static trampoline per UART
    static LC29HDriver *instance_[2];  // indexed by uart_get_index(), set by begin()
    static void uart0Irq();
    static void uart1Irq();
    void onIrq();  // interrupt context: RX FIFO -> rx_ring_ (+ errors), tx_ring_ -> TX FIFO
    void fillTxFifo();
#ifdef LC29H_DEBUG_TIMING
    void notePending(uint16_t w);
#endif

    uint8_t rx_ring_[LC29H_RX_RING_SIZE];
    volatile uint16_t rx_w_ = 0;  // written only by onIrq()
    uint16_t rx_r_ = 0;           // written only by the parse

    uint8_t tx_ring_[LC29H_TX_RING_SIZE];
    volatile uint32_t tx_w_ = 0;  // free-running; written only by writeCorrections()
    volatile uint32_t tx_r_ = 0;  // free-running; written by onIrq() (and writeCorrections() with the IRQ off)

    // Line assembler (survives partial sentences between parse calls). Sentences the driver does not use are
    // skipped after their header (line_skip_)
    char line_[LC29H_LINE_MAX];
    uint16_t line_len_ = 0;
    uint16_t line_done_len_ = 0;  // length of the complete line in line_
    bool line_skip_ = false;
    uint16_t nmea_r_ = 0;  // getNMEA(): read index into the caller's ring
    bool assembleLine(char ch);
    void handleLine(const char *line, uint16_t len, uint32_t now_ms);
    void dispatchSentence(const char *line, struct RtkState *rtk_state);  // getNMEA() only

    // Newest GGA, PQTMTAR and VTG as received (written by the parse under lock_); getState() converts them
    // into state_, which keeps the values of empty fields
    char state_gga_[LC29H_LINE_MAX] = "";
    char state_tar_[LC29H_LINE_MAX] = "";
    char state_vtg_[LC29H_LINE_MAX] = "";
    RtkState state_ = {};
    void storeStateText(char *dst, const char *line, uint16_t len);
    void updateState(struct RtkState &st, char **fields, int n);

    // An epoch as the parse stores it: the sentences as received. takeEpoch() fills epoch_'s values from them
    struct EpochText {
        GnssEpoch epoch_;  // sentences_, seq_, close_reason_ (and the debug timing) are set by the parse
        char gga_[LC29H_LINE_MAX];
        char tar_[LC29H_LINE_MAX];
        char epe_[LC29H_LINE_MAX];
    };

    // Epoch state (written only by the parse, except the queue: lock_)
    EpochText open_epoch_ = {};  // epoch being assembled
    bool epoch_open_ = false;
    uint32_t epoch_opened_ms_ = 0;
    uint16_t epoch_timeout_ms_ = GNSS_EPOCH_TIMEOUT_MS;
    uint8_t epoch_sentences_ = GNSS_SENTENCE_DEFAULT;
    EpochText ready_[GNSS_EPOCH_QUEUE_MAX] = {};  // closed epochs not yet taken, oldest at ready_head_
    uint8_t ready_head_ = 0;
    uint8_t ready_count_ = 0;
    uint8_t seq_ = 0;  // seq_ of the newest closed epoch
    uint8_t epochs_closed_ = 0;
    uint32_t now_ms_ = 0;  // time of the current parse

    void countSentence(const char *line, bool ok);
    void convertEpoch(EpochText *t);
    void closeEpoch(uint8_t reason);
    void openEpoch(uint32_t now_ms);
    void prepareEpoch(uint8_t sentence, uint32_t now_ms);
    void parseGGAFields(GnssEpoch &e, char **fields, int n);
    void parsePQTMTARFields(GnssEpoch &e, char **fields, int n);
    void parsePQTMEPEFields(GnssEpoch &e, char **fields, int n);
    static int splitFields(char *line, char **fields, int max_fields);
    static bool parseUtcMs(const char *field, uint32_t *utc_ms);

    void parseGGA(const char *data, struct RtkState *rtk_state);
    void parseVTG(const char *data, struct RtkState *rtk_state);
    void parsePQTMTAR(const char *data, struct RtkState *rtk_state);
    bool validateChecksum(const char *message);
    double convertToDegrees(double value);
};
