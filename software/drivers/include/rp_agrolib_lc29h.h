#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "hardware/dma.h"
#include "pico/critical_section.h"
#include "pico/stdlib.h"

#include "rp_agrolib_uart.h"

// Ring sizes. They change the size of LC29HDriver, so override them on the library target
// (target_compile_definitions(rp_agrolib_lc29h PUBLIC ...)), never only in the firmware
#ifndef LC29H_RX_RING_SIZE
#define LC29H_RX_RING_SIZE 1024  // bytes of module output kept for the parser (~0.3-0.4 s at 10 Hz)
#endif
// The DMA writes the RX ring in its ring mode, which wraps on a power of two (at most 2^15)
static_assert((LC29H_RX_RING_SIZE & (LC29H_RX_RING_SIZE - 1)) == 0 && LC29H_RX_RING_SIZE <= 32768,
              "LC29H_RX_RING_SIZE must be a power of two, at most 32768");
#ifndef LC29H_TX_RING_SIZE
#define LC29H_TX_RING_SIZE 8192  // bytes of corrections waiting for the UART (~8 s at ~1 KB/s)
#endif

// Longest sentence kept (characters, with the terminating '\0'); longer lines are dropped (LC29HStats::long_lines)
#define LC29H_LINE_MAX 128

// RX DMA transfer count. The RP2040 DMA stops when the count reaches 0 (it has no endless mode), so the parse
// starts it again whenever fewer than LC29H_DMA_RESTART_BELOW transfers are left (~5 days at the module's ~5 KB/s)
#ifndef LC29H_DMA_COUNT
#define LC29H_DMA_COUNT 0xFFFFFFFFu
#endif
#ifndef LC29H_DMA_RESTART_BELOW
#define LC29H_DMA_RESTART_BELOW 0x80000000u
#endif

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
#define GNSS_VALID_ALTITUDE (1 << 6)     // GGA altitude present and fix quality >= 1
#define GNSS_VALID_SPEED (1 << 7)        // VTG speed present and its mode is not N (not valid)

// GnssEpoch::sentences_ (which sentences of the epoch arrived) and setEpochSentences()
#define GNSS_SENTENCE_GGA (1 << 0)
#define GNSS_SENTENCE_PQTMTAR (1 << 1)
#define GNSS_SENTENCE_PQTMEPE (1 << 2)
#define GNSS_SENTENCE_VTG (1 << 3)
#define GNSS_SENTENCE_ALL (GNSS_SENTENCE_GGA | GNSS_SENTENCE_PQTMTAR | GNSS_SENTENCE_PQTMEPE | GNSS_SENTENCE_VTG)
// Position + heading. VTG (speed) and PQTMEPE are opt-in: VTG comes right after GGA, so adding it does not make
// an epoch wait longer
#define GNSS_SENTENCE_DEFAULT (GNSS_SENTENCE_GGA | GNSS_SENTENCE_PQTMTAR)

// Default for setEpochTimeoutMs(). Measured on the LC29H EA at 10 Hz: PQTMTAR arrives ~50 ms after GGA,
// so this must be above that and below the 100 ms fix period
#define GNSS_EPOCH_TIMEOUT_MS 90

// Closed epochs kept for popOldestEpoch(); when full, the oldest is dropped (LC29HStats::overwritten)
#define GNSS_EPOCH_QUEUE_MAX 8

/// One GNSS solution: the GGA, VTG, PQTMTAR and PQTMEPE sentences the module prints for the same fix
struct GnssEpoch {
    uint32_t utc_ms_;          // UTC time of the fix, ms since 00:00
    double latitude_;          // deg, south negative
    double longitude_;         // deg, west negative
    double altitude_;          // m above mean sea level (GGA)
    uint8_t fix_quality_;      // GGA quality: 0 invalid, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float, 6 dead reckoning
    uint8_t num_satellites_;   // satellites used in the position
    double heading_;           // deg, 0-360
    uint8_t heading_quality_;  // PQTMTAR quality, same values as fix_quality_
    double heading_acc_;       // deg
    double baseline_m_;        // distance between the two antennas reported by the module
    double epe_north_;         // m
    double epe_east_;          // m
    double epe_down_;          // m
    double speed_kph_;         // speed over ground, km/h (VTG)
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
    uint32_t rx_bytes;            // bytes moved by the DMA from the UART to the RX ring
    uint32_t rx_dropped;          // bytes dropped: the DMA went round the RX ring past bytes not parsed yet
                                  // (the parser was not called often enough)
    uint32_t uart_overrun;        // parse periods in which the UART lost bytes (its RX FIFO was full)
    uint32_t uart_framing_break;  // parse periods with framing or break errors
    uint32_t dma_restarts;        // RX DMA started again before its count ran out (every ~5 days)
    uint32_t rtcm_bytes;          // correction bytes queued by writeCorrections()
    uint32_t rtcm_dropped;        // correction frames dropped: no room in the TX ring
    uint32_t ok_gga, ok_tar, ok_epe;  // valid checksum, by sentence
    uint32_t ok_other;                // other lines: VTG (checked), and the sentences the driver does not use,
                                      // skipped after their header without checking the checksum
    uint32_t bad_gga, bad_tar, bad_epe, bad_other;  // checksum failed (or no checksum), by sentence prefix
    uint32_t closed[5];                             // epochs closed, by GNSS_CLOSE_* (index 0 unused)
    uint32_t overwritten;  // closed epochs dropped for a newer one before popOldestEpoch() read them (queue full)
#ifdef LC29H_DEBUG_TIMING
    // Debug only; maxima since the previous formatStatsDebugLines()
    uint32_t irq_max_us;           // longest UART interrupt (TX: corrections)
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

    /// Sets up the UART (8N1, RX/TX FIFOs on), one DMA channel that moves every received byte into the RX ring
    /// (no CPU, no interrupt), the UART interrupt (sends the corrections) and a timer that parses the received
    /// sentences every parse_period_ms (in the timer interrupt). From here on the driver owns this UART's
    /// interrupt: the firmware must not call uart_enable_interrupt() on it. parse_period_ms = 0: no timer, the
    /// firmware calls process() itself. Returns false if the UART's interrupt already has a handler (another
    /// driver or the firmware), no DMA channel is free or no timer is free.
    bool begin(int rx_pin, int tx_pin, uint baudrate, uint16_t parse_period_ms = LC29H_PARSE_PERIOD_MS);

    /// After begin(): queues a correction frame for the module and returns at once; the UART interrupt
    /// sends it. Drops the whole frame (stats().rtcm_dropped) if the TX ring has no room. Nothing else
    /// may write to this UART's TX, or its bytes get mixed into a frame.
    /// Without begin(): writes the frame and waits until it is sent, as before.
    void writeCorrections(uint8_t *data, uint16_t size);

    // Reading the epochs. All three are safe from anywhere (the parse timer and either core): they copy under
    // the driver's lock and convert the text to numbers outside it, in the caller's context.

    /// Every fix, in order: removes the oldest closed epoch from the queue and copies it into *epoch. Returns
    /// false if the queue is empty. Call it in a loop until it returns false: several epochs may be waiting.
    /// The queue keeps the newest GNSS_EPOCH_QUEUE_MAX; older ones are dropped (stats().overwritten).
    bool popOldestEpoch(struct GnssEpoch *epoch);

    /// The newest fix, whenever needed: copies the newest closed epoch into *epoch without removing anything
    /// from the queue (popOldestEpoch() still returns it in its turn). Returns false until the first epoch
    /// closes. Each epoch is converted once; calls that find the same epoch copy the converted result. A new
    /// epoch is told by its seq_.
    bool getMostRecentEpoch(struct GnssEpoch *epoch);

    /// The newest fix as RtkState (the struct of the old getNMEA()), from getMostRecentEpoch(): every field
    /// comes from the same fix. A field that is not valid in that epoch (its GNSS_VALID_* bit is 0, e.g. the
    /// position without a fix) keeps the value it had, as before; all are 0 until the first epoch.
    void getState(struct RtkState *rtk_state);

    [[deprecated("renamed popOldestEpoch()")]] bool takeEpoch(struct GnssEpoch *epoch) {
        return popOldestEpoch(epoch);
    }

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
    [[deprecated("use begin() + popOldestEpoch() / getMostRecentEpoch() / getState()")]]
    void getNMEA(uint16_t size, struct RtkState *rtk_state, uint8_t *data);
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
    ///   DU,t,rx_bytes,uart_overrun,uart_framing_break,rx_ring_max_pending,irq_max_us (TX)
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

    // Shared between the parse (timer interrupt) and the firmware: the epoch queue, the newest epoch and its
    // converted copy, state_ and the settings
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
    void onIrq();  // interrupt context: tx_ring_ -> TX FIFO
    void fillTxFifo();

    // RX: the DMA channel writes rx_ring_ (aligned to its size for the DMA ring mode); the parse reads it
    int rx_dma_ = -1;
    uint32_t rx_dma_last_count_ = 0;  // DMA transfer count at the previous parse
    uint16_t rxWriteIndex() const;
    void restartRxDma(uint16_t w);
    void countUartErrors();
#ifdef LC29H_DEBUG_TIMING
    void notePending(uint16_t w);
#endif

    alignas(LC29H_RX_RING_SIZE) uint8_t rx_ring_[LC29H_RX_RING_SIZE] = {};
    uint16_t rx_r_ = 0;  // written only by the parse

    uint8_t tx_ring_[LC29H_TX_RING_SIZE] = {};
    volatile uint32_t tx_w_ = 0;  // free-running; written only by writeCorrections()
    volatile uint32_t tx_r_ = 0;  // free-running; written by onIrq() (and writeCorrections() with the IRQ off)

    // Line assembler (survives partial sentences between parse calls). Sentences the driver does not use are
    // skipped after their header (line_skip_)
    char line_[LC29H_LINE_MAX] = {};
    uint16_t line_len_ = 0;
    uint16_t line_done_len_ = 0;  // length of the complete line in line_
    bool line_skip_ = false;
    uint16_t nmea_r_ = 0;  // getNMEA(): read index into the caller's ring
    bool assembleLine(char ch);
    void handleLine(const char *line, uint16_t len, uint32_t now_ms);
    void dispatchSentence(const char *line, struct RtkState *rtk_state);  // getNMEA() only

    // An epoch as the parse stores it: the sentences as received. convertEpoch() fills epoch_'s values from them
    struct EpochText {
        GnssEpoch epoch_;  // sentences_, seq_, close_reason_ (and the debug timing) are set by the parse
        char gga_[LC29H_LINE_MAX];
        char tar_[LC29H_LINE_MAX];
        char epe_[LC29H_LINE_MAX];
        char vtg_[LC29H_LINE_MAX];
    };

    // Epoch state (written only by the parse, except the queue: lock_)
    EpochText open_epoch_ = {};  // epoch being assembled
    bool epoch_open_ = false;
    uint32_t epoch_opened_ms_ = 0;
    uint16_t epoch_timeout_ms_ = GNSS_EPOCH_TIMEOUT_MS;
    uint8_t epoch_sentences_ = GNSS_SENTENCE_DEFAULT;
    EpochText ready_[GNSS_EPOCH_QUEUE_MAX] = {};  // closed epochs not yet popped, oldest at ready_head_
    uint8_t ready_head_ = 0;
    uint8_t ready_count_ = 0;
    uint8_t seq_ = 0;  // seq_ of the newest closed epoch

    // The newest closed epoch is ready_[latest_idx_]: the next close always writes the slot after it, so it stays
    // there (popped or not) until a newer one replaces it. closes_ counts every close (32 bits: unlike the 8-bit
    // seq_ it does not wrap in practice), so it tells whether latest_ (converted) is still the newest
    uint8_t latest_idx_ = 0;
    uint32_t closes_ = 0;
    GnssEpoch latest_ = {};
    uint32_t latest_closes_ = 0;  // closes_ of latest_; 0: none converted yet
    RtkState state_ = {};          // getState(): fields not valid in the newest epoch keep their value
    uint32_t state_closes_ = 0;    // closes_ of the epoch state_ was last updated from
    bool newestEpoch(struct GnssEpoch *epoch, uint32_t *closes);
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
    void parseVTGFields(GnssEpoch &e, char **fields, int n);
    static int splitFields(char *line, char **fields, int max_fields);
    static bool parseUtcMs(const char *field, uint32_t *utc_ms);

    void parseGGA(const char *data, struct RtkState *rtk_state);
    void parseVTG(const char *data, struct RtkState *rtk_state);
    void parsePQTMTAR(const char *data, struct RtkState *rtk_state);
    bool validateChecksum(const char *message);
    double convertToDegrees(double value);
};
