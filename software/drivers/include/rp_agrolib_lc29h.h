#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

    /// Sets up the UART (8N1, RX/TX FIFOs on), the RX ring and the UART interrupt. From here on the
    /// driver owns this UART's interrupt: the firmware must not call uart_enable_interrupt() on it.
    /// Returns false if the interrupt already has a handler (another driver or the firmware).
    bool begin(int rx_pin, int tx_pin, uint baudrate);

    /// After begin(): queues a correction frame for the module and returns at once; the UART interrupt
    /// sends it. Drops the whole frame (stats().rtcm_dropped) if the TX ring has no room. Nothing else
    /// may write to this UART's TX, or its bytes get mixed into a frame.
    /// Without begin(): writes the frame and waits until it is sent, as before.
    void writeCorrections(uint8_t *data, uint16_t size);

    /// After begin(): parses every complete NMEA sentence that arrived since the previous call, in arrival
    /// order, so rtk_state ends up holding the NEWEST GGA, PQTMTAR and VTG. Call it periodically (e.g. from
    /// a 10 ms timer). Use either this or processStreamEpochs(), not both: each one consumes the ring.
    void processStream(struct RtkState *rtk_state);

    void getNMEA(uint16_t size, struct RtkState *rtk_state, uint8_t *data);
    void getPQMT(uint16_t in_size, char *pqmt, uint8_t *data, uint16_t *out_size);
    void getPair(uint16_t in_size, char *pair, uint8_t *data, uint16_t *out_size);

    /// Counters (32-bit reads are atomic on the RP2040).
    const LC29HStats &stats() const { return stats_; }

private:
    uart_inst_t *uart_id_;
    bool begun_ = false;
    LC29HStats stats_ = {};

    // UART interrupt: the SDK handler takes no argument, so one static trampoline per UART
    static LC29HDriver *instance_[2];  // indexed by uart_get_index(), set by begin()
    static void uart0Irq();
    static void uart1Irq();
    void onIrq();  // interrupt context: RX FIFO -> rx_ring_ (+ errors), tx_ring_ -> TX FIFO
    void fillTxFifo();

    uint8_t rx_ring_[LC29H_RX_RING_SIZE];
    volatile uint16_t rx_w_ = 0;  // written only by onIrq()
    uint16_t rx_r_ = 0;           // written only by the parse call

    uint8_t tx_ring_[LC29H_TX_RING_SIZE];
    volatile uint32_t tx_w_ = 0;  // free-running; written only by writeCorrections()
    volatile uint32_t tx_r_ = 0;  // free-running; written by onIrq() (and writeCorrections() with the IRQ off)

    // Line assembler (survives partial sentences between calls)
    char line_[128];
    uint16_t line_len_ = 0;
    bool assembleLine(char ch);
    void dispatchSentence(const char *line, struct RtkState *rtk_state);

    void parseGGA(const char *data, struct RtkState *rtk_state);
    void parseVTG(const char *data, struct RtkState *rtk_state);
    void parsePQTMTAR(const char *data, struct RtkState *rtk_state);
    bool validateChecksum(const char *message);
    double convertToDegrees(double value);
};
