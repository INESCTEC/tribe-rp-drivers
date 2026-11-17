#include "../include/rp_agrolib_lc29h.h"

#include "hardware/irq.h"
#include "hardware/sync.h"

LC29HDriver *LC29HDriver::instance_[2] = {nullptr, nullptr};

LC29HDriver::LC29HDriver(uart_inst_t *uart_id) : uart_id_(uart_id) {}

bool LC29HDriver::begin(int rx_pin, int tx_pin, uint baudrate, uint16_t parse_period_ms) {
    uint index = uart_get_index(uart_id_);
    int irq = index == 0 ? UART0_IRQ : UART1_IRQ;
    if (instance_[index] != nullptr || irq_has_handler(irq)) {
        return false;
    }
    rx_dma_ = dma_claim_unused_channel(false);
    if (rx_dma_ < 0) {
        return false;
    }
    critical_section_init(&lock_);
    uart_setup(uart_id_, rx_pin, tx_pin, baudrate, 8, 1, UART_PARITY_NONE, true);
    instance_[index] = this;
    irq_set_exclusive_handler(irq, index == 0 ? uart0Irq : uart1Irq);

    // The UART interrupt only sends the corrections: TX interrupt (enabled only while corrections wait) when the
    // TX FIFO drains to <= 1/2 (16 bytes). No RX interrupt: the DMA below moves the received bytes
    uart_set_irqs_enabled(uart_id_, false, false);
    hw_write_masked(&uart_get_hw(uart_id_)->ifls, 2 << UART_UARTIFLS_TXIFLSEL_LSB, UART_UARTIFLS_TXIFLSEL_BITS);
    irq_set_enabled(irq, true);

    // RX by DMA: every byte goes from the UART FIFO into rx_ring_ as soon as it arrives, without the CPU, so no
    // byte waits for an interrupt (however long other interrupts take). The write address wraps on the ring.
    // IRQ_QUIET and no DMA interrupt: see restartRxDma() (RP2040-E13)
    dma_channel_config cfg = dma_channel_get_default_config(rx_dma_);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_ring(&cfg, true, __builtin_ctz(LC29H_RX_RING_SIZE));
    channel_config_set_dreq(&cfg, uart_get_dreq_num(uart_id_, false));
    channel_config_set_irq_quiet(&cfg, true);
    uart_get_hw(uart_id_)->icr = UART_UARTRIS_OERIS_BITS | UART_UARTRIS_BERIS_BITS | UART_UARTRIS_FERIS_BITS;
    dma_channel_configure(rx_dma_, &cfg, rx_ring_, &uart_get_hw(uart_id_)->dr, LC29H_DMA_COUNT, true);
    rx_dma_last_count_ = LC29H_DMA_COUNT;
    begun_ = true;

    // Negative period: from the start of one call to the start of the next, whatever the call lasts
    if (parse_period_ms > 0 && !add_repeating_timer_ms(-(int32_t) parse_period_ms, parseTimerCb, this, &parse_timer_)) {
        return false;
    }
    return true;
}

bool LC29HDriver::parseTimerCb(struct repeating_timer *t) {
    ((LC29HDriver *) t->user_data)->process(to_ms_since_boot(get_absolute_time()));
    return true;
}

void LC29HDriver::uart0Irq() {
    instance_[0]->onIrq();
}

void LC29HDriver::uart1Irq() {
    instance_[1]->onIrq();
}

void LC29HDriver::onIrq() {
#ifdef LC29H_DEBUG_TIMING
    uint32_t t_in = time_us_32();
#endif
    uart_hw_t *hw = uart_get_hw(uart_id_);

    // TX only (reception is by DMA): refill from the corrections ring; stop the TX interrupt when there is nothing left, since it
    // would keep firing on an empty FIFO
    fillTxFifo();
    if (tx_r_ == tx_w_) {
        hw_clear_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
#ifdef LC29H_DEBUG_TIMING
    uint32_t dt = time_us_32() - t_in;
    if (dt > stats_.irq_max_us) {
        stats_.irq_max_us = dt;
    }
#endif
}

void LC29HDriver::fillTxFifo() {
    uart_hw_t *hw = uart_get_hw(uart_id_);
    while (tx_r_ != tx_w_ && !(hw->fr & UART_UARTFR_TXFF_BITS)) {
        hw->dr = tx_ring_[tx_r_ % LC29H_TX_RING_SIZE];
        tx_r_ = tx_r_ + 1;
    }
}

void LC29HDriver::writeCorrections(uint8_t *data, uint16_t size) {
    if (!begun_) {
        uart_write_blocking(uart_id_, data, size);
        return;
    }

    if (LC29H_TX_RING_SIZE - (tx_w_ - tx_r_) < size) {
        stats_.rtcm_dropped++;  // never expected: ~1 KB/s in, ~46 KB/s out at 460800 baud
        return;
    }
    uint32_t w = tx_w_;
    for (uint16_t i = 0; i < size; i++) {
        tx_ring_[(w + i) % LC29H_TX_RING_SIZE] = data[i];
    }
    tx_w_ = w + size;
    stats_.rtcm_bytes += size;

    // Fill the TX FIFO now and let the interrupt send the rest. The UART interrupt is off meanwhile, so
    // the handler cannot empty the ring and disable TX between the fill and the enable. The PL011 TX
    // interrupt fires when the FIFO level falls through the threshold, so the FIFO must be filled first
    int irq = uart_get_index(uart_id_) == 0 ? UART0_IRQ : UART1_IRQ;
    irq_set_enabled(irq, false);
    fillTxFifo();
    if (tx_r_ != tx_w_) {
        hw_set_bits(&uart_get_hw(uart_id_)->imsc, UART_UARTIMSC_TXIM_BITS);
    }
    irq_set_enabled(irq, true);
}

uint8_t LC29HDriver::process(uint32_t now_ms) {
#ifdef LC29H_DEBUG_TIMING
    uint32_t t_in = time_us_32();
    if (last_parse_us_ && t_in - last_parse_us_ > stats_.parse_gap_max_us) {
        stats_.parse_gap_max_us = t_in - last_parse_us_;
    }
    last_parse_us_ = t_in;
#endif
    if (!begun_) {
        return 0;
    }
    epochs_closed_ = 0;
    now_ms_ = now_ms;

    // Where the DMA has written up to, and how many bytes it wrote since the previous parse
    uint32_t count = dma_channel_hw_addr(rx_dma_)->transfer_count;
    uint16_t w = rxWriteIndex();
    uint32_t arrived = rx_dma_last_count_ - count;
    rx_dma_last_count_ = count;
    stats_.rx_bytes += arrived;
    if (arrived >= LC29H_RX_RING_SIZE) {
        // The DMA went round the ring past bytes not parsed yet (parse calls too rare): drop them and resync
        stats_.rx_dropped += arrived;
        rx_r_ = w;
        line_len_ = 0;
        line_skip_ = false;
    }
    countUartErrors();
#ifdef LC29H_DEBUG_TIMING
    notePending(w);
#endif
    while (rx_r_ != w) {
        char ch = (char) rx_ring_[rx_r_];
        rx_r_ = (rx_r_ + 1) % LC29H_RX_RING_SIZE;
        if (assembleLine(ch)) {
            handleLine(line_, line_done_len_, now_ms);
        }
    }

    // The DMA stops when its transfer count reaches 0 (the RP2040 DMA has no endless mode), and the received bytes
    // would then be lost. So it is started again from here, long before: ~every 5 days at what the module sends
    if (count < LC29H_DMA_RESTART_BELOW) {
        restartRxDma(w);
    }

    // An epoch that never completes (a sentence missing or lost) is closed as it is
    if (epoch_open_ && (now_ms - epoch_opened_ms_) >= epoch_timeout_ms_) {
        closeEpoch(GNSS_CLOSE_TIMEOUT);
    }

#ifdef LC29H_DEBUG_TIMING
    uint32_t dt = time_us_32() - t_in;
    if (dt > stats_.parse_max_us) {
        stats_.parse_max_us = dt;
    }
#endif
    return epochs_closed_;
}

uint16_t LC29HDriver::rxWriteIndex() const {
    return (uint16_t) ((dma_channel_hw_addr(rx_dma_)->write_addr - (uint32_t) rx_ring_) % LC29H_RX_RING_SIZE);
}

void LC29HDriver::restartRxDma(uint16_t w) {
    // Stop the channel and start it again with a full count, from where it stopped in the ring. Bytes that arrive
    // meanwhile (a few us) wait in the UART RX FIFO (32 bytes, ~600 us at 460800 baud), so none is lost.
    // RP2040-E13: after an abort, the ABORT bit clears before a transfer in flight (byte read, not yet written) is
    // finished. dma_channel_abort() waits on CTRL.BUSY instead (pico-sdk), so the write address read below is final.
    // E13 may also set the channel's completion flag: harmless, the channel is IRQ_QUIET and no DMA interrupt is used
    dma_channel_abort(rx_dma_);
    uint16_t next = rxWriteIndex();
    stats_.rx_bytes += (uint16_t) (next - w) % LC29H_RX_RING_SIZE;  // written after this parse read the count
    dma_channel_set_write_addr(rx_dma_, &rx_ring_[next], false);
    dma_channel_set_trans_count(rx_dma_, LC29H_DMA_COUNT, true);
    rx_dma_last_count_ = LC29H_DMA_COUNT;
    stats_.dma_restarts++;
}

void LC29HDriver::countUartErrors() {
    // With the DMA, a received byte's error flags are not read with it: the UART's raw status keeps them until
    // cleared, so they are counted once per parse period in which they happened. The overrun should never happen
    // (the DMA empties the FIFO as bytes arrive)
    uart_hw_t *hw = uart_get_hw(uart_id_);
    uint32_t ris = hw->ris;
    if (ris & UART_UARTRIS_OERIS_BITS) {
        stats_.uart_overrun++;
    }
    if (ris & (UART_UARTRIS_BERIS_BITS | UART_UARTRIS_FERIS_BITS)) {
        stats_.uart_framing_break++;
    }
    hw->icr = ris & (UART_UARTRIS_OERIS_BITS | UART_UARTRIS_BERIS_BITS | UART_UARTRIS_FERIS_BITS);
}

void LC29HDriver::handleLine(const char *line, uint16_t len, uint32_t now_ms) {
    // Interrupt context, kept short: no conversion to numbers here (atof and the like are slow on the RP2040,
    // which has no floating-point unit). The sentences are only checked, copied as text and grouped into epochs;
    // the methods that read the epochs convert them in the caller's context
    bool ok = line[0] == '$' && validateChecksum(line);
    countSentence(line, ok);
    if (!ok) {
        return;
    }

    if (strncmp(line, "$GNGGA", 6) == 0) {
        // GGA starts the epoch of a new fix: whatever is still open belongs to the previous one
        if (epoch_open_) {
            closeEpoch(GNSS_CLOSE_NEXT_GGA);
        }
        prepareEpoch(GNSS_SENTENCE_GGA, now_ms);
        memcpy(open_epoch_.gga_, line, len + 1);
    } else if (strncmp(line, "$PQTMTAR", 8) == 0) {
        // Sentences outside the set are not part of epochs: they never open, join or close one
        if (!(epoch_sentences_ & GNSS_SENTENCE_PQTMTAR)) {
            return;
        }
        // The LC29H EA stamps PQTMTAR ~49 ms after the GGA of the same fix, so its time is not
        // compared with the epoch's: it belongs to the epoch that is open, like PQTMEPE
        prepareEpoch(GNSS_SENTENCE_PQTMTAR, now_ms);
#ifdef LC29H_DEBUG_TIMING
        open_epoch_.epoch_.tar_after_ms_ = (uint16_t) (now_ms - epoch_opened_ms_);
#endif
        memcpy(open_epoch_.tar_, line, len + 1);
    } else if (strncmp(line, "$PQTMEPE", 8) == 0) {
        if (!(epoch_sentences_ & GNSS_SENTENCE_PQTMEPE)) {
            return;
        }
        // PQTMEPE has no time field: it belongs to the epoch that is open
        prepareEpoch(GNSS_SENTENCE_PQTMEPE, now_ms);
#ifdef LC29H_DEBUG_TIMING
        open_epoch_.epoch_.epe_after_ms_ = (uint16_t) (now_ms - epoch_opened_ms_);
#endif
        memcpy(open_epoch_.epe_, line, len + 1);
    } else if (strncmp(line, "$GNVTG", 6) == 0) {
        if (!(epoch_sentences_ & GNSS_SENTENCE_VTG)) {
            return;
        }
        // VTG has no time field: the module sends it right after the GGA of the same fix (before PQTMEPE and
        // PQTMTAR), so it belongs to the epoch that is open
        prepareEpoch(GNSS_SENTENCE_VTG, now_ms);
        memcpy(open_epoch_.vtg_, line, len + 1);
    } else {
        return;
    }

    if ((open_epoch_.epoch_.sentences_ & epoch_sentences_) == epoch_sentences_) {
        closeEpoch(GNSS_CLOSE_COMPLETE);
    }
}

bool LC29HDriver::newestEpoch(GnssEpoch *epoch, uint32_t *closes) {
    // Under the lock: whether the converted copy is still the newest (then copy it, ~100 B) or the newest epoch's
    // text (~600 B). The queue (head, count) is not touched
    EpochText t;
    critical_section_enter_blocking(&lock_);
    uint32_t newest = closes_;
    bool converted = newest != 0 && latest_closes_ == newest;
    if (converted) {
        *epoch = latest_;
    } else if (newest != 0) {
        t = ready_[latest_idx_];
    }
    critical_section_exit(&lock_);
    *closes = newest;
    if (newest == 0 || converted) {
        return newest != 0;
    }

    convertEpoch(&t);
    critical_section_enter_blocking(&lock_);
    // Kept only if still the newest: another caller may have stored a newer one meanwhile
    if (closes_ == newest) {
        latest_ = t.epoch_;
        latest_closes_ = newest;
    }
    critical_section_exit(&lock_);
    *epoch = t.epoch_;
    return true;
}

bool LC29HDriver::getMostRecentEpoch(GnssEpoch *epoch) {
    uint32_t closes;
    return newestEpoch(epoch, &closes);
}

void LC29HDriver::getState(RtkState *rtk_state) {
    GnssEpoch e = {};
    uint32_t closes;
    bool got = newestEpoch(&e, &closes);
    // hhmmss.sss as the old getNMEA() gave it (the GGA time field as a number)
    uint32_t s = e.utc_ms_ / 1000;
    double utc = (s / 3600) * 10000.0 + (s / 60 % 60) * 100.0 + s % 60 + (e.utc_ms_ % 1000) / 1000.0;

    critical_section_enter_blocking(&lock_);
    // Only from a newer epoch than the last one used: two callers must not put an older fix back
    if (got && closes != state_closes_) {
        state_closes_ = closes;
        if (e.flags_ & GNSS_VALID_TIME)
            state_.utc_ = utc;
        if (e.flags_ & GNSS_VALID_POSITION) {
            state_.latitude_ = e.latitude_;
            state_.longitude_ = e.longitude_;
        }
        if (e.flags_ & GNSS_VALID_ALTITUDE)
            state_.altitude_ = e.altitude_;
        if (e.sentences_ & GNSS_SENTENCE_GGA) {
            state_.fix_status_ = e.fix_quality_;
            state_.num_satellites_ = e.num_satellites_;
        }
        if (e.flags_ & GNSS_VALID_HEADING)
            state_.heading_ = e.heading_;
        if (e.flags_ & GNSS_VALID_SPEED)
            state_.speed_kph_ = e.speed_kph_;
    }
    *rtk_state = state_;
    critical_section_exit(&lock_);
}

void LC29HDriver::setEpochSentences(uint8_t sentences) {
    if (begun_)
        critical_section_enter_blocking(&lock_);
    epoch_sentences_ = sentences | GNSS_SENTENCE_GGA;
    if (begun_)
        critical_section_exit(&lock_);
}

void LC29HDriver::setEpochTimeoutMs(uint16_t ms) {
    if (begun_)
        critical_section_enter_blocking(&lock_);
    epoch_timeout_ms_ = ms;
    if (begun_)
        critical_section_exit(&lock_);
}

bool LC29HDriver::assembleLine(char ch) {
    // Returns true when line_ holds a complete line to handle (without the line ending). Sentences the driver
    // does not use (most of what the module sends) are skipped after their 6-character header, so the parse
    // spends almost nothing on them
    if (ch == '\n' || ch == '\r') {
        bool complete = line_len_ > 0 && !line_skip_;
        if (line_len_ > 0 || line_skip_) {
            stats_.lines++;
        }
        if (complete) {
            line_[line_len_] = '\0';
            line_done_len_ = line_len_;
        }
        line_len_ = 0;
        line_skip_ = false;
        return complete;
    }

    if (ch == '$') {
        // Start of a new sentence: resync (recovers from lost bytes/overrun)
        line_len_ = 0;
        line_skip_ = false;
    }
    if (line_skip_) {
        return false;
    }

    if (line_len_ < sizeof(line_) - 1) {
        line_[line_len_++] = ch;
        if (line_len_ == 6 && strncmp(line_, "$GNGGA", 6) != 0 && strncmp(line_, "$GNVTG", 6) != 0 &&
            strncmp(line_, "$PQTMT", 6) != 0 && strncmp(line_, "$PQTME", 6) != 0) {
            line_[0] == '$' ? stats_.ok_other++ : stats_.bad_other++;
            line_skip_ = true;
            line_len_ = 0;
        }
    } else {
        line_len_ = 0;  // oversized garbage without newline: drop and resync
        stats_.long_lines++;
    }
    return false;
}

void LC29HDriver::dispatchSentence(const char *line, RtkState *rtk_state) {
    if (line[0] != '$' || !validateChecksum(line)) {
        return;
    }

    if (strncmp(line, "$GNGGA", 6) == 0) {
        parseGGA(line, rtk_state);
    } else if (strncmp(line, "$PQTMTAR", 8) == 0) {
        parsePQTMTAR(line, rtk_state);
    } else if (strncmp(line, "$GNVTG", 6) == 0) {
        parseVTG(line, rtk_state);
    }
}

bool LC29HDriver::popOldestEpoch(struct GnssEpoch *epoch) {
    // The parse (timer interrupt, possibly on the other core) adds to the queue while this removes from it:
    // the copy and the head/count update must be one step, or an epoch can be overwritten mid-copy or the count
    // lose an update. The critical section blocks interrupts on this core and the other core's access. Only
    // bytes are copied under it (a few us); the conversion to numbers is done after, outside it
    EpochText t;
    critical_section_enter_blocking(&lock_);
    bool got = ready_count_ > 0;
    if (got) {
        t = ready_[ready_head_];
        ready_head_ = (ready_head_ + 1) % GNSS_EPOCH_QUEUE_MAX;
        ready_count_--;
    }
    critical_section_exit(&lock_);
    if (got) {
        convertEpoch(&t);
        *epoch = t.epoch_;
    }
    return got;
}

void LC29HDriver::convertEpoch(EpochText *t) {
    GnssEpoch &e = t->epoch_;
    char *fields[24];
    if (e.sentences_ & GNSS_SENTENCE_GGA) {
        int n = splitFields(t->gga_, fields, 24);
        if (n > 1 && parseUtcMs(fields[1], &e.utc_ms_)) {
            e.flags_ |= GNSS_VALID_TIME;
        }
        parseGGAFields(e, fields, n);
    }
    if (e.sentences_ & GNSS_SENTENCE_PQTMTAR) {
        parsePQTMTARFields(e, fields, splitFields(t->tar_, fields, 24));
    }
    if (e.sentences_ & GNSS_SENTENCE_PQTMEPE) {
        parsePQTMEPEFields(e, fields, splitFields(t->epe_, fields, 24));
    }
    if (e.sentences_ & GNSS_SENTENCE_VTG) {
        parseVTGFields(e, fields, splitFields(t->vtg_, fields, 24));
    }
}

void LC29HDriver::countSentence(const char *line, bool ok) {
    // Classified by prefix, so a sentence whose checksum failed is still attributed to its type
    if (strncmp(line, "$GNGGA", 6) == 0)
        ok ? stats_.ok_gga++ : stats_.bad_gga++;
    else if (strncmp(line, "$PQTMTAR", 8) == 0)
        ok ? stats_.ok_tar++ : stats_.bad_tar++;
    else if (strncmp(line, "$PQTMEPE", 8) == 0)
        ok ? stats_.ok_epe++ : stats_.bad_epe++;
    else
        ok ? stats_.ok_other++ : stats_.bad_other++;
}

void LC29HDriver::prepareEpoch(uint8_t sentence, uint32_t now_ms) {
    // A second copy of a sentence means the open epoch is finished (e.g. its GGA was lost)
    if (epoch_open_ && (open_epoch_.epoch_.sentences_ & sentence) != 0) {
        closeEpoch(GNSS_CLOSE_REPEAT);
    }

    if (!epoch_open_) {
        openEpoch(now_ms);
    }
    open_epoch_.epoch_.sentences_ |= sentence;
}

void LC29HDriver::openEpoch(uint32_t now_ms) {
    open_epoch_.epoch_ = {};
#ifdef LC29H_DEBUG_TIMING
    open_epoch_.epoch_.opened_ms_ = now_ms;
    open_epoch_.epoch_.tar_after_ms_ = open_epoch_.epoch_.epe_after_ms_ = 0xFFFF;
#endif
    epoch_open_ = true;
    epoch_opened_ms_ = now_ms;
}

void LC29HDriver::closeEpoch(uint8_t reason) {
    critical_section_enter_blocking(&lock_);
    if (ready_count_ >= GNSS_EPOCH_QUEUE_MAX) {
        // Queue full: the oldest waiting epoch was never taken
        stats_.overwritten++;
        ready_head_ = (ready_head_ + 1) % GNSS_EPOCH_QUEUE_MAX;
        ready_count_--;
    }
    open_epoch_.epoch_.close_reason_ = reason;
#ifdef LC29H_DEBUG_TIMING
    open_epoch_.epoch_.closed_ms_ = now_ms_;
#endif
    open_epoch_.epoch_.seq_ = ++seq_;
    stats_.closed[reason]++;
    latest_idx_ = (ready_head_ + ready_count_) % GNSS_EPOCH_QUEUE_MAX;
    ready_[latest_idx_] = open_epoch_;
    ready_count_++;
    closes_++;
    critical_section_exit(&lock_);
    epoch_open_ = false;
    epochs_closed_++;
}

void LC29HDriver::parseGGAFields(GnssEpoch &e, char **fields, int n) {
    // $GNGGA,1 UTC,2 Lat,3 N/S,4 Lon,5 E/W,6 Quality,7 NumSatUsed,8 HDOP,9 Alt,...
    if (n > 6 && fields[6][0] != '\0') {
        e.fix_quality_ = (uint8_t) atoi(fields[6]);
    }
    if (n > 7 && fields[7][0] != '\0') {
        int sats = atoi(fields[7]);
        e.num_satellites_ = (uint8_t) (sats > 255 ? 255 : sats);
    }

    bool has_position = n > 5 && fields[2][0] != '\0' && fields[3][0] != '\0' && fields[4][0] != '\0' &&
                        fields[5][0] != '\0';
    if (has_position) {
        e.latitude_ = convertToDegrees(atof(fields[2]));
        if (fields[3][0] == 'S') {
            e.latitude_ = -e.latitude_;
        }
        e.longitude_ = convertToDegrees(atof(fields[4]));
        if (fields[5][0] == 'W') {
            e.longitude_ = -e.longitude_;
        }
        if (e.fix_quality_ >= 1) {
            e.flags_ |= GNSS_VALID_POSITION;
        }
    }
    if (n > 9 && fields[9][0] != '\0') {
        e.altitude_ = atof(fields[9]);
        if (e.fix_quality_ >= 1) {
            e.flags_ |= GNSS_VALID_ALTITUDE;
        }
    }
}

void LC29HDriver::parsePQTMTARFields(GnssEpoch &e, char **fields, int n) {
    // $PQTMTAR,1 MsgVer,2 Time,3 Quality,4 Res,5 Length,6 Pitch,7 Res,8 Heading,9 Acc_Pitch,10 Res,
    // 11 Acc_Heading,12 UsedSV
    if (n > 3 && fields[3][0] != '\0') {
        e.heading_quality_ = (uint8_t) atoi(fields[3]);
    }
    if (n > 5 && fields[5][0] != '\0') {
        e.baseline_m_ = atof(fields[5]);
    }
    if (n > 8 && fields[8][0] != '\0' && e.heading_quality_ >= 1) {
        e.heading_ = atof(fields[8]);
        e.flags_ |= GNSS_VALID_HEADING;
    }
    if (n > 11 && fields[11][0] != '\0') {
        e.heading_acc_ = atof(fields[11]);
        e.flags_ |= GNSS_VALID_HEADING_ACC;
    }
}

void LC29HDriver::parsePQTMEPEFields(GnssEpoch &e, char **fields, int n) {
    // $PQTMEPE,1 MsgVer,2 EPE_North,3 EPE_East,4 EPE_Down,5 EPE_2D,6 EPE_3D
    if (n > 4 && fields[2][0] != '\0' && fields[3][0] != '\0' && fields[4][0] != '\0') {
        e.epe_north_ = atof(fields[2]);
        e.epe_east_ = atof(fields[3]);
        e.epe_down_ = atof(fields[4]);
        e.flags_ |= GNSS_VALID_EPE;
    }
}

void LC29HDriver::parseVTGFields(GnssEpoch &e, char **fields, int n) {
    // $GNVTG,1 COG_T,2 T,3 COG_M,4 M,5 SOG_kn,6 N,7 SOG_kmh,8 K,9 Mode (N: not valid)
    if (n > 7 && fields[7][0] != '\0' && !(n > 9 && fields[9][0] == 'N')) {
        e.speed_kph_ = atof(fields[7]);
        e.flags_ |= GNSS_VALID_SPEED;
    }
}

int LC29HDriver::splitFields(char *line, char **fields, int max_fields) {
    // In place: ',' and '*' become '\0'. Unlike strtok, empty fields are kept, so field numbers match the
    // manual
    int n = 0;
    fields[n++] = line;
    for (char *p = line; *p != '\0' && n < max_fields; p++) {
        if (*p == '*') {
            *p = '\0';
            break;
        }
        if (*p == ',') {
            *p = '\0';
            fields[n++] = p + 1;
        }
    }
    return n;
}

bool LC29HDriver::parseUtcMs(const char *field, uint32_t *utc_ms) {
    // hhmmss.sss
    for (int i = 0; i < 6; i++) {
        if (field[i] < '0' || field[i] > '9') {
            return false;
        }
    }
    uint32_t h = (field[0] - '0') * 10 + (field[1] - '0');
    uint32_t m = (field[2] - '0') * 10 + (field[3] - '0');
    uint32_t s = (field[4] - '0') * 10 + (field[5] - '0');
    if (h > 23 || m > 59 || s > 60) {
        return false;
    }

    uint32_t ms = 0;
    if (field[6] == '.') {
        uint32_t scale = 100;
        for (int i = 7; scale > 0 && field[i] >= '0' && field[i] <= '9'; i++) {
            ms += (field[i] - '0') * scale;
            scale /= 10;
        }
    }

    *utc_ms = ((h * 60 + m) * 60 + s) * 1000 + ms;
    return true;
}

void LC29HDriver::getNMEA(uint16_t size, RtkState *rtk_state, uint8_t *data) {
    // Before: re-scanned the buffer from the start on every call and took the FIRST sentence of each type,
    // i.e. the oldest since the last wrap. Now: only the bytes that arrived since the previous call
    if (size < nmea_r_) {
        nmea_r_ = 0;  // the caller's ring wrapped
        line_len_ = 0;
    }
    while (nmea_r_ < size) {
        if (assembleLine((char) data[nmea_r_++])) {
            dispatchSentence(line_, rtk_state);
        }
    }
}

void LC29HDriver::parseGGA(const char *data, struct RtkState *rtk_state) {
    // Split NMEA string into tokens using "," as delimiter
    char *token;
    char *str = strdup(data);  // To not modify the original string
    token = strtok(str, ",");

    // Iterate through tokens
    int token_index = 0;
    while (token != NULL) {
        // Parse relevant tokens for GGA sentence
        switch (token_index) {
            case 1:  // UTC of position fix
                rtk_state->utc_ = atof(token);
                break;
            case 2:  // Latitude
                rtk_state->latitude_ = convertToDegrees(atof(token));
                break;
            case 3:  // North or South
                if (token[0] == 'S')
                    rtk_state->latitude_ = -rtk_state->latitude_;
                break;
            case 4:  // Longitude
                rtk_state->longitude_ = convertToDegrees(atof(token));
                break;
            case 5:  // East or West
                if (token[0] == 'W')
                    rtk_state->longitude_ = -rtk_state->longitude_;
                break;
            case 6:  // Fix Status
                rtk_state->fix_status_ = atoi(token);
                break;
            case 7:  // Number of Satellites
                rtk_state->num_satellites_ = atoi(token);
                break;
            case 9:  // Altitude
                rtk_state->altitude_ = atof(token);
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        token_index++;
    }
    free(str);  // Free memory allocated by strdup
}

void LC29HDriver::parsePQTMTAR(const char *data, struct RtkState *rtk_state) {
    // Split NMEA string into tokens using "," as delimiter
    char *token;
    char *str = strdup(data);  // To not modify the original string
    token = strtok(str, ",");

    // Iterate through tokens
    int token_index = 0;
    while (token != NULL) {
        // Parse relevant tokens for GGA sentence
        switch (token_index) {
            case 6:  // Heading
                rtk_state->heading_ = atof(token);
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        token_index++;
    }
    free(str);  // Free memory allocated by strdup
}

void LC29HDriver::parseVTG(const char *data, struct RtkState *rtk_state) {
    // Split NMEA string into tokens using "," as delimiter
    char *token;
    char *str = strdup(data);  // To not modify the original string
    token = strtok(str, ",");

    // Iterate through tokens
    int token_index = 0;
    while (token != NULL) {
        // Parse relevant tokens for VTG sentence
        switch (token_index) {
            case 3:  // In case it enters here and it's equal to 'M', means that empty spot was
                     // found so we have to add 1 to index
                if (token[0] == 'M')
                    token_index++;
                break;
            case 7:  // Speed over ground (km/h)
                rtk_state->speed_kph_ = atof(token);
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        token_index++;
    }
    free(str);  // Free memory allocated by strdup
}

double LC29HDriver::convertToDegrees(double value) {
    // Extract degrees and minutes
    int degrees = (int) value / 100;           // Extract integer part as degrees
    double minutes = value - (degrees * 100);  // Extract fractional part as minutes

    // Convert minutes to degrees
    double minutes_in_degrees = minutes / 60.0;

    // Calculate total degrees
    double total_degrees = degrees + minutes_in_degrees;

    return total_degrees;
}

void LC29HDriver::getPQMT(uint16_t in_size, char *pqmt, uint8_t *data, uint16_t *out_size) {
    // Convert uint8_t array to string
    char pqmt_string[in_size + 1];
    sprintf(pqmt_string, "%s", (char *) data);

    char *ptr = strstr(pqmt_string, "$PQMT");
    if (ptr != NULL) {
        // Find the position of the new line from the found index
        char *new_line_ptr = strchr(ptr, '\n');
        if (new_line_ptr != NULL) {
            // Calculate the size of the section to be copied
            *out_size = new_line_ptr - ptr;

            // Copy the section to the new array
            memcpy(pqmt, ptr, *out_size);
            pqmt[*out_size] = '\0';  // Add the null terminator
        }
    }
}

void LC29HDriver::getPair(uint16_t in_size, char *pair, uint8_t *data, uint16_t *out_size) {
    // Convert uint8_t array to string
    char pqmt_string[in_size + 1];
    sprintf(pqmt_string, "%s", (char *) data);

    char *ptr = strstr(pqmt_string, "$PAIR");
    if (ptr != NULL) {
        // Find the position of the new line from the found index
        char *new_line_ptr = strchr(ptr, '\n');
        if (new_line_ptr != NULL) {
            // Calculate the size of the section to be copied
            *out_size = new_line_ptr - ptr;

            // Copy the section to the new array
            memcpy(pair, ptr, *out_size);
            pair[*out_size] = '\0';  // Add the null terminator
        }
    }
}

bool LC29HDriver::validateChecksum(const char *message) {
    int calculated_checksum = 0;
    for (int i = 1; message[i] != '*' && message[i] != '\0'; i++) {
        calculated_checksum ^= message[i];
    }

    char *checksum_ptr = strchr(message, '*');
    if (checksum_ptr) {
        int received_checksum = strtol(checksum_ptr + 1, NULL, 16);
        return calculated_checksum == received_checksum;
    }
    return false;
}
#ifdef LC29H_DEBUG_TIMING
void LC29HDriver::notePending(uint16_t w) {
    uint32_t pending = (w + LC29H_RX_RING_SIZE - rx_r_) % LC29H_RX_RING_SIZE;
    if (pending > stats_.rx_ring_max_pending) {
        stats_.rx_ring_max_pending = pending;
    }
}

int LC29HDriver::formatEpochDebugLine(const GnssEpoch *e, char *buf, int size) const {
    int n = snprintf(buf, size, "E,%u,%lu,%x,%x,%u,%lu,%u,%u,%lu", e->seq_, (unsigned long) e->utc_ms_,
                     e->sentences_, e->flags_, e->close_reason_, (unsigned long) (e->opened_ms_ % 10000000u),
                     e->tar_after_ms_, e->epe_after_ms_, (unsigned long) (e->closed_ms_ - e->opened_ms_));
    return n < size ? n : size - 1;
}

// uint32_t is unsigned long on the RP2040 (no-op there) but unsigned int on a PC: the cast keeps %lu right on both
static inline unsigned long ul(uint32_t x) { return (unsigned long) x; }

int LC29HDriver::formatStatsDebugLines(uint32_t now_ms, char *buf, int size) {
    // Snapshot and reset the maxima together (they are written in interrupts)
    uint32_t irq = save_and_disable_interrupts();
    uint32_t irq_max = stats_.irq_max_us, ring_max = stats_.rx_ring_max_pending;
    uint32_t parse_max = stats_.parse_max_us, parse_gap_max = stats_.parse_gap_max_us;
    stats_.irq_max_us = stats_.rx_ring_max_pending = stats_.parse_max_us = stats_.parse_gap_max_us = 0;
    restore_interrupts(irq);

    const LC29HStats &s = stats_;
    unsigned long t = now_ms % 10000000u;
    int n = snprintf(buf, size,
                     "DU,%lu,%lu,%lu,%lu,%lu,%lu\n"
                     "DS,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n"
                     "DB,%lu,%lu,%lu,%lu,%lu\n"
                     "DC,%lu,%lu,%lu,%lu,%lu,%lu\n"
                     "DL,%lu,%lu,%lu,%lu,%lu,%lu\n",
                     t, ul(s.rx_bytes), ul(s.uart_overrun), ul(s.uart_framing_break), ul(ring_max), ul(irq_max),
                     t, ul(s.lines), ul(s.ok_gga), ul(s.ok_tar), ul(s.ok_epe), ul(s.ok_other), ul(s.long_lines),
                     t, ul(s.bad_gga), ul(s.bad_tar), ul(s.bad_epe), ul(s.bad_other),
                     t, ul(s.closed[GNSS_CLOSE_COMPLETE]), ul(s.closed[GNSS_CLOSE_NEXT_GGA]),
                     ul(s.closed[GNSS_CLOSE_REPEAT]), ul(s.closed[GNSS_CLOSE_TIMEOUT]), ul(s.overwritten),
                     t, ul(s.rx_dropped), ul(s.rtcm_bytes), ul(s.rtcm_dropped), ul(parse_max), ul(parse_gap_max));
    return n < size ? n : size - 1;
}
#endif
