#include "../include/rp_agrolib_lc29h.h"

#include "hardware/irq.h"

LC29HDriver *LC29HDriver::instance_[2] = {nullptr, nullptr};

LC29HDriver::LC29HDriver(uart_inst_t *uart_id) : uart_id_(uart_id) {}

bool LC29HDriver::begin(int rx_pin, int tx_pin, uint baudrate) {
    uint index = uart_get_index(uart_id_);
    int irq = index == 0 ? UART0_IRQ : UART1_IRQ;
    if (instance_[index] != nullptr || irq_has_handler(irq)) {
        return false;
    }
    uart_setup(uart_id_, rx_pin, tx_pin, baudrate, 8, 1, UART_PARITY_NONE, true);
    instance_[index] = this;
    irq_set_exclusive_handler(irq, index == 0 ? uart0Irq : uart1Irq);

    // RX interrupt at >= 4 bytes in the FIFO or after 32 idle bit periods (SDK); TX interrupt (enabled
    // only while corrections wait) when the TX FIFO drains to <= 1/2 (16 bytes)
    uart_set_irqs_enabled(uart_id_, true, false);
    hw_write_masked(&uart_get_hw(uart_id_)->ifls, 2 << UART_UARTIFLS_TXIFLSEL_LSB, UART_UARTIFLS_TXIFLSEL_BITS);
    irq_set_enabled(irq, true);
    begun_ = true;
    return true;
}

void LC29HDriver::uart0Irq() {
    instance_[0]->onIrq();
}

void LC29HDriver::uart1Irq() {
    instance_[1]->onIrq();
}

void LC29HDriver::onIrq() {
    uart_hw_t *hw = uart_get_hw(uart_id_);

    // RX: drain the whole FIFO. The data register carries the byte's error flags in bits 8-11
    while (!(hw->fr & UART_UARTFR_RXFE_BITS)) {
        uint32_t dr = hw->dr;
        if (dr & UART_UARTDR_OE_BITS) {
            stats_.uart_overrun++;  // the FIFO was full: bytes before this one were lost
        }
        if (dr & (UART_UARTDR_FE_BITS | UART_UARTDR_BE_BITS)) {
            stats_.uart_framing_break++;
        }
        uint16_t next = (rx_w_ + 1) % LC29H_RX_RING_SIZE;
        if (next == rx_r_) {
            stats_.rx_dropped++;  // ring full: keep the older bytes, the parser will resync on '$'
            continue;
        }
        rx_ring_[rx_w_] = (uint8_t) dr;
        rx_w_ = next;
        stats_.rx_bytes++;
    }

    // TX: refill from the corrections ring; stop the TX interrupt when there is nothing left, since it
    // would keep firing on an empty FIFO
    fillTxFifo();
    if (tx_r_ == tx_w_) {
        hw_clear_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
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

void LC29HDriver::getNMEA(uint16_t size, RtkState *rtk_state, uint8_t *data) {
    // Convert uint8_t array to string
    char nmea_string[size + 1];
    sprintf(nmea_string, "%s", (char *) data);

    char *ptr = strstr(nmea_string, "$GNGGA");
    if (ptr != NULL) {
        // Find the position of the new line from the found index
        char *new_line_ptr = strchr(ptr, '\n');
        if (new_line_ptr != NULL) {
            // Calculate the size of the section to be copied
            size_t section_length = new_line_ptr - ptr;

            // Create an array to store the copied section
            char section[section_length + 1];

            // Copy the section to the new array
            memcpy(section, ptr, section_length);
            section[section_length] = '\0';  // Add the null terminator

            // Parse GGA and store it
            if (validateChecksum(section)) {
                parseGGA(section, rtk_state);
            }
        }
    }

    ptr = strstr(nmea_string, "$PQTMTAR");
    if (ptr != NULL) {
        // Find the position of the new line from the found index
        char *new_line_ptr = strchr(ptr, '\n');
        if (new_line_ptr != NULL) {
            // Calculate the size of the section to be copied
            size_t section_length = new_line_ptr - ptr;

            // Create an array to store the copied section
            char section[section_length + 1];

            // Copy the section to the new array
            memcpy(section, ptr, section_length);
            section[section_length] = '\0';  // Add the null terminator

            // Parse GGA and store it
            if (validateChecksum(section)) {
                parsePQTMTAR(section, rtk_state);
            }
        }
    }

    ptr = strstr(nmea_string, "$GNVTG");
    if (ptr != NULL) {
        // Find the position of the new line from the found index
        char *new_line_ptr = strchr(ptr, '\n');
        if (new_line_ptr != NULL) {
            // Calculate the size of the section to be copied
            size_t section_length = new_line_ptr - ptr;

            // Create an array to store the copied section
            char section[section_length + 1];

            // Copy the section to the new array
            memcpy(section, ptr, section_length);
            section[section_length] = '\0';  // Add the null terminator

            // Parse VTG and store it
            if (validateChecksum(section)) {
                parseVTG(section, rtk_state);
            }
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