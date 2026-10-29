#include <stdio.h>

#include "pico/stdlib.h"

#include "hardware/uart.h"

#include <cstring>
#include <iostream>

typedef void (*uart_callback)();

int uart_setup(uart_inst_t *bus_id, int rx_pin, int tx_pin, uint baudrate, int data_bits,
               int stop_bits, uart_parity_t parity);
// Same as above, with fifo: true keeps the 32-byte RX/TX FIFOs on, so a byte survives ~32 character times of
// interrupt delay instead of one (460800 baud: ~700 us instead of ~22 us). The interrupt handler must then
// drain everything readable (while (uart_is_readable(...))). uart_setup() above keeps the FIFOs off.
int uart_setup(uart_inst_t *bus_id, int rx_pin, int tx_pin, uint baudrate, int data_bits,
               int stop_bits, uart_parity_t parity, bool fifo);
void uart_enable_interrupt(uart_inst_t *bus_id, int uart_irq, uart_callback callback);
bool uart_read_char(uart_inst_t *bus_id, char *c);
void uart_write_char(uart_inst_t *bus_id, char c);
void uart_write_string(uart_inst_t *bus_id, char *s);