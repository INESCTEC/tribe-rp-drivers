#include <stdio.h>

#include "pico/stdlib.h"

#include "hardware/uart.h"

#include <cstring>
#include <iostream>

typedef void (*uart_callback)();

int uart_setup(uart_inst_t *bus_id, int rx_pin, int tx_pin, uint baudrate, int data_bits,
               int stop_bits, uart_parity_t parity);
void uart_enable_interrupt(uart_inst_t *bus_id, int uart_irq, uart_callback callback);
bool uart_read_char(uart_inst_t *bus_id, char *c);
void uart_write_char(uart_inst_t *bus_id, char c);
void uart_write_string(uart_inst_t *bus_id, char *s);