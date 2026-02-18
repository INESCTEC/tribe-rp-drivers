#include "../include/rp_agrolib_uart.h"

int uart_setup(uart_inst_t *bus_id, int rx_pin, int tx_pin, uint baudrate, int data_bits,
               int stop_bits, uart_parity_t parity) {

    // There is no guarantee that the baudrate requested will be returned
    // The nearest will be chosen, and this function will return the configured baud rate.
    uint actual_baud = uart_init(bus_id, baudrate);
    printf("Baudrate returned: %d\n", actual_baud);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(bus_id, false, false);

    // Set our data format
    uart_set_format(bus_id, data_bits, stop_bits, parity);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(bus_id, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = bus_id == uart0 ? UART0_IRQ : UART1_IRQ;

    return UART_IRQ;
}

void uart_enable_interrupt(uart_inst_t *bus_id, int uart_irq, uart_callback callback) {

    // Set up and enable the interrupt handlers
    irq_set_exclusive_handler(uart_irq, callback);
    irq_set_enabled(uart_irq, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(bus_id, true, false);
}

bool uart_read_char(uart_inst_t *bus_id, char *c) {
    if (!uart_is_readable(bus_id))
        return false;
    *c = uart_getc(bus_id);
    return true;
}

void uart_write_char(uart_inst_t *bus_id, char c) {
    uart_putc(bus_id, c);
}

void uart_write_string(uart_inst_t *bus_id, char *s) {
    for (int i = 0; i < strlen(s); i++) {
        uart_write_char(bus_id, (char) s[i]);
        // std::cout << s << std::endl;
        // printf("%x\n", s[i]);
    }
}