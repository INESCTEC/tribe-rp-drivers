#include <stdio.h>
#include <string.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "hardware/irq.h"

// Core 1 interrupt Handler
void core1_interrupt_handler() {

    // Receive Raw Value, Convert and Print Temperature Value
    while (multicore_fifo_rvalid()) {
        uint16_t raw = multicore_fifo_pop_blocking();
        printf("Temp = %f C\n", (float) raw);
    }

    multicore_fifo_clear_irq();  // Clear interrupt
}

// Core 1 Main Code
void core1_entry() {
    // Configure Core 1 Interrupt
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_interrupt_handler);

    irq_set_enabled(SIO_IRQ_PROC1, true);

    // Infinte While Loop to wait for interrupt
    while (1) {
        tight_loop_contents();
    }
}

// Core 0 Main Code
int main(void) {
    stdio_init_all();

    multicore_launch_core1(
            core1_entry);  // Start core 1 - Do this before any interrupt configuration

    // Primary Core 0 Loop
    while (1) {
        uint16_t raw = 23;
        multicore_fifo_push_blocking(raw);
        sleep_ms(1000);
    }
}