#include <stdio.h>
#include <string.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "hardware/irq.h"

int global = 0;


// Core 1 Main Code
void core1_entry() {
    // Infinte While Loop to wait for interrupt
    while (1) {
        printf("Global = %d C\n", global);
        sleep_ms(1000);
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
        // multicore_fifo_push_blocking(raw);
        global++;
        sleep_ms(1000);
    }
}