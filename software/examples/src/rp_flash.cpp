#include <stdio.h>

#include "pico/stdlib.h"

#include "rp_agrolib_flash.h"

#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)  // address of last sector

int main() {
    stdio_init_all();

    Flash flash;

    int first_empty_page = -1;

    uint8_t data[FLASH_PAGE_SIZE / sizeof(uint8_t)] = {"Ola_"};
    data[0] = 'O';
    data[1] = 'l';
    data[2] = 'a';
    data[3] = '_';
    uint8_t reading[5];

    // Give time to initialize
    sleep_ms(3000);

    while (true) {
        // Get first empty page to possibly program
        first_empty_page = flash.findFirstEmptyPageInSector(FLASH_TARGET_OFFSET);
        printf("Found first empty page at %d.\n", first_empty_page);
        if (first_empty_page > 0) {
            flash.read(FLASH_TARGET_OFFSET + ((first_empty_page - 1) * FLASH_PAGE_SIZE), reading,
                       5);
            printf("Read '%s' from page at position %d.\n", reading, first_empty_page - 1);
        }

        // Get char from USB
        int c = getchar_timeout_us(100);
        if (c == PICO_ERROR_TIMEOUT) {
        } else {
            if (c == (int) 'z' ||
                first_empty_page < 0)  // Erase flash if 'z' is received or if there is no empty
                                       // page left in the sector
            {
                flash.erase(FLASH_TARGET_OFFSET);
                printf("Erased flash.\n");
            } else if (c == (int) 'r' &&
                       first_empty_page > 0)  // Tries to program a page that is not empty if 'r' is
                                              // received -> gives error (expected!)
            {
                flash.programSinglePage(
                        FLASH_TARGET_OFFSET + ((first_empty_page - 1) * FLASH_PAGE_SIZE), data, 4);
            } else if (first_empty_page >
                       -1)  // Programs next empty page if anything else is received through USB
            {
                data[4] = (uint8_t) c;
                flash.programSinglePage(FLASH_TARGET_OFFSET + (first_empty_page * FLASH_PAGE_SIZE),
                                        data, 5);
            }
        }

        sleep_ms(1000);
    }

    return 0;
}
