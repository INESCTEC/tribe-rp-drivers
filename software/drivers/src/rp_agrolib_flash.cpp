// See https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/ for more
// information about this!

#include "../include/rp_agrolib_flash.h"

/*
 * This function finds the first empty page inside a sector
 */
int Flash::findFirstEmptyPageInSector(int start_addr) {
    int page, addr, first_empty_page = -1;
    int *p;
    for (page = 0; page < FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE; ++page) {
        addr = XIP_BASE + start_addr + (page * FLASH_PAGE_SIZE);
        p = (int *) addr;
        // 0xFFFFFFFF cast as an int is -1 so this is how we detect an empty page
        if (*p == -1 && first_empty_page < 0) {
            first_empty_page = page;
            break;
        }
    }
    return first_empty_page;
}

/*
 * This function finds the first empty page
 * Start: PICO_FLASH_SIZE_BYTES
 * End: __flash_binary_end (XIP_BASE + BINARY_SIZE)
 */
int Flash::findFirstEmptyPage(uint32_t start_addr, uint32_t max_addr) {
    int page, addr, first_empty_page = -1;
    int *p;

    int max_pages = (start_addr - max_addr) / FLASH_PAGE_SIZE;
    int pages_per_sector = FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE;

    for (page = 0; page < max_pages; ++page) {
        addr = XIP_BASE + start_addr - (page * FLASH_PAGE_SIZE);
        p = (int *) addr;
        // 0xFFFFFFFF cast as an int is -1 so this is how we detect an empty page
        if (*p == -1 && first_empty_page < 0) {
            first_empty_page = page;
            break;
        }
    }
    return first_empty_page;
}

/*
 * This function programs a single page if possible (if the page is empty) without erasing the
 * entire sector
 */
int Flash::programSinglePage(int start_addr, uint8_t *data, int size) {
    if (size > FLASH_PAGE_SIZE) {
        printf("Err flash prog single: 'size' bigger than 'FLASH_PAGE_SIZE'\n");
        return -1;
    }

    int addr = XIP_BASE + start_addr;
    int *p = (int *) addr;
    if (*p != -1) {
        printf("Err flash prog single: page is not empty\n");
        return -2;
    }

    printf("Address addr: %d\n", addr);
    printf("Address start_addr (xip_base + start_addr): %d\n", start_addr);

    uint8_t buf[1 * (FLASH_PAGE_SIZE / sizeof(uint8_t))];
    for (size_t i = 0; i < size; ++i)
        buf[i] = data[i];

    uint32_t interrupts;
    interrupts = save_and_disable_interrupts();
    flash_range_program(start_addr, (uint8_t *) buf, FLASH_PAGE_SIZE);
    restore_interrupts(interrupts);
    return 0;
}

/*
 * This function erase an entire sector
 */
void Flash::erase(int start_addr) {
    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(start_addr, FLASH_SECTOR_SIZE);
    restore_interrupts(interrupts);
}

// TODO: Missing address validation
// TODO: Sector erased must be multiple of 2048 (FLASH_SECTOR_SIZE)
// TODO: Page programmed must be multiple of 256 (FLASH_PAGE_SIZE)
// TODO: Ensure that address starts at multiples
/*
 * This function programs and erases multiple pages
 */
void Flash::programMultiplePages(int start_addr, uint8_t *data, int size) {
    int multiples_pages = size / FLASH_PAGE_SIZE;

    uint8_t buf[(multiples_pages + 1) * (FLASH_PAGE_SIZE / sizeof(uint8_t))];
    for (size_t i = 0; i < size; ++i) {
        buf[i] = data[i];
    }

    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(start_addr, FLASH_SECTOR_SIZE);
    flash_range_program(start_addr, (uint8_t *) buf, FLASH_PAGE_SIZE);
    restore_interrupts(interrupts);
}

/*
 * This function read data from flash
 */
void Flash::read(int start_addr, uint8_t *reading, int nbytes) {
    uint8_t *p;
    int start = XIP_BASE + start_addr;
    for (size_t i = 0; i < nbytes; ++i) {
        p = (uint8_t *) (start + i);
        reading[i] = *p;
    }
    reading[nbytes] = '\0';
}
