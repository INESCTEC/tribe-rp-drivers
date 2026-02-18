#pragma once

#include <stdio.h>

#include "pico/stdlib.h"
extern "C" {
#include "hardware/flash.h"
#include "hardware/sync.h"
};

extern char __flash_binary_end;

#define SAFE_START_ADDR (XIP_BASE + FLASH_TARGET_OFFSET)
#define MAX_FLASH_ADDR (16 * 1024 * 1024)   // 16MB flash (from XIP_BASE to the end)
#define BINARY_END_ADDR __flash_binary_end  // XIP_BASE + BINARY_SIZE

class Flash {
public:
    int findFirstEmptyPageInSector(int start_addr);
    int findFirstEmptyPage(uint32_t start_addr, uint32_t max_addr);
    int programSinglePage(int start_addr, uint8_t *data, int size);
    void erase(int start_addr);
    void programMultiplePages(int start_addr, uint8_t *data, int size);
    void read(int start_addr, uint8_t *reading, int nbytes);

private:
};
