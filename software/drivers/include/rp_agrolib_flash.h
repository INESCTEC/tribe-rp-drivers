#pragma once

#include <stdio.h>
#include "pico/stdlib.h"
extern "C" {
  #include "hardware/sync.h"
  #include "hardware/flash.h"
};

extern char __flash_binary_end;

#define SAFE_START_ADDR (XIP_BASE + FLASH_TARGET_OFFSET)
#define MAX_FLASH_ADDR (16 * 1024 * 1024)  // 16MB flash (from XIP_BASE to the end)
#define BINARY_END_ADDR __flash_binary_end // XIP_BASE + BINARY_SIZE

class Flash {
  public:
    int findFirstEmptyPageInSector(int flash_offs);
    int findFirstEmptyPage(uint32_t upperLimitOffset, uint32_t lowerLimitOffset);
    int programSinglePage(int flash_offs, uint8_t* data, int size);
    void erase(int flash_offs);
    void programMultiplePages(int flash_offs, uint8_t* data, int size);
    void read(int flash_offs, uint8_t* reading, int nbytes);

  private:
};
