/*
 * This file includes code provided by STMicroelectronics,
 * originally licensed under the BSD License (https://www.st.com/SLA0103).
 *
 * Modifications made by INESC TEC on communication functions:
 * - RdByte, RdWord, RdDWord, WrByte, WrWord, WrDWord, WaitMs
 */


#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/**
 * @brief I2C instance.
 */
extern i2c_inst_t* vl_i2c_inst_;

/**
* VL53L4CD device instance.
*/

typedef uint16_t Dev_t;

/**
 * @brief Error instance.
 */
typedef uint8_t VL53L4CD_Error;

/**
 * @brief If the macro below is defined, the device will be programmed to run
 * with I2C Fast Mode Plus (up to 1MHz). Otherwise, default max value is 400kHz.
 */

//#define VL53L4CD_I2C_FAST_MODE_PLUS


/**
 * @brief Read 8 bits through I2C.
 */

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t RegisterAdress, uint8_t *value);

/**
 * @brief Read 16 bits through I2C.
 */

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t RegisterAdress, uint16_t *value);

/**
 * @brief Read 32 bits through I2C.
 */

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t *value);

/**
 * @brief Write 8 bits through I2C.
 */

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t RegisterAdress, uint8_t value);

/**
 * @brief Write 16 bits through I2C.
 */

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAdress, uint16_t value);

/**
 * @brief Write 32 bits through I2C.
 */

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t value);

/**
 * @brief Wait during N milliseconds.
 */

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs);

/**
 * @brief Read data from I2C
 * 
 * @param DeviceAddr 
 * @param RegisterAddress 
 * @param values 
 * @param size 
 * @return uint8_t 
 */
uint8_t VL53L4CD_I2CRead(uint8_t DeviceAddr, uint16_t RegisterAddress, uint8_t *values, uint32_t size);

/**
 * @brief Write data to I2C
 * 
 * @param DeviceAddr 
 * @param RegisterAddress 
 * @param values 
 * @param size 
 * @return uint8_t 
 */
uint8_t VL53L4CD_I2CWrite(uint8_t DeviceAddr, uint16_t RegisterAddress, uint8_t *values, uint32_t size);

#endif	// _PLATFORM_H_