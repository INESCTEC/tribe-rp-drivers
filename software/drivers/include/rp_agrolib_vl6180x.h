// picovl6180x.h
// VL6180X Time-of-Flight + Ambient Light Sensor driver (Pico SDK)

#pragma once

#include <stdio.h>
#include "pico/stdlib.h"
#include "rp_agrolib_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PICOVL6180X_DEFAULT_ADDR 0x29

#ifndef rp_agrolib_vl6180x_h
#define rp_agrolib_vl6180x_h



typedef enum {
    PICOVL6180X_OK = 0,
    PICOVL6180X_ERR_I2C,
    PICOVL6180X_ERR_TIMEOUT,
    PICOVL6180X_ERR_INVALID_ARG,
    PICOVL6180X_ERR_NOT_INITIALIZED,
} picovl6180x_status_t;

typedef enum {
    PICOVL6180X_RANGE_ERROR_NONE        = 0,
    PICOVL6180X_RANGE_ERROR_SYSERR_1    = 1,
    PICOVL6180X_RANGE_ERROR_SYSERR_2    = 2,
    PICOVL6180X_RANGE_ERROR_SYSERR_3    = 3,
    PICOVL6180X_RANGE_ERROR_SYSERR_4    = 4,
    PICOVL6180X_RANGE_ERROR_SYSERR_5    = 5,
    PICOVL6180X_RANGE_ERROR_ECEFAIL     = 6,
    PICOVL6180X_RANGE_ERROR_NOCONVERGE  = 7,
    PICOVL6180X_RANGE_ERROR_RANGEIGNORE = 8,
    PICOVL6180X_RANGE_ERROR_SNR         = 11,
    PICOVL6180X_RANGE_ERROR_RAWUFLOW    = 12,
    PICOVL6180X_RANGE_ERROR_RAWOFLOW    = 13,
    PICOVL6180X_RANGE_ERROR_RANGEUFLOW  = 14,
    PICOVL6180X_RANGE_ERROR_RANGEOFLOW  = 15,
} picovl6180x_range_error_t;

typedef enum {
    PICOVL6180X_ALS_GAIN_20    = 0x00,
    PICOVL6180X_ALS_GAIN_10    = 0x01,
    PICOVL6180X_ALS_GAIN_5     = 0x02,
    PICOVL6180X_ALS_GAIN_2_5   = 0x03,
    PICOVL6180X_ALS_GAIN_1_67  = 0x04,
    PICOVL6180X_ALS_GAIN_1_25  = 0x05,
    PICOVL6180X_ALS_GAIN_1     = 0x06,
    PICOVL6180X_ALS_GAIN_40    = 0x07,
} picovl6180x_als_gain_t;

typedef struct {
    i2c_inst_t *i2c;
    uint8_t     addr;        
    bool        initialized;

    uint8_t     scaling;     // 1..3
    int8_t      ptp_offset; 

    uint32_t    timeout_ms;  
} picovl6180x_t;

// subset of registers 
typedef enum {
    PICOVL6180X_REG_SYSTEM__INTERRUPT_CLEAR            = 0x0015,
    PICOVL6180X_REG_SYSTEM__FRESH_OUT_OF_RESET         = 0x0016,

    PICOVL6180X_REG_SYSRANGE__START                    = 0x0018,
    PICOVL6180X_REG_SYSRANGE__CROSSTALK_VALID_HEIGHT   = 0x0021,
    PICOVL6180X_REG_SYSRANGE__PART_TO_PART_OFFSET      = 0x0024,
    PICOVL6180X_REG_SYSRANGE__RANGE_CHECK_ENABLES      = 0x002D,
    PICOVL6180X_REG_SYSRANGE__INTERMEASUREMENT_PERIOD  = 0x01B,
    PICOVL6180X_REG_SYSRANGE__MAX_CONVERGENCE_TIME     = 0x01C,


    PICOVL6180X_REG_SYSALS__START                      = 0x0038,
    PICOVL6180X_REG_SYSALS__ANALOGUE_GAIN              = 0x003F,
    PICOVL6180X_REG_SYSALS__INTEGRATION_PERIOD         = 0x0040,

    PICOVL6180X_REG_RESULT__RANGE_STATUS               = 0x004D,
    PICOVL6180X_REG_RESULT__RANGE_VAL                  = 0x0062,
    PICOVL6180X_REG_RESULT__INTERRUPT_STATUS_GPIO      = 0x004F,
    PICOVL6180X_REG_RESULT__ALS_VAL                    = 0x0050,
    
    PICOVL6180X_REG_RANGE_SCALER                       = 0x0096,

    PICOVL6180X_REG_I2C_SLAVE__DEVICE_ADDRESS          = 0x0212,
} picovl6180x_reg_t;
//Write 8-bit value to register
picovl6180x_status_t write_reg8(picovl6180x_t *dev, uint16_t reg, uint8_t val); 
//Write 16-bit value to register
 picovl6180x_status_t write_reg16(picovl6180x_t *dev, uint16_t reg, uint16_t val);
//Read 8-bit value from register
 picovl6180x_status_t read_reg8(picovl6180x_t *dev, uint16_t reg, uint8_t *val_out);
//Read 16-bit value from registerr
picovl6180x_status_t read_reg16(picovl6180x_t *dev, uint16_t reg, uint16_t *val_out);
// -------------------- Bring-up --------------------

//Debug to check Return Rate, SNR
picovl6180x_status_t picovl6180x_read_return_rate(picovl6180x_t *dev, float *mcps_out);
// Split init 
picovl6180x_status_t picovl6180x_init(picovl6180x_t *dev, i2c_inst_t *i2c, uint8_t addr);

// Recommended defaults 
picovl6180x_status_t picovl6180x_configure_default(picovl6180x_t *dev);

// One-call: init + defaults 
picovl6180x_status_t picovl6180x_begin(picovl6180x_t *dev, i2c_inst_t *i2c, uint8_t addr);

// -------------------- Address / timeout --------------------

picovl6180x_status_t picovl6180x_set_address(picovl6180x_t *dev, uint8_t new_addr);
uint8_t          picovl6180x_get_address(const picovl6180x_t *dev);

void             picovl6180x_set_timeout_ms(picovl6180x_t *dev, uint32_t timeout_ms);
uint32_t         picovl6180x_get_timeout_ms(const picovl6180x_t *dev);

// -------------------- Scaling --------------------

picovl6180x_status_t picovl6180x_set_scaling(picovl6180x_t *dev, uint8_t scaling);
uint8_t          picovl6180x_get_scaling(const picovl6180x_t *dev);

// -------------------- Optional knobs --------------------
picovl6180x_status_t picovl6180x_set_range_period_ms(picovl6180x_t *dev, uint16_t ms);
picovl6180x_status_t picovl6180x_set_als_period_ms(picovl6180x_t *dev, uint16_t ms);

picovl6180x_status_t picovl6180x_set_als_integration_ms(picovl6180x_t *dev, uint16_t ms);
picovl6180x_status_t picovl6180x_get_als_integration_ms(picovl6180x_t *dev, uint16_t *ms);
picovl6180x_status_t picovl6180x_set_snr_check(picovl6180x_t *dev, bool enable);
picovl6180x_status_t picovl6180x_set_range_ignore(picovl6180x_t *dev, bool enable);
picovl6180x_status_t picovl6180x_set_crosstalk_compensation(picovl6180x_t *dev, uint16_t compensation_value, uint8_t valid_height);

// -------------------- Reads --------------------
//
// Range:
//   - mm_out: scaled distance in mm
//   - err_out: decoded range error
//   - status_raw_out: full RESULT__RANGE_STATUS byte
picovl6180x_status_t picovl6180x_read_range(picovl6180x_t *dev,
                                   uint16_t *mm_out,
                                   picovl6180x_range_error_t *err_out,
                                   uint8_t *status_raw_out);

// ALS:
//   - raw_out: raw ALS counts
//   - lux_out: lux computed from same sample
picovl6180x_status_t picovl6180x_read_als(picovl6180x_t *dev,
                                 uint16_t *raw_out,
                                 float *lux_out);

// -------------------- ALS gain --------------------

picovl6180x_status_t picovl6180x_set_als_gain(picovl6180x_t *dev, picovl6180x_als_gain_t gain);
picovl6180x_status_t picovl6180x_get_als_gain(const picovl6180x_t *dev, picovl6180x_als_gain_t *gain);

// -------------------- String helpers --------------------

const char *picovl6180x_range_error_str(picovl6180x_range_error_t err);
const char *picovl6180x_status_str(picovl6180x_status_t st);

// -------------------- Optional Pico helpers --------------------

#ifndef PICOVL6180X_NO_PICO_HELPERS

picovl6180x_status_t picovl6180x_hw_begin_default(picovl6180x_t *dev,
                                           i2c_inst_t *i2c,
                                           uint8_t addr,
                                           uint sda_pin,
                                           uint scl_pin,
                                           uint32_t baud_hz,
                                           bool enable_pullups);

picovl6180x_status_t picovl6180x_hw_begin(picovl6180x_t *dev,
                                   i2c_inst_t *i2c,
                                   uint sda_pin,
                                   uint scl_pin);

#endif 

#ifdef __cplusplus
}
#endif
#endif
