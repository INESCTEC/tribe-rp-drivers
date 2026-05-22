#include "../include/rp_agrolib_vl6180x.h"

#include <stddef.h>


#ifndef PICOVL6180X_NO_PICO_HELPERS
#include "hardware/gpio.h"
#endif

// -------------------- Internal registers --------------------

#define REG_READOUT__AVERAGING_SAMPLE_PERIOD      0x010A
#define REG_SYSRANGE__VHV_REPEAT_RATE             0x0031
#define REG_SYSRANGE__VHV_RECALIBRATE             0x002E
#define REG_SYSRANGE__INTERMEASUREMENT_PERIOD     0x001B
#define REG_SYSRANGE__MAX_CONVERGENCE_TIME        0x001C

#define REG_SYSALS__INTERMEASUREMENT_PERIOD       0x003E

#define REG_SYSTEM__INTERRUPT_CONFIG_GPIO         0x0014
#define REG_INTERLEAVED_MODE__ENABLE              0x02A3

// Values for 1x/2x/3x scaling 
static const uint16_t kScalerValues[] = {0, 253, 127, 84};

// Used by common implementations
static const uint8_t kDefaultCrosstalkValidHeight = 20;

// -------------------- I2C helpers --------------------

static picovl6180x_status_t write_reg8(picovl6180x_t *dev, uint16_t reg, uint8_t value) {
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), value };
    int rc = i2c_write_blocking(dev->i2c, dev->addr, buf, 3, false);
    return (rc == 3) ? PICOVL6180X_OK : PICOVL6180X_ERR_I2C;
}

static picovl6180x_status_t write_reg16(picovl6180x_t *dev, uint16_t reg, uint16_t value) {
    uint8_t buf[4] = {
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)
    };
    int rc = i2c_write_blocking(dev->i2c, dev->addr, buf, 4, false);
    return (rc == 4) ? PICOVL6180X_OK : PICOVL6180X_ERR_I2C;
}

static picovl6180x_status_t read_reg8(picovl6180x_t *dev, uint16_t reg, uint8_t *out) {
    uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    int rc = i2c_write_blocking(dev->i2c, dev->addr, addr, 2, true);
    if (rc != 2) return PICOVL6180X_ERR_I2C;

    rc = i2c_read_blocking(dev->i2c, dev->addr, out, 1, false);
    return (rc == 1) ? PICOVL6180X_OK : PICOVL6180X_ERR_I2C;
}

static picovl6180x_status_t read_reg16(picovl6180x_t *dev, uint16_t reg, uint16_t *out) {
    uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    uint8_t data[2];

    int rc = i2c_write_blocking(dev->i2c, dev->addr, addr, 2, true);
    if (rc != 2) return PICOVL6180X_ERR_I2C;

    rc = i2c_read_blocking(dev->i2c, dev->addr, data, 2, false);
    if (rc != 2) return PICOVL6180X_ERR_I2C;

    *out = ((uint16_t)data[0] << 8) | (uint16_t)data[1];

    //printf("DEBUG I2C: reg=0x%04X, data[0]=0x%02X, data[1]=0x%02X\n", reg, data[0], data[1]);
    return PICOVL6180X_OK;
}

// Poll RESULT__INTERRUPT_STATUS_GPIO until ready.
static picovl6180x_status_t poll_ready(picovl6180x_t *dev, uint8_t mask, uint8_t ready_value) {
    const bool use_timeout = (dev->timeout_ms > 0);
    absolute_time_t deadline = use_timeout ? make_timeout_time_ms(dev->timeout_ms) : nil_time;

    while (true) {
        uint8_t st = 0;
        picovl6180x_status_t rc = read_reg8(dev, PICOVL6180X_REG_RESULT__INTERRUPT_STATUS_GPIO, &st);
        if (rc != PICOVL6180X_OK) return rc;

        if ((st & mask) == ready_value) return PICOVL6180X_OK;

        if (use_timeout && time_reached(deadline)) return PICOVL6180X_ERR_TIMEOUT;

        sleep_ms(1);
    }
}

// -------------------- Helpers --------------------

static uint8_t clamp_u8(int v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

static uint8_t ms_to_intermeas_reg(uint16_t ms) {
    return clamp_u8((int)(ms / 10));
}

static float als_gain_to_float(picovl6180x_als_gain_t g) {
    switch (g) {
        case PICOVL6180X_ALS_GAIN_20:   return 20.0f;
        case PICOVL6180X_ALS_GAIN_10:   return 10.32f;
        case PICOVL6180X_ALS_GAIN_5:    return 5.21f;
        case PICOVL6180X_ALS_GAIN_2_5:  return 2.60f;
        case PICOVL6180X_ALS_GAIN_1_67: return 1.72f;
        case PICOVL6180X_ALS_GAIN_1_25: return 1.28f;
        case PICOVL6180X_ALS_GAIN_1:    return 1.01f;
        case PICOVL6180X_ALS_GAIN_40:   return 40.0f;
        default:                        return 1.01f;
    }
}

static picovl6180x_status_t compute_lux_from_raw(picovl6180x_t *dev, uint16_t raw, float *lux_out) {
    uint16_t integ_ms = 0;
    picovl6180x_status_t rc = read_reg16(dev, PICOVL6180X_REG_SYSALS__INTEGRATION_PERIOD, &integ_ms);
    if (rc != PICOVL6180X_OK) return rc;
    if (integ_ms == 0) return PICOVL6180X_ERR_INVALID_ARG;

    picovl6180x_als_gain_t gain_sel;
    rc = picovl6180x_get_als_gain(dev, &gain_sel);
    if (rc != PICOVL6180X_OK) return rc;

    float gain = als_gain_to_float(gain_sel);
    float integ_factor = 100.0f / (float)integ_ms;
    *lux_out = 0.32f * ((float)raw / gain) * integ_factor;
    return PICOVL6180X_OK;
}

// Return the Return Rate ,this is a value that can be used to adjust the crosstalk compensation dynamically based on the environment.
picovl6180x_status_t picovl6180x_read_return_rate(picovl6180x_t *dev, float *mcps_out) {
    uint16_t raw_rate;
    // 9.7 format
    picovl6180x_status_t rc = read_reg16(dev, 0x0066, &raw_rate);
    if (rc == PICOVL6180X_OK && mcps_out) {
        *mcps_out = (float)raw_rate / 128.0f; // Converte para Mega counts per second
    }
    return rc;
}

picovl6180x_status_t picovl6180x_calculate_snr(picovl6180x_t *dev, float *mcps_out){
uint32_t return_signal;
    //0x06C 32bit register for the range return signal count / 0x074 raneg return ambient count 32bits


}
picovl6180x_status_t picovl6180x_set_crosstalk_compensation(picovl6180x_t *dev, uint16_t value){
    picovl6180x_status_t rc = write_reg16(dev, 0x001E, value);
    if (rc == PICOVL6180X_OK)
    {
        printf("Crosstalk compensation set to %d.\n", value);
        return rc;
    }else
    {
        printf("Failed to set crosstalk compensation. Status code: %d\n", rc);
        return rc;
    } 
}
// -------------------- Bring-up --------------------

picovl6180x_status_t picovl6180x_init(picovl6180x_t *dev, i2c_inst_t *i2c, uint8_t addr) {
    if (!dev || !i2c || addr > 0x7F) return PICOVL6180X_ERR_INVALID_ARG;

    dev->i2c = i2c;
    dev->addr = addr;
    dev->timeout_ms = 0;
    dev->initialized = false;

    uint8_t ptp = 0;
    picovl6180x_status_t rc = read_reg8(dev, PICOVL6180X_REG_SYSRANGE__PART_TO_PART_OFFSET, &ptp);
    if (rc != PICOVL6180X_OK) return rc;
    dev->ptp_offset = (int8_t)ptp;

    uint8_t fresh = 0;
    rc = read_reg8(dev, PICOVL6180X_REG_SYSTEM__FRESH_OUT_OF_RESET, &fresh);
    if (rc != PICOVL6180X_OK) return rc;

    if (fresh == 1) {
        dev->scaling = 1;

        // init sequence 
        const struct { uint16_t reg; uint8_t val; } init_seq[] = {
            {0x0207, 0x01}, {0x0208, 0x01}, {0x0096, 0x00}, {0x0097, 0xFD},
            {0x00E3, 0x01}, {0x00E4, 0x03}, {0x00E5, 0x02}, {0x00E6, 0x01},
            {0x00E7, 0x03}, {0x00F5, 0x02}, {0x00D9, 0x05}, {0x00DB, 0xCE},
            {0x00DC, 0x03}, {0x00DD, 0xF8}, {0x009F, 0x00}, {0x00A3, 0x3C},
            {0x00B7, 0x00}, {0x00BB, 0x3C}, {0x00B2, 0x09}, {0x00CA, 0x09},
            {0x0198, 0x01}, {0x01B0, 0x17}, {0x01AD, 0x00}, {0x00FF, 0x05},
            {0x0100, 0x05}, {0x0199, 0x05}, {0x01A6, 0x1B}, {0x01AC, 0x3E},
            {0x01A7, 0x1F}, {0x0030, 0x00}
        };

        for (size_t i = 0; i < sizeof(init_seq)/sizeof(init_seq[0]); i++) {
            rc = write_reg8(dev, init_seq[i].reg, init_seq[i].val);
            if (rc != PICOVL6180X_OK) return rc;
        }

        rc = write_reg8(dev, PICOVL6180X_REG_SYSTEM__FRESH_OUT_OF_RESET, 0);
        if (rc != PICOVL6180X_OK) return rc;

    } else {
        // Scaling from RANGE_SCALER
        uint16_t s = 0;
        rc = read_reg16(dev, PICOVL6180X_REG_RANGE_SCALER, &s);
        if (rc != PICOVL6180X_OK) return rc;

        if (s == kScalerValues[3]) dev->scaling = 3;
        else if (s == kScalerValues[2]) dev->scaling = 2;
        else dev->scaling = 1;
    }

    dev->initialized = true;
    return PICOVL6180X_OK;
}

picovl6180x_status_t picovl6180x_configure_default(picovl6180x_t *dev) {
    if (!dev) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;

    picovl6180x_status_t rc = write_reg8(dev, REG_READOUT__AVERAGING_SAMPLE_PERIOD, 0x30);
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg8(dev, PICOVL6180X_REG_SYSALS__ANALOGUE_GAIN, 0x40 | (uint8_t)PICOVL6180X_ALS_GAIN_1);
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg8(dev, REG_SYSRANGE__VHV_REPEAT_RATE, 0xFF);
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg16(dev, PICOVL6180X_REG_SYSALS__INTEGRATION_PERIOD, 100);
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg8(dev, REG_SYSRANGE__VHV_RECALIBRATE, 0x01);
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg8(dev, REG_SYSRANGE__INTERMEASUREMENT_PERIOD, ms_to_intermeas_reg(100));
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg8(dev, REG_SYSALS__INTERMEASUREMENT_PERIOD, ms_to_intermeas_reg(500));
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg8(dev, REG_SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24);
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg8(dev, REG_SYSRANGE__MAX_CONVERGENCE_TIME, 0x31);
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg8(dev, REG_INTERLEAVED_MODE__ENABLE, 0x00);
    if (rc != PICOVL6180X_OK) return rc;

    return picovl6180x_set_scaling(dev, 1);
}

picovl6180x_status_t picovl6180x_begin(picovl6180x_t *dev, i2c_inst_t *i2c, uint8_t addr) {
    picovl6180x_status_t rc = picovl6180x_init(dev, i2c, addr);
    if (rc != PICOVL6180X_OK) return rc;

    rc = picovl6180x_configure_default(dev);
    if (rc != PICOVL6180X_OK) return rc;

    dev->timeout_ms = 500;
    return PICOVL6180X_OK;
}

// -------------------- Address / timeout --------------------

picovl6180x_status_t picovl6180x_set_address(picovl6180x_t *dev, uint8_t new_addr) {
    if (!dev) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;
    if (new_addr > 0x7F) return PICOVL6180X_ERR_INVALID_ARG;

    picovl6180x_status_t rc = write_reg8(dev, PICOVL6180X_REG_I2C_SLAVE__DEVICE_ADDRESS,
                                        (uint8_t)(new_addr & 0x7F));
    if (rc == PICOVL6180X_OK) dev->addr = (uint8_t)(new_addr & 0x7F);
    return rc;
}

uint8_t picovl6180x_get_address(const picovl6180x_t *dev) {
    return dev ? dev->addr : 0;
}

void picovl6180x_set_timeout_ms(picovl6180x_t *dev, uint32_t timeout_ms) {
    if (dev) dev->timeout_ms = timeout_ms;
}

uint32_t picovl6180x_get_timeout_ms(const picovl6180x_t *dev) {
    return dev ? dev->timeout_ms : 0;
}

// -------------------- Scaling --------------------

picovl6180x_status_t picovl6180x_set_scaling(picovl6180x_t *dev, uint8_t scaling) {
    if (!dev) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;
    if (scaling < 1 || scaling > 3) return PICOVL6180X_ERR_INVALID_ARG;

    dev->scaling = scaling;

    picovl6180x_status_t rc = write_reg16(dev, PICOVL6180X_REG_RANGE_SCALER, kScalerValues[scaling]);
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg8(dev, PICOVL6180X_REG_SYSRANGE__PART_TO_PART_OFFSET,
                    (uint8_t)((int16_t)dev->ptp_offset / (int16_t)scaling));
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg8(dev, PICOVL6180X_REG_SYSRANGE__CROSSTALK_VALID_HEIGHT,
                    (uint8_t)(kDefaultCrosstalkValidHeight / scaling));
    if (rc != PICOVL6180X_OK) return rc;

    uint8_t rce = 0;
    rc = read_reg8(dev, PICOVL6180X_REG_SYSRANGE__RANGE_CHECK_ENABLES, &rce);
    if (rc != PICOVL6180X_OK) return rc;

    rce = (uint8_t)((rce & 0xFE) | (scaling == 1 ? 1 : 0));
    return write_reg8(dev, PICOVL6180X_REG_SYSRANGE__RANGE_CHECK_ENABLES, rce);
}

uint8_t picovl6180x_get_scaling(const picovl6180x_t *dev) {
    return dev ? dev->scaling : 0;
}

// -------------------- Optional knobs --------------------

picovl6180x_status_t picovl6180x_set_range_period_ms(picovl6180x_t *dev, uint16_t ms) {
    if (!dev) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;
    return write_reg8(dev, REG_SYSRANGE__INTERMEASUREMENT_PERIOD, ms_to_intermeas_reg(ms));
}

picovl6180x_status_t picovl6180x_set_als_period_ms(picovl6180x_t *dev, uint16_t ms) {
    if (!dev) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;
    return write_reg8(dev, REG_SYSALS__INTERMEASUREMENT_PERIOD, ms_to_intermeas_reg(ms));
}

picovl6180x_status_t picovl6180x_set_als_integration_ms(picovl6180x_t *dev, uint16_t ms) {
    if (!dev) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;
    if (ms == 0) return PICOVL6180X_ERR_INVALID_ARG;
    return write_reg16(dev, PICOVL6180X_REG_SYSALS__INTEGRATION_PERIOD, ms);
}

picovl6180x_status_t picovl6180x_get_als_integration_ms(picovl6180x_t *dev, uint16_t *ms) {
    if (!dev || !ms) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;
    return read_reg16(dev, PICOVL6180X_REG_SYSALS__INTEGRATION_PERIOD, ms);
}

picovl6180x_status_t picovl6180x_set_snr_check(picovl6180x_t *dev, bool enable) {

    if (!dev) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;

    uint8_t rce = 0;
    picovl6180x_status_t rc = read_reg8(dev, PICOVL6180X_REG_SYSRANGE__RANGE_CHECK_ENABLES, &rce);
    if (rc != PICOVL6180X_OK) return rc;

    if (enable) {
        //limite de SNR para 0.1 (10 * 16 = 160) 
        rc = write_reg8(dev, 0x002C, 160); // SYSRANGE__MAX_AMBIENT_LEVEL_MULT
        if (rc != PICOVL6180X_OK) return rc;

        // Bit 4 (0x10) SNR check enable
        rce |= 0x10; 
    } else {
      
        rce &= ~0x10;
    }

    return write_reg8(dev, PICOVL6180X_REG_SYSRANGE__RANGE_CHECK_ENABLES, rce);
}

picovl6180x_status_t picovl6180x_set_range_ignore(picovl6180x_t *dev, bool enable) {
    if (!dev) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;

    uint8_t rce = 0;
    picovl6180x_status_t rc = read_reg8(dev, PICOVL6180X_REG_SYSRANGE__RANGE_CHECK_ENABLES, &rce);
    if (rc != PICOVL6180X_OK) return rc;

    if (enable) {
        //Bit 1 (0x02) Range ignore enable
        rce |= 0x02; 
    } else {
        rce &= ~0x02;
    }

    return write_reg8(dev, PICOVL6180X_REG_SYSRANGE__RANGE_CHECK_ENABLES, rce);
}
// -------------------- Reads --------------------

picovl6180x_status_t picovl6180x_read_range(picovl6180x_t *dev,
                                           uint16_t *mm_out,
                                           picovl6180x_range_error_t *err_out,
                                           uint8_t *status_raw_out)
{
    if (!dev) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;

    if (!mm_out && !err_out && !status_raw_out) return PICOVL6180X_ERR_INVALID_ARG;

    picovl6180x_status_t rc = write_reg8(dev, PICOVL6180X_REG_SYSRANGE__START, 0x01);
    if (rc != PICOVL6180X_OK) return rc;

    rc = poll_ready(dev, 0x07, 0x04);
    if (rc != PICOVL6180X_OK) return rc;

    uint8_t status = 0;
    rc = read_reg8(dev, PICOVL6180X_REG_RESULT__RANGE_STATUS, &status);
    if (rc != PICOVL6180X_OK) return rc;

    uint8_t raw = 0;
    rc = read_reg8(dev, PICOVL6180X_REG_RESULT__RANGE_VAL, &raw);
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg8(dev, PICOVL6180X_REG_SYSTEM__INTERRUPT_CLEAR, 0x01);
    if (rc != PICOVL6180X_OK) return rc;

    if (status_raw_out) *status_raw_out = status;
    if (err_out) *err_out = (picovl6180x_range_error_t)(status >> 4);
    if (mm_out) *mm_out = (uint16_t)dev->scaling * (uint16_t)raw;

    return PICOVL6180X_OK;
}

picovl6180x_status_t picovl6180x_read_als(picovl6180x_t *dev,
                                         uint16_t *raw_out,
                                         float *lux_out)
{
    if (!dev) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;

    if (!raw_out && !lux_out) return PICOVL6180X_ERR_INVALID_ARG;

    picovl6180x_status_t rc = write_reg8(dev, PICOVL6180X_REG_SYSALS__START, 0x01);
    if (rc != PICOVL6180X_OK) return rc;

    rc = poll_ready(dev, 0x38, 0x20);
    if (rc != PICOVL6180X_OK) return rc;

    uint16_t raw = 0;
    rc = read_reg16(dev, PICOVL6180X_REG_RESULT__ALS_VAL, &raw);
    if (rc != PICOVL6180X_OK) return rc;

    rc = write_reg8(dev, PICOVL6180X_REG_SYSTEM__INTERRUPT_CLEAR, 0x02);
    if (rc != PICOVL6180X_OK) return rc;

    if (raw_out) *raw_out = raw;

    if (lux_out) {
        float lux = 0.0f;
        rc = compute_lux_from_raw(dev, raw, &lux);
        if (rc != PICOVL6180X_OK) return rc;
        *lux_out = lux;
    }

    return PICOVL6180X_OK;
}

// -------------------- ALS gain --------------------

picovl6180x_status_t picovl6180x_set_als_gain(picovl6180x_t *dev, picovl6180x_als_gain_t gain) {
    if (!dev) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;
    if ((uint8_t)gain > 0x07) return PICOVL6180X_ERR_INVALID_ARG;

    return write_reg8(dev, PICOVL6180X_REG_SYSALS__ANALOGUE_GAIN, (uint8_t)(0x40 | (uint8_t)gain));
}

picovl6180x_status_t picovl6180x_get_als_gain(const picovl6180x_t *dev, picovl6180x_als_gain_t *gain) {
    if (!dev || !gain) return PICOVL6180X_ERR_INVALID_ARG;
    if (!dev->initialized) return PICOVL6180X_ERR_NOT_INITIALIZED;

    uint8_t v = 0;
    picovl6180x_status_t rc = read_reg8((picovl6180x_t *)dev, PICOVL6180X_REG_SYSALS__ANALOGUE_GAIN, &v);
    if (rc != PICOVL6180X_OK) return rc;

    *gain = (picovl6180x_als_gain_t)(v & 0x07);
    return PICOVL6180X_OK;
}

// -------------------- Error Helpers --------------------

const char *picovl6180x_status_str(picovl6180x_status_t st) {
    switch (st) {
        case PICOVL6180X_OK:                  return "OK";
        case PICOVL6180X_ERR_I2C:             return "I2C_ERROR";
        case PICOVL6180X_ERR_TIMEOUT:         return "TIMEOUT";
        case PICOVL6180X_ERR_INVALID_ARG:     return "INVALID_ARG";
        case PICOVL6180X_ERR_NOT_INITIALIZED: return "NOT_INITIALIZED";
        default:                              return "UNKNOWN";
    }
}

const char *picovl6180x_range_error_str(picovl6180x_range_error_t err) {
    switch (err) {
        case PICOVL6180X_RANGE_ERROR_NONE:        return "VALID";
        case PICOVL6180X_RANGE_ERROR_SYSERR_1:    return "SYSERR_1";
        case PICOVL6180X_RANGE_ERROR_SYSERR_2:    return "SYSERR_2";
        case PICOVL6180X_RANGE_ERROR_SYSERR_3:    return "SYSERR_3";
        case PICOVL6180X_RANGE_ERROR_SYSERR_4:    return "SYSERR_4";
        case PICOVL6180X_RANGE_ERROR_SYSERR_5:    return "SYSERR_5";
        case PICOVL6180X_RANGE_ERROR_ECEFAIL:     return "ECEFAIL";
        case PICOVL6180X_RANGE_ERROR_NOCONVERGE:  return "NO_CONVERGE";
        case PICOVL6180X_RANGE_ERROR_RANGEIGNORE: return "RANGE_IGNORE";
        case PICOVL6180X_RANGE_ERROR_SNR:         return "SNR_LOW";
        case PICOVL6180X_RANGE_ERROR_RAWUFLOW:    return "RAW_UNDERFLOW";
        case PICOVL6180X_RANGE_ERROR_RAWOFLOW:    return "RAW_OVERFLOW";
        case PICOVL6180X_RANGE_ERROR_RANGEUFLOW:  return "RANGE_UNDERFLOW";
        case PICOVL6180X_RANGE_ERROR_RANGEOFLOW:  return "RANGE_OVERFLOW";
        default:                                  return "UNKNOWN";
    }
}

// -------------------- Optional Pico helpers --------------------

#ifndef PICOVL6180X_NO_PICO_HELPERS

picovl6180x_status_t picovl6180x_hw_begin_default(picovl6180x_t *dev,
                                                   i2c_inst_t *i2c,
                                                   uint8_t addr,
                                                   uint sda_pin,
                                                   uint scl_pin,
                                                   uint32_t baud_hz,
                                                   bool enable_pullups)
{
    if (!dev || !i2c) return PICOVL6180X_ERR_INVALID_ARG;
    if (addr > 0x7F) return PICOVL6180X_ERR_INVALID_ARG;
    if (baud_hz == 0) return PICOVL6180X_ERR_INVALID_ARG;

    i2c_init(i2c, baud_hz);

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    if (enable_pullups) {
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);
    }

    return picovl6180x_begin(dev, i2c, addr);
}

picovl6180x_status_t picovl6180x_hw_begin(picovl6180x_t *dev,
                                           i2c_inst_t *i2c,
                                           uint sda_pin,
                                           uint scl_pin)
{
    return picovl6180x_hw_begin_default(dev, i2c,
                                         PICOVL6180X_DEFAULT_ADDR,
                                         sda_pin, scl_pin,
                                         400000u,
                                         true);
}

#endif

