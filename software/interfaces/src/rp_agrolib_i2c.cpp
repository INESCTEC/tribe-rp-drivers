#include "../include/rp_agrolib_i2c.h"

// Write 1 byte to the specified register
int reg_write(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf,
              const uint8_t nbytes) {

    int num_bytes_written = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    memcpy(&msg[1], buf, nbytes);

    // Write data to register(s) over I2C
    num_bytes_written = i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

    return num_bytes_written;
}

// Write 1 byte to the specified register
int reg_write_raw(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf, int nbytes) {

    int num_bytes_written = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 1; i <= nbytes; i++) {
        msg[i] = buf[i - 1];
    }

    // Write data to register(s) over I2C
    num_bytes_written = i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

    return num_bytes_written;
}

// Write a set of bytes to the specified register with timeout in us
// The time that the function will wait for the entire transaction to complete.
// Note, an individual timeout of this value divided by the length of data is applied for each byte
// transfer. So if the first or subsequent bytes fails to transfer within that sub timeout, the
// function will return with an error.
int reg_write_timeout(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *data,
                      const uint8_t nbytes, uint timeout_us) {
    int num_bytes_written = 0;
    uint8_t msg[nbytes + 1];

    if (nbytes < 1)
        return 0;

    msg[0] = reg;
    memcpy(&msg[1], data, nbytes);

    num_bytes_written = i2c_write_timeout_us(i2c, addr, msg, (nbytes + 1), false, timeout_us);

    return num_bytes_written;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf,
             const uint8_t nbytes) {

    int num_bytes_read = 0;
    int num_bytes_written = 0;

    if (nbytes < 1) {
        return 0;
    }

    num_bytes_written = i2c_write_blocking(i2c, addr, &reg, 1, true);
    // sleep_us(20);
    int timeout = 1000;
    do {
        num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);
        timeout--;
        if (timeout == 0)
            break;
        sleep_ms(1);
    } while (num_bytes_read == PICO_ERROR_GENERIC || num_bytes_read < nbytes);

    return num_bytes_read;
}

// Read a set of bytes from the specified register with timeout in us
// Timeout is the time that the function will wait for the entire transaction to complete
int reg_read_timeout(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *data,
                     const uint8_t nbytes, uint timeout_us) {

    int num_bytes_read = 0;
    int num_bytes_written = 0;

    if (nbytes < 1)
        return 0;

    uint32_t half_timeout_us = timeout_us / 2;

    num_bytes_written = i2c_write_timeout_us(i2c, addr, &reg, 1, false, timeout_us);
    if (num_bytes_written == PICO_ERROR_GENERIC) {
        printf("PICO_ERROR_GENERIC:%d\n", PICO_ERROR_GENERIC);
        return PICO_ERROR_GENERIC;
    } else if (num_bytes_written == PICO_ERROR_TIMEOUT) {
        printf("PICO_ERROR_TIMEOUT:%d\n", PICO_ERROR_TIMEOUT);
        return PICO_ERROR_TIMEOUT;
    }

    int timeout = 1000;
    do {
        num_bytes_read = i2c_read_timeout_us(i2c, addr, data, nbytes, false, timeout_us);
        timeout--;
        if (timeout == 0)
            break;
        sleep_ms(1);
    } while (num_bytes_read == PICO_ERROR_GENERIC || num_bytes_read == PICO_ERROR_TIMEOUT);


    printf("num bytes written:%d\n", num_bytes_written);
    printf("num bytes read:%d\n", num_bytes_read);
    if (num_bytes_read == PICO_ERROR_GENERIC) {
        printf("PICO_ERROR_GENERIC:%d\n", PICO_ERROR_GENERIC);
        return PICO_ERROR_GENERIC;
    } else if (num_bytes_read == PICO_ERROR_TIMEOUT) {
        printf("PICO_ERROR_TIMEOUT:%d\n", PICO_ERROR_TIMEOUT);
        return PICO_ERROR_TIMEOUT;
    }

    return 0;
}

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void scan_bus(i2c_inst_t *i2c) {
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr)) {
            ret = PICO_ERROR_GENERIC;
        } else {
            ret = i2c_read_blocking(i2c, addr, &rxdata, 1, false);
        }
        sleep_ms(1);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
}


/************************************/
/******i2c 0/1 setup functions*******/
/************************************/

// example:
// i2c_inst_t* i2c_1 = init_i2c1(14, 15, 100 * 1000, true);

i2c_inst_t *init_i2c0(const uint8_t sda_pin, const uint8_t scl_pin, const uint32_t baudrate,
                      const bool internal_pullups) {
    i2c_inst_t *i2c = i2c0;

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    if (internal_pullups) {
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);
    }

    i2c_init(i2c, baudrate);

    return i2c;
}

i2c_inst_t *init_i2c1(const uint8_t sda_pin, const uint8_t scl_pin, const uint32_t baudrate,
                      const bool internal_pullups) {
    i2c_inst_t *i2c = i2c1;

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    if (internal_pullups) {
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);
    }

    i2c_init(i2c, baudrate);

    return i2c;
}

bool i2c_setup(i2c_inst_t *i2c_id, const uint8_t sda_pin, const uint8_t scl_pin,
               const uint32_t baudrate, const bool internal_pullups) {
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    if (internal_pullups) {
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);
    }

    i2c_init(i2c_id, baudrate);

    return true;
}
/************************************/
/************************************/
/************************************/
