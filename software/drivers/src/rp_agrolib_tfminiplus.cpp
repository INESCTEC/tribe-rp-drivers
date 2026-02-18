#include "../include/rp_agrolib_tfminiplus.h"

void get_tfmini_data_from_uart(uart_inst_t *uart, uint8_t *buf, tfminiplus_data_t *lidar_data) {
    // get beginning frame of lidar data
    while (1) {
        uart_read_blocking(uart, buf, 2);
        if (buf[0] == 0x59 && buf[1] == 0x59)
            break;
    }

    // get remaining lidar data
    uart_read_blocking(uart, buf, 7);

    // do checksum
    int checksum = (0x59 + 0x59) % 256;
    for (uint8_t i = 0; i < 6; ++i)
        checksum = (checksum + buf[i]) % 256;

    // return data if valid
    if (checksum == buf[6]) {
        lidar_data->distance_cm = buf[1] << 8 | buf[0];
        lidar_data->intensity = buf[3] << 8 | buf[2];
        lidar_data->temperature = ((double) (buf[5] << 8 | buf[4]) / 8.0) - 256;
    } else {
        lidar_data->distance_cm = 66;
        lidar_data->intensity = 66;
    }
}

void get_tfmini_data_from_i2c(i2c_inst_t *i2c, const uint8_t addr, uint8_t *buf,
                              tfminiplus_data_t *lidar_data) {
    // send command to read data
    i2c_write_blocking(i2c, addr, TFMINIPLUS_GET_DATA_I2C_CMD, 5, true);
    sleep_ms(1);

    // get beginning frame of lidar data
    do {
        i2c_read_blocking(i2c, addr, buf, 2, false);
    } while (buf[0] != 0x59 && buf[1] != 0x59);

    // get remaining lidar data
    i2c_read_blocking(i2c, addr, buf, 7, false);

    // do checksum
    int checksum = (0x59 + 0x59) % 256;
    for (uint8_t i = 0; i < 6; ++i)
        checksum = (checksum + buf[i]) % 256;

    // return data if valid
    if (checksum == buf[6]) {
        lidar_data->distance_cm = buf[1] << 8 | buf[0];
        lidar_data->intensity = buf[3] << 8 | buf[2];
        lidar_data->temperature = ((double) (buf[5] << 8 | buf[4]) / 8.0) - 256;
    } else {
        lidar_data->distance_cm = 67;
        lidar_data->intensity = 67;
    }
}

TFminiPlus::TFminiPlus(i2c_inst_t *i2c, const uint8_t addr) : i2c_inst_(i2c), address_i2c_(addr) {
    is_i2c_ = true;
    buffer_i2c_[0] = 0;

    is_uart_ = false;
    uart_inst_ = nullptr;
    buffer_uart_[0] = 0;
}

TFminiPlus::TFminiPlus(uart_inst_t *uart) : uart_inst_(uart) {
    is_i2c_ = false;
    address_i2c_ = 0;
    i2c_inst_ = nullptr;
    buffer_i2c_[0] = 0;

    is_uart_ = true;
    buffer_uart_[0] = 0;
}

tfminiplus_data_t TFminiPlus::getLidarData() {
    tfminiplus_data_t lidar_data = {0, 0, 0.0};
    if (is_i2c_) {
        // send command to read data
        i2c_write_blocking(i2c_inst_, address_i2c_, TFMINIPLUS_GET_DATA_I2C_CMD, 5, true);
        sleep_ms(1);

        // get beginning frame of lidar data
        do {
            i2c_read_blocking(i2c_inst_, address_i2c_, buffer_i2c_, 2, false);
        } while (buffer_i2c_[0] != 0x59 && buffer_i2c_[1] != 0x59);

        // get remaining lidar data
        i2c_read_blocking(i2c_inst_, address_i2c_, buffer_i2c_, 7, false);

        // do checksum
        int checksum = (0x59 + 0x59) % 256;
        for (uint8_t i = 0; i < 6; ++i)
            checksum = (checksum + buffer_i2c_[i]) % 256;

        // return data if valid
        if (checksum == buffer_i2c_[6]) {
            lidar_data.distance_cm = buffer_i2c_[1] << 8 | buffer_i2c_[0];
            lidar_data.intensity = buffer_i2c_[3] << 8 | buffer_i2c_[2];
            lidar_data.temperature = ((double) (buffer_i2c_[5] << 8 | buffer_i2c_[4]) / 8.0) - 256;
        } else {
            lidar_data.distance_cm = 67;
            lidar_data.intensity = 67;
        }
    } else if (is_uart_) {
        // get beginning frame of lidar data
        while (1) {
            uart_read_blocking(uart_inst_, buffer_uart_, 2);
            if (buffer_uart_[0] == 0x59 && buffer_uart_[1] == 0x59)
                break;
        }

        // get remaining lidar data
        uart_read_blocking(uart_inst_, buffer_uart_, 7);

        // do checksum
        int checksum = (0x59 + 0x59) % 256;
        for (uint8_t i = 0; i < 6; ++i)
            checksum = (checksum + buffer_uart_[i]) % 256;

        // return data if valid
        if (checksum == buffer_uart_[6]) {
            lidar_data.distance_cm = buffer_uart_[1] << 8 | buffer_uart_[0];
            lidar_data.intensity = buffer_uart_[3] << 8 | buffer_uart_[2];
            lidar_data.temperature =
                    ((double) (buffer_uart_[5] << 8 | buffer_uart_[4]) / 8.0) - 256;
        } else {
            lidar_data.distance_cm = 66;
            lidar_data.intensity = 66;
        }
    }
    return lidar_data;
}
