#include "hardware/irq.h"

#include "tps_config.h"

#ifdef LOW_REGION_BIN
#include "low_region_bin.h"
#endif

#include "rp_agrolib_tps25750.h"
#include "rp_agrolib_uart.h"

#define TPS_I2C_CONFIG NEGOTIATE_HIGH_VOLTAGE
#define TPS_I2C_ADDR TPS_INDEX1_ADDR

#define BAUDRATE 100  // KHz
#define SDA_PIN 14
#define SCL_PIN 15
// #define SDA_PIN 0
// #define SCL_PIN 1

#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define START_UPDATE 0
#define READY_FOR_PATCH 1
#define PBMS 2
#define WRITE_PATCH_BUNDLE 3
#define PBMC 4
#define PBME 5
#define VERIFY_PATCH_LOAD 6
#define PATCH_LOADED 7
#define APP_RUN 8

void write(char *data, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        uart_write_char(UART_ID, data[i]);
        // printf("%x\n", data[i]);
    }
}

int main() {
    stdio_init_all();

    gpio_init(SDA_PIN);
    gpio_init(SCL_PIN);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    int uart_irq =
            uart_setup(UART_ID, UART_RX_PIN, UART_TX_PIN, BAUD_RATE, DATA_BITS, STOP_BITS, PARITY);

    i2c_inst_t *i2c = i2c1;
    // i2c_inst_t *i2c = i2c0;
    uint baud = i2c_init(i2c, BAUDRATE * 1000);
    RPTPS25750 tps(i2c);

    // tps.clear_dead_battery_flag(TPS_I2C_ADDR);
    // tps.hard_reset(TPS_I2C_ADDR);
    // tps.go_to_patch_mode(TPS_I2C_ADDR);

    /*sleep_ms(3000);
    scan_bus(i2c);
    sleep_ms(3000);*/

    sleep_ms(2000);

    //------READ LOW REGION BINARY FILE FROM LOW_REGION_BIN.H
    uint8_t *low_region_bin_data = (uint8_t *) tps25750x_lowRegion_i2c_array;
    int low_region_bin_size = gSizeLowRegionArray;

    //------UPDATE PATCH BUNDLE STATE MACHINE
    int state = START_UPDATE;
    bool flag = 0;
    while (true) {
        uint8_t read[100];
        int ready, n, n_bytes_read, n_bytes_written, loaded;
        bool success;
        char print_buf[100];

        switch (state) {
            case START_UPDATE:
                sleep_ms(3000);

                n = tps.get_mode(TPS_I2C_ADDR, read);
                memset(read, 0, sizeof(read));

                n = tps.get_status(TPS_I2C_ADDR, read);
                memset(read, 0, sizeof(read));

                n = tps.get_boot_status(TPS_I2C_ADDR, read);
                memset(read, 0, sizeof(read));

                n = tps.get_intmask1(TPS_I2C_ADDR, read);
                memset(read, 0, sizeof(read));

                n = tps.get_intclear1(TPS_I2C_ADDR, read);
                memset(read, 0, sizeof(read));

                ready = tps.is_ready_for_patch(TPS_I2C_ADDR, read);
                if (ready == 1) {
                    sprintf(print_buf, "\nReady for patch");
                    write(print_buf, strlen(print_buf));
                    state = READY_FOR_PATCH;
                } else
                    sleep_ms(10);

                memset(read, 0, sizeof(read));
                n = tps.get_boot_status(TPS_I2C_ADDR, read);
                std::cout << "Boot status: " << read << std::endl;
                sprintf(print_buf, "\nBoot status: ");
                write(print_buf, strlen(print_buf));
                for (int ii = 0; ii < read[0] + 1; ii++) {
                    sprintf(print_buf, " %x", read[ii]);
                    write(print_buf, strlen(print_buf));
                }

                memset(read, 0, sizeof(read));
                n = tps.get_power_path_status(TPS_I2C_ADDR, read);
                std::cout << "Power path status: " << read << std::endl;
                sprintf(print_buf, "\nPath: ");
                write(print_buf, strlen(print_buf));
                for (int ii = 0; ii < read[0] + 1; ii++) {
                    sprintf(print_buf, " %x", read[ii]);
                    write(print_buf, strlen(print_buf));
                }

                break;

            case READY_FOR_PATCH:
                // sleep_ms(1000);
                n = tps.get_mode(TPS_I2C_ADDR, read);

                if (n != -1) {
                    n_bytes_read = read[0];
                    read[n_bytes_read + 1] = '\0';
                    int res = std::memcmp(&read[1], "PTCH", 4);
                    if (res == 0) {
                        state = PBMS;
                        // tps.clear_dead_battery_flag(TPS_I2C_ADDR);
                    } else
                        sleep_ms(10);
                } else
                    sleep_ms(10);
                break;

            case PBMS:
                sprintf(print_buf, "\nPBMS: ");
                write(print_buf, strlen(print_buf));
                success = tps.start_patch_burst_mode(TPS_I2C_ADDR, low_region_bin_size);
                if (success)
                    state = WRITE_PATCH_BUNDLE;
                else
                    state = START_UPDATE;
                break;

            case WRITE_PATCH_BUNDLE:
                sprintf(print_buf, "\nWrite: ");
                write(print_buf, strlen(print_buf));
                // scan_bus(i2c);
                // sleep_ms(3000);
                // sleep_ms(10);
                success = tps.write_patch_bundle(TPS_BUNDLE_SLAVE_ADDR, low_region_bin_data,
                                                 low_region_bin_size);
                if (success)
                    state = PBMC;
                else
                    state = PBME;
                break;

            case PBMC:
                sprintf(print_buf, "\nComplete: ");
                write(print_buf, strlen(print_buf));
                // sleep_us(500);
                // scan_bus(i2c);
                sleep_ms(1000);
                success = tps.patch_burst_mode_download_complete(TPS_I2C_ADDR);
                if (success) {
                    state = VERIFY_PATCH_LOAD;
                    // success = tps.end_patch_burst_mode(TPS_I2C_ADDR);
                    // if(success)
                    // std::cout << "End success" << std::endl;
                    // std::cout << "GO TO VERIFY PATCH" << std::endl;
                }
                // else
                // state = PBME; //or START_UPDATE?
                break;

            case PBME:
                sprintf(print_buf, "\nPBME: ");
                write(print_buf, strlen(print_buf));
                success = tps.end_patch_burst_mode(TPS_I2C_ADDR);
                state = START_UPDATE;
                break;

            case VERIFY_PATCH_LOAD:  // does not have timeout
                sprintf(print_buf, "\nVerify: ");
                write(print_buf, strlen(print_buf));
                sleep_ms(10);
                loaded = tps.is_patch_loaded(TPS_I2C_ADDR, read);

                if (loaded == 1)
                    state = PATCH_LOADED;
                else
                    sleep_ms(1000);
                break;

            case PATCH_LOADED:
                sprintf(print_buf, "\nPLoaded: ");
                write(print_buf, strlen(print_buf));
                sleep_ms(10);

                /*
                std::cout << std::endl;
                memset(read, 0, sizeof(read));
                n = tps.get_device_capabilities(TPS_I2C_ADDR,read);
                std::cout << "Device Cap: " << read << std::endl;
                memset(read, 0, sizeof(read));
                n = tps.get_boot_status(TPS_I2C_ADDR,read);
                std::cout << "Boot: " << read << std::endl;
                memset(read, 0, sizeof(read));
                n = tps.get_status(TPS_I2C_ADDR,read);
                std::cout << "Status: " << read << std::endl;
                memset(read, 0, sizeof(read));
                n = tps.get_power_path_status(TPS_I2C_ADDR,read);
                std::cout << "Path: " << read << std::endl;
                memset(read, 0, sizeof(read));
                n = tps.get_power_status(TPS_I2C_ADDR,read);
                std::cout << "Power: " << read << std::endl;
                memset(read, 0, sizeof(read));
                n = tps.get_pd_status(TPS_I2C_ADDR,read);
                std::cout << "PD: " << read << std::endl;
                memset(read, 0, sizeof(read));
                n = tps.get_mode(TPS_I2C_ADDR,read);
                std::cout << "Mode: " << read << std::endl;*/

                if (n != -1) {
                    n_bytes_read = read[0];
                    read[n_bytes_read + 1] = '\0';
                    int res = std::memcmp(&read[1], "APP", 3);
                    if (res == 0) {
                        state = APP_RUN;
                        sleep_ms(2000);
                        tps.swap_to_source(TPS_I2C_ADDR);
                    } else
                        sleep_ms(10);
                } else
                    sleep_ms(10);
                break;

            case APP_RUN:
                sleep_ms(1000);
                std::cout << "APP RUN" << std::endl;

                memset(read, 0, sizeof(read));
                n = tps.get_device_capabilities(TPS_I2C_ADDR, read);
                std::cout << "Device Cap: " << read << std::endl;
                sprintf(print_buf, "\nDevice Cap: ");
                write(print_buf, strlen(print_buf));
                for (int ii = 0; ii < read[0] + 1; ii++) {
                    sprintf(print_buf, " %x", read[ii]);
                    write(print_buf, strlen(print_buf));
                }

                memset(read, 0, sizeof(read));
                n = tps.get_boot_status(TPS_I2C_ADDR, read);
                std::cout << "Boot status: " << read << std::endl;
                sprintf(print_buf, "\nBoot status: ");
                write(print_buf, strlen(print_buf));
                for (int ii = 0; ii < read[0] + 1; ii++) {
                    sprintf(print_buf, " %x", read[ii]);
                    write(print_buf, strlen(print_buf));
                }

                memset(read, 0, sizeof(read));
                n = tps.get_typec_state(TPS_I2C_ADDR, read);
                std::cout << "TypeC state: " << read << std::endl;
                sprintf(print_buf, "\nTypeC state: ");
                write(print_buf, strlen(print_buf));
                for (int ii = 0; ii < read[0] + 1; ii++) {
                    sprintf(print_buf, " %x", read[ii]);
                    write(print_buf, strlen(print_buf));
                }

                memset(read, 0, sizeof(read));
                n = tps.get_status(TPS_I2C_ADDR, read);
                std::cout << "Status: " << read << std::endl;
                sprintf(print_buf, "\nStatus:");
                write(print_buf, strlen(print_buf));
                for (int ii = 0; ii < read[0] + 1; ii++) {
                    sprintf(print_buf, " %x", read[ii]);
                    write(print_buf, strlen(print_buf));
                }

                memset(read, 0, sizeof(read));
                n = tps.get_power_path_status(TPS_I2C_ADDR, read);
                std::cout << "Power path status: " << read << std::endl;
                sprintf(print_buf, "\nPath: ");
                write(print_buf, strlen(print_buf));
                for (int ii = 0; ii < read[0] + 1; ii++) {
                    sprintf(print_buf, " %x", read[ii]);
                    write(print_buf, strlen(print_buf));
                }
                if (read[5] == 0x80) {
                    if (flag == 0) {
                        tps.clear_dead_battery_flag(TPS_I2C_ADDR);
                        flag = 1;
                        sprintf(print_buf, "\nClear Flag");
                        write(print_buf, strlen(print_buf));
                    }
                } else {
                    flag = 0;
                }

                memset(read, 0, sizeof(read));
                n = tps.get_power_status(TPS_I2C_ADDR, read);
                std::cout << "Power status: " << read << std::endl;
                sprintf(print_buf, "\nPower: ");
                write(print_buf, strlen(print_buf));
                for (int ii = 0; ii < read[0] + 1; ii++) {
                    sprintf(print_buf, " %x", read[ii]);
                    write(print_buf, strlen(print_buf));
                }
                if (read[1] == 0x0d) {
                    if (flag == 0) {
                        tps.swap_to_ufp(TPS_I2C_ADDR);
                        flag = 1;
                        sprintf(print_buf, "\nSwapUFP ");
                        write(print_buf, strlen(print_buf));
                    }
                } else {
                    flag = 0;
                }

                memset(read, 0, sizeof(read));
                n = tps.get_pd_status(TPS_I2C_ADDR, read);
                std::cout << "PD: " << read << std::endl;
                sprintf(print_buf, "\nPD status: ");
                write(print_buf, strlen(print_buf));
                for (int ii = 0; ii < read[0] + 1; ii++) {
                    sprintf(print_buf, " %x", read[ii]);
                    write(print_buf, strlen(print_buf));
                }


                // uart_write_char(UART_ID, 'a');

                // tps.swap_to_ufp(TPS_I2C_ADDR);

                break;
        }

        memset(read, 0, sizeof(read));
        n = tps.get_mode(TPS_I2C_ADDR, read);
        sprintf(print_buf, "\n\nMode: ");
        write(print_buf, strlen(print_buf));
        for (int ii = 0; ii < read[0] + 1; ii++) {
            sprintf(print_buf, " %c", read[ii]);
            write(print_buf, strlen(print_buf));
        }

        // std::cout << "State: " << state << std::endl;
        // sleep_ms(1);
    }
}
