#include "../include/rp_agrolib_tps25750.h"

/*
-- 1.Each i2c from tps read has:
        byte[0]: return n_bytes read
        byte[1]...byte[n_bytes]: return information register in LSB

-- 2.Device modes:
        APP: PD controller is fully functioning in the app firmware
        BOOT: Device booting in dead battery
        PTCH: Device in patch mode (waiting for patch bundle)

-- 3.In PTCH and BOOT modes only the following registers are available:
        [x] REG_MODE
        [ ] REG_TYPE
        [ ] REG_VERSION
        [ ] REG_CMD1
        [ ] REG_DATA1
        [x] REG_DEV_CAPABILITIES
        [x] REG_INT_EVENT1
        [ ] REG_INT_MASK1
        [ ] REG_INT_CLEAR1
        [x] REG_BOOT_STATUS
        [x] REG_DEV_INFO
        [ ] 4CC tasks
*/

void RPTPS25750::state_machine_it(uint8_t &sm_pd_state, uint8_t *low_region_bin_data,
                                  int low_region_bin_size, int debug_on,
                                  uart_inst_t *DEBUG_UART_ID) {
    uint8_t read[100];
    int ready, n, n_bytes_read, n_bytes_written, loaded;
    bool success;
    char print_buf[100];

    switch (sm_pd_state) {
        case SM_START_UPDATE:
            sleep_ms(3000);

            ready = is_ready_for_patch(TPS_I2C_ADDR, read);
            if (ready == 1) {
                sm_pd_state = SM_READY_FOR_PATCH;
            } else
                sleep_ms(10);

            break;

        case SM_READY_FOR_PATCH:
            n = get_mode(TPS_I2C_ADDR, read);

            if (n != -1) {
                n_bytes_read = read[0];
                read[n_bytes_read + 1] = '\0';
                int res = std::memcmp(&read[1], "PTCH", 4);
                if (res == 0) {
                    sm_pd_state = SM_PBMS;
                } else
                    sleep_ms(10);
            } else
                sleep_ms(10);
            break;

        case SM_PBMS:
            success = start_patch_burst_mode(TPS_I2C_ADDR, low_region_bin_size);
            if (success)
                sm_pd_state = SM_WRITE_PATCH_BUNDLE;
            else
                sm_pd_state = SM_START_UPDATE;
            break;

        case SM_WRITE_PATCH_BUNDLE:
            success = write_patch_bundle(TPS_BUNDLE_SLAVE_ADDR, low_region_bin_data,
                                         low_region_bin_size);
            if (success)
                sm_pd_state = SM_PBMC;
            else
                sm_pd_state = SM_PBME;
            break;

        case SM_PBMC:
            sleep_ms(1000);
            success = patch_burst_mode_download_complete(TPS_I2C_ADDR);
            if (success) {
                sm_pd_state = SM_VERIFY_PATCH_LOAD;
                // success = end_patch_burst_mode(TPS_I2C_ADDR);
            }
            // else
            // sm_pd_state = PBME; //or START_UPDATE?
            break;

        case SM_PBME:
            success = end_patch_burst_mode(TPS_I2C_ADDR);
            sm_pd_state = SM_START_UPDATE;
            break;

        case SM_VERIFY_PATCH_LOAD:  // does not have timeout
            sleep_ms(10);
            loaded = is_patch_loaded(TPS_I2C_ADDR, read);

            if (loaded == 1)
                sm_pd_state = SM_PATCH_LOADED;
            else
                sleep_ms(1000);
            break;

        case SM_PATCH_LOADED:
            sleep_ms(10);

            memset(read, 0, sizeof(read));
            n = get_mode(TPS_I2C_ADDR, read);

            if (n != -1) {
                n_bytes_read = read[0];
                read[n_bytes_read + 1] = '\0';
                int res = std::memcmp(&read[1], "APP", 3);
                if (res == 0) {
                    sm_pd_state = SM_APP_RUN;
                    sleep_ms(2000);
                    swap_to_source(TPS_I2C_ADDR);
                } else
                    sleep_ms(10);
            } else
                sleep_ms(10);
            break;

        case SM_APP_RUN:
            if (is_set_dead_battery_flag(TPS_I2C_ADDR, read) == 1) {
                clear_dead_battery_flag(TPS_I2C_ADDR);
                if (debug_on) {
                    uart_write_string(DEBUG_UART_ID, (char *) "\n[INFO] Clear Flag");
                }
            }

            if (is_data_role(TPS_I2C_ADDR, read) == 1) {
                swap_to_ufp(TPS_I2C_ADDR);
                if (debug_on) {
                    uart_write_string(DEBUG_UART_ID, (char *) "\n[INFO] Swap to UFP");
                }
            }

            if (debug_on) {
                debug(TPS_I2C_ADDR, REG_DEV_CAPABILITIES, DEBUG_UART_ID);
                debug(TPS_I2C_ADDR, REG_STATUS, DEBUG_UART_ID);
                debug(TPS_I2C_ADDR, REG_POWER_PATH_STATUS, DEBUG_UART_ID);
                debug(TPS_I2C_ADDR, REG_BOOT_STATUS, DEBUG_UART_ID);
                debug(TPS_I2C_ADDR, REG_PWR_STATUS, DEBUG_UART_ID);
                debug(TPS_I2C_ADDR, REG_PD_STATUS, DEBUG_UART_ID);
                debug(TPS_I2C_ADDR, REG_TYPEC_STATE, DEBUG_UART_ID);
            }

            break;
    }

    if (debug_on) {
        debug(TPS_I2C_ADDR, REG_MODE, DEBUG_UART_ID);
        sprintf(print_buf, "\n\n[INFO] State: %d", sm_pd_state);
        uart_write_string(DEBUG_UART_ID, print_buf);
    }
}

int RPTPS25750::read_tps(uint8_t dev_addr, uint8_t reg_addr, uint8_t *i2cread, int n) {
    // Should be read number of register bytes (n) +1
    // because the first byte returned is the number of bytes read
    int n_bytes_read = reg_read(i2c, dev_addr, reg_addr, i2cread, n + 1);
    if (n_bytes_read == PICO_ERROR_GENERIC || n_bytes_read < n + 1) {
        return -1;
    }
    return n_bytes_read;
}

int RPTPS25750::write_tps(uint8_t dev_addr, uint8_t reg_addr, uint8_t *i2cwrite, int n) {
    // n data bytes as argument (excluding reg_addr)
    uint8_t data[n + 1];
    data[0] = n;
    memcpy(&data[1], i2cwrite, n);
    int n_bytes_written = reg_write(i2c, dev_addr, reg_addr, data, n + 1);
    if (n_bytes_written == PICO_ERROR_GENERIC) {
        return -1;
    }
    return n_bytes_written;
}

int RPTPS25750::write_tps1(uint8_t dev_addr, uint8_t reg_addr, uint8_t *i2cwrite, int n) {
    int n_bytes_written = reg_write_raw(i2c, dev_addr, reg_addr, i2cwrite, n);
    if (n_bytes_written == PICO_ERROR_GENERIC) {
        return -1;
    }
    return n_bytes_written;
}

int RPTPS25750::debug(uint8_t dev_addr, uint8_t reg_addr, uart_inst_t *bus_id) {
    int n_bytes_read = 0;
    uint8_t read[100];
    char print_buf[100];
    uint8_t print_char;

    memset(read, 0, sizeof(read));
    memset(read, 0, sizeof(print_buf));

    switch (reg_addr) {
        case REG_MODE:
            n_bytes_read = get_mode(dev_addr, read);
            sprintf(print_buf, "\n[0x03] Mode: ");
            break;

        case REG_DEV_CAPABILITIES:
            n_bytes_read = get_device_capabilities(dev_addr, read);
            sprintf(print_buf, "\n[0x0D] Device Capabilities: ");
            break;

        case REG_STATUS:
            n_bytes_read = get_status(dev_addr, read);
            sprintf(print_buf, "\n[0x1A] Status: ");
            break;

        case REG_POWER_PATH_STATUS:
            n_bytes_read = get_power_path_status(dev_addr, read);
            sprintf(print_buf, "\n[0x26] Power Path Status: ");
            break;

        case REG_BOOT_STATUS:
            n_bytes_read = get_boot_status(dev_addr, read);
            sprintf(print_buf, "\n[0x2D] Boot Status: ");
            break;

        case REG_PWR_STATUS:
            n_bytes_read = get_power_status(dev_addr, read);
            sprintf(print_buf, "\n[0x3F] Power Status: ");
            break;

        case REG_PD_STATUS:
            n_bytes_read = get_pd_status(dev_addr, read);
            sprintf(print_buf, "\n[0x40] PD Status: ");
            break;

        case REG_TYPEC_STATE:
            n_bytes_read = get_typec_state(dev_addr, read);
            sprintf(print_buf, "\n[0x69] TypeC State: ");
            break;
    }

    uart_write_string(bus_id, print_buf);

    for (int ii = 0; ii < read[0] + 1; ii++) {
        sprintf(print_buf, "0x%x ", read[ii]);
        uart_write_string(bus_id, print_buf);
    }

    return n_bytes_read;
}

int RPTPS25750::get_mode(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_MODE, i2cread, N_BYTES_MODE);
    return n;
}

// Get hardware and firmware version information of the PD controller
int RPTPS25750::get_device_info(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_DEV_INFO, i2cread, N_BYTES_DEV_INFO);
    return n;
}

// Power Status register is not available while in PTCH mode
int RPTPS25750::get_power_status(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_PWR_STATUS, i2cread, N_BYTES_PWR_STATUS);
    return n;
}

int RPTPS25750::get_device_capabilities(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_DEV_CAPABILITIES, i2cread, N_BYTES_DEV_CAPABILITIES);
    return n;
}

int RPTPS25750::get_boot_status(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_BOOT_STATUS, i2cread, N_BYTES_BOOT_STATUS);
    return n;
}

int RPTPS25750::get_status(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_STATUS, i2cread, N_BYTES_STATUS);
    return n;
}

int RPTPS25750::get_power_path_status(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_POWER_PATH_STATUS, i2cread, N_BYTES_POWER_PATH_STATUS);
    return n;
}

int RPTPS25750::get_pd_status(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_PD_STATUS, i2cread, N_BYTES_PD_STATUS);
    return n;
}

int RPTPS25750::get_typec_state(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_TYPEC_STATE, i2cread, N_BYTES_TYPEC_STATE);
    return n;
}

void RPTPS25750::handle_power_status(uint8_t *get) {
    uint8_t pwr_connection = get[1] & 0x01;
    uint8_t src_sink = (get[1] >> 1) & 0x01;
    uint8_t typec_current = (get[1] >> 2) & 0x03;
    uint8_t chrg_detect_status = (get[1] >> 4) & 0x0F;
    uint8_t chrg_advertise_status = get[2] & 0x03;

    switch (pwr_connection) {
        case 0:
            std::cout << "No connection" << std::endl;
            break;
        case 1:
            std::cout << "Connection ok" << std::endl;
            break;
    }

    if (pwr_connection == 1) {
        switch (src_sink) {
            case 0:
                std::cout << "Connection requests power: PD controller is the source" << std::endl;
                break;
            case 1:
                std::cout << "Connection provides power: PD controler as sink" << std::endl;
                break;
        }
        switch (typec_current) {
            case 0:
                std::cout << "USB default current" << std::endl;
                break;
            case 1:
                std::cout << "1.5A" << std::endl;
                break;
            case 2:
                std::cout << "3.0A" << std::endl;
                break;
            case 3:
                std::cout << "Explicit PD contract sets current" << std::endl;
                break;
        }
        switch (chrg_detect_status) {
            case 0:
                std::cout << "Charge detection disabled or not run" << std::endl;
                break;
            case 1:
                std::cout << "Charger detection in progress" << std::endl;
                break;
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                std::cout << "Charger detection completed" << std::endl;
                break;
        }
        switch (chrg_advertise_status) {
            case 0:
                std::cout << "Charger advertise disabled or not run" << std::endl;
                break;
            case 1:
                std::cout << "Charger advertisement in process" << std::endl;
                break;
            case 2:
                std::cout << "Charger advertisement completed" << std::endl;
                break;
        }
    } else {
        std::cout << "All other status bits are not valid" << std::endl;
    }
}

void RPTPS25750::handle_device_capabilities(uint8_t *get) {
    // get[0] is n_bytes read
    uint8_t pwr_role = get[1] & 0x03;
    uint8_t usb_pd_capability = (get[1] >> 2) & 0x01;

    switch (pwr_role) {
        case 0:
            std::cout << "Both source and sink roles supported (DRP)" << std::endl;
            break;
        case 1:
            std::cout << "Source-only supported" << std::endl;
            break;
        case 2:
            std::cout << "No information about power role capability" << std::endl;
            break;
        case 3:
            std::cout << "Source-only supported" << std::endl;
            break;
    }
    switch (usb_pd_capability) {
        case 0:
            std::cout << "USB Power Delivery supported" << std::endl;
            break;
        case 1:
            std::cout << "USB Power Delivery not supported" << std::endl;
            break;
    }
}

void RPTPS25750::handle_boot_status(uint8_t *get) {
    // get[0] is n_bytes read
    uint8_t patch_config_source = (get[4] >> 5) & 0x07;
    uint8_t patch_download_error = (get[2] >> 2) & 0x01;
    uint8_t patch_header_error = get[1] & 0x01;

    switch (patch_config_source) {
        case 0:
            std::cout << "No configuration has been loaded" << std::endl;
            break;
        case 5:
            std::cout << "A configuration has been loaded from EEPROM" << std::endl;
            break;
        case 6:
            std::cout << "A configuration has been loaded from I2C" << std::endl;
            break;
    }
    switch (patch_download_error) {
        case 0:
            std::cout << "Patch download completed successfully" << std::endl;
            break;
        case 1:
            std::cout << "Patch download error occurs" << std::endl;
            break;
    }
    switch (patch_header_error) {
        case 0:
            std::cout << "Patch bundle header ok" << std::endl;
            break;
        case 1:
            std::cout << "Patch bundle header error" << std::endl;
            break;
    }
}

int RPTPS25750::is_ready_for_patch(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_INT_EVENT1, i2cread, N_BYTES_INT_EVENT1);

    /*for(int i=0;i<n;i++){
        printf("%x ", i2cread[i]);
    }*/

    if (n != -1) {
        // Read bit 1 of byte 11 (ReadyForPatch)
        uint8_t ready_for_patch_bit = (i2cread[11] >> 1) & 0x01;
        return ready_for_patch_bit;
    }
    return n;
}

int RPTPS25750::is_set_dead_battery_flag(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_BOOT_STATUS, i2cread, N_BYTES_BOOT_STATUS);
    if (n != -1) {
        // Read bit 2 of byte 1 (DeadBatteryFlag)
        uint8_t dead_battery_flag_bit = (i2cread[1] >> 2) & 0x01;
        return dead_battery_flag_bit;
    }
    return n;
}

int RPTPS25750::is_data_role(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_STATUS, i2cread, N_BYTES_STATUS);
    if (n != -1) {
        // Read bit 6 of byte 1 (DataRole)
        uint8_t data_role_bit = (i2cread[1] >> 6) & 0x01;
        return data_role_bit;
    }
    return n;
}

int RPTPS25750::write_input_data(uint8_t dev_addr, uint8_t *i2cwrite, int size) {
    int n_bytes_written = 0;
    if (i2cwrite == NULL) {
        uint8_t dummy[2] = {};
        // n_bytes_written = write_tps1(dev_addr, REG_DATA1, dummy, sizeof(dummy));
    } else {
        n_bytes_written = write_tps(dev_addr, REG_DATA1, i2cwrite, size);
        // printf("[WRITE][%d] %x %x ",n_bytes_written, REG_DATA1,size);
    }

    /*for(int i=0;i<n_bytes_written-2;i++){
        printf("%x ", i2cwrite[i]);
    }
    printf("\n");*/
    return n_bytes_written;
}

int RPTPS25750::write_command(uint8_t dev_addr, uint8_t *i2cwrite) {
    int n_bytes_written = write_tps(dev_addr, REG_CMD1, i2cwrite, 4);
    /*printf("[WRITE][%d] %x %x ",n_bytes_written, REG_CMD1, 4);
    for(int i=0;i<n_bytes_written-2;i++){
        printf("%x ", i2cwrite[i]);
    }
    printf("\n");*/
    return n_bytes_written;
}

int RPTPS25750::get_command(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_CMD1, i2cread, N_BYTES_CMD1);
    /*printf("[WRITE][%d] %x\n",1,REG_CMD1);
    printf("[READ][%d] ",n);
    for(int i=0;i<n;i++){
        printf("%x ", i2cread[i]);
    }
    printf("\n");*/
    return n;
}

int RPTPS25750::get_data(uint8_t dev_addr, uint8_t *i2cread, int size) {
    int n = read_tps(dev_addr, REG_DATA1, i2cread, size);
    /*printf("[WRITE][%d] %x\n",1,REG_DATA1);
    printf("[READ][%d] ",n);
    for(int i=0;i<n;i++){
        printf("%x ", i2cread[i]);
    }
    printf("\n");*/
    return n;
}

bool RPTPS25750::execute_4CC_command(uint8_t dev_addr, uint8_t *cmd, uint8_t *input, int in_size,
                                     uint8_t *output, int out_size) {
    int n_data_written = write_input_data(dev_addr, input, in_size);
    if (n_data_written == -1)
        return false;
    // sleep_ms(10);

    /* uint8_t read[100];
    int n = get_mode(TPS_INDEX1_ADDR,read); */

    int n_cmd_written = write_command(dev_addr, cmd);
    // sleep_ms(10);
    uint8_t cmd_read[N_BYTES_CMD1 + 1];
    bool cmd_success = false;

    int timeout = 500;  // milliseconds
    while (!cmd_success) {

        // uint8_t read[100];
        // int n = get_mode(TPS_INDEX1_ADDR,read);
        // std::cout << "Read: " << read << std::endl;

        // std::memset(cmd_read, 0, sizeof(cmd_read));
        // std::cout << "Get command" << std::endl;
        int n_cmd_read = get_command(dev_addr, cmd_read);
        uint8_t empty[4] = {0x00, 0x00, 0x00, 0x00};
        // std::cout << "CMD read: " << cmd_read << std::endl;

        /* if(std::memcmp(&read[1],"APP",3)==0){
            cmd_success = true;
        }
        else */
        if (std::memcmp(&cmd_read[1], "!CMD", 4) == 0) {
            std::cout << "!CMD " << cmd_read << std::endl;
            // return false;
        } else if (std::memcmp(&cmd_read[1], empty, 4) == 0) {
            std::cout << "CMD SUCCESS" << std::endl;
            cmd_success = true;
        }
        if (timeout == 0) {
            std::cout << "TIMEOUT" << std::endl;
            return false;
        }
        timeout--;
        sleep_ms(10);
    }

    // std::cout << "Get data: " << cmd_read << std::endl;
    int n_data_read = get_data(dev_addr, output, out_size);
    if (n_data_read == -1) {
        std::cout << "DATA FAILED" << std::endl;
        return false;
    }
    sleep_ms(10);

    return true;
}

bool RPTPS25750::swap_to_dfp(uint8_t dev_addr) {
    uint8_t *input = NULL;
    uint8_t output[2];

    bool success = execute_4CC_command(dev_addr, (uint8_t *) "SWDF", input, 2, output, 2);
    uint8_t taskresult = output[1] & 0x0F;

    switch (taskresult) {
        case 0:
            std::cout << "Task completed successfully" << std::endl;
            break;
        case 1:
            std::cout << "Task timed-out or aborted by 'ABRT' request" << std::endl;
            break;
        case 3:
            std::cout << "Task rejected" << std::endl;
            break;
        case 4:
            std::cout << "Task rejected because the Rx buffer was locked" << std::endl;
            break;
    }

    if (!success)
        return false;
    else
        return true;
}

bool RPTPS25750::swap_to_ufp(uint8_t dev_addr) {
    uint8_t *input = NULL;
    uint8_t output[2];

    bool success = execute_4CC_command(dev_addr, (uint8_t *) "SWUF", input, 2, output, 2);
    uint8_t taskresult = output[1] & 0x0F;

    switch (taskresult) {
        case 0:
            std::cout << "Task completed successfully" << std::endl;
            break;
        case 1:
            std::cout << "Task timed-out or aborted by 'ABRT' request" << std::endl;
            break;
        case 3:
            std::cout << "Task rejected" << std::endl;
            break;
        case 4:
            std::cout << "Task rejected because the Rx buffer was locked" << std::endl;
            break;
    }

    if (!success)
        return false;
    else
        return true;
}

bool RPTPS25750::swap_to_source(uint8_t dev_addr) {
    uint8_t *input = NULL;
    uint8_t output[2];

    bool success = execute_4CC_command(dev_addr, (uint8_t *) "SWSr", input, 2, output, 2);
    uint8_t taskresult = output[1] & 0x0F;

    switch (taskresult) {
        case 0:
            std::cout << "Task completed successfully" << std::endl;
            break;
        case 1:
            std::cout << "Task timed-out or aborted by 'ABRT' request" << std::endl;
            break;
        case 3:
            std::cout << "Task rejected" << std::endl;
            break;
        case 4:
            std::cout << "Task rejected because the Rx buffer was locked" << std::endl;
            break;
    }

    if (!success)
        return false;
    else
        return true;
}

bool RPTPS25750::start_patch_burst_mode(uint8_t dev_addr, uint32_t bundle_size) {
    /*
    -- INPUT DATAX
        bytes[0-3]: Low Region Binary bundle size in bytes (assuming little-endian)
        byte[4]: I2C bundle slave address (different from address defined by ADCINx pins)
        byte[5]: Burst mode timeout
    */
    uint8_t input[6];
    input[0] = (uint8_t) (bundle_size & 0xFF);  // LSB
    input[1] = (uint8_t) ((bundle_size >> 8) & 0xFF);
    input[2] = (uint8_t) ((bundle_size >> 16) & 0xFF);
    input[3] = (uint8_t) ((bundle_size >> 24) & 0xFF);
    input[4] = TPS_BUNDLE_SLAVE_ADDR;
    input[5] = TPS_BUNDLE_TIMEOUT;

    uint8_t output[43];
    bool success = execute_4CC_command(dev_addr, (uint8_t *) "PBMs", input, 6, output, 41);
    uint8_t patch_start_status = output[1];

    switch (patch_start_status) {
        case 0:
            // std::cout << "Patch start success" << std::endl;
            break;
        case 4:
            std::cout << "Invalid bundle size" << std::endl;
            break;
        case 5:
            std::cout << "Invalid slave address" << std::endl;
            break;
        case 6:
            std::cout << "Invalid timeout value" << std::endl;
            break;
    }

    // std::cout << "return" << std::endl;
    // std::cout << "success: " << success << std::endl;

    if (!success)
        return false;
    else
        return true;
}

bool RPTPS25750::patch_burst_mode_download_complete(uint8_t dev_addr) {
    uint8_t *input = NULL;
    uint8_t output[43];
    bool success = execute_4CC_command(dev_addr, (uint8_t *) "PBMc", input, 0, output, 41);

    // printf("Patch burst mode download complete output %x %x\n", output[0], output[3]);

    if (!success)
        return false;
    else
        return true;
}

bool RPTPS25750::write_patch_bundle(uint8_t patch_addr, uint8_t *data, int size) {
    // std::cout << "write patch" << std::endl;
    int n = write_tps1(patch_addr, data[0], &data[1], size - 1);
    // std::cout << "n: " << n << std::endl;
    if (n == -1)
        return false;
    // printf("[WRITE][%d] ",n);
    /* for(int i=0;i<n;i++){
        printf("%x ", data[i]);
    }
    printf("\n"); */

    // printf("Patch bundle bytes written: %d\n",n);
    return true;
}

bool RPTPS25750::end_patch_burst_mode(uint8_t dev_addr) {
    uint8_t *input = NULL;
    uint8_t output[2];
    bool success = execute_4CC_command(dev_addr, (uint8_t *) "PBMe", input, 0, output, 2);
    uint8_t taskresult = output[1] & 0x0F;

    switch (taskresult) {
        case 0:
            // std::cout << "Task completed successfully" << std::endl;
            break;
        case 1:
            std::cout << "Task timed-out or aborted by 'ABRT' request" << std::endl;
            break;
        case 3:
            std::cout << "Task rejected" << std::endl;
            break;
        case 4:
            std::cout << "Task rejected because the Rx buffer was locked" << std::endl;
            break;
    }

    if (!success)
        return false;
    else
        return true;
}

bool RPTPS25750::clear_dead_battery_flag(uint8_t dev_addr) {
    uint8_t *input = NULL;
    uint8_t output[2];
    bool success = execute_4CC_command(dev_addr, (uint8_t *) "DBfg", input, 0, output, 0);

    if (!success)
        return false;
    else
        return true;
}

bool RPTPS25750::go_to_patch_mode(uint8_t dev_addr) {
    uint8_t *input = NULL;
    uint8_t output[2];
    bool success = execute_4CC_command(dev_addr, (uint8_t *) "GO2P", input, 0, output, 2);
    uint8_t taskresult = output[1] & 0x0F;

    switch (taskresult) {
        case 0:
            std::cout << "Task completed successfully" << std::endl;
            break;
        case 1:
            std::cout << "Task timed-out or aborted by 'ABRT' request" << std::endl;
            break;
        case 3:
            std::cout << "Task rejected" << std::endl;
            break;
        case 4:
            std::cout << "Task rejected because the Rx buffer was locked" << std::endl;
            break;
    }

    if (!success)
        return false;
    else
        return true;
}

bool RPTPS25750::hard_reset(uint8_t dev_addr) {
    uint8_t *input = NULL;
    uint8_t output[2];
    bool success = execute_4CC_command(dev_addr, (uint8_t *) "PTCr", input, 0, output, 2);
    uint8_t taskresult = output[1] & 0x0F;

    switch (taskresult) {
        case 0:
            std::cout << "Task completed successfully" << std::endl;
            break;
        case 1:
            std::cout << "Task timed-out or aborted by 'ABRT' request" << std::endl;
            break;
        case 3:
            std::cout << "Task rejected" << std::endl;
            break;
        case 4:
            std::cout << "Task rejected because the Rx buffer was locked" << std::endl;
            break;
    }

    if (!success)
        return false;
    else
        return true;
}

int RPTPS25750::is_patch_loaded(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_INT_EVENT1, i2cread, N_BYTES_INT_EVENT1);
    /*printf("[WRITE][%d] %x\n",1,REG_INT_EVENT1);
    printf("[READ][%d] ",n);
    for(int i=0;i<n;i++){
        printf("%x ", i2cread[i]);
    }
    printf("\n");*/
    if (n != -1) {
        // Read bit 1 of byte 11 (ReadyForPatch)
        uint8_t patch_loaded = i2cread[11] & 0x01;
        return patch_loaded;
    }
    return n;
}

int RPTPS25750::get_intevent1(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_INT_EVENT1, i2cread, N_BYTES_INT_EVENT1);
    /*printf("[WRITE][%d] %x\n",1,REG_INT_EVENT1);
    printf("[READ][%d] ",n);
    for(int i=0;i<n;i++){
        printf("%x ", i2cread[i]);
    }
    printf("\n");*/
    return n;
}

int RPTPS25750::get_intmask1(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_INT_MASK1, i2cread, N_BYTES_INT_MASK1);
    /*printf("[WRITE][%d] %x\n",1,REG_INT_MASK1);
    printf("[READ][%d] ",n);
    for(int i=0;i<n;i++){
        printf("%x ", i2cread[i]);
    }
    printf("\n");*/
    return n;
}

int RPTPS25750::get_intclear1(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_INT_CLEAR1, i2cread, N_BYTES_INT_CLEAR1);
    /*printf("[WRITE][%d] %x\n",1,REG_INT_CLEAR1);
    printf("[READ][%d] ",n);
    for(int i=0;i<n;i++){
        printf("%x ", i2cread[i]);
    }
    printf("\n");*/
    return n;
}

int RPTPS25750::check_usb_connection(uint8_t dev_addr, uint8_t *i2cread) {
    int n = read_tps(dev_addr, REG_PWR_STATUS, i2cread, N_BYTES_PWR_STATUS);
    if (n != -1) {
        // Bit 0 of byte 1 indicates USB connection
        uint8_t pwr_connection = i2cread[1] & 0x01;
        return pwr_connection;  // 1 = connected, 0 = disconnected
    }
    return -1;  // error
}
