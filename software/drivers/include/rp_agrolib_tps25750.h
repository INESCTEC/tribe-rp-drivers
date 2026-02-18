#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "rp_agrolib_i2c.h"
#include "rp_agrolib_uart.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>

#define ALWAYS_ENABLE_SINK 0
#define NEGOTIATE_HIGH_VOLTAGE 1
#define SAFE_MODE 2

#define TPS_I2C_ADDR TPS_INDEX1_ADDR

#define TPS_INDEX1_ADDR 0x20
#define TPS_INDEX2_ADDR 0x21
#define TPS_INDEX3_ADDR 0x22
#define TPS_INDEX4_ADDR 0x23

#define TPS_BUNDLE_SLAVE_ADDR 0x35
#define TPS_BUNDLE_TIMEOUT 0x32  // recommended (5 seconds - LSB of 100ms)

#define REG_MODE 0x03
#define REG_DEV_INFO 0x2F
#define REG_INT_EVENT1 0x14
#define REG_INT_MASK1 0x16
#define REG_INT_CLEAR1 0x18
#define REG_PWR_STATUS 0x3F
#define REG_DEV_CAPABILITIES 0x0D
#define REG_BOOT_STATUS 0x2D
#define REG_CMD1 0x08
#define REG_DATA1 0x09
#define REG_STATUS 0x1A
#define REG_POWER_PATH_STATUS 0x26
#define REG_PD_STATUS 0x40
#define REG_TYPEC_STATE 0x69
#define REG_PORT_CONTROL 0x29

#define N_BYTES_MODE 4
#define N_BYTES_DEV_INFO 40
#define N_BYTES_INT_MASK1 11
#define N_BYTES_INT_CLEAR1 11
#define N_BYTES_INT_EVENT1 11
#define N_BYTES_PWR_STATUS 2
#define N_BYTES_DEV_CAPABILITIES 4
#define N_BYTES_BOOT_STATUS 5
#define N_BYTES_POWER_PATH_STATUS 5
#define N_BYTES_PD_STATUS 4
#define N_BYTES_STATUS 5
#define N_BYTES_TYPEC_STATE 4
#define N_BYTES_CMD1 4
#define N_BYTES_DATA1 64
#define N_BYTES_PORT_CONTROL 4

#define SM_START_UPDATE 0
#define SM_READY_FOR_PATCH 1
#define SM_PBMS 2
#define SM_WRITE_PATCH_BUNDLE 3
#define SM_PBMC 4
#define SM_PBME 5
#define SM_VERIFY_PATCH_LOAD 6
#define SM_PATCH_LOADED 7
#define SM_APP_RUN 8

class RPTPS25750 {
public:
    void state_machine_it(uint8_t &sm_pd_state, uint8_t *low_region_bin_data,
                          int low_region_bin_size, int debug_on, uart_inst_t *DEBUG_UART_ID);

    int read_tps(uint8_t dev_addr, uint8_t reg_addr, uint8_t *i2cread, int n);
    int write_tps(uint8_t dev_addr, uint8_t reg_addr, uint8_t *i2cwrite, int n);
    int write_tps1(uint8_t dev_addr, uint8_t reg_addr, uint8_t *i2cwrite, int n);
    int debug(uint8_t dev_addr, uint8_t reg_addr, uart_inst_t *bus_id);
    int get_mode(uint8_t addr, uint8_t *i2cread);
    int get_device_info(uint8_t dev_addr, uint8_t *i2cread);
    int get_power_status(uint8_t dev_addr, uint8_t *i2cread);
    int get_device_capabilities(uint8_t dev_addr, uint8_t *i2cread);
    int get_boot_status(uint8_t dev_addr, uint8_t *i2cread);
    int get_status(uint8_t dev_addr, uint8_t *i2cread);
    int get_power_path_status(uint8_t dev_addr, uint8_t *i2cread);
    int get_pd_status(uint8_t dev_addr, uint8_t *i2cread);
    int get_typec_state(uint8_t dev_addr, uint8_t *i2cread);
    void handle_power_status(uint8_t *get);
    void handle_device_capabilities(uint8_t *get);
    void handle_boot_status(uint8_t *get);
    int is_ready_for_patch(uint8_t dev_addr, uint8_t *i2cread);
    int is_set_dead_battery_flag(uint8_t dev_addr, uint8_t *i2cread);
    int is_data_role(uint8_t dev_addr, uint8_t *i2cread);
    int write_input_data(uint8_t dev_addr, uint8_t *i2cwrite, int size);
    int write_command(uint8_t dev_addr, uint8_t *i2cwrite);
    int get_command(uint8_t dev_addr, uint8_t *i2cread);
    int get_data(uint8_t dev_addr, uint8_t *i2cread, int size);
    bool execute_4CC_command(uint8_t dev_addr, uint8_t *cmd, uint8_t *input, int in_size,
                             uint8_t *output, int out_size);
    bool swap_to_dfp(uint8_t dev_addr);
    bool swap_to_ufp(uint8_t dev_addr);
    bool swap_to_source(uint8_t dev_addr);
    bool start_patch_burst_mode(uint8_t dev_addr, uint32_t bundle_size);
    bool patch_burst_mode_download_complete(uint8_t dev_addr);
    bool write_patch_bundle(uint8_t patch_addr, uint8_t *data, int size);
    bool end_patch_burst_mode(uint8_t dev_addr);
    bool clear_dead_battery_flag(uint8_t dev_addr);
    bool go_to_patch_mode(uint8_t dev_addr);
    bool hard_reset(uint8_t dev_addr);
    int is_patch_loaded(uint8_t dev_addr, uint8_t *i2cread);
    int get_intevent1(uint8_t dev_addr, uint8_t *i2cread);
    int get_intmask1(uint8_t dev_addr, uint8_t *i2cread);
    int get_intclear1(uint8_t dev_addr, uint8_t *i2cread);
    int check_usb_connection(uint8_t dev_addr, uint8_t *i2cread);

    RPTPS25750(i2c_inst_t *_i2c) : i2c(_i2c) {}

private:
    i2c_inst_t *i2c;
};

/*
GO2P: Forces PD controller to return to 'PTCH' mode and wait for patch over I2C
      (must only be used when Negotiate HighVoltage is used)
DBfg: Clear Dead Battery Flag

0x0D REG_DEV_CAPABILITIES [p.18]
bit 1:0 = 00b: source and sink supported (DRP)
bit 2 = 0b: power delivery supported

0x14 REG_INT_EVENT1 [RO][p.20]
bit 1 = 1: a PD Hard Reset has been performed
bit 11*8+1 = 1: device ready for a patch bundle from the host
(other bits represent error events, see carefully)

0x16 REG_INT_MASK1 [RW]
0x18 REG_INT_CLEAR1 [RW]

0x1A REG_STATUS [RO]
bit 0 = 1: A plug is connected
bit 3:1 = 110b: Connection present, no Ra detected. Can be an Rd (but no Ra) or an Rp detected with
no previous Ra detection, includes PD Controller that connected in Attached.SNK bit 4 = 0: plug on
CC1 bit 5 = 1: PD is source bit 6 = 0: UFP bit 21:20 = 10b: Within expected limits. The limits are
determined based on the USB PD negotiated value. bit 23:22 = 00b: No host present. This means that
no far-end device is presently providing VBUS or the PD Controller power role is Source. bit 25:24 =
00b: PD Controller is not in a legacy (non PD) mode

0x26 REG_POWER_PATH_STATUS [RO]
bit 4*8+15:14 = 10b: PD Controller is powered from VBUS. The Dead Battery flag is set
bit 4*8+15:14 = 01b: PD Controller is powered from VIN_3V3
(other bits represent cable and PP5V status)

0x29 PORT_CONTROL [RW]
bit 1:0 = 00b: USB default current
bit 1:0 = 01b: 1.5A
bit 1:0 = 10b: 3A
(other bits are related to charger)

0x2D REG_BOOT_STATUS [RO]
bit 0 = 1: Asserted when a patch bundle header errors.
bit 2 = 1: Dead Battery flag indicator. This bit is asserted when the PD Controller booted in
dead-battery mode. (other bits represent boot status and errors)

0x3F REG_PWR_STATUS [RO]
(bits represent charger information and redundant other data)

0x40 REG_PD_STATUS [RO]
bit 3:2: CC Pull-up value. The pull-up value detected by PD Controller when in CC Pull-down mode
bit 5:4: Present Type-C power role. The PD Controller is acting under this Type-C power role
bit 6 = 1: source
bit 12:8: reason for soft reset
bit 21:16: reason for hard reset
bit 27:22: reason for error recovery
bit 30:28: reason for data reset

0x69 REG_TYPEC_STATE [RO]
bit 1:0: CC pin used for PD communication
bit 9:8: state of CC1 pin
bit 17:16: state of CC2 pin
bit 31:24: present state of type-C state machine

0x70 REG_SLEEP_CONFIG [RW]
bit 0: If this bit is asserted the PD controller will enter sleep modes after device is idle for
Sleep Time.


*/
