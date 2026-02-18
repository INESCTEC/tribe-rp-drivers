#include "rp_agrolib_tps25750.h"

#define TPS_I2C_CONFIG NEGOTIATE_HIGH_VOLTAGE
#define TPS_I2C_ADDR TPS_INDEX1_ADDR

#define BAUDRATE 100  // KHz
#define SDA_PIN 14
#define SCL_PIN 15

int main() {
    stdio_init_all();

    gpio_init(SDA_PIN);
    gpio_init(SCL_PIN);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    i2c_inst_t *i2c = i2c1;
    uint baud = i2c_init(i2c, BAUDRATE * 1000);

    RPTPS25750 tps(i2c);

    sleep_ms(3000);
    scan_bus(i2c);
    sleep_ms(3000);

    uint8_t read[100];

    //------GET MODE
    if (tps.get_mode(TPS_I2C_ADDR, read) == -1)
        printf("[I2C]::Get Mode::Error reading address %x\n", TPS_I2C_ADDR);
    else {
        int n_bytes = read[0];
        read[n_bytes + 1] = '\0';
        for (int i = 1; i < strlen((const char *) read); i++) {
            char c = read[i];
            printf("%c", c);
        }
        printf("\n");
    }

    //------GET DEVICE INFO
    if (tps.get_device_info(TPS_I2C_ADDR, read) == -1)
        printf("[I2C]::Get Device Info::Error reading address %x\n", TPS_I2C_ADDR);
    else {
        int n_bytes = read[0];
        read[n_bytes + 1] = '\0';
        for (int i = 1; i < strlen((const char *) read); i++) {
            char c = read[i];
            printf("%c", c);
        }
        printf("\n");
    }

    //------GET POWER STATUS
    if (tps.get_power_status(TPS_I2C_ADDR, read) == -1)
        printf("[I2C]::Get Power Status::Error reading address %x\n", TPS_I2C_ADDR);
    else {
        int n_bytes = read[0];
        read[n_bytes + 1] = '\0';
        for (int i = 1; i < strlen((const char *) read); i++) {
            char c = read[i];
            printf("%c", c);
        }
        printf("\n");
        tps.handle_power_status(read);
    }

    //------GET DEVICE CAPABILITIES
    if (tps.get_device_capabilities(TPS_I2C_ADDR, read) == -1)
        printf("[I2C]::Get Device Capabilities::Error reading address %x\n", TPS_I2C_ADDR);
    else {
        int n_bytes = read[0];
        read[n_bytes + 1] = '\0';
        for (int i = 1; i < strlen((const char *) read); i++) {
            char c = read[i];
            printf("%c", c);
        }
        printf("\n");
        tps.handle_device_capabilities(read);
    }

    //------GET BOOT STATUS
    if (tps.get_boot_status(TPS_I2C_ADDR, read) == -1)
        printf("[I2C]::Get Device Capabilities::Error reading address %x\n", TPS_I2C_ADDR);
    else {
        int n_bytes = read[0];
        read[n_bytes + 1] = '\0';
        for (int i = 1; i < strlen((const char *) read); i++) {
            char c = read[i];
            printf("%c", c);
        }
        printf("\n");
        tps.handle_boot_status(read);
    }

    //------IS READY FOR PATCH?
    int ready = tps.is_ready_for_patch(TPS_I2C_ADDR, read);
    if (ready == -1)
        printf("[I2C]::Ready for patch?::Error reading address %x\n", TPS_I2C_ADDR);
    else if (ready == 0) {
        printf("Device is not ready for a patch bundle from the host\n");
    } else {
        printf("Device is ready for a patch bundle from the host\n");
    }

    while (true) {
        sleep_ms(2000);
    }
}
