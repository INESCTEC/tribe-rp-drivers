#include "rp_agrolib_hamamatsu_xenon.h"


int main() {

    stdio_init_all();
    HamamatsuXenon lamp(1, 500);

    sleep_ms(1500);
    printf("Starting\n");
    sleep_ms(1500);
    bool gerbro = true;
    while (1) {
        // lamp.multiShot(10);
        if (gerbro) {
            gerbro = false;
            lamp.timeFrameShot(2000);
        }
        sleep_ms(1000);
    }
}