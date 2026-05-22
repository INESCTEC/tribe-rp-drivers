
// MLX90641 refresh rates (Control register 0x800D bits 10:7):
// -----------------------------------------------------------
// Bit    Freq      Sec/frame          POR Delay (ms)  Sample Every (ms)
// 0x00 = 0.5 Hz    2 sec              4080 ms         2400 ms
// 0x01 = 1 Hz      1 sec/frame        2080 ms         1200 ms
// 0x02 = 2 Hz      0.5 sec/frame      1080 ms         600 ms (default)
// 0x03 = 4 Hz      0.25 sec/frame     580 ms          300 ms
// 0x04 = 8 Hz      0.125 sec/frame    330 ms          150 ms
// 0x05 = 16 Hz     0.0625 sec/frame   205 ms           75 ms
// 0x06 = 32 Hz     0.03125 sec/frame  143 ms           38 ms
// 0x07 = 64 Hz     0.015625 sec/frame 112 ms           19 ms

#include "../include/rp_agrolib_mlx90641_thermal.h"
#include <stdio.h>
#include "pico/time.h"

#define I2C_ID i2c1
#define BAUDRATE 400000
#define SDA_PIN 14
#define SCL_PIN 15
#define I2C_ADDR 0x33

//#define DEBUG                             // show calculated and example values for calibration constants
#define OFFSET 0.0                          // posthoc cheap temperature adjustment (shift)
#define REFRESH_RATE 0x04                   // 0x00 (0.5 Hz) to 0x07 (64 Hz). Default: 0x03 (4 Hz)
#define SAMPLE_DELAY 150                    // delay between reading samples (see refresh rate table)
#define POR_DELAY SAMPLE_DELAY * 2.0 * 1.2  // delay required after power on reset (see refresh rate table)
MLX90641 myIRcam;  // declare an instance of class MLX90641

// helper function to convert pixel row, col to 1D index array
int pixelAddr(int row, int col){
  return (row*16)+col; // convert row, col to 1D array index in 1D array of pixels (e.g. badPixels[])

  }

int main() {
  stdio_init_all();
  sleep_ms(2000);  // Wait for serial monitor to open
  //printf("MLX90641 ESP32 Calibrated Read\n");
  i2c_inst_t* i2c_1 = init_i2c1(14, 15, 100 * 1000, true);
  if(!i2c_setup(i2c_1, SDA_PIN, SCL_PIN, BAUDRATE, true)){
    return 0;
  } else {
    //printf("I2C setup successful.\n");
  }
  if (myIRcam.setRefreshRate(REFRESH_RATE)) {  // set the page refresh rate (sampling frequency)
    printf("Refresh rate adjusted to 0x%02X.\n", REFRESH_RATE);
  } else {
    printf("Error on adjusting refresh rate.\n");
  }
  sleep_ms(POR_DELAY);  // Power on reset delay (POR), see table above
  //printf("MLX90641 ESP32 Calibrated Read\n");
  // Read full EEPROM (0x2400..0x272F)
  if (!myIRcam.readEEPROMBlock(0x2400, EEPROM_WORDS, myIRcam.eeData)) {
    //printf("EEPROM read failed!\n");
    while (1) sleep_ms(1000);
  }

  // Mark bad pixels separately here (row indexes 0...11, col indexes 0..15)
  //myIRcam.badPixels[pixelAddr(9,14)]=true;    // mark pixel bad at row 9, column 14
  //myIRcam.badPixels[pixelAddr(11,0)]=true;    // mark pixel bad at row 11, column 0

  // Check EEPROM data:
#ifdef DEBUG
  printf("setup() First 16 words of EEPROM:");
  for (int i = 0; i < 16; i++) {
    printf("EEPROM value at address: 0x" + String(0x2400 + i, HEX) + ", value: 0x" + String(eeData[i], HEX));
  }
  printf("setup() Suspicious EEPROM value check:");
  for (int i = 0; i < EEPROM_WORDS; i++) {
    if (myIRcam.eeData[i] == 0x0000 || myIRcam.eeData[i] == 0xFFFF) {
      printf("EEPROM value suspicious at address: 0x" + String(0x2400 + i, HEX) + ", value: 0x" + String(eeData[i], HEX));
    }
  }
#endif
  myIRcam.Vdd = myIRcam.readVdd();  // This should be close to 3.3V. Can read once in setup.
  myIRcam.Ta = myIRcam.readTa();    // should happen inside the loop
  //printf("Ambient temperature on start: ");
  //printf("%0.2f\n", myIRcam.Ta);      // This should be close to ambient temperature (21°C?)
  myIRcam.readPixelOffset();          // only needs to be read once
  myIRcam.readAlpha();                // read sensitivities (fills alpha_pixel[])
  myIRcam.readKta();                  // read Kta coefficients (fills Kta[])
  myIRcam.readKv();                   // read Kv coefficients (fills Kv[])
  myIRcam.KsTa = myIRcam.readKsTa();  // read KsTa coefficient
  //printf("Finished: read KsTA.");
  myIRcam.readCT();                               // read 8 corner temperatures
  myIRcam.readKsTo();                             // read 8 KsTo coefficients
  myIRcam.readAlphaCorrRange();                   // read sensitivity correction coefficients
  //myIRcam.Emissivity = myIRcam.readEmissivity();  // read Emissivity coefficient
  myIRcam.Emissivity = 0.98;                    // un-comment to over-write Emissivity with hard-coded value here (e.g. 0.95)
  myIRcam.alpha_CP = myIRcam.readAlpha_CP();      // read Sensitivity alpha_CP coefficient
  myIRcam.pix_OS_ref_CP = myIRcam.readOff_CP();   // read offset CP (also called pix_OS_ref_CP)
  myIRcam.Kv_CP = myIRcam.readKv_CP();            // read Kv CP
  myIRcam.KTa_CP = myIRcam.readKTa_CP();          // read KTa_CP
  myIRcam.TGC = myIRcam.readTGC();                // read TGC - do this last (leaves setup function for some odd reason)
  /*#ifdef DEBUG                  // uncomment this if you need a pixel map (or consult the datasheet)
  printf("Printing pixel address memory map: ");
  for (int i = 0; i < 192; i++) {
    Serial.print(i);
    Serial.print(", 0x0");
    Serial.print(myIRcam.pix_addr_S0(i), HEX);
    Serial.print(", 0x0");
    printf(myIRcam.pix_addr_S1(i), HEX);
    delay(100);
  }
  #endif*/
  while (1) {

    uint32_t pollStart = to_ms_since_boot(get_absolute_time());
    bool dataAvailable = false;

    while (to_ms_since_boot(get_absolute_time()) - pollStart < 150) {  // 500ms max wait (adjust to > 1000 * 1.2 * (1/refresh_rate in Hz))
      if (myIRcam.isNewDataAvailable()){
        dataAvailable = true;
       break;
    }
      sleep_ms(10);  // Yield to prevent watchdog/I2C lockup
    }
  if (myIRcam.isNewDataAvailable()) {
    myIRcam.clearNewDataBit();
    myIRcam.readTempC();              // read the temperature
    //printf("Ta: %.2f ºC \n", myIRcam.Ta);      // print ambient temperature
    myIRcam.printFrame(myIRcam.T_o, myIRcam.Ta);  // print temperature frame to Serial Monitor
  } else {
    //printf("Timeout: No new data\n");
  }
  sleep_ms(SAMPLE_DELAY);  // wait for new reading (adjust to desired sample frequency, see refresh rate table in setRefreshRate() for ranges)
#ifdef DEBUG            // when debugging, it helps to only see the first reading, so you can scroll through the constants.
  while (1)
    ;
#endif
}
  return 0;
}
