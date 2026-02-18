#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "rp_agrolib_loraE5.h"

#include <iostream>

#define UART_ID uart0
#define BAUD_RATE 9600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 16
#define UART_RX_PIN 17

#define LoRa_APPKEY "2B7E151628AED2A609CF4F3CABF71588" /*Custom key for this App*/
#define LoRa_FREQ_standard EU868                       /*International frequency band. see*/
#define LoRa_DR                                                                                    \
    DR4 /*DR5=5.2kbps //data rate. see at                                                          \
           https://www.thethingsnetwork.org/docs/lorawan/regional-parameters/    */
#define LoRa_DEVICE_CLASS                                                                          \
    CLASS_C /*CLASS_A for power restriction/low power nodes. Class C for other device applications \
             */
#define LoRa_PORT_BYTES                                                                            \
    8 /*node Port for binary values to send, allowing the app to know it is recieving bytes*/
#define LoRa_PORT_STRING                                                                           \
    7 /*Node Port for string messages to send, allowing the app to know it is recieving            \
         characters/text */
#define LoRa_POWER 14 /*Node Tx (Transmition) power*/
#define LoRa_CHANNEL                                                                               \
    0 /*Node selected Tx channel. Default is 0, we use 2 to show only to show how to set up*/
#define LoRa_ADR_FLAG                                                                              \
    false /*ADR(Adaptative Dara Rate) status flag (True or False). Use False if your Node is       \
             moving*/
#define Tx_delay_s 9.5      /*delay between transmitions expressed in seconds*/
#define PAYLOAD_FIRST_TX 10 /*bytes to send into first packet*/
#define Tx_and_ACK_RX_timeout                                                                      \
    6000 /*6000 for SF12,4000 for SF11,3000 for SF11, 2000 for SF9/8/, 1500 for SF7. All examples  \
            consering 50 bytes payload and BW125*/

LoRaE5Class lorae5;
char response[256];
char lora_buffer[256];
short *rssi;

int main() {
    stdio_init_all();

    sleep_ms(5000);

    uart_setup(UART_ID, UART_RX_PIN, UART_TX_PIN, BAUD_RATE, DATA_BITS, STOP_BITS, PARITY);

    // lorae5.getId(response,DevEui);
    // lorae5.getVersion(response);

    lorae5.initP2PMode(868, SF12, BW125, 12, 15, LoRa_POWER);
    lorae5.enableread();  // enable continuous reading

    while (1) {

        int n = lorae5.receivePacketP2PMode((unsigned char *) lora_buffer, 256, rssi, 1000);

        sleep_ms(1);
    }
}