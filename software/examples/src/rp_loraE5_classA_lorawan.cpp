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

// #define LoRa_APPKEY "2B7E151628AED2A609CF4F3CABF71588" /*Custom key for this App*/
#define LoRa_APPKEY "00000000000000000000000000000000" /*Custom key for this App*/
#define LoRa_FREQ_standard EU868                       /*International frequency band. see*/
#define LoRa_DR                                                                                    \
    DR5 /*DR5=5.2kbps //data rate. see at                                                          \
           https://www.thethingsnetwork.org/docs/lorawan/regional-parameters/    */
#define LoRa_DEVICE_CLASS                                                                          \
    CLASS_A /*CLASS_A for power restriction/low power nodes. Class C for other device applications \
             */
#define LoRa_PORT_BYTES                                                                            \
    8 /*node Port for binary values to send, allowing the app to know it is recieving bytes*/
#define LoRa_PORT_STRING                                                                           \
    7 /*Node Port for string messages to send, allowing the app to know it is recieving            \
         characters/text */
#define LoRa_POWER 6 /*Node Tx (Transmition) power*/
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

unsigned char appkey[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
                          0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38};
unsigned char nwkkey[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
                          0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38};

unsigned char buffer_binary[128] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10,
                                    11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
char buffer_char[50] = "I am sending this message to a LoRa Gateway.";

int main() {
    stdio_init_all();

    sleep_ms(5000);

    uart_setup(UART_ID, UART_RX_PIN, UART_TX_PIN, BAUD_RATE, DATA_BITS, STOP_BITS, PARITY);

    lorae5.getId(response, DevEui);
    lorae5.getId(response, DevAddr);
    lorae5.getId(response, AppEui);
    lorae5.getVersion(response);

    lorae5.setDeviceMode(LWOTAA); /*LWOTAA or LWABP. We use LWOTAA in this example*/
    // lorae5.setDataRate((_data_rate_t)LoRa_DR, (_physical_type_t)LoRa_FREQ_standard);
    lorae5.setSpreadFactor(SF9, BW125, (_physical_type_t) LoRa_FREQ_standard);
    lorae5.setKey((char *) LoRa_APPKEY, NULL,
                  (char *) LoRa_APPKEY); /*Only App key is seeted when using OOTA*/
    lorae5.setClassType((_class_type_t) LoRa_DEVICE_CLASS); /*set device class*/
    lorae5.setPort(LoRa_PORT_BYTES); /*set the default port for transmiting data*/
    lorae5.setPower(LoRa_POWER);     /*sets the Tx power*/
    lorae5.setChannel(LoRa_CHANNEL); /*selects the channel*/
    // lorae5.setAdaptiveDataRate(LoRa_ADR_FLAG);/*Enables adaptative data rate*/

    // lorae5.setDeviceWakeUp(); //do something if in sleep state
    lorae5.setOTAAJoin(JOIN, 10000);  // Join to a gateway

    while (1) {
        lorae5.setDeviceWakeUp();  // do something if in sleep state
        lorae5.setOTAAJoin(JOIN, 10000);

        /*sending a string message*/
        lorae5.setPort(LoRa_PORT_STRING); /*set port configured in reception Gateway for expecting
                                             Strings*/
        lorae5.transferPacketWithConfirmed(buffer_char, Tx_and_ACK_RX_timeout);

        /*sending bytes message*/
        // lorae5.setPort(LoRa_PORT_BYTES);/*set port configured in reception Gateway for expecting
        // bytes*/
        // lorae5.transferPacketWithConfirmed(buffer_binary,PAYLOAD_FIRST_TX,Tx_and_ACK_RX_timeout);

        /*power down the LoRa module until next Tx Transmition (Tx) cicle*/
        lorae5.setDeviceLowPower();

        /*sleeps until the next tx cicle*/
        sleep_ms((unsigned int) (Tx_delay_s * 1000));
    }
}