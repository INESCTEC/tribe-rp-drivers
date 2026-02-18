#include "../include/rp_agrolib_usb.h"

/*Reads from the USB port and returns false if no data was read*/
bool read_usb(uint8_t *buffer) {
    int i = 0;

    while (i < 256) {  // this size can be bigger if necessary
        int receivedChar = getchar_timeout_us(50);

        if (receivedChar < 0) {
            // No character received within the timeout
            break;
        }

        buffer[i++] = (uint8_t) receivedChar;
    }

    buffer[i] = '\0';

    return i > 0;  // Return true if at least one character was read
}

/*Checks the beggining of a buffer with a string, returns true if they match*/
/*example use for checking the start characters of a message*/
// uint8_t buffer[10] = {'\0'};
// bool hasMessage = read_usb(buffer);
// if(hasMessage){
// bool validMessage = check_string(buffer, "&!")
//}
/*Check end of string*/
// uint8_t endstring[2] = {'\0'};
// memcpy(endstring,buffer[sizeof(buffer)-3],2);
// bool validMessage = check_string(endstring, "$$")
bool check_string(const uint8_t *buffer, const char *string) {

    for (int i = 0; i < strlen(string); i++) {
        if (buffer[i] != string[i]) {
            return false;
        }
    }

    return true;
}