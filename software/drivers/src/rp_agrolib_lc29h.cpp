#include "../include/rp_agrolib_lc29h.h"

LC29HDriver::LC29HDriver(uart_inst_t *uart_id) : uart_id_(uart_id) {}

void LC29HDriver::writeCorrections(uint8_t *data, uint8_t size) {
    uart_write_blocking(uart_id_, data, size);
}

void LC29HDriver::getNMEA(uint16_t size, RtkState *rtk_state, uint8_t *data) {
    // Convert uint8_t array to string
    char nmea_string[size + 1];
    sprintf(nmea_string, "%s", (char *) data);

    char *ptr = strstr(nmea_string, "$GNGGA");
    if (ptr != NULL) {
        // Find the position of the new line from the found index
        char *new_line_ptr = strchr(ptr, '\n');
        if (new_line_ptr != NULL) {
            // Calculate the size of the section to be copied
            size_t section_length = new_line_ptr - ptr;

            // Create an array to store the copied section
            char section[section_length + 1];

            // Copy the section to the new array
            memcpy(section, ptr, section_length);
            section[section_length] = '\0';  // Add the null terminator

            // Parse GGA and store it
            if (validateChecksum(section)) {
                parseGGA(section, rtk_state);
            }
        }
    }

    ptr = strstr(nmea_string, "$PQTMTAR");
    if (ptr != NULL) {
        // Find the position of the new line from the found index
        char *new_line_ptr = strchr(ptr, '\n');
        if (new_line_ptr != NULL) {
            // Calculate the size of the section to be copied
            size_t section_length = new_line_ptr - ptr;

            // Create an array to store the copied section
            char section[section_length + 1];

            // Copy the section to the new array
            memcpy(section, ptr, section_length);
            section[section_length] = '\0';  // Add the null terminator

            // Parse GGA and store it
            if (validateChecksum(section)) {
                parsePQTMTAR(section, rtk_state);
            }
        }
    }

    ptr = strstr(nmea_string, "$GNVTG");
    if (ptr != NULL) {
        // Find the position of the new line from the found index
        char *new_line_ptr = strchr(ptr, '\n');
        if (new_line_ptr != NULL) {
            // Calculate the size of the section to be copied
            size_t section_length = new_line_ptr - ptr;

            // Create an array to store the copied section
            char section[section_length + 1];

            // Copy the section to the new array
            memcpy(section, ptr, section_length);
            section[section_length] = '\0';  // Add the null terminator

            // Parse VTG and store it
            if (validateChecksum(section)) {
                parseVTG(section, rtk_state);
            }
        }
    }
}

void LC29HDriver::parseGGA(const char *data, struct RtkState *rtk_state) {
    // Split NMEA string into tokens using "," as delimiter
    char *token;
    char *str = strdup(data);  // To not modify the original string
    token = strtok(str, ",");

    // Iterate through tokens
    int token_index = 0;
    while (token != NULL) {
        // Parse relevant tokens for GGA sentence
        switch (token_index) {
            case 1:  // UTC of position fix
                rtk_state->utc_ = atof(token);
                break;
            case 2:  // Latitude
                rtk_state->latitude_ = convertToDegrees(atof(token));
                break;
            case 3:  // North or South
                if (token[0] == 'S')
                    rtk_state->latitude_ = -rtk_state->latitude_;
                break;
            case 4:  // Longitude
                rtk_state->longitude_ = convertToDegrees(atof(token));
                break;
            case 5:  // East or West
                if (token[0] == 'W')
                    rtk_state->longitude_ = -rtk_state->longitude_;
                break;
            case 6:  // Fix Status
                rtk_state->fix_status_ = atoi(token);
                break;
            case 7:  // Number of Satellites
                rtk_state->num_satellites_ = atoi(token);
                break;
            case 9:  // Altitude
                rtk_state->altitude_ = atof(token);
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        token_index++;
    }
    free(str);  // Free memory allocated by strdup
}

void LC29HDriver::parsePQTMTAR(const char *data, struct RtkState *rtk_state) {
    // Split NMEA string into tokens using "," as delimiter
    char *token;
    char *str = strdup(data);  // To not modify the original string
    token = strtok(str, ",");

    // Iterate through tokens
    int token_index = 0;
    while (token != NULL) {
        // Parse relevant tokens for GGA sentence
        switch (token_index) {
            case 6:  // Heading
                rtk_state->heading_ = atof(token);
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        token_index++;
    }
    free(str);  // Free memory allocated by strdup
}

void LC29HDriver::parseVTG(const char *data, struct RtkState *rtk_state) {
    // Split NMEA string into tokens using "," as delimiter
    char *token;
    char *str = strdup(data);  // To not modify the original string
    token = strtok(str, ",");

    // Iterate through tokens
    int token_index = 0;
    while (token != NULL) {
        // Parse relevant tokens for VTG sentence
        switch (token_index) {
            case 3:  // In case it enters here and it's equal to 'M', means that empty spot was
                     // found so we have to add 1 to index
                if (token[0] == 'M')
                    token_index++;
                break;
            case 7:  // Speed over ground (km/h)
                rtk_state->speed_kph_ = atof(token);
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        token_index++;
    }
    free(str);  // Free memory allocated by strdup
}

double LC29HDriver::convertToDegrees(double value) {
    // Extract degrees and minutes
    int degrees = (int) value / 100;           // Extract integer part as degrees
    double minutes = value - (degrees * 100);  // Extract fractional part as minutes

    // Convert minutes to degrees
    double minutes_in_degrees = minutes / 60.0;

    // Calculate total degrees
    double total_degrees = degrees + minutes_in_degrees;

    return total_degrees;
}

void LC29HDriver::getPQMT(uint16_t in_size, char *pqmt, uint8_t *data, uint16_t *out_size) {
    // Convert uint8_t array to string
    char pqmt_string[in_size + 1];
    sprintf(pqmt_string, "%s", (char *) data);

    char *ptr = strstr(pqmt_string, "$PQMT");
    if (ptr != NULL) {
        // Find the position of the new line from the found index
        char *new_line_ptr = strchr(ptr, '\n');
        if (new_line_ptr != NULL) {
            // Calculate the size of the section to be copied
            *out_size = new_line_ptr - ptr;

            // Copy the section to the new array
            memcpy(pqmt, ptr, *out_size);
            pqmt[*out_size] = '\0';  // Add the null terminator
        }
    }
}

void LC29HDriver::getPair(uint16_t in_size, char *pair, uint8_t *data, uint16_t *out_size) {
    // Convert uint8_t array to string
    char pqmt_string[in_size + 1];
    sprintf(pqmt_string, "%s", (char *) data);

    char *ptr = strstr(pqmt_string, "$PAIR");
    if (ptr != NULL) {
        // Find the position of the new line from the found index
        char *new_line_ptr = strchr(ptr, '\n');
        if (new_line_ptr != NULL) {
            // Calculate the size of the section to be copied
            *out_size = new_line_ptr - ptr;

            // Copy the section to the new array
            memcpy(pair, ptr, *out_size);
            pair[*out_size] = '\0';  // Add the null terminator
        }
    }
}

bool LC29HDriver::validateChecksum(const char *message) {
    int calculated_checksum = 0;
    for (int i = 1; message[i] != '*' && message[i] != '\0'; i++) {
        calculated_checksum ^= message[i];
    }

    char *checksum_ptr = strchr(message, '*');
    if (checksum_ptr) {
        int received_checksum = strtol(checksum_ptr + 1, NULL, 16);
        return calculated_checksum == received_checksum;
    }
    return false;
}