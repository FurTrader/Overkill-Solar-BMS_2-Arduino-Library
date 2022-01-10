#ifndef BMS2_OPTIONS_H
#define BMS2_OPTIONS_H

// Memory-saving options: Comment these out if you do not need them.
// It will save a slight bit of SRAM, which is very precious on 8-bit AVRs.
#define BMS_OPTION_PRODUCTION_DATE      // Comment this out if reading the production date is not needed.
#define BMS_OPTION_SW_VERSION           // Comment this out if reading the software version is not needed.
#define BMS_OPTION_NAME                 // Comment this out if reading the BMS name is not needed.
#define BMS_OPTION_FAULT_COUNTS         // Comment this out to not store the number of times a fault occurred.
#define BMS_OPTION_DEBUG                // Uncomment this to output debug logging data on the Serial monitor port.
// #define BMS_OPTION_DEBUG_STATE_MACHINE  // Uncomment this to output debug the state machine
// #define BMS_OPTION_DEBUG_WRITE  // Uncomment this to output debug the write command
#define BMS_OPTION_DEBUG_PARAM  // Uncomment this to output debug the write command


#define BMS_TIMEOUT         500  // The longest time to wait, in milliseconds for a response

#define BMS_MAX_CELLS       16  // Preallocates this number of cells voltages in the array
#define BMS_MAX_NTCs        4   // Preallocates this number of temperatures in the array
#define BMS_MAX_RX_DATA_LEN 64  // Preallocates this number of bytes to store RX data field


#endif  // BMS2_OPTIONS_H 
