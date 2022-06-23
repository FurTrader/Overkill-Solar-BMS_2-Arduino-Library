/* Copyright 2020 Neil Jansen (njansen1@gmail.com)

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef BMS2_H
#define BMS2_H

#include "Arduino.h"
#include "bms2_options.h"

// Constants
#define BMS_STARTBYTE 0xDD
#define BMS_STOPBYTE  0x77
#define BMS_READ      0xA5
#define BMS_WRITE     0x5A

#define BMS_PARAM_TIMEOUT 500  // milliseconds

// Command codes (registers)
#define BMS_REG_BASIC_SYSTEM_INFO 0x03
#define BMS_REG_CELL_VOLTAGES     0x04
#define BMS_REG_NAME              0x05
#define BMS_REG_CTL_MOSFET        0xE1

// State machine states
#define BMS_STATE_WAIT_FOR_START_BYTE   0x00
#define BMS_STATE_WAIT_FOR_STATUS_BYTE  0x01
#define BMS_STATE_WAIT_FOR_CMD_CODE     0x02
#define BMS_STATE_WAIT_FOR_LENGTH       0x03
#define BMS_STATE_WAIT_FOR_DATA         0x04
#define BMS_STATE_WAIT_FOR_CHECKSUM_MSB 0x05
#define BMS_STATE_WAIT_FOR_CHECKSUM_LSB 0x06
#define BMS_STATE_WAIT_FOR_STOP_BYTE    0x07
#define BMS_STATE_ERROR                 0xFF

// replace min() because it doesnt work on the esp32 when the arguments have different data types
#define __min(a,b) ((a)<(b)?(a):(b))


typedef struct SoftwareVersion {
    uint8_t major;
    uint8_t minor;
} SoftwareVersion;


typedef struct Date {
    uint16_t year;
    uint8_t month;
    uint8_t day;
} Date;

typedef struct ProtectionStatus {
    bool single_cell_overvoltage_protection;
    bool single_cell_undervoltage_protection;
    bool whole_pack_overvoltage_protection;
    bool whole_pack_undervoltage_protection;
    bool charging_over_temperature_protection;
    bool charging_low_temperature_protection;
    bool discharge_over_temperature_protection;
    bool discharge_low_temperature_protection;
    bool charging_overcurrent_protection;
    bool discharge_overcurrent_protection;
    bool short_circuit_protection;
    bool front_end_detection_ic_error;
    bool software_lock_mos;
} ProtectionStatus;

typedef struct FaultCount {
    uint8_t single_cell_overvoltage_protection;
    uint8_t single_cell_undervoltage_protection;
    uint8_t whole_pack_overvoltage_protection;
    uint8_t whole_pack_undervoltage_protection;
    uint8_t charging_over_temperature_protection;
    uint8_t charging_low_temperature_protection;
    uint8_t discharge_over_temperature_protection;
    uint8_t discharge_low_temperature_protection;
    uint8_t charging_overcurrent_protection;
    uint8_t discharge_overcurrent_protection;
    uint8_t short_circuit_protection;
    uint8_t front_end_detection_ic_error;
    uint8_t software_lock_mos;
} FaultCount;

typedef struct DelayParamTuple {
    uint8_t msb;
    uint8_t lsb;
} DelayParamTuple;


// 0x03 Basic Info
typedef struct BasicInfo {
    uint16_t voltage;  // The total voltage, stored as units of 10 mV
    int16_t  current;  // The total current, stored in units 10mA
    uint16_t balance_capacity;
    uint16_t rate_capacity;
    uint16_t cycle_count;
    uint16_t production_date;
    uint32_t balance_status;  // The cell balance statuses, stored as a bitfield
    uint16_t protection_status;
    uint8_t  software_version;
    uint8_t  remaining_soc;
    uint8_t  mosfet_status;
    uint8_t  num_cells;
    uint8_t  num_ntcs;
    uint16_t ntc_temps[BMS_MAX_NTCs];
} BasicInfo;


typedef struct {
    // Config Parameters
    uint16_t cell_cnt;              // 0x2f
    uint16_t ntc_config;            // 0x2e -- each bit corresponds to 1 ntc.
    union {                         // 0x2d
        uint16_t func_config;
        struct {
            uint16_t _dummy2:10;
            uint16_t led_num:1;
            uint16_t led_en:1;
            uint16_t chg_balance_en:1;
            uint16_t balance_en:1;
            uint16_t scrl:1;
            uint16_t Switch:1; // *sigh* keyword.
        };
    };
    // uint16_t shunt_res;             // 0x2c
    // uint16_t bal_ctl;               // 0xe2

    // Capacity Parameters
    uint16_t design_cap;            // 0x10
    uint16_t cycle_cap;             // 0x11
    uint16_t dsg_rate;              // 0x14
    uint16_t cap_100;               // 0x12
    uint16_t cap_80;                // 0x32
    uint16_t cap_60;                // 0x33
    uint16_t cap_40;                // 0x34
    uint16_t cap_20;                // 0x35
    uint16_t cap_0;                 // 0x13

    // Balance Parameters
    uint16_t bal_start;             // 0x2a
    uint16_t bal_window;            // 0x2b

    // BMS Metadata
    // uint16_t mfg_date;              // 0x15 -- not broken out ; device doesn't care.
    // uint16_t serial_num;            // 0x16
    // uint16_t cycle_cnt;             // 0x17
    // char     mfg_name[32];          // 0xa0
    // char     device_name[32];       // 0xa1
    // char     barcode[32];           // 0xa2
    struct {                        // 0xaa
        uint16_t sc;
        uint16_t chgoc;
        uint16_t dsgoc;
        uint16_t covp;
        uint16_t cuvp;
        uint16_t chgot;
        uint16_t chgut;
        uint16_t dsgot;
        uint16_t dsgut;
        uint16_t povp;
        uint16_t puvp;
    } error_cnts;

    // Protection Params (Voltage)
    uint16_t povp;                  // 0x20
    uint16_t povp_rel;              // 0x21
    uint16_t puvp;                  // 0x22
    uint16_t puvp_rel;              // 0x23
    union {                         // 0x3c
        uint16_t pack_v_delays;
        struct {
            uint8_t puvp_delay;
            uint8_t povp_delay;
        };
    };

    uint16_t covp;                  // 0x24
    uint16_t covp_rel;              // 0x25
    uint16_t cuvp;                  // 0x26
    uint16_t cuvp_rel;              // 0x27
    union {                         // 0x3d
        uint16_t cell_v_delays;
        struct {
            uint8_t cuvp_delay;
            uint8_t covp_delay;
        };
    };

    // Protection Params (Current)
    uint16_t chgoc;                 // 0x28
    union {                         // 0x3e
        uint16_t chgoc_delays;
        struct {
            uint8_t chgoc_delay;
            uint8_t chgoc_rel;
        };
    };
    uint16_t dsgoc;                 // 0x29
    union {                         // 0x3f
        uint16_t dsgoc_delays;
        struct {
            uint8_t dsgoc_delay;
            uint8_t dsgoc_rel;
        };
    };

    // Protection Params (Temperature)
    uint16_t chgot;                 // 0x18
    uint16_t chgot_rel;             // 0x19
    uint16_t chgut;                 // 0x1a
    uint16_t chgut_rel;             // 0x1b
    union {                         // 0x3a
        uint16_t chg_t_delays;
        struct {
            uint8_t chgut_delay;
            uint8_t chgot_delay;
        };
    };
    uint16_t dsgot;                 // 0x1c
    uint16_t dsgot_rel;             // 0x1d
    uint16_t dsgut;                 // 0x1e
    uint16_t dsgut_rel;             // 0x1f
    union {                         // 0x3b
        uint16_t dsg_t_delays;
        struct {
            uint8_t dsgut_delay;
            uint8_t dsgot_delay;
        };
    };

    // Hardware Protection Parameters
    uint16_t covp_high;             // 0x36
    uint16_t cuvp_high;             // 0x37
    union {                         // 0x38
        uint16_t sc_dsgoc2;
        struct {
            uint8_t sc_dsgoc_x2:1;
            uint8_t _dummy0:2;
            uint8_t sc_delay:2;
            uint8_t sc:3;

            uint8_t dsgoc2_delay:4;
            uint8_t dsgoc2:4;
        };
    };
    union {                         // 0x39
        uint16_t cxvp_high_delay_sc_rel;
        struct {
            uint8_t cuvp_high_delay:2;
            uint8_t covp_high_delay:2;
            uint8_t _dummy1:4;

            uint8_t sc_rel;
        };
    };
    uint16_t other;
} eeprom_data_t;


class OverkillSolarBms2
{
public:
    OverkillSolarBms2();  // constructor

    // #######################################################################
    // Top-level control

    void begin(Stream *port);  // Start processing.  Call this in the sketch's setup() function
    void end();    // End processing.  Call this to stop querying the BMS and processing data.

    void main_task(bool query); // Call this as fast as possible within the sketch's loop() function

    void set_query_rate(uint16_t rate);  // Set the

    bool get_comm_error_state();  // Returns true if the BMS is not responding

    // #######################################################################
    // 0x03 Basic Info

    void query_0x03_basic_info();

    float get_voltage();  // Returns the total voltage, in volts
    float get_current();  // Returns the instantaneous current, in amps
    float get_balance_capacity();  // Returns the balance capacity, in amp hours
    float get_rate_capacity();  // Returns the rate capacity, in amp hours
    uint16_t get_cycle_count();  // Returns the cycle count (number of charge/discharge cycles)

    Date get_production_date();  // Return the production date (day, month, year)

    bool get_balance_status(uint8_t cell_index);  // Returns the balance status of the specified cell index
    ProtectionStatus get_protection_status(); // Returns the protection status
    bool get_protection_status_summary();  // Returns True if any protection status bits are currently active

    FaultCount get_fault_counts();  // Get individual fault counts
    uint32_t   get_fault_count();  // Get the cumulative fault count
    void       clear_fault_counts();

    SoftwareVersion get_software_version();  // Returns the software version (major, minor)
    uint8_t get_state_of_charge();  // returns the state of charge, in percent (0-100)
    bool get_discharge_mosfet_status();  // Returns true if the discharge FET is enabled
    bool get_charge_mosfet_status();  // Returns true if the charge FET is enabled
    uint8_t get_num_cells();  // Returns the # of cells in which the BMS is configured for
    uint8_t get_num_ntcs();  // Returns the # of temperature sensors
    float get_ntc_temperature(uint8_t ntc_index);  // Returns the temperature, in celsius, of the specified temp sensor index

    // #######################################################################
    // 0x04 Cell Voltages

    void query_0x04_cell_voltages();
    float get_cell_voltage(uint8_t cell_index);  // Returns the cell voltage, in volts, of the specified cell index

    // #######################################################################
    // 0x05 BMS Name

    String get_bms_name();  // Returns the BMS name

    // #######################################################################
    // 0xE1 MOSFET Control
    // Assume "true" means the MOSFET is ON (passing current) and "false" is OFF
    void set_0xE1_mosfet_control(bool charge, bool discharge);  // Controls the charge and discharge MOSFETs
    void set_0xE1_mosfet_control_charge(bool charge);
    void set_0xE1_mosfet_control_discharge(bool discharge);

    // #######################################################################
    // Config Parameters
    // #######################################################################

    // Do the thing that the BMS needs to do the you-know-what
    bool enter_factory_mode();
    bool exit_factory_mode(bool save);

    // #######################################################################
    // Low-level communication methods:
    void write(uint8_t read, uint8_t command_code, uint8_t* data, uint8_t length);  // Write to BMS

    uint16_t read_int_param(         uint8_t cmd_code);
    uint32_t read_int_param(         uint8_t cmd_code, float scale);
    void     write_int_param(        uint8_t cmd_code, uint16_t value, bool save=true);
    void     write_int_param(        uint8_t cmd_code, uint32_t value, float scale, bool save=true);
    float    read_float_param(       uint8_t cmd_code, float scale);
    void     write_float_param(      uint8_t cmd_code, float value, float scale, bool save=true);
    float    read_temperature_param( uint8_t cmd_code);
    void     write_temperature_param(uint8_t cmd_code, float value, bool save=true);

    bool param_success();
    void param_clear_errors();

    // Set all non-calibration params
    void print_params(eeprom_data_t* params);
    void get_params  (eeprom_data_t* params);
    void set_params  (eeprom_data_t* params);

    // # of cells
    uint8_t  get_0x2F_num_cells();
    void     set_0x2F_num_cells(uint8_t num_cells, bool save=true);

    // NTC Settings (16-bitfield)
    uint16_t get_0x2E_ntc_settings();
    void     set_0x2E_ntc_settings(uint16_t flags, bool save=true);

    uint32_t get_0x2C_shunt_resistor_value();
    void     set_0x2C_shunt_resistor_value(uint32_t milliohms, bool save=true);

    // Configuration Flags
    uint16_t get_0x2D_config_flags();
    void     set_0x2D_config_flags(uint16_t flags, bool save=true);

    // #######################################################################
    // Capacity Parameters

    // Designed capacity
    uint32_t get_0x10_designed_capacity();
    void     set_0x10_designed_capacity(uint32_t capacity, bool save=true);

    // Cycle capacity
    uint32_t get_0x11_cycle_capacity();
    void     set_0x11_cycle_capacity(uint32_t capacity, bool save=true);

    // Full Charge Voltage
    uint16_t get_0x12_full_charge_voltage();
    void     set_0x12_full_charge_voltage(uint16_t voltage, bool save=true);

    // End of Discharge Voltage
    uint16_t get_0x13_end_of_discharge_voltage();
    void     set_0x13_end_of_discharge_voltage(uint16_t voltage, bool save=true);

    // Discharge rate
    float    get_0x14_discharge_rate();
    void     set_0x14_discharge_rate(float rate, bool save=true);

    // 80/60/40/20% capacity voltage
    uint16_t get_0x3x_capacity_voltage(uint8_t percent);
    void     set_0x3x_capacity_voltage(uint8_t percent, uint16_t voltage, bool save=true);


    // #######################################################################
    // Balance Parameters

    // Start voltage
    uint16_t get_0x2A_start_voltage();
    void     set_0x2A_start_voltage(uint16_t voltage, bool save=true);

    // Delta to balance
    uint16_t get_0x2B_delta_to_balance();
    void     set_0x2B_delta_to_balance(uint16_t delta, bool save=true);

    // #######################################################################
    // BMS Metadata

    String   get_0xA2_barcode();
    void     set_0xA2_barcode(String barcode, bool save=true);

    // Manufacture date (15:9=year, 8:5=month, 4:0=day)
    String   get_0xA1_bms_name();
    void     set_0xA1_bms_name(String bmsname, bool save=true);

    Date     get_0x15_mfg_date();
    void     set_0x15_mfg_date(Date date, bool save=true);

    uint16_t get_serial_number();
    void     set_serial_number(uint16_t sn, bool save=true);
    // TODO: 0x17 Cycle count
    // TODO: 0xA0 Manufacturer name
    // TODO: 0xAA Error condition counts

    // #######################################################################
    // Protection Params (Voltage)

    // Battery over voltage (trigger value)
    uint32_t get_0x20_batt_over_volt_trig();
    void     set_0x20_batt_over_volt_trig(uint32_t trigger, bool save=true);

    // Battery over voltage (release value)
    uint32_t get_0x21_batt_over_volt_release();
    void     set_0x21_batt_over_volt_release(uint32_t release, bool save=true);

    // Battery under voltage (trigger value)
    uint32_t get_0x22_batt_under_volt_trig();
    void     set_0x22_batt_under_volt_trig(uint32_t trigger, bool save=true);

    // Battery under voltage (release value)
    uint32_t get_0x23_batt_under_volt_release();
    void     set_0x23_batt_under_volt_release(uint32_t release, bool save=true);

    // Delays (MSB=Battery under-voltage, LSB=Battery over-voltage)
    DelayParamTuple get_0x3C_delay_batt_volt();
    void            set_0x3C_delay_batt_volt(DelayParamTuple delay, bool save=true);

    // Cell over voltage (trigger value)
    uint16_t get_0x24_cell_over_volt_trig();
    void     set_0x24_cell_over_volt_trig(uint16_t trigger, bool save=true);

    // Cell over voltage (release value)
    uint16_t get_0x25_cell_over_volt_release();
    void     set_0x25_cell_over_volt_release(uint16_t release, bool save=true);

    // Cell under voltage (trigger value)
    uint16_t get_0x26_cell_under_volt_trig();
    void     set_0x26_cell_under_volt_trig(uint16_t trigger, bool save=true);

    // Cell under voltage (release value)
    uint16_t get_0x27_cell_under_volt_release();
    void     set_0x27_cell_under_volt_release(uint16_t release, bool save=true);

    // Delays (MSB=Cell under voltage, LSB=Cell over voltage)
    DelayParamTuple get_0x3D_delay_cell_volt();
    void            set_0x3D_delay_cell_volt(DelayParamTuple delay, bool save=true);

    // #######################################################################
    // Protection Params (Current)

    // Charge over current (trigger value)
    uint32_t get_0x28_charge_over_current_trig();        
    void     set_0x28_charge_over_current_trig(uint32_t trigger, bool save=true);           

    // Delays (MSB=Charge over-current delay, LSB=Charge over-current release delay)
    DelayParamTuple get_0x3E_delay_charge_current_delay(); 
    void            set_0x3E_delay_charge_current_delay(DelayParamTuple delay, bool save=true);

    // Discharge over current (release value)
    uint32_t get_0x29_discharge_over_current_release();  
    void     set_0x29_discharge_over_current_release(uint32_t release, bool save=true);     

    // Delays (MSB=Discharge over-current delay, LSB=Discharge over-current reset delay)
    DelayParamTuple get_0x3F_delay_discharge_current_delay(); 
    void            set_0x3F_delay_discharge_current_delay(DelayParamTuple delay, bool save=true);


    // #######################################################################
    // Protection Params (Temperature)

    // Charge over-temperature (trigger)
    float    get_0x18_charge_over_temp_trig();           
    void     set_0x18_charge_over_temp_trig(float trig, bool save=true);

    // Charge over-temperature (release)
    float    get_0x19_charge_over_temp_release();        
    void     set_0x19_charge_over_temp_release(float release, bool save=true);

    // Charge under-temperature (trigger)
    float    get_0x1A_charge_under_temp_trig();          
    void     set_0x1A_charge_under_temp_trig(float trig, bool save=true);

    // Charge under-temperature (release)
    float    get_0x1B_charge_under_temp_release();       
    void     set_0x1B_charge_under_temp_release(float release, bool save=true);

    // Delays (MSB=Charge under temp, LSB=Charge over-temp)
    DelayParamTuple get_0x3A_delay_charge_temp();        
    void            set_0x3A_delay_charge_temp(DelayParamTuple delay, bool save=true);            

    // Discharge over-temperature (trigger)
    float    get_0x1C_discharge_over_temp_trig();        
    void     set_0x1C_discharge_over_temp_trig(float trig, bool save=true);

    // Discharge over-temperature (release)
    float    get_0x1D_discharge_over_temp_release();     
    void     set_0x1D_discharge_over_temp_release(float release, bool save=true);

    // Discharge under-temperature (trigger)
    float    get_0x1E_discharge_under_temp_trig();       
    void     set_0x1E_discharge_under_temp_trig(float trig, bool save=true);

    // Discharge under-temperature (release)
    float    get_0x1F_discharge_under_temp_release();    
    void     set_0x1F_discharge_under_temp_release(float release, bool save=true);

    // Delays (MSB=Discharge under-temp, LSB=Discharge over-temp)
    DelayParamTuple get_0x3B_delay_discharge_temp();     
    void            set_0x3B_delay_discharge_temp(DelayParamTuple delay, bool save=true);         

    // #######################################################################
    // Hardware Protection Parameters

    // Secondary cell overvoltage protection.  scale=1, units=mV
    // uint32_t get_0x36_hw_cell_overvoltage();
    // void set_0x36_hw_cell_overvoltage();

    // 0x37 Secondary cell undervoltage protection.  scale=1, units=mV
    // uint32_t get_0x37_hw_cell_undervoltage();
    // void set_0x37_hw_cell_undervoltage();

    // 0x38 Short-circuit and Secondary overcurrent settings
    // 0x39 Secondary cell under/over voltage release times, and short-circuit release time

    // #######################################################################
    // Calibration
    void set_0xBx_cell_calibration(uint8_t cell, float voltage, bool save=true);
    void set_0xAD_idle_current_calibration(bool save=true);  // Only call when NOT charging or discharging!
    void set_0xAE_charge_current_calibration(float current, bool save=true);
    void set_0xAF_discharge_current_calibration(float current, bool save=true);
    void set_0xDx_temp_calibration(uint8_t ntc, float temperature, bool save=true);
    void set_0xE0_capacity_remaining_calibration(float capacity, bool save=true);

    // #######################################################################
    void print_config_params();
    void print_capacity_params();
    void print_balance_params();
    void print_bms_metadata();
    void print_protection_params_voltage();
    void print_protection_params_current();
    void print_protection_params_charge_temperature();
    void print_protection_params_discharge_temperature();

#ifdef BMS_OPTION_DEBUG
    void debug();  // Calling this method will print out the received data to the main serial port
    void debug_params();  // Calling this method will print out the received data to the main serial port
#endif
    uint8_t get_rx_errors();  // Returns the # of RX errors that have occured while trying to

private:
    bool m_is_initialized;
    Stream* m_serial;  // Reference to the BMS serial port

    // ########################################################################
    // Store the data internally in raw format, as received from the BMS.
    BasicInfo m_0x03_basic_info;
    uint16_t  m_0x04_cell_voltages[BMS_MAX_CELLS];
    String    m_0x05_bms_name;
    uint8_t   m_0xE1_mosfet_control[2];
    // String    m_0xA2_barcode;    // Barcode
    // String    m_0xA1_bms_name;   // BMS Name
    uint16_t  m_param;  // The last param to be read
    FaultCount m_fault_count;
    // ########################################################################
    uint16_t m_last_protection_status;
    bool has_new_fault_occured(uint8_t index);

    // Background tasks
    void serial_rx_task();
    uint32_t m_last_query_time;
    uint32_t m_last_10ms_time;

    // RX framer, state data
    uint8_t  m_rx_state;  // RX framer state machine state
    uint8_t  m_rx_cmd_code;  // current RX frame's command code (register)
    uint8_t  m_rx_status;  // current RX frame's status
    uint8_t  m_rx_length;  // current RX frame's length
    uint8_t  m_rx_data[BMS_MAX_RX_DATA_LEN];  // current RX frame's data
    uint8_t  m_rx_data_index;  // Current RX frame's data index (starts at zero)
    uint16_t m_rx_checksum;  // current RX frame's checksum
    uint8_t  m_num_rx_errors;  // Current number of RX framing errors encountered

    uint16_t m_tx_query_rate;
    uint32_t m_last_0x00_timestamp;
    uint32_t m_last_0x01_timestamp;
    uint32_t m_last_0x03_timestamp;
    uint32_t m_last_0x04_timestamp;
    uint32_t m_last_param_timestamp;
    bool     m_param_success;
    bool     m_param_timeout;
    bool     m_in_factory_mode;


    uint16_t atomic_param_read(uint8_t cmd_code);
    uint16_t atomic_param_read(uint8_t cmd_code, uint32_t timeout);

    // #######################################################################
    // Do not call; these will be called by the RX task function when needed
    void handle_rx_0x00();
    void handle_rx_0x01();
    void handle_rx_0x03();
    void handle_rx_0x04();
    void handle_rx_0x05();
    void handle_rx_0xA2();
    void handle_rx_0xA1();
    void handle_rx_param();

};

#endif  // BMS2_H
