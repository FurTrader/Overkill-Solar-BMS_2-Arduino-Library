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

#include "bms2.h"

// constructor
OverkillSolarBms2::OverkillSolarBms2() {
    m_is_initialized = false;
    m_in_factory_mode = false;

    m_0x03_basic_info.voltage = 0;
    m_0x03_basic_info.current = 0;
    m_0x03_basic_info.balance_capacity = 0;
    m_0x03_basic_info.rate_capacity = 0;
    m_0x03_basic_info.cycle_count = 0;
    m_0x03_basic_info.production_date = 0;
    m_0x03_basic_info.balance_status = 0;
    m_0x03_basic_info.protection_status = 0;
    m_0x03_basic_info.software_version = 0;
    m_0x03_basic_info.remaining_soc = 0;
    m_0x03_basic_info.mosfet_status = 0;
    m_0x03_basic_info.num_cells = 0;
    m_0x03_basic_info.num_ntcs = 0;

    for (uint8_t i=0; i<BMS_MAX_NTCs; i++) {
        m_0x03_basic_info.ntc_temps[i] = 0;
    }

    for (uint8_t i=0; i<BMS_MAX_CELLS; i++) {
        m_0x04_cell_voltages[i] = 0;
    }

    m_0xE1_mosfet_control[0] = 0;
    m_0xE1_mosfet_control[1] = 0;

    m_param = 0;

    m_0x05_bms_name = String("");
    // m_0xA2_barcode = String("");
    // m_0xA1_bms_name = String("");

    m_param_success = true;
    m_param_timeout = false;
}

// ###########################################################################

// Start processing
void OverkillSolarBms2::begin(Stream *port) {
    #ifdef BMS_OPTION_DEBUG
        // Serial.println("OverkillSolarBMS Begin!");
    #endif
    m_serial = port;
    m_is_initialized = true;
    m_rx_state = BMS_STATE_WAIT_FOR_START_BYTE;
    uint32_t now = millis();
    m_last_query_time = now;
    m_last_10ms_time = now;
    m_last_0x00_timestamp = now;
    m_last_0x01_timestamp = now;
    m_last_0x03_timestamp = now;
    m_last_0x04_timestamp = now;
    m_last_param_timestamp = now;
    m_tx_query_rate = 1000;
}

// End processing
void OverkillSolarBms2::end() {
    m_is_initialized = false;
}

// Call this as fast as possible within the sketch's loop() function
void OverkillSolarBms2::main_task(bool query) {
    if (m_is_initialized) {
        uint32_t now = millis();

        // 10 millisecond task (serial data task)
        if (now - m_last_10ms_time >= 10) {
            serial_rx_task();
            m_last_10ms_time = now;
        }

        // query task
        if (query) {
            if (now - m_last_query_time >= m_tx_query_rate) {
                query_0x03_basic_info();
                query_0x04_cell_voltages();
                m_last_query_time = now;
            }
        }
    }
}

bool OverkillSolarBms2::get_comm_error_state() {
    uint32_t now = millis();
    bool err_state = false;
    if (now - m_last_0x03_timestamp >= (m_tx_query_rate * 2)) {
        err_state = true;
    }
    if (now - m_last_0x04_timestamp >= (m_tx_query_rate * 2)) {
        err_state = true;
    }
    return err_state;
}

void OverkillSolarBms2::set_query_rate(uint16_t rate) {
    m_tx_query_rate = rate;
}


// ###########################################################################
// 0x03 Basic Info & Status
// ###########################################################################

float OverkillSolarBms2::get_voltage() {
    // Voltage is stored internally as a uint16_t in units of 10mV
    return m_0x03_basic_info.voltage * 0.01;
}

float OverkillSolarBms2::get_current() {
    // Current is stored internally as an int16_t in units of 10mA
    // Serial.print("Raw current:    ");
    // Serial.print(m_0x03_basic_info.current, DEC);
    // Serial.println(" x 10mA");
    // Serial.print("Scaled current: ");
    // Serial.print((float)m_0x03_basic_info.current * 0.01, DEC);
    // Serial.println(" A");
    return (float)m_0x03_basic_info.current * 0.01;
}

float OverkillSolarBms2::get_balance_capacity() {
    // Capacity is stored internally as a uint16_t in units of 10mAh
    return m_0x03_basic_info.balance_capacity * 0.01;
}

float OverkillSolarBms2::get_rate_capacity() {
    // Capacity is stored internally as a uint16_t in units of 10mAh
    return m_0x03_basic_info.rate_capacity * 0.01;
}

uint16_t OverkillSolarBms2::get_cycle_count() {
    return m_0x03_basic_info.cycle_count;
}

Date OverkillSolarBms2::get_production_date() {
    // Production date is stored internally as a uint16_t, bit-packed as follows:
    //         1111110000000000
    // Field   5432109876543210  # bits  offset
    // ======  ================  ======  ======
    // Day:               xxxxx  5       0
    // Month:         xxxx       4       5
    // Year:   xxxxxxx           7       9
    Date date;
    date.year = 2000 + ((m_0x03_basic_info.production_date >> 9) & 0b1111111);
    date.month = (m_0x03_basic_info.production_date >> 5) & 0b1111;
    date.day = m_0x03_basic_info.production_date & 0b11111;
    return date;
}

bool OverkillSolarBms2::get_balance_status(uint8_t cell_index) {
    if (cell_index <= 31) {
        return (m_0x03_basic_info.balance_status >> cell_index) & 1;
    }
    else {
        return false;
    }
}

bool OverkillSolarBms2::has_new_fault_occured(uint8_t index) {
    return (bool)((((m_last_protection_status ^ m_0x03_basic_info.protection_status) & m_0x03_basic_info.protection_status) >> index) & 1);
}


ProtectionStatus OverkillSolarBms2::get_protection_status() {
    ProtectionStatus status;
    status.single_cell_overvoltage_protection   = (bool)((m_0x03_basic_info.protection_status)       & 1);
    status.single_cell_undervoltage_protection  = (bool)((m_0x03_basic_info.protection_status >> 1)  & 1);
    status.whole_pack_overvoltage_protection    = (bool)((m_0x03_basic_info.protection_status >> 2)  & 1);
    status.whole_pack_undervoltage_protection   = (bool)((m_0x03_basic_info.protection_status >> 3)  & 1);
    status.charging_over_temperature_protection = (bool)((m_0x03_basic_info.protection_status >> 4)  & 1);
    status.charging_low_temperature_protection  = (bool)((m_0x03_basic_info.protection_status >> 5)  & 1);
    status.discharge_over_temperature_protection= (bool)((m_0x03_basic_info.protection_status >> 6)  & 1);
    status.discharge_low_temperature_protection = (bool)((m_0x03_basic_info.protection_status >> 7)  & 1);
    status.charging_overcurrent_protection      = (bool)((m_0x03_basic_info.protection_status >> 8)  & 1);
    status.discharge_overcurrent_protection     = (bool)((m_0x03_basic_info.protection_status >> 9)  & 1);
    status.short_circuit_protection             = (bool)((m_0x03_basic_info.protection_status >> 10) & 1);
    status.front_end_detection_ic_error         = (bool)((m_0x03_basic_info.protection_status >> 11) & 1);
    status.software_lock_mos                    = (bool)((m_0x03_basic_info.protection_status >> 12) & 1);
    return status;
}

FaultCount OverkillSolarBms2::get_fault_counts() {
    return m_fault_count;
}

uint32_t OverkillSolarBms2::get_fault_count() {
    uint32_t count = 0;
    count += m_fault_count.single_cell_overvoltage_protection;
    count += m_fault_count.single_cell_undervoltage_protection;
    count += m_fault_count.whole_pack_overvoltage_protection;
    count += m_fault_count.whole_pack_undervoltage_protection;
    count += m_fault_count.charging_over_temperature_protection;
    count += m_fault_count.charging_low_temperature_protection;
    count += m_fault_count.discharge_over_temperature_protection;
    count += m_fault_count.discharge_low_temperature_protection;
    count += m_fault_count.charging_overcurrent_protection;
    count += m_fault_count.discharge_overcurrent_protection;
    count += m_fault_count.short_circuit_protection;
    count += m_fault_count.front_end_detection_ic_error;
    count += m_fault_count.software_lock_mos;
    return count;
}

void OverkillSolarBms2::clear_fault_counts() {
    m_fault_count.single_cell_overvoltage_protection    = 0;
    m_fault_count.single_cell_undervoltage_protection   = 0;
    m_fault_count.whole_pack_overvoltage_protection     = 0;
    m_fault_count.whole_pack_undervoltage_protection    = 0;
    m_fault_count.charging_over_temperature_protection  = 0;
    m_fault_count.charging_low_temperature_protection   = 0;
    m_fault_count.discharge_over_temperature_protection = 0;
    m_fault_count.discharge_low_temperature_protection  = 0;
    m_fault_count.charging_overcurrent_protection       = 0;
    m_fault_count.discharge_overcurrent_protection      = 0;
    m_fault_count.short_circuit_protection              = 0;
    m_fault_count.front_end_detection_ic_error          = 0;
    m_fault_count.software_lock_mos                     = 0;
}


bool OverkillSolarBms2::get_protection_status_summary() {
    return (bool)m_0x03_basic_info.protection_status & 0b1111111111111;
}


SoftwareVersion OverkillSolarBms2::get_software_version() {
    SoftwareVersion version;
    version.major = (m_0x03_basic_info.software_version >> 4) & 0b1111;
    version.minor = m_0x03_basic_info.software_version & 0b1111;
    return version;
}

uint8_t OverkillSolarBms2::get_state_of_charge() {
    // note: the result is not bounds-checked, I assume that the BMS could
    // return higher than 100%, but rather than hiding that here and
    // bounding it to 100, it is returned as-is.
    return m_0x03_basic_info.remaining_soc;
}

bool OverkillSolarBms2::get_discharge_mosfet_status() {
    return (m_0x03_basic_info.mosfet_status >> 1) & 1;
}

bool OverkillSolarBms2::get_charge_mosfet_status() {
    return m_0x03_basic_info.mosfet_status & 1;
}

uint8_t OverkillSolarBms2::get_num_cells() {
    return m_0x03_basic_info.num_cells;
}

uint8_t OverkillSolarBms2::get_num_ntcs() {
    return m_0x03_basic_info.num_ntcs;
}

// Returns the temperature, in celsius
float OverkillSolarBms2::get_ntc_temperature(uint8_t ntc_index) {
    if (ntc_index + 1 <= BMS_MAX_NTCs) {
        float temp = m_0x03_basic_info.ntc_temps[ntc_index];
        temp *= 0.1;  // Convert fixed-precision int 0.1 degrees K per LSB to float degrees K
        temp -= 273.15;  // Convert Kelvin to Celsius
        return temp;
    }
    else {
        return 1.0 / 0.0;  // NaN
    }
}

// ###########################################################################
// 0x04 Cell Voltages
// ###########################################################################

// Returns the cell voltage, in volts
float OverkillSolarBms2::get_cell_voltage(uint8_t cell_index) {
    if (cell_index + 1 <= BMS_MAX_CELLS) {
        float voltage = m_0x04_cell_voltages[cell_index];
        voltage *= 0.001;  // Convert millivolts to volts
        return voltage;
    }
    else {
        return 1.0 / 0.0;  // NaN
    }
}

// ###########################################################################
// 0x05 BMS Name
// ###########################################################################

String OverkillSolarBms2::get_bms_name() {
    // Clear the bms_name string
    m_0x05_bms_name = String("");

    #ifdef BMS_OPTION_DEBUG
        // Serial.println("Query 0x05 BMS Name");
    #endif
    uint8_t length = 0;
    uint8_t *data = NULL;
    write(BMS_READ, BMS_REG_NAME, data, length);

    uint32_t t0 = millis();

    // Handle the incoming serial data until the BMS name is received
    while(1) {
        serial_rx_task();
        delay(10);

        // Wait for the RX task to receive the message, parse it and store
        // it in the bms_name
        if (m_0x05_bms_name.length() > 0) {
            break;
        }

        // Timeout if it took too long
        if (millis() - t0 >= BMS_TIMEOUT ) {
            break;
        }
    }
    return m_0x05_bms_name;
}

// ###########################################################################
// 0xE1 Set MOSFET
// ###########################################################################

// bit 0: Set to disable charge FET
// bit 1: Set to disable discharge FET

void OverkillSolarBms2::set_0xE1_mosfet_control(bool charge, bool discharge) {
    #ifdef BMS_OPTION_DEBUG
        // Serial.println("Set 0xE1 MOSFET Control");
    #endif
    if (charge) {
        m_0xE1_mosfet_control[1] &= 0b10;  // Disable bit zero
    }
    else {
        m_0xE1_mosfet_control[1] |= 0b01;  // Enable bit zero
    }

    if (discharge) {
        m_0xE1_mosfet_control[1] &= 0b01;  // Disable bit 1
    }
    else {
        m_0xE1_mosfet_control[1] |= 0b10;  // Enable bit 1
    }

    write(BMS_WRITE, BMS_REG_CTL_MOSFET, m_0xE1_mosfet_control, 2);
}

void OverkillSolarBms2::set_0xE1_mosfet_control_charge(bool charge) {
    if (charge) {
        m_0xE1_mosfet_control[1] &= 0b10;  // Disable bit zero
    }
    else {
        m_0xE1_mosfet_control[1] |= 0b01;  // Enable bit zero
    }
    write(BMS_WRITE, BMS_REG_CTL_MOSFET, m_0xE1_mosfet_control, 2);
}

void OverkillSolarBms2::set_0xE1_mosfet_control_discharge(bool discharge) {
    if (discharge) {
        m_0xE1_mosfet_control[1] &= 0b01;  // Disable bit 1
    }
    else {
        m_0xE1_mosfet_control[1] |= 0b10;  // Enable bit 1
    }
    write(BMS_WRITE, BMS_REG_CTL_MOSFET, m_0xE1_mosfet_control, 2);
}


#ifdef BMS_OPTION_DEBUG
void OverkillSolarBms2::debug() {
    // Serial.println("==============================================");
    // Serial.print("Voltage:           ");
    // Serial.print(get_voltage(), 3);
    // Serial.println(" V");

    // Serial.print("Current:           ");
    // Serial.print(get_current(), 3);
    // Serial.println(" A");

    // Serial.print("Balance capacity:  ");
    // Serial.print(get_balance_capacity(), 3);
    // Serial.println(" Ah");

    // Serial.print("Rate capacity:     ");
    // Serial.print(get_rate_capacity(), 3);
    // Serial.println(" Ah");

    // Serial.print("Cycle count:       ");
    // Serial.println(get_cycle_count() , DEC);

    // Serial.print("Production Date:   ");
    Date date = get_production_date();
    // Serial.print(date.day, DEC);
    // Serial.print("/");
    // Serial.print(date.month, DEC);
    // Serial.print("/");
    // Serial.println(date.year, DEC);

    // Serial.println("Protection Status: ");
    ProtectionStatus prot_status = get_protection_status();
    // Serial.print("  software_lock_mos:                    ");
    // Serial.println(prot_status.software_lock_mos, DEC);
    // Serial.print("  front_end_detection_ic_error:         ");
    // Serial.println(prot_status.front_end_detection_ic_error, DEC);
    // Serial.print("  short_circuit_protection:             ");
    // Serial.println(prot_status.short_circuit_protection, DEC);
    // Serial.print("  discharge_overcurrent_protection:     ");
    // Serial.println(prot_status.discharge_overcurrent_protection, DEC);
    // Serial.print("  charging_overcurrent_protection:      ");
    // Serial.println(prot_status.charging_overcurrent_protection, DEC);
    // Serial.print("  discharge_low_temperature_protection: ");
    // Serial.println(prot_status.discharge_low_temperature_protection, DEC);
    // Serial.print("  discharge_over_temperature_protection:");
    // Serial.println(prot_status.discharge_over_temperature_protection, DEC);
    // Serial.print("  charging_low_temperature_protection:  ");
    // Serial.println(prot_status.charging_low_temperature_protection, DEC);
    // Serial.print("  charging_over_temperature_protection: ");
    // Serial.println(prot_status.charging_over_temperature_protection, DEC);
    // Serial.print("  whole_pack_undervoltage_protection:   ");
    // Serial.println(prot_status.whole_pack_undervoltage_protection, DEC);
    // Serial.print("  whole_pack_overvoltage_protection:    ");
    // Serial.println(prot_status.whole_pack_overvoltage_protection, DEC);
    // Serial.print("  single_cell_undervoltage_protection:  ");
    // Serial.println(prot_status.single_cell_undervoltage_protection, DEC);
    // Serial.print("  single_cell_overvoltage_protection:   ");
    // Serial.println(prot_status.single_cell_overvoltage_protection, DEC);

    // Serial.print("Software version:  ");
    SoftwareVersion version = get_software_version();
    // Serial.print(version.major, DEC);
    // Serial.print(".");
    // Serial.println(version.minor, DEC);

    // Serial.print("State of Charge:   ");
    // Serial.print(get_state_of_charge(), DEC);
    // Serial.println("%");

    // Serial.print("Discharge MOSFET:  ");
    // Serial.println(get_discharge_mosfet_status()?"ON":"OFF");

    // Serial.print("Charge MOSFET:     ");
    // Serial.println(get_charge_mosfet_status()?"ON":"OFF");

    // Serial.print("# of cells:        ");
    // Serial.println(get_num_cells(), DEC);

    // Serial.print("# of temp sensors: ");
    // Serial.println(get_num_ntcs(), DEC);

    // Serial.println("Temperatures:");
    for (uint8_t i=0; i < __min(BMS_MAX_NTCs, get_num_ntcs()); i++) {
        // Serial.print("  ");
        // Serial.print(get_ntc_temperature(i), 1);
        // Serial.println(" deg C");
    }

    // Serial.println("Cell Voltages & Balance Status: ");
    for (uint8_t i=0; i < __min(BMS_MAX_CELLS, get_num_cells()); i++) {
        // Serial.print("  ");
        // Serial.print(get_cell_voltage(i), 3);  // Returns the cell voltage, in volts
        // Serial.print("V  ");
        // Serial.println(get_balance_status(i)?"(balancing)":"(not balancing)");
    }

    // Serial.print("BMS Name:         ");
    // Serial.println(get_bms_name());
    // // Serial.println("==============================================");
    // Serial.println();
}
#endif


// ############################################################################
// Config Parameters
// ############################################################################

void OverkillSolarBms2::print_params(eeprom_data_t* params) {
    // Config Parameters
    // Serial.print(F("0x2F  ")); // Serial.println(params->cell_cnt, HEX);
    // Serial.print(F("0x2E  ")); // Serial.println(params->ntc_config, HEX);
    // Serial.print(F("0x2D  ")); // Serial.println(params->func_config, HEX);
    // // Serial.print(F("0x2C  ")); // Serial.println(params->shunt_res, HEX);
    // // Serial.print(F("0xE2  ")); // Serial.println(params->bal_ctl, HEX);

    // Capacity Parameters
    // Serial.print(F("0x10  ")); // Serial.println(params->design_cap, HEX);
    // Serial.print(F("0x11  ")); // Serial.println(params->cycle_cap, HEX);
    // Serial.print(F("0x14  ")); // Serial.println(params->dsg_rate, HEX);
    // Serial.print(F("0x12  ")); // Serial.println(params->cap_100, HEX);
    // Serial.print(F("0x32  ")); // Serial.println(params->cap_80, HEX);
    // Serial.print(F("0x33  ")); // Serial.println(params->cap_60, HEX);
    // Serial.print(F("0x34  ")); // Serial.println(params->cap_40, HEX);
    // Serial.print(F("0x35  ")); // Serial.println(params->cap_20, HEX);
    // Serial.print(F("0x13  ")); // Serial.println(params->cap_0, HEX);

    // Balance Parameters
    // Serial.print(F("0x2a  ")); // Serial.println(params->bal_start, HEX);
    // Serial.print(F("0x2b  ")); // Serial.println(params->bal_window, HEX);

    // BMS Metadata
    // // Serial.print(F("0x15  ")); // Serial.println(params->mfg_date, HEX);
    // // Serial.print(F("0x16  ")); // Serial.println(params->serial_num, HEX);
    // // Serial.print(F("0x17  ")); // Serial.println(params->cycle_cnt, HEX);
    // // Serial.print(F("0xA0  ")); // Serial.println(params->mfg_name);
    // // Serial.print(F("0xA1  ")); // Serial.println(params->device_name);
    // // Serial.print(F("0xA2  ")); // Serial.println(params->barcode);
    // // Serial.print(F("0xAA  ")); // Serial.println(params->error_cnts, HEX);

    // Protection Params (Voltage)
    // Serial.print(F("0x20  ")); // Serial.println(params->povp, HEX);
    // Serial.print(F("0x21  ")); // Serial.println(params->povp_rel, HEX);
    // Serial.print(F("0x22  ")); // Serial.println(params->puvp, HEX);
    // Serial.print(F("0x23  ")); // Serial.println(params->puvp_rel, HEX);
    // Serial.print(F("0x3C  ")); // Serial.println(params->pack_v_delays, HEX);
    // Serial.print(F("0x24  ")); // Serial.println(params->covp, HEX);
    // Serial.print(F("0x25  ")); // Serial.println(params->covp_rel, HEX);
    // Serial.print(F("0x26  ")); // Serial.println(params->cuvp, HEX);
    // Serial.print(F("0x27  ")); // Serial.println(params->cuvp_rel, HEX);
    // Serial.print(F("0x3D  ")); // Serial.println(params->cell_v_delays, HEX);

    // Protection Params (Current)
    // Serial.print(F("0x28  ")); // Serial.println(params->chgoc, HEX);
    // Serial.print(F("0x3E  ")); // Serial.println(params->chgoc_delays, HEX);
    // Serial.print(F("0x29  ")); // Serial.println(params->dsgoc, HEX);
    // Serial.print(F("0x3F  ")); // Serial.println(params->dsgoc_delays, HEX);

    // Protection Params (Temperature)
    // Serial.print(F("0x18  ")); // Serial.println(params->chgot, HEX);
    // Serial.print(F("0x19  ")); // Serial.println(params->chgot_rel, HEX);
    // Serial.print(F("0x1A  ")); // Serial.println(params->chgut, HEX);
    // Serial.print(F("0x1B  ")); // Serial.println(params->chgut_rel, HEX);
    // Serial.print(F("0x3A  ")); // Serial.println(params->chg_t_delays, HEX);
    // Serial.print(F("0x1C  ")); // Serial.println(params->dsgot, HEX);
    // Serial.print(F("0x1D  ")); // Serial.println(params->dsgot_rel, HEX);
    // Serial.print(F("0x1E  ")); // Serial.println(params->dsgut, HEX);
    // Serial.print(F("0x1F  ")); // Serial.println(params->dsgut_rel, HEX);
    // Serial.print(F("0x3B  ")); // Serial.println(params->dsg_t_delays, HEX);

    // Hardware Protection Parameters
    // // Serial.print(F("0x36  ")); // Serial.println(params->covp_high, HEX);
    // // Serial.print(F("0x37  ")); // Serial.println(params->cuvp_high, HEX);
    // // Serial.print(F("0x38  ")); // Serial.println(params->sc_dsgoc2, HEX);
    // // Serial.print(F("0x39  ")); // Serial.println(params->cxvp_high_delay_sc_rel, HEX);
}

void OverkillSolarBms2::get_params(eeprom_data_t* params) {
    enter_factory_mode();

    // Config Parameters
    params->cell_cnt    = read_int_param(0x2F);
    params->ntc_config  = read_int_param(0x2E);
    params->func_config = read_int_param(0x2D);
    // params->shunt_res   = read_int_param(0x2C);
    // params->bal_ctl     = read_int_param(0xE2);

    // Capacity Parameters
    params->design_cap  = read_int_param(0x10);
    params->cycle_cap   = read_int_param(0x11);
    params->dsg_rate    = read_int_param(0x14);
    params->cap_100     = read_int_param(0x12);
    params->cap_80      = read_int_param(0x32);
    params->cap_60      = read_int_param(0x33);
    params->cap_40      = read_int_param(0x34);
    params->cap_20      = read_int_param(0x35);
    params->cap_0       = read_int_param(0x13);

    // Balance Parameters
    params->bal_start   = read_int_param(0x2a);
    params->bal_window  = read_int_param(0x2b);

    // BMS Metadata
    // params->mfg_date    = read_int_param(0x15);
    // params->serial_num  = read_int_param(0x16);
    // params->cycle_cnt   = read_int_param(0x17);
    // params->mfg_name    = read_int_param(0xA0);
    // params->device_name = read_int_param(0xA1);
    // params->barcode     = read_int_param(0xA2);
    // params->error_cnts  = read_int_param(0xAA);

    // Protection Params (Voltage)
    params->povp          = read_int_param(0x20);
    params->povp_rel      = read_int_param(0x21);
    params->puvp          = read_int_param(0x22);
    params->puvp_rel      = read_int_param(0x23);
    params->pack_v_delays = read_int_param(0x3C);
    params->covp          = read_int_param(0x24);
    params->covp_rel      = read_int_param(0x25);
    params->cuvp          = read_int_param(0x26);
    params->cuvp_rel      = read_int_param(0x27);
    params->cell_v_delays = read_int_param(0x3D);

    // Protection Params (Current)
    params->chgoc         = read_int_param(0x28);
    params->chgoc_delays  = read_int_param(0x3E);
    params->dsgoc         = read_int_param(0x29);
    params->dsgoc_delays  = read_int_param(0x3F);

    // Protection Params (Temperature)
    params->chgot         = read_int_param(0x18);
    params->chgot_rel     = read_int_param(0x19);
    params->chgut         = read_int_param(0x1A);
    params->chgut_rel     = read_int_param(0x1B);
    params->chg_t_delays  = read_int_param(0x3A);
    params->dsgot         = read_int_param(0x1C);
    params->dsgot_rel     = read_int_param(0x1D);
    params->dsgut         = read_int_param(0x1E);
    params->dsgut_rel     = read_int_param(0x1F);
    params->dsg_t_delays  = read_int_param(0x3B);

    // Hardware Protection Parameters
    // params->covp_high     = read_int_param(0x36);
    // params->cuvp_high     = read_int_param(0x37);
    // params->sc_dsgoc2     = read_int_param(0x38);
    // params->cxvp_high_delay_sc_rel = read_int_param(0x39);
    exit_factory_mode(false);
}

void OverkillSolarBms2::set_params(eeprom_data_t* params) {

    eeprom_data_t old_params;
    get_params(&old_params);

    enter_factory_mode();

    // Config Parameters
    if (params->cell_cnt      != old_params.cell_cnt)     { write_int_param(0x2F, params->cell_cnt,     false); }
    if (params->ntc_config    != old_params.ntc_config)   { write_int_param(0x2E, params->ntc_config,   false); }
    if (params->func_config   != old_params.func_config)  { write_int_param(0x2D, params->func_config,  false); }

    // Capacity Parameters
    if (params->design_cap    != old_params.design_cap)   { write_int_param(0x10, params->design_cap,   false); }
    if (params->cycle_cap     != old_params.cycle_cap)    { write_int_param(0x11, params->cycle_cap,    false); }
    if (params->dsg_rate      != old_params.dsg_rate)     { write_int_param(0x14, params->dsg_rate,     false); }
    if (params->cap_100       != old_params.cap_100)      { write_int_param(0x12, params->cap_100,      false); }
    if (params->cap_80        != old_params.cap_80)       { write_int_param(0x32, params->cap_80,       false); }
    if (params->cap_60        != old_params.cap_60)       { write_int_param(0x33, params->cap_60,       false); }
    if (params->cap_40        != old_params.cap_40)       { write_int_param(0x34, params->cap_40,       false); }
    if (params->cap_20        != old_params.cap_20)       { write_int_param(0x35, params->cap_20,       false); }
    if (params->cap_0         != old_params.cap_0)        { write_int_param(0x13, params->cap_0,        false); }

    // Balance Parameters
    if (params->bal_start     != old_params.bal_start)    { write_int_param(0x2a, params->bal_start,    false); }
    if (params->bal_window    != old_params.bal_window)   { write_int_param(0x2b, params->bal_window,   false); }

    // Protection Params (Voltage)
    if (params->povp          != old_params.povp)         { write_int_param(0x20, params->povp,         false); }
    if (params->povp_rel      != old_params.povp_rel)     { write_int_param(0x21, params->povp_rel,     false); }
    if (params->puvp          != old_params.puvp)         { write_int_param(0x22, params->puvp,         false); }
    if (params->puvp_rel      != old_params.puvp_rel)     { write_int_param(0x23, params->puvp_rel,     false); }
    if (params->pack_v_delays != old_params.pack_v_delays){ write_int_param(0x3C, params->pack_v_delays,false); }
    if (params->covp          != old_params.covp)         { write_int_param(0x24, params->covp,         false); }
    if (params->covp_rel      != old_params.covp_rel)     { write_int_param(0x25, params->covp_rel,     false); }
    if (params->cuvp          != old_params.cuvp)         { write_int_param(0x26, params->cuvp,         false); }
    if (params->cuvp_rel      != old_params.cuvp_rel)     { write_int_param(0x27, params->cuvp_rel,     false); }
    if (params->cell_v_delays != old_params.cell_v_delays){ write_int_param(0x3D, params->cell_v_delays,false); }

    // Protection Params (Current)
    if (params->chgoc         != old_params.chgoc)        { write_int_param(0x28, params->chgoc,        false); }
    if (params->chgoc_delays  != old_params.chgoc_delays) { write_int_param(0x3E, params->chgoc_delays, false); }
    if (params->dsgoc         != old_params.dsgoc)        { write_int_param(0x29, params->dsgoc,        false); }
    if (params->dsgoc_delays  != old_params.dsgoc_delays) { write_int_param(0x3F, params->dsgoc_delays, false); }

    // Protection Params (Temperature)
    if (params->chgot         != old_params.chgot)        { write_int_param(0x18, params->chgot,        false); }
    if (params->chgot_rel     != old_params.chgot_rel)    { write_int_param(0x19, params->chgot_rel,    false); }
    if (params->chgut         != old_params.chgut)        { write_int_param(0x1A, params->chgut,        false); }
    if (params->chgut_rel     != old_params.chgut_rel)    { write_int_param(0x1B, params->chgut_rel,    false); }
    if (params->chg_t_delays  != old_params.chg_t_delays) { write_int_param(0x3A, params->chg_t_delays, false); }
    if (params->dsgot         != old_params.dsgot)        { write_int_param(0x1C, params->dsgot,        false); }
    if (params->dsgot_rel     != old_params.dsgot_rel)    { write_int_param(0x1D, params->dsgot_rel,    false); }
    if (params->dsgut         != old_params.dsgut)        { write_int_param(0x1E, params->dsgut,        false); }
    if (params->dsgut_rel     != old_params.dsgut_rel)    { write_int_param(0x1F, params->dsgut_rel,    false); }
    if (params->dsg_t_delays  != old_params.dsg_t_delays) { write_int_param(0x3B, params->dsg_t_delays, false); }

    bool success = param_success();
    #ifdef BMS_OPTION_DEBUG_PARAM
    // Serial.println("Exit factory mode");
    #endif
    if (success) {
        exit_factory_mode(true); // Write to EEPROM
    }
    else {
        exit_factory_mode(false); // Do not write to EEPROM
    }
}

// # of cells
uint8_t OverkillSolarBms2::get_0x2F_num_cells() {
    uint16_t resp = read_int_param(0x2F);
    return (uint8_t)(resp & 0xFF);
}
void OverkillSolarBms2::set_0x2F_num_cells(uint8_t num_cells, bool save) {
    write_int_param(0x2F, (uint16_t)num_cells, save);
}

// NTC Settings (16-bitfield)
uint16_t OverkillSolarBms2::get_0x2E_ntc_settings() {
    return read_int_param(0x2E);
}
void OverkillSolarBms2::set_0x2E_ntc_settings(uint16_t flags, bool save) {
    write_int_param(0x2E, flags, save);
}

// Configuration Flags (16-bitfield)
uint16_t OverkillSolarBms2::get_0x2D_config_flags() {
    return read_int_param(0x2D);
}
void OverkillSolarBms2::set_0x2D_config_flags(uint16_t flags, bool save) {
    write_int_param(0x2D, flags, save);
}

// Shunt resistor value (milliohms)
uint32_t OverkillSolarBms2::get_0x2C_shunt_resistor_value() {
    return read_int_param(0x2C, 0.1);
}
void OverkillSolarBms2::set_0x2C_shunt_resistor_value(uint32_t milliohms, bool save) {
    write_int_param(0x2C, milliohms, 0.1, save);
}

// TODO: 0xE2 Balance control

// ############################################################################
// Capacity Parameters
// ############################################################################

// Designed capacity, in milliamp hours
uint32_t OverkillSolarBms2::get_0x10_designed_capacity() {
    return read_int_param(0x10, 10);
}
void OverkillSolarBms2::set_0x10_designed_capacity(uint32_t capacity, bool save) {
    write_int_param(0x10, capacity, 10, save);
}

// Cycle capacity, in milliamp hours
uint32_t OverkillSolarBms2::get_0x11_cycle_capacity() {
    return read_int_param(0x11, 10);
}
void OverkillSolarBms2::set_0x11_cycle_capacity(uint32_t capacity, bool save) {
    write_int_param(0x11, capacity, 10, save);
}

// Full Charge Voltage, in millivolts
uint16_t OverkillSolarBms2::get_0x12_full_charge_voltage() {
    return read_int_param(0x12);
}
void OverkillSolarBms2::set_0x12_full_charge_voltage(uint16_t voltage, bool save) {
    write_int_param(0x12, voltage, save);
}

// End of Discharge Voltage (0% capacity voltage), in millivolts
uint16_t OverkillSolarBms2::get_0x13_end_of_discharge_voltage() {
    return read_int_param(0x13);
}
void OverkillSolarBms2::set_0x13_end_of_discharge_voltage(uint16_t voltage, bool save) {
    write_int_param(0x13, voltage, save);
}

// Discharge rate, in percent
float OverkillSolarBms2::get_0x14_discharge_rate() {
    return read_float_param(0x14, 0.1);
}
void OverkillSolarBms2::set_0x14_discharge_rate(float percent, bool save) {
    write_float_param(0x14, percent, 0.1, save);
}

// 80/60/40/20% capacity voltage, in millivolts
uint16_t OverkillSolarBms2::get_0x3x_capacity_voltage(uint8_t percent) {
    uint8_t cmd_code = 0x32;
    switch (percent) {
        case 80:
            cmd_code = 0x32;
            break;
        case 60:
            cmd_code = 0x33;
            break;
        case 40:
            cmd_code = 0x34;
            break;
        case 20:
            cmd_code = 0x35;
            break;
        default:
            // Serial.println("get_0x3x_capacity_voltage() percent must be 80, 60, 40, or 20");
            m_param_success &= false;
            break;
    }

    return read_int_param(cmd_code);
}
void OverkillSolarBms2::set_0x3x_capacity_voltage(uint8_t percent, uint16_t voltage, bool save) {
    // 80/60/40/20% capacity voltage, mV
    switch (percent) {
        case 80:
            write_int_param(0x32, voltage, save);
            break;
        case 60:
            write_int_param(0x33, voltage, save);
            break;
        case 40:
            write_int_param(0x34, voltage, save);
            break;
        case 20:
            write_int_param(0x35, voltage, save);
            break;
        case 0:
            write_int_param(0x13, voltage, save);
            break;
        default:
            break;
    }
}


// ############################################################################
// Balance Parameters
// ############################################################################

// Start voltage.  Scale=1, unit=mV
uint16_t OverkillSolarBms2::get_0x2A_start_voltage() {
    return read_int_param(0x2A);
}
void OverkillSolarBms2::set_0x2A_start_voltage(uint16_t voltage, bool save) {
    // Start voltage
    write_int_param(0x2A, (uint16_t)voltage, save);
}

// Delta to balance.  Scale=1, units=mV
uint16_t OverkillSolarBms2::get_0x2B_delta_to_balance() {
    return read_int_param(0x2B);
}
void OverkillSolarBms2::set_0x2B_delta_to_balance(uint16_t delta, bool save) {
    write_int_param(0x2B, delta, save);
}


// ############################################################################
// BMS Metadata
// ############################################################################

// TODO: 0x15 Manufacture date (15:9=year, 8:5=month, 4:0=day)
// TODO: 0x16 Serial number
// TODO: 0x17 Cycle count
// TODO: 0xA0 Manufacturer name
// TODO: 0xAA Error condition counts

// Barcode
String OverkillSolarBms2::get_0xA2_barcode() {
    String barcode = "";
    return barcode;
}
void OverkillSolarBms2::set_0xA2_barcode(String barcode, bool save) {
}


// BMS Name
String OverkillSolarBms2::get_0xA1_bms_name() {
    String name = "";
    return name;
}
void OverkillSolarBms2::set_0xA1_bms_name(String bmsname, bool save) {
}


uint16_t OverkillSolarBms2::get_serial_number() {
    return 0;
}
void OverkillSolarBms2::set_serial_number(uint16_t sn, bool save) {
}

// Manufacture date (15:9=year, 8:5=month, 4:0=day)
// 5432109876543210
// 1111110000000000
// yyyyyyymmmmddddd
Date OverkillSolarBms2::get_0x15_mfg_date() {

    uint16_t resp = read_int_param(0x15);
    Date date;
    date.year = (resp >> 9) & 0b111111;
    date.month = (resp >> 5) & 0b1111;
    date.day = resp & 0b11111;
    return date;
}
void OverkillSolarBms2::set_0x15_mfg_date(Date date, bool save) {
    uint16_t _date = 0;
    _date |= ((date.year & 0b111111) << 9);
    _date |= ((date.month & 0b1111) << 5);
    _date |= ((date.day & 0b11111));
    write_int_param(0x15, (uint16_t)_date, save);
}



// ############################################################################
// Protection Params (Voltage)

// Battery Voltage ------------------------------------------------------------

// Battery over voltage (trigger value), in millivolts
uint32_t OverkillSolarBms2::get_0x20_batt_over_volt_trig() {
    return read_int_param(0x20, 10);
}
void OverkillSolarBms2::set_0x20_batt_over_volt_trig(uint32_t trigger, bool save) {
    write_int_param(0x20, trigger, 10, save);
}

// Battery over voltage (release value), in millivolts
uint32_t OverkillSolarBms2::get_0x21_batt_over_volt_release() {
    return read_int_param(0x21, 10);
}
void OverkillSolarBms2::set_0x21_batt_over_volt_release(uint32_t release, bool save) {
    write_int_param(0x21, release, 10, save);
}

// Battery under voltage (trigger value), in millivolts
uint32_t OverkillSolarBms2::get_0x22_batt_under_volt_trig() {
    return read_int_param(0x22, 10);
}
void OverkillSolarBms2::set_0x22_batt_under_volt_trig(uint32_t trigger, bool save) {
    write_int_param(0x22, trigger, 10, save);
}

// Battery under voltage (release value), in millivolts
uint32_t OverkillSolarBms2::get_0x23_batt_under_volt_release() {
    return read_int_param(0x23, 10);
}
void OverkillSolarBms2::set_0x23_batt_under_volt_release(uint32_t release, bool save) {
    write_int_param(0x23, (uint16_t)release, save);
}

// Delays (MSB=Battery under-voltage, LSB=Battery over-voltage), in seconds
// NOTE: Param contains 2x 8-bit values packed into a 16-bit register.
DelayParamTuple OverkillSolarBms2::get_0x3C_delay_batt_volt() {
    uint16_t resp = read_int_param(0x3C);
    DelayParamTuple temps;
    temps.lsb = resp & 0xFF;
    temps.msb = resp >> 8;
    return temps;
}
void OverkillSolarBms2::set_0x3C_delay_batt_volt(DelayParamTuple delay, bool save) {
    write_int_param(0x3C, (uint16_t)(((delay.msb & 0xF) << 4) | (delay.lsb & 0xF)), save);
}

// Cell Voltage ---------------------------------------------------------------

// Cell over voltage (trigger value), in millivolts
uint16_t OverkillSolarBms2::get_0x24_cell_over_volt_trig() {
    return read_int_param(0x24);
}
void OverkillSolarBms2::set_0x24_cell_over_volt_trig(uint16_t trigger, bool save) {
    write_int_param(0x24, trigger, save);
}

// Cell over voltage (release value), in millivolts
uint16_t OverkillSolarBms2::get_0x25_cell_over_volt_release() {
    return read_int_param(0x25);
}
void OverkillSolarBms2::set_0x25_cell_over_volt_release(uint16_t release, bool save) {
    write_int_param(0x25, release, save);
}

// Cell under voltage (trigger value), in millivolts
uint16_t OverkillSolarBms2::get_0x26_cell_under_volt_trig() {
    return read_int_param(0x26);
}
void OverkillSolarBms2::set_0x26_cell_under_volt_trig(uint16_t trigger, bool save) {
    write_int_param(0x26, trigger, save);
}

// Cell under voltage (release value), in millivolts
uint16_t OverkillSolarBms2::get_0x27_cell_under_volt_release() {
    return read_int_param(0x27);
}
void OverkillSolarBms2::set_0x27_cell_under_volt_release(uint16_t release, bool save) {
    write_int_param(0x27, release, save);
}

// Delays (MSB=Cell under voltage, LSB=Cell over voltage), in seconds
// NOTE: Param contains 2x 8-bit values packed into a 16-bit register.
DelayParamTuple OverkillSolarBms2::get_0x3D_delay_cell_volt() {
    uint16_t resp = read_int_param(0x3D);
    DelayParamTuple temps;
    temps.lsb = resp & 0xFF;
    temps.msb = resp >> 8;
    return temps;
}
void OverkillSolarBms2::set_0x3D_delay_cell_volt(DelayParamTuple delay, bool save) {
    write_int_param(0x3D, (uint16_t)(((delay.msb & 0xF) << 4) | (delay.lsb & 0xF)), save);
}

// ############################################################################
// Protection Params (Current)

// Charge current -------------------------------------------------------------

// Charge over current (trigger value), in milliamps
uint32_t OverkillSolarBms2::get_0x28_charge_over_current_trig() {
    return read_int_param(0x28, 10);
}
void OverkillSolarBms2::set_0x28_charge_over_current_trig(uint32_t trigger, bool save) {
    write_int_param(0x28, trigger, 10, save);
}

// Delays (MSB=Charge over-current delay, LSB=Charge over-current release delay), in seconds
// NOTE: Param contains 2x 8-bit values packed into a 16-bit register.
DelayParamTuple OverkillSolarBms2::get_0x3E_delay_charge_current_delay() {
    uint16_t resp = read_int_param(0x3E);
    DelayParamTuple temps;
    temps.lsb = resp & 0xFF;
    temps.msb = resp >> 8;
    return temps;
}
void OverkillSolarBms2::set_0x3E_delay_charge_current_delay(DelayParamTuple delay, bool save) {
    write_int_param(0x3E, (uint16_t)(((delay.msb & 0xF) << 4) | (delay.lsb & 0xF)), save);
}

// Discharge current ----------------------------------------------------------

// Discharge over current (release value), in milliamps
// Set/get value will is always positive
uint32_t OverkillSolarBms2::get_0x29_discharge_over_current_release() {
    uint16_t resp = read_int_param(0x29);
    return (uint32_t)((65536 - (int32_t)resp) * 10);
}
void OverkillSolarBms2::set_0x29_discharge_over_current_release(uint32_t release, bool save) {
    int16_t _release = (release / 10) * -1;
    write_int_param(0x29, (uint16_t)_release, save);
}

// Delays (MSB=Discharge over-current delay, LSB=Discharge over-current reset delay), in seconds
// NOTE: Param contains 2x 8-bit values packed into a 16-bit register.
DelayParamTuple OverkillSolarBms2::get_0x3F_delay_discharge_current_delay() {
    uint16_t resp = read_int_param(0x3F);
    DelayParamTuple temps;
    temps.lsb = resp & 0xFF;
    temps.msb = resp >> 8;
    return temps;
}
void OverkillSolarBms2::set_0x3F_delay_discharge_current_delay(DelayParamTuple delay, bool save) {
    write_int_param(0x3F, (uint16_t)(((delay.msb & 0xF) << 4) | (delay.lsb & 0xF)), save);
}

// ############################################################################
// Protection Params (Temperature)

// Charge under/over temp -----------------------------------------------------

// Charge over-temperature (trigger), in degrees C
float OverkillSolarBms2::get_0x18_charge_over_temp_trig() {
    return read_temperature_param(0x18);
}
void OverkillSolarBms2::set_0x18_charge_over_temp_trig(float trig, bool save) {
    write_temperature_param(0x18, trig, save);
}

// Charge over-temperature (release), in degrees C
float OverkillSolarBms2::get_0x19_charge_over_temp_release() {
    return read_temperature_param(0x19);
}
void OverkillSolarBms2::set_0x19_charge_over_temp_release(float release, bool save) {
    write_temperature_param(0x19, release, save);
}

// Charge under-temperature (trigger), in degrees C
float OverkillSolarBms2::get_0x1A_charge_under_temp_trig() {
    return read_temperature_param(0x1A);
}
void OverkillSolarBms2::set_0x1A_charge_under_temp_trig(float trig, bool save) {
    write_temperature_param(0x1A, trig, save);
}

// Charge under-temperature (release), in degrees C
float OverkillSolarBms2::get_0x1B_charge_under_temp_release() {
    return read_temperature_param(0x1B);
}
void OverkillSolarBms2::set_0x1B_charge_under_temp_release(float release, bool save) {
    write_temperature_param(0x1B, release, save);
}

// Delays (MSB=Charge under temp, LSB=Charge over-temp), in seconds
DelayParamTuple OverkillSolarBms2::get_0x3A_delay_charge_temp() {
    uint16_t resp = read_int_param(0x3A);
    DelayParamTuple temps;
    temps.lsb = resp & 0xFF;
    temps.msb = resp >> 8;
    return temps;
}
void OverkillSolarBms2::set_0x3A_delay_charge_temp(DelayParamTuple delay, bool save) {
    write_int_param(0x3A, (uint16_t)(((delay.msb & 0xF) << 4) | (delay.lsb & 0xF)), save);
}

// Discharge under/over temp --------------------------------------------------

// Discharge over-temperature (trigger), in degrees C
float OverkillSolarBms2::get_0x1C_discharge_over_temp_trig() {
    return read_temperature_param(0x1C);
}
void OverkillSolarBms2::set_0x1C_discharge_over_temp_trig(float trig, bool save) {
    write_temperature_param(0x1C, trig, save);
}

// Discharge over-temperature (release), in degrees C
float OverkillSolarBms2::get_0x1D_discharge_over_temp_release() {
    return read_temperature_param(0x1D);
}
void OverkillSolarBms2::set_0x1D_discharge_over_temp_release(float release, bool save) {
    write_temperature_param(0x1D, release, save);
}

// Discharge under-temperature (trigger), in degrees C
float OverkillSolarBms2::get_0x1E_discharge_under_temp_trig() {
    return read_temperature_param(0x1E);
}
void OverkillSolarBms2::set_0x1E_discharge_under_temp_trig(float trig, bool save) {
    write_temperature_param(0x1E, trig, save);
}

// Discharge under-temperature (release), in degrees C
float OverkillSolarBms2::get_0x1F_discharge_under_temp_release() {
    return read_temperature_param(0x1F);
}
void OverkillSolarBms2::set_0x1F_discharge_under_temp_release(float release, bool save) {
    write_temperature_param(0x1F, release, save);
}

// Delays (MSB=Discharge under-temp, LSB=Discharge over-temp), in seconds
// NOTE: Param contains 2x 8-bit values packed into a 16-bit register.
DelayParamTuple OverkillSolarBms2::get_0x3B_delay_discharge_temp() {
    uint16_t resp = read_int_param(0x3B);
    DelayParamTuple temps;
    temps.lsb = resp & 0xFF;
    temps.msb = resp >> 8;
    return temps;
}
void OverkillSolarBms2::set_0x3B_delay_discharge_temp(DelayParamTuple delay, bool save) {
    write_int_param(0x3B, (uint16_t)(((delay.msb & 0xF) << 4) | (delay.lsb & 0xF)));
}

// ############################################################################
// Hardware Protection Parameters

// TODO: 0x36 Secondary cell overvoltage protection
// TODO: 0x37 Secondary cell undervoltage protection
// TODO: 0x38 Short-circuit and Secondary overcurrent settings
// TODO: 0x39 Secondary cell under/over voltage release times, and short-circuit release time


// ############################################################################
// Calibration, Voltages

// Specify voltage in VOLTS
void OverkillSolarBms2::set_0xBx_cell_calibration(uint8_t cell, float voltage, bool save) {
    if (cell >= 32) {
        m_param_success = false;
        return;
    }
    write_float_param(0xB0 + cell, voltage, 0.001, save);  // 1 mV scale
}

// ############################################################################
// Calibration, Current

// Only call when NOT charging or discharging!
void OverkillSolarBms2::set_0xAD_idle_current_calibration(bool save) {
    write_int_param(0xAD, 0x00, save);
}

// Specify current in AMPS
void OverkillSolarBms2::set_0xAE_charge_current_calibration(float current, bool save) {
    write_float_param(0xAE, current, 0.01, save);
}

// Specify current in AMPS
void OverkillSolarBms2::set_0xAF_discharge_current_calibration(float current, bool save) {
    // NOTE: Always write the current as positive
    write_float_param(0xAF, abs(current), 0.01, save);
}


// ############################################################################
// Calibration, Temperature

// Specify current in Degrees Celsius
void OverkillSolarBms2::set_0xDx_temp_calibration(uint8_t ntc, float temperature, bool save) {
    if (ntc >= 8) {
        m_param_success = false;
        return;
    }
    write_temperature_param(0xD0 + ntc, temperature, save);
}


// ############################################################################
// Calibration, Capacity, in milliamp hours

// Specify capacity in Amp hours
void OverkillSolarBms2::set_0xE0_capacity_remaining_calibration(float capacity, bool save) {
    write_float_param(0xE0, capacity, 0.01, save);
}

// ###########################################################################
// Messages to and from the BMS

void OverkillSolarBms2::query_0x03_basic_info() {
    #ifdef BMS_OPTION_DEBUG
        // Serial.println("Query 0x03 Basic Info");
    #endif

    // CLear data before read
    m_0x03_basic_info.voltage = 0;
    m_0x03_basic_info.current = 0;
    m_0x03_basic_info.balance_capacity = 0;
    m_0x03_basic_info.rate_capacity = 0;
    m_0x03_basic_info.cycle_count = 0;
    m_0x03_basic_info.production_date = 0;
    m_0x03_basic_info.balance_status = 0;
    m_0x03_basic_info.protection_status = 0;
    m_0x03_basic_info.software_version = 0;
    m_0x03_basic_info.remaining_soc = 0;
    m_0x03_basic_info.mosfet_status = 0;
    m_0x03_basic_info.num_cells = 0;
    m_0x03_basic_info.num_ntcs = 0;

    for (uint8_t i=0; i<BMS_MAX_NTCs; i++) {
        m_0x03_basic_info.ntc_temps[i] = 0;
    }


    uint8_t length = 0;
    uint8_t *data = NULL;

    m_param_timeout = false;

    for (uint8_t i=0; i<3; i++) {  // 3 attempts
        write(BMS_READ, BMS_REG_BASIC_SYSTEM_INFO, data, length);

        uint32_t t0 = millis();

        // Handle the incoming serial data and block until the basic info is received
        while(1) {
            serial_rx_task();
            delay(10);

            if (m_last_0x03_timestamp > t0) {
                break;
            }

            // Timeout if it took too long
            if (millis() - t0 >= BMS_TIMEOUT ) {
                m_param_success = false;
                m_param_timeout = true;
                break;
            }
        }

        if (m_param_success) {
            break;
        }
        else {
            if (i < 3) {
                m_param_success = true;
            }
        }
    }
}

void OverkillSolarBms2::query_0x04_cell_voltages() {
    #ifdef BMS_OPTION_DEBUG
        // Serial.println("Query 0x04 Cell Voltages");
    #endif

    // Clear register before read
    for (uint8_t i=0; i<BMS_MAX_CELLS; i++) {
        m_0x04_cell_voltages[i] = 0;
    }

    uint8_t length = 0;
    uint8_t *data = NULL;
    write(BMS_READ, BMS_REG_CELL_VOLTAGES, data, length);

    m_param_timeout = false;
    uint32_t t0 = millis();
    // Handle the incoming serial data and block until the voltages are received
    while(1) {
        serial_rx_task();
        delay(10);

        if (m_last_0x04_timestamp > t0) {
            break;
        }

        // Timeout if it took too long
        if (millis() - t0 >= BMS_TIMEOUT ) {
            m_param_success = false;
            m_param_timeout = true;
            break;
        }
    }
}

// ###########################################################################
// Proprietery params

// Write the byte sequence 0x5678 to 0x00 to enter "Factory Mode."  
// In this mode, the param registers can be accessed.
bool OverkillSolarBms2::enter_factory_mode() {
    #ifdef BMS_OPTION_DEBUG_PARAM
    // Serial.print(F("\nEnter factory mode..."));
    #endif
    uint8_t data[2];
    data[0] = 0x56;
    data[1] = 0x78;
    bool success = true;
    write(BMS_WRITE, 0x00, (uint8_t*)&data, 2);
    atomic_param_read(0x00, 100);
    if (m_param_timeout) {
        success = false;
        #ifdef BMS_OPTION_DEBUG_PARAM
        // Serial.print(F(" enter_factory_mode_timeout "));
        #endif
    }
    else if (m_rx_status != 0x00) {
        success = false;
        #ifdef BMS_OPTION_DEBUG_PARAM
        // Serial.print(F(" enter_factory_mode_NAK "));
        #endif
    }
    else {
        #ifdef BMS_OPTION_DEBUG_PARAM
        // Serial.println(F("Success"));
        #endif
        success = true;
        m_in_factory_mode = true;
    }
    return success;
}

// Write the byte sequence 0x0000 to 0x01 to exit "Factory Mode," and update the values in the EEPROM.
// Write the byte sequence 0x2828 to 0x01 to exit "Factory Mode," update the values in the EEROM, and reset the "Error Counts" (0xAA) register to zeroes
bool OverkillSolarBms2::exit_factory_mode(bool save) {
    #ifdef BMS_OPTION_DEBUG_PARAM
    // Serial.print(F("\nExit factory mode..."));
    #endif
    uint8_t data[2];
    if (save) {
        data[0] = 0x28;
        data[1] = 0x28;
    }
    else {
        data[0] = 0x00;
        data[1] = 0x00;
    }
    bool success = true;
    write(BMS_WRITE, 0x01, (uint8_t*)&data, 2);
    atomic_param_read(0x01, 100);
    if (m_param_timeout) {
        success = false;
        #ifdef BMS_OPTION_DEBUG_PARAM
        // Serial.print(F(" exit_factory_mode_timeout "));
        #endif
    }
    else if (m_rx_status != 0x00) {
        success = false;
        #ifdef BMS_OPTION_DEBUG_PARAM
        // Serial.print(F(" exit_factory_mode_NAK "));
        #endif
    }
    else {
        #ifdef BMS_OPTION_DEBUG_PARAM
        // Serial.println(F("Success"));
        #endif
        success = true;
        m_in_factory_mode = false;
    }
    delay(save?3500:1000);
    return success;
}


uint16_t OverkillSolarBms2::read_int_param(uint8_t cmd_code) {
    bool success = true;
    uint16_t resp = 0;

    if (!m_in_factory_mode) {
        enter_factory_mode();
    }

    #ifdef BMS_OPTION_DEBUG_PARAM
        // Serial.print(F("read_int_param("));
        // Serial.print(cmd_code, HEX);
        // Serial.print(F("): "));
    #endif
    for (uint8_t i=0; i<5; i++) {
        write(BMS_READ, cmd_code, NULL, 0);
        delay(10);
        resp = atomic_param_read(cmd_code, 500);

        if (m_param_timeout) {
            success = false;
            #ifdef BMS_OPTION_DEBUG_PARAM
            // Serial.print(F(" timeout "));
            #endif
        }
        else if (m_rx_status != 0x00) {
            success = false;
            #ifdef BMS_OPTION_DEBUG_PARAM
            // Serial.print(" NAK ");
            #endif
            exit_factory_mode(false);
            enter_factory_mode();
        }
        else {
            success = true;
            break;
        }
    }

    #ifdef BMS_OPTION_DEBUG_PARAM
        if (success) {
            // Serial.print(F("0x"));
            // Serial.println(resp, HEX);
        }
        else {
            // Serial.println(" Fail! ");
        }
    #endif

    m_param_success &= success;
    return resp;
}

uint32_t OverkillSolarBms2::read_int_param(uint8_t cmd_code, float scale) {
    uint16_t resp = read_int_param(cmd_code);
    if (m_param_success) {
        return (uint32_t)(resp * scale);
    }
    else {
        // // Serial.print("HERE! resp was:");
        // // Serial.println(resp, DEC);
        return 0;
    }
}

void OverkillSolarBms2::write_int_param(uint8_t cmd_code, uint16_t value, bool save) {
    uint8_t data[2];
    data[0] = (uint8_t)((value >> 8) & 0xFF);
    data[1] = (uint8_t)(value & 0xFF);

    bool old_param_success = m_param_success;
    bool success;
    // uint16_t resp;

    #ifdef BMS_OPTION_DEBUG_PARAM
        // Serial.print(F("\nwrite_int_param("));
        // Serial.print(cmd_code, HEX);
        // Serial.print(F(", "));
        // Serial.print(value, HEX);
        // Serial.print(F("): "));
    #endif
    for (uint8_t i=0; i<5; i++) {
        success = true;
        m_param_success = true;

        if (!m_in_factory_mode) {
            success &= enter_factory_mode();
        }

        write(BMS_WRITE, cmd_code, (uint8_t*)&data, 2);
        delay(300);
        atomic_param_read(cmd_code, 100);

        if (m_param_timeout) {
            success = false;
            #ifdef BMS_OPTION_DEBUG_PARAM
            // Serial.println(" Timeout");
            #endif
        }
        else if (m_rx_status != 0x00) {
            success = false;
            #ifdef BMS_OPTION_DEBUG_PARAM
            // Serial.println(" NAK ");
            #endif
            exit_factory_mode(true);
            enter_factory_mode();
        }
        else {
            // if (resp == value)
            success = true;
            // #ifdef BMS_OPTION_DEBUG_PARAM
            // Serial.println("Write Param Success!");
            // #endif
            if (save) {
                success &= exit_factory_mode(true);
            }
            break;
        }
    }
    m_param_success = old_param_success & success;
}

void OverkillSolarBms2::write_int_param(uint8_t cmd_code, uint32_t value, float scale, bool save) {
    write_int_param(cmd_code, (uint16_t)(value / scale), save);
}


float OverkillSolarBms2::read_float_param(uint8_t cmd_code, float scale) {
    return (float)(read_int_param(cmd_code) * scale);
}
void OverkillSolarBms2::write_float_param(uint8_t cmd_code, float value, float scale, bool save) {
    write_int_param(cmd_code, (uint16_t)(value / scale), save);
}

// Native units are 0.1 degress K.
float OverkillSolarBms2::read_temperature_param(uint8_t cmd_code) {
    uint16_t t = read_int_param(cmd_code);
    if (m_param_success) {
        return ((float)t * 0.1) - 273.15;
    }
    else {
        return 0.0;
    }
}
void OverkillSolarBms2::write_temperature_param(uint8_t cmd_code, float value, bool save) {
    write_int_param(cmd_code, (uint16_t)((value + 271.15) / 0.1));
}


bool OverkillSolarBms2::param_success() {
    return m_param_success;
}

void OverkillSolarBms2::param_clear_errors() {
    m_param_success = true;
}

uint16_t OverkillSolarBms2::atomic_param_read(uint8_t cmd_code, uint32_t timeout) {
    uint32_t t0 = millis();
    m_param = 0;
    m_param_timeout = false;
    delay(1); // Theory goes this will cause extremly-fast replies evaluate if (t>t0) correctly

    while(1) {
        // Handle the incoming serial data and block until the reply is received
        serial_rx_task();
        delay(10);

        if (cmd_code == 0x00) {
            if (m_last_0x00_timestamp > t0) {
                // // Serial.println("atomic_param_read(): Success");
                break;
            }
        }
        else if (cmd_code == 0x01) {
            if (m_last_0x01_timestamp > t0) {
                break;
            }
        }
        else {
            if (m_last_param_timestamp > t0) {
                break;
            }
        }

        // Timeout if it took too long
        if (millis() - t0 >= timeout ) {
            m_param_timeout = true;
            break;
        }
    }
    return m_param;  // RX task will conveniently write the param value to this pointer
}

uint16_t OverkillSolarBms2::atomic_param_read(uint8_t cmd_code) {
    return atomic_param_read(cmd_code, BMS_PARAM_TIMEOUT);
}


// ###########################################################################
// Low-level read/write methods

// Write to BMS
void OverkillSolarBms2::write(uint8_t rw, uint8_t command_code, uint8_t* data, uint8_t length) {
    uint16_t checksum = 0;

    #ifdef BMS_OPTION_DEBUG_WRITE
        // Serial.print("<< ");
    #endif

    // Write the start byte, 0xDD
    m_serial->write(BMS_STARTBYTE);
    #ifdef BMS_OPTION_DEBUG_WRITE
        // Serial.print(BMS_STARTBYTE, HEX);
        // Serial.print(" ");
    #endif

    // Write the status byte, which indicates whether a read or write is being requested
    m_serial->write(rw);  // 0xA5 (read) or 0x5A (write)
    #ifdef BMS_OPTION_DEBUG_WRITE
        // Serial.print(rw, HEX);
        // Serial.print(" ");
    #endif

    // Write the command code (the register address)
    m_serial->write(command_code);
    checksum += command_code;
    #ifdef BMS_OPTION_DEBUG_WRITE
        // Serial.print(command_code, HEX);
        // Serial.print(" ");
    #endif

    // Write the length
    m_serial->write(length);
    checksum += length;
    #ifdef BMS_OPTION_DEBUG_WRITE
        // Serial.print(length, HEX);
        // Serial.print(" ");
    #endif

    // Write the data
    for (uint8_t i=0; i< length; i++) {
        m_serial->write(data[i]);
        #ifdef BMS_OPTION_DEBUG_WRITE
            // Serial.print(data[i], HEX);
            // Serial.print(" ");
        #endif
        checksum += data[i];
    }

    // Write the checksum
    checksum = (uint16_t)((0x10000UL) - (uint32_t)checksum);

    uint8_t checksum_msb = (uint8_t)((checksum >> 8) & 0xFF);
    m_serial->write(checksum_msb);
    #ifdef BMS_OPTION_DEBUG_WRITE
        // Serial.print(checksum_msb, HEX);
        // Serial.print(" ");
    #endif

    uint8_t checksum_lsb = (uint8_t)(checksum & 0xFF);
    m_serial->write(checksum_lsb);
    #ifdef BMS_OPTION_DEBUG_WRITE
        // Serial.print(checksum_lsb, HEX);
        // Serial.print(" ");
    #endif

    // Write the stop byte, 0x77
    m_serial->write(BMS_STOPBYTE);
    #ifdef BMS_OPTION_DEBUG_WRITE
        // Serial.println(BMS_STOPBYTE, HEX);
    #endif
}

void OverkillSolarBms2::serial_rx_task() {
    int bytes_available = m_serial->available();
    if (bytes_available > 0) {
        #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
            // Serial.print(bytes_available, DEC);
            // Serial.println(" bytes available to read");
        #endif
        for (int i=0; i < bytes_available; i++) {
            int c = m_serial->read();
            if (c == -1) {
                continue;
            }

            // // Serial.print("\n>> ");
            // // Serial.println(c, HEX);

            if (m_rx_state == BMS_STATE_WAIT_FOR_START_BYTE) {

                #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                    // Serial.print("[WAIT_FOR_START_BYTE]:  ");
                    // Serial.println(c, HEX);
                #endif
                if (c == BMS_STARTBYTE) {
                    m_rx_cmd_code = 0;
                    m_rx_status = 0;
                    m_rx_length = 0;
                    m_rx_data_index = 0;
                    m_rx_checksum = 0;

                    // Reset all of the state variables
                    m_rx_state = BMS_STATE_WAIT_FOR_CMD_CODE;
                }
                else {
                    // Error
                    // // Serial.println("");
                    // // Serial.println("Framing error! Not a start byte!");
                    m_num_rx_errors += 1;
                    m_rx_state = BMS_STATE_WAIT_FOR_START_BYTE;
                }
            }
            else if (m_rx_state == BMS_STATE_WAIT_FOR_CMD_CODE) {
                m_rx_cmd_code = c;
                #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                    // Serial.print("[WAIT_FOR_CMD_CODE]:    ");
                    // Serial.println(c, HEX);
                #endif
                m_rx_state = BMS_STATE_WAIT_FOR_STATUS_BYTE;
            }
            else if (m_rx_state == BMS_STATE_WAIT_FOR_STATUS_BYTE) {
                m_rx_status = c;
                #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                    // Serial.print("[WAIT_FOR_STATUS_BYTE]: ");
                    // Serial.println(c, HEX);
                #endif

                if (m_rx_status == 0x00) {
                    // The BMS should set the status to 0x00 if it is OK.
                }

                else if (m_rx_status == 0x80) {
                    #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                        // Serial.println("");
                        // Serial.println("RX error! Status byte should have been 0x00, but got 0x80!");
                    #endif
                    // An error occured
                    m_num_rx_errors += 1;
                }
                else {
                    // Any code other to us is still an error
                    #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                        // Serial.println("");
                        // Serial.println("RX error! Status byte should have been 0x00 or 0x80!");
                    #endif
                    m_num_rx_errors += 1;
                }
                m_rx_state = BMS_STATE_WAIT_FOR_LENGTH;
            }
            else if (m_rx_state == BMS_STATE_WAIT_FOR_LENGTH) {
                m_rx_length = c;
                #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                    // Serial.print("[WAIT_FOR_LENGTH]:      ");
                    // Serial.println(c, DEC);
                #endif
                if (m_rx_length > BMS_MAX_RX_DATA_LEN) {
                    // Error
                    // // Serial.println("");
                    m_num_rx_errors += 1;
                }
                if (m_rx_length == 0) {
                    // No data bytes, don't wait for data
                    m_rx_state = BMS_STATE_WAIT_FOR_CHECKSUM_MSB;
                }
                else {
                    m_rx_state = BMS_STATE_WAIT_FOR_DATA;
                }
            }
            else if (m_rx_state == BMS_STATE_WAIT_FOR_DATA) {
                #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                    // Serial.print("[WAIT_FOR_DATA]:        ");
                    // Serial.print(c, HEX);
                #endif
                if (m_rx_data_index + 1 <= BMS_MAX_RX_DATA_LEN) {
                    m_rx_data[m_rx_data_index] = c;
                }
                if (m_rx_data_index + 1 >= m_rx_length) {
                    m_rx_state = BMS_STATE_WAIT_FOR_CHECKSUM_MSB;
                }
                else {
                    // Keep the m_rx_state in BMS_STATE_WAIT_FOR_DATA,
                    // until all data bytes have been received
                    m_rx_data_index += 1;
                }
                #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                    // Serial.print(", rx_data_index=");
                    // Serial.println(m_rx_data_index, DEC);
                #endif
            }
            else if (m_rx_state == BMS_STATE_WAIT_FOR_CHECKSUM_MSB) {
                #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                    // Serial.print("[WAIT_FOR_CHECKSUM_MSB]: ");
                    // Serial.println(c, HEX);
                #endif
                m_rx_checksum = (c << 8) & 0xFF00;
                m_rx_state = BMS_STATE_WAIT_FOR_CHECKSUM_LSB;
            }
            else if (m_rx_state == BMS_STATE_WAIT_FOR_CHECKSUM_LSB) {
                #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                    // Serial.print("[WAIT_FOR_CHECKSUM_LSB]: ");
                    // Serial.println(c, HEX);
                #endif
                m_rx_checksum |= c;
                m_rx_state = BMS_STATE_WAIT_FOR_STOP_BYTE;
            }
            else if (m_rx_state == BMS_STATE_WAIT_FOR_STOP_BYTE) {
                #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                    // Serial.print("[WAIT_FOR_STOP_BYTE]: ");
                    // Serial.println(c, HEX);
                #endif

                // Calculate the checksum of the length and data bytes.
                // If it matches the received checksum, then continue.
                // Otherwise, flag it as an error and stop.
                uint16_t calc_checksum = 0;
                calc_checksum += m_rx_status;
                calc_checksum += m_rx_length;
                for (uint8_t i=0; i < m_rx_length; i++) {
                    calc_checksum += m_rx_data[i];
                }
                calc_checksum = (uint16_t)((0x10000UL) - (uint32_t)calc_checksum);

                #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                    if (m_rx_checksum != calc_checksum) {
                        // Serial.println("");
                        // Serial.println("Checksum did not calculate!");
                        // Serial.print("  Calculated: ");
                        // Serial.println(calc_checksum, HEX);
                        // Serial.print("  Received: ");
                        // Serial.println(m_rx_checksum, HEX);
                    }
                #endif

                if (m_rx_checksum == calc_checksum && m_rx_status == 0x00) {
                    if (c == BMS_STOPBYTE) {
                        // Everything looks OK, handle the data
                        #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
                            // Serial.println("Got a complete msg");
                        #endif
                        switch (m_rx_cmd_code) {
                            case 0x00:
                                handle_rx_0x00();
                                break;
                            case 0x01:
                                handle_rx_0x01();
                                break;
                            case BMS_REG_BASIC_SYSTEM_INFO:  // 0x03
                                handle_rx_0x03();
                                break;
                            case BMS_REG_CELL_VOLTAGES:  // 0x04
                                handle_rx_0x04();
                                break;
                            case BMS_REG_NAME:  // 0x05
                                handle_rx_0x05();
                                break;
                            // Config params
                            case 0x2F:
                            case 0x2E:
                            case 0x2D:
                            // Capacity params
                            case 0x10:
                            case 0x11:
                            case 0x14:
                            case 0x12:
                            case 0x32:
                            case 0x33:
                            case 0x34:
                            case 0x35:
                            case 0x13:
                            // Balance params
                            case 0x2A:
                            case 0x2B:
                            // Protection params (voltage)
                            case 0x20:
                            case 0x21:
                            case 0x22:
                            case 0x23:
                            case 0x3C:
                            case 0x24:
                            case 0x25:
                            case 0x26:
                            case 0x27:
                            case 0x3D:
                            // Protection params (current)
                            case 0x28:
                            case 0x3E:
                            case 0x29:
                            case 0x3F:
                            // Protection params (temperature)
                            case 0x18:
                            case 0x19:
                            case 0x1A:
                            case 0x1B:
                            case 0x3A:
                            case 0x1C:
                            case 0x1D:
                            case 0x1E:
                            case 0x1F:
                            case 0x3B:
                            // HW protection params
                            case 0x36:
                            case 0x37:
                            case 0x38:
                            case 0x39:
                            // Calibration, Voltages
                            case 0xB0:
                            case 0xB1:
                            case 0xB2:
                            case 0xB3:
                            case 0xB4:
                            case 0xB5:
                            case 0xB6:
                            case 0xB7:
                            case 0xB8:
                            case 0xB9:
                            case 0xBA:
                            case 0xBB:
                            case 0xBC:
                            case 0xBD:
                            case 0xBE:
                            case 0xBF:
                            case 0xC0:
                            case 0xC1:
                            case 0xC2:
                            case 0xC3:
                            case 0xC4:
                            case 0xC5:
                            case 0xC6:
                            case 0xC7:
                            case 0xC8:
                            case 0xC9:
                            case 0xCA:
                            case 0xCB:
                            case 0xCC:
                            case 0xCD:
                            case 0xCE:
                            case 0xCF:
                            // Calibration, Current
                            case 0xAD:
                            case 0xAE:
                            case 0xAF:
                            // Calibration, Temperature
                            case 0xD0:
                            case 0xD1:
                            case 0xD2:
                            case 0xD3:
                            case 0xD4:
                            case 0xD5:
                            case 0xD6:
                            case 0xD7:
                            // Calibration, Capacity
                            case 0xE0:
                                handle_rx_param();
                                break;
                            case 0xA2:
                                handle_rx_0xA2();
                                break;
                            case 0xA1:
                                handle_rx_0xA1();
                                break;
                            default:
                                #ifdef BMS_OPTION_DEBUG
                                    // Serial.print(F("Skipping unknown register: "));
                                    // Serial.println(m_rx_cmd_code, HEX);
                                #endif
                                break;
                        }

                    }
                    #ifdef BMS_OPTION_DEBUG
                        else {
                            // Serial.println("");
                            // Serial.println("Framing error!  Expected 0x77 stop byte!");
                            m_num_rx_errors += 1;
                        }
                    #endif
                }
                else {
                    m_num_rx_errors += 1;
                }

                m_rx_state = BMS_STATE_WAIT_FOR_START_BYTE;
            }
            // #ifdef BMS_OPTION_DEBUG
            else {
                // Serial.print(F("THIS SHOULD NEVER HAPPEN!  The rx_state was: "));
                // Serial.println(m_rx_state, DEC);
            }  // rx_state
            // #endif
        }  // for i in bytes_available
        #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
            // Serial.print("The rx_state is: ");
            // Serial.print(m_rx_state, DEC);
            // Serial.println("...");
        #endif
    }  // if bytes_available > 0
}

void OverkillSolarBms2::handle_rx_0x00() {
    #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
        // Serial.println("Got a 0x00 (Enter Factory Mode) reply");
    #endif
    if (m_rx_length == 0) {
        m_last_0x00_timestamp = millis();
    }
    else {
        // Serial.print("ERROR! Got a reply to 0x00 (Enter Factory mode) but length was ");
        // Serial.println(m_rx_length);
    }
}

void OverkillSolarBms2::handle_rx_0x01() {
    #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
        // Serial.println(F("Got a 0x01 (Exit Factory Mode ) reply"));
    #endif
    if (m_rx_length == 0) {
        m_last_0x01_timestamp = millis();
    }
    else {
        // Serial.print(F("ERROR! Got a reply to 0x01 (Exit Factory mode) but length was "));
        // Serial.println(m_rx_length);
    }
}

void OverkillSolarBms2::handle_rx_0x03() {
    #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
        // Serial.println("Got an 0x03 Basic Info msg");
    #endif
    m_0x03_basic_info.voltage           = (uint16_t)(m_rx_data[0]  << 8) | (uint16_t)(m_rx_data[1]);  // 0-1   Total voltage
    m_0x03_basic_info.current           = (uint16_t)(m_rx_data[2]  << 8) | (uint16_t)(m_rx_data[3]);  // 2-3   Current
    m_0x03_basic_info.balance_capacity  = (uint16_t)(m_rx_data[4]  << 8) | (uint16_t)(m_rx_data[5]);  // 4-5   Balance capacity
    m_0x03_basic_info.rate_capacity     = (uint16_t)(m_rx_data[6]  << 8) | (uint16_t)(m_rx_data[7]);  // 6-7   Rate capacity
    m_0x03_basic_info.cycle_count       = (uint16_t)(m_rx_data[8]  << 8) | (uint16_t)(m_rx_data[9]);  // 8-9   Cycle count
    m_0x03_basic_info.production_date   = (uint16_t)(m_rx_data[10] << 8) | (uint16_t)(m_rx_data[11]);  // 10-11 Production Date
    m_0x03_basic_info.balance_status    = (uint16_t)(m_rx_data[12] << 8) | (uint16_t)(m_rx_data[13]);  // 12-13, 14-15 Balance Status
    m_0x03_basic_info.protection_status = (uint16_t)(m_rx_data[16] << 8) | (uint16_t)(m_rx_data[17]);  // 16-17 Protection status

    // See if there are any new faults.  If so, then increment the count.
    if (has_new_fault_occured(0))  { m_fault_count.single_cell_overvoltage_protection    += 1; }
    if (has_new_fault_occured(1))  { m_fault_count.single_cell_undervoltage_protection   += 1; }
    if (has_new_fault_occured(2))  { m_fault_count.whole_pack_undervoltage_protection    += 1; }
    if (has_new_fault_occured(3))  { m_fault_count.single_cell_overvoltage_protection    += 1; }
    if (has_new_fault_occured(4))  { m_fault_count.charging_over_temperature_protection  += 1; }
    if (has_new_fault_occured(5))  { m_fault_count.charging_low_temperature_protection   += 1; }
    if (has_new_fault_occured(6))  { m_fault_count.discharge_over_temperature_protection += 1; }
    if (has_new_fault_occured(7))  { m_fault_count.discharge_low_temperature_protection  += 1; }
    if (has_new_fault_occured(8))  { m_fault_count.charging_overcurrent_protection       += 1; }
    if (has_new_fault_occured(9))  { m_fault_count.discharge_overcurrent_protection      += 1; }
    if (has_new_fault_occured(10)) { m_fault_count.short_circuit_protection              += 1; }
    if (has_new_fault_occured(11)) { m_fault_count.front_end_detection_ic_error          += 1; }
    if (has_new_fault_occured(12)) { m_fault_count.software_lock_mos                     += 1; }

    m_0x03_basic_info.software_version = m_rx_data[18];  // 18    Software version
    m_0x03_basic_info.remaining_soc    = m_rx_data[19];  // 19    Remaining state of charge
    m_0x03_basic_info.mosfet_status    = m_rx_data[20];  // 20    MOSFET status
    m_0x03_basic_info.num_cells        = m_rx_data[21];  // 21    # of batteries in series
    m_0x03_basic_info.num_ntcs         = m_rx_data[22];  // 22    # of NTCs

    for (uint8_t i=0; i < __min(BMS_MAX_NTCs, m_0x03_basic_info.num_ntcs); i++) {
        uint8_t ntc_index = 23 + (i * 2);
        m_0x03_basic_info.ntc_temps[i] = (uint16_t)(m_rx_data[ntc_index] << 8) | (uint16_t)(m_rx_data[ntc_index + 1]);
    }
    m_last_0x03_timestamp = millis();
}

void OverkillSolarBms2::handle_rx_0x04() {
    #ifdef BMS_OPTION_DEBUG_STATE_MACHINE
        // Serial.println("Got an 0x04 Cell Voltage msg");
    #endif
    for (uint8_t i=0; i < __min(BMS_MAX_CELLS, m_0x03_basic_info.num_cells); i++) {
        m_0x04_cell_voltages[i] = (uint16_t)(m_rx_data[i * 2] << 8) | (uint16_t)(m_rx_data[(i * 2) + 1]);
    }
    m_last_0x04_timestamp = millis();
}

void OverkillSolarBms2::handle_rx_0x05() {
    m_0x05_bms_name = String("");
    for (uint8_t i=0; i < __min(BMS_MAX_RX_DATA_LEN, m_rx_length); i++) {
        m_0x05_bms_name += (char)m_rx_data[i];
    }
}

void OverkillSolarBms2::handle_rx_param() {
    // uint16_t resp;
    // // Serial.print("Got RX param.  cmd_code:");
    // // Serial.print(m_rx_cmd_code, HEX);
    // // Serial.print(", length: ");
    // // Serial.print(m_rx_length);
    if (m_rx_length == 0) {  // Reply to write command
        m_last_param_timestamp = millis();
    }
    else if (m_rx_length == 2) {  // Reply to read command
        m_param = (uint16_t)(m_rx_data[0] << 8) | (uint16_t)(m_rx_data[1]);
        // // Serial.print(", rx_data: ");
        // // Serial.println(m_param, HEX);
        m_last_param_timestamp = millis();
    }
    else {
        #ifdef BMS_OPTION_DEBUG_PARAM
            // Serial.print(F("ERROR! Got a reply to 0x"));
            // Serial.print(m_rx_cmd_code, HEX);
            // Serial.print(F(" but length was "));
            // Serial.println(m_rx_length);
        #endif
    }
    // // Serial.println("");
}

void OverkillSolarBms2::handle_rx_0xA2() {
    // TODO: Handle the barcode here
    m_last_param_timestamp = millis();
}

void OverkillSolarBms2::handle_rx_0xA1() {
    // TODO: Handle the BMS name here
    m_last_param_timestamp = millis();
}


// ############################################################################
// Debug

void OverkillSolarBms2::print_config_params() {
    // Serial.print(F("# cells:                     "));
    // Serial.println(get_0x2F_num_cells());

    // Serial.print(F("Config flags:                "));
    // Serial.println(get_0x2D_config_flags(), BIN);
}

void OverkillSolarBms2::print_capacity_params() {
    // Serial.print(F("Designed capacity:           "));
    // Serial.println(get_0x10_designed_capacity());

    // Serial.print(F("Cycle_capacity:              "));
    // Serial.println(get_0x11_cycle_capacity());

    // Serial.print(F("Full charge voltage:         "));
    // Serial.println(get_0x12_full_charge_voltage());

    // Serial.print(F("End-of-discharge voltage:    "));
    // Serial.println(get_0x13_end_of_discharge_voltage());

    // Serial.print(F("Discharge rate:              "));
    // Serial.println(get_0x14_discharge_rate());

    // Serial.print(F("Capacity voltage, 80%:       "));
    // Serial.println(get_0x3x_capacity_voltage(80));
    // Serial.print(F("Capacity voltage, 60%:       "));
    // Serial.println(get_0x3x_capacity_voltage(60));
    // Serial.print(F("Capacity voltage, 40%:       "));
    // Serial.println(get_0x3x_capacity_voltage(40));
    // Serial.print(F("Capacity voltage, 20%:       "));
    // Serial.println(get_0x3x_capacity_voltage(20));
}

void OverkillSolarBms2::print_balance_params() {
    // Serial.print(F("Start voltage:               "));
    // Serial.println(get_0x2A_start_voltage());
    // Serial.print(F("Delta to balance:            "));
    // Serial.println(get_0x2B_delta_to_balance());
}

void OverkillSolarBms2::print_bms_metadata() {
}

void OverkillSolarBms2::print_protection_params_voltage() {
    // Serial.print(F("batt over volt trig:         "));
    // Serial.println(get_0x20_batt_over_volt_trig());
    // Serial.print(F("batt over volt release:      "));
    // Serial.println(get_0x21_batt_over_volt_release());
    // Serial.print(F("batt under volt trig:        "));
    // Serial.println(get_0x22_batt_under_volt_trig());
    // Serial.print(F("batt under volt release:     "));
    // Serial.println(get_0x23_batt_under_volt_release());

    DelayParamTuple delay_batt_volt = get_0x3C_delay_batt_volt();
    // Serial.print(F("delay battery under-volt:                  "));
    // Serial.println(delay_batt_volt.msb);
    // Serial.print(F("delay battery over-volt:                   "));
    // Serial.println(delay_batt_volt.lsb);

    // Serial.print(F("cell over volt trig:         "));
    // Serial.println(get_0x24_cell_over_volt_trig());
    // Serial.print(F("cell over volt release:      "));
    // Serial.println(get_0x25_cell_over_volt_release());
    // Serial.print(F("cell under volt trig:        "));
    // Serial.println(get_0x26_cell_under_volt_trig());
    // Serial.print(F("cell under volt release:     "));
    // Serial.println(get_0x27_cell_under_volt_release());

    DelayParamTuple delay_cell_volt = get_0x3D_delay_cell_volt();
    // Serial.print(F("delay cell under-volt:                     "));
    // Serial.println(delay_cell_volt.msb);
    // Serial.print(F("delay cell over-volt:                      "));
    // Serial.println(delay_cell_volt.lsb);
}

void OverkillSolarBms2::print_protection_params_current() {
    DelayParamTuple c;
    // Serial.print(F("charge over current trig:    "));
    // Serial.println(get_0x28_charge_over_current_trig());

    c = get_0x3E_delay_charge_current_delay();
    // Serial.print(F("delay charge over-current delay:           "));
    // Serial.println(c.msb);
    // Serial.print(F("delay charge over-current release delay:   "));
    // Serial.println(c.lsb);

    // Serial.print(F("charge over current release: "));
    // Serial.println(get_0x29_discharge_over_current_release());

    c = get_0x3F_delay_discharge_current_delay();
    // Serial.print(F("delay discharge over-current delay:        "));
    // Serial.println(c.msb);
    // Serial.print(F("delay discharge over-current release delay:"));
    // Serial.println(c.lsb);
}

void OverkillSolarBms2::print_protection_params_charge_temperature() {
    // Serial.print(F("charge over temp trig:                     "));
    // Serial.println(get_0x18_charge_over_temp_trig());
    // Serial.print(F("charge over temp release:                  "));
    // Serial.println(get_0x19_charge_over_temp_release());
    // Serial.print(F("charge under temp trig:                    "));
    // Serial.println(get_0x1A_charge_under_temp_trig());
    // Serial.print(F("charge under temp release:                 "));
    // Serial.println(get_0x1B_charge_under_temp_release());

    DelayParamTuple delay_charge_temp = get_0x3A_delay_charge_temp();
    // Serial.print(F("delay charge under-temp:                   "));
    // Serial.println(delay_charge_temp.msb);
    // Serial.print(F("delay charge over-temp:                    "));
    // Serial.println(delay_charge_temp.lsb);
}

void OverkillSolarBms2::print_protection_params_discharge_temperature() {
    // Serial.print(F("discharge over temp trig:                  "));
    // Serial.println(get_0x1C_discharge_over_temp_trig());
    // Serial.print(F("discharge over temp release:               "));
    // Serial.println(get_0x1D_discharge_over_temp_release());
    // Serial.print(F("discharge under temp trig:                 "));
    // Serial.println(get_0x1E_discharge_under_temp_trig());
    // Serial.print(F("discharge under temp release:              "));
    // Serial.println(get_0x1F_discharge_under_temp_release());

    DelayParamTuple delay_discharge_temp = get_0x3B_delay_discharge_temp();
    // Serial.print(F("delay discharge under-temp:                "));
    // Serial.println(delay_discharge_temp.msb);
    // Serial.print(F("delay discharge over-temp:                 "));
    // Serial.println(delay_discharge_temp.lsb);
}
