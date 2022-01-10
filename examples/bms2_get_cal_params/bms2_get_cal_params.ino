#include "bms2.h"
#include "analog_board.h"

#define LOAD_SWITCH_TTL    A0
#define CHARGER_SWITCH_TTL A1

OverkillSolarBms2 bms = OverkillSolarBms2();

AnalogBoard analog_board = AnalogBoard();

void setup() {
    pinMode(LOAD_SWITCH_TTL, OUTPUT);
    pinMode(CHARGER_SWITCH_TTL, OUTPUT);
    digitalWrite(LOAD_SWITCH_TTL, LOW);  // Relay off
    digitalWrite(CHARGER_SWITCH_TTL, LOW);  // Relay off

    delay(1000);
    Serial.begin(115200);
    Serial2.begin(9600);
    Wire.begin();

    bms.begin(&Serial2);
    analog_board.begin(4);

    while(1) {
        bms.main_task(true);
        if (millis() >= 5000) {
            break;
        }
        delay(10);
    }
    Serial.println("BMS Params:");

    float c1 = analog_board.measure_cell_voltage(0);
    float c2 = analog_board.measure_cell_voltage(1);
    float c3 = analog_board.measure_cell_voltage(2);
    float c4 = analog_board.measure_cell_voltage(3);

    float v_offset = 0.0;

    Serial.print("Cell 1: ");  Serial.println(c1 + v_offset);
    Serial.print("Cell 2: ");  Serial.println(c2 + v_offset);
    Serial.print("Cell 3: ");  Serial.println(c3 + v_offset);
    Serial.print("Cell 4: ");  Serial.println(c4 + v_offset);

    bms.enter_factory_mode();
    bms.set_0xBx_cell_calibration(0, c1 + v_offset, false);
    bms.set_0xBx_cell_calibration(1, c2 + v_offset, false);
    bms.set_0xBx_cell_calibration(2, c3 + v_offset, false);
    bms.set_0xBx_cell_calibration(3, c4 + v_offset, false);
    bms.exit_factory_mode(true);

//    bms.get_0xAE_charge_current_calibration();
//    bms.get_0xAF_discharge_current_calibration();
//    bms.get_0xDx_temp_calibration();

}

void loop() {
    delay(1000);
}
