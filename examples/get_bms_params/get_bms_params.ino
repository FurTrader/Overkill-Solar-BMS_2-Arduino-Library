#include "bms2.h"

#define LOAD_SWITCH_TTL    A0
#define CHARGER_SWITCH_TTL A1

OverkillSolarBms2 bms = OverkillSolarBms2();

void setup() {
    pinMode(LOAD_SWITCH_TTL, OUTPUT);
    pinMode(CHARGER_SWITCH_TTL, OUTPUT);
    digitalWrite(LOAD_SWITCH_TTL, LOW);  // Relay off
    digitalWrite(CHARGER_SWITCH_TTL, LOW);  // Relay off

    delay(1000);
    Serial.begin(115200);
    Serial2.begin(9600);

    bms.begin(&Serial2);

    while(1) {
        bms.main_task(true);
        if (millis() >= 5000) {
            break;
        }
        delay(10);
    }
    Serial.println("BMS Params:");

    eeprom_data_t params;
    bms.get_params(&params);
    bms.print_params(&params);
    delay(1000);
    bms.get_params(&params);
    bms.print_params(&params);
    delay(1000);
    bms.get_params(&params);
    bms.print_params(&params);

}

void loop() {
    delay(1000);
}