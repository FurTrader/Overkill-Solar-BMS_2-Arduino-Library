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
    bms.enter_factory_mode();
    bms.set_0xAD_idle_current_calibration(false);
    bms.exit_factory_mode(true);
}

void loop() {
    delay(1000);
}
