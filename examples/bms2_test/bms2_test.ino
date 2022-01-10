#include <pindefs.h>
#include <bms2.h>

OverkillSolarBms2 bms;

uint32_t timestamp_1000ms = 0;

void setup() {
    pinMode(BMS_SWITCH_TTL, OUTPUT);
    pinMode(BMS2_SWITCH_TTL, OUTPUT);
    digitalWrite(BMS_SWITCH_TTL, LOW);  // Relay off
    digitalWrite(BMS2_SWITCH_TTL, LOW);  // Relay off

    Serial.begin(115200);
    // Serial2.begin(9600);
    Serial3.begin(9600);
    delay(1000);

    Serial.println(F("------------------------------"));
    Serial.println(F("BMS sketch init"));

    bms.begin(&Serial3);
    // bms.begin(&Serial2);

    while(1) {
        bms.main_task(true);
        if (millis() >= 5000) {
            break;
        }
        delay(10);
    }
}

void loop() {
//    Serial.println(F("Main task"));

    while(1) {
        if (millis() - timestamp_1000ms >= 1000) {
            bms.print_config_params();
            bms.print_capacity_params();
            bms.print_balance_params();
            bms.print_protection_params_voltage();
            bms.print_protection_params_current();
            bms.print_protection_params_charge_temperature();
            bms.print_protection_params_discharge_temperature();
            Serial.println("------");
            timestamp_1000ms = millis();
        }
    }

    delay(2000);

    // bms.main_task(false);
    // delay(250);
}
