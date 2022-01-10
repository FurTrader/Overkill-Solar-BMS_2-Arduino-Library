#include <pindefs.h>
#include <bms2.h>

OverkillSolarBms2 bms;

uint32_t timestamp_1000ms = 0;

void setup() {
    pinMode(LOAD_SWITCH_TTL, OUTPUT);
    pinMode(CHARGER_SWITCH_TTL, OUTPUT);
    digitalWrite(LOAD_SWITCH_TTL, LOW);  // Relay off
    digitalWrite(CHARGER_SWITCH_TTL, LOW);  // Relay off

    Serial.begin(115200);
    Serial2.begin(9600);
    delay(1000);

    Serial.println(F("------------------------------"));
    Serial.println(F("BMS sketch init"));

    bms.begin(&Serial2);

    while(1) {
        bms.main_task(true);
        if (millis() >= 5000) {
            break;
        }
        delay(10);
    }

    Serial.println("READ ======================");
    Serial.print((float)millis() / 1000.0);
    Serial.print(": 0x10 Designed Capacity: ");
    Serial.println(bms.get_0x10_designed_capacity());

    Serial.print((float)millis() / 1000.0);
    Serial.print(": 0x11 Cycle Capacity: ");
    Serial.println(bms.get_0x11_cycle_capacity());
    Serial.println("READ ======================");

    Serial.println("WRITE ======================");
    bms.set_0x10_designed_capacity(122220);
    Serial.println("------");
    bms.set_0x11_cycle_capacity( 82220);
    Serial.println("WRITE ======================");

    bms.param_clear_errors();

    for (int i=0; i<5; i++) {
        Serial.println("READ ======================");
        Serial.print((float)millis() / 1000.0);
        Serial.print(": 0x10 Designed Capacity: ");
        Serial.println(bms.get_0x10_designed_capacity());
    
        Serial.print((float)millis() / 1000.0);
        Serial.print(": 0x11 Cycle Capacity: ");
        Serial.println(bms.get_0x11_cycle_capacity());
        delay(500);
    }

}

void loop() {
//    Serial.println(F("Main task"));

    delay(1000);
}
