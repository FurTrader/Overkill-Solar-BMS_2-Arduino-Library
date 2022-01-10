#include "bms2.h"

#define LOAD_SWITCH_TTL    A0
#define CHARGER_SWITCH_TTL A1


void availableMemory() {
    int size = 8192; // Use 2048 with ATmega328
    byte *buf;

    while ((buf = (byte *) malloc(--size)) == NULL)
        ;

    free(buf);

    Serial.print("Available Memory:");
    Serial.print(size);
    Serial.println(" bytes");

    // return size;
}


OverkillSolarBms2 bms = OverkillSolarBms2();

void setup() {
    pinMode(LOAD_SWITCH_TTL, OUTPUT);
    pinMode(CHARGER_SWITCH_TTL, OUTPUT);
    digitalWrite(LOAD_SWITCH_TTL, LOW);  // Relay off
    digitalWrite(CHARGER_SWITCH_TTL, LOW);  // Relay off

    delay(1000);
    Serial.begin(115200);
    Serial2.begin(9600);

    availableMemory();

    bms.begin(&Serial2);

    availableMemory();

    while(1) {
        bms.main_task(true);
        if (millis() >= 5000) {
            break;
        }
        delay(10);
    }
    Serial.println("BMS Params:");

    availableMemory();

    eeprom_data_t params;

    availableMemory();

    Serial.println("Get Params");
    bms.get_params(&params);
    Serial.println("Print Params");
    bms.print_params(&params);

    availableMemory();

    // 12V 120A BMS
    params.cell_cnt      = 0x0004;  // 0x2F
    params.ntc_config    = 0x0003;  // 0x2E
    params.func_config   = 0x000F;  // 0x2D
    params.design_cap    = 0x2FBE;  // 0x10
    params.cycle_cap     = 0x201E;  // 0x11
    params.dsg_rate      = 0x0002;  // 0x14
    params.cap_100       = 0x0D48;  // 0x12
    params.cap_80        = 0x0D16;  // 0x32
    params.cap_60        = 0x0CE4;  // 0x33
    params.cap_40        = 0x0C4E;  // 0x34
    params.cap_20        = 0x0C1C;  // 0x35
    params.cap_0         = 0x0BB8;  // 0x13
    params.bal_start     = 0x0D48;  // 0x2a
    params.bal_window    = 0x000F;  // 0x2b
    params.povp          = 0x05B4;  // 0x20
    params.povp_rel      = 0x0578;  // 0x21
    params.puvp          = 0x03E8;  // 0x22
    params.puvp_rel      = 0x04B0;  // 0x23
    params.pack_v_delays = 0x0202;  // 0x3C
    params.covp          = 0x0E42;  // 0x24
    params.covp_rel      = 0x0DAC;  // 0x25
    params.cuvp          = 0x09C4;  // 0x26
    params.cuvp_rel      = 0x0BB8;  // 0x27
    params.cell_v_delays = 0x0202;  // 0x3D
    params.chgoc         = 0x32C8;  // 0x28
    params.chgoc_delays  = 0x0A20;  // 0x3E
    params.dsgoc         = 0xCD38;  // 0x29
    params.dsgoc_delays  = 0x0A20;  // 0x3F
    params.chgot         = 0x0D35;  // 0x18
    params.chgot_rel     = 0x0CD1;  // 0x19
    params.chgut         = 0x0AB5;  // 0x1A
    params.chgut_rel     = 0x0B0F;  // 0x1B
    params.chg_t_delays  = 0x0202;  // 0x3A
    params.dsgot         = 0x0D99;  // 0x1C
    params.dsgot_rel     = 0x0D35;  // 0x1D
    params.dsgut         = 0x0A47;  // 0x1E
    params.dsgut_rel     = 0x0AAB;  // 0x1F
    params.dsg_t_delays  = 0x0202;  // 0x3B

    Serial.println("Set Params");
    bms.set_params(&params);

    delay(1000);

    availableMemory();

    Serial.println("Get Params");
    bms.get_params(&params);
    availableMemory();
    bms.print_params(&params);

}

void loop() {
    delay(1000);
}