#include "BatteryMonitor.h"

BatteryMonitor battery;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    BatteryMonitorConfig cfg;
    cfg.i2cAddr = 0x41;
    cfg.shuntOhms = 0.015f;
    cfg.maxCurrentA = 10.0f;
    cfg.sampleIntervalMs = 100;

    if (!battery.begin(cfg, Wire)) {
        Serial.println("Could not initialize INA228");
        while (true) delay(1000);
    }

    Serial.println("INA228 initialized");
}

void loop() {
    if (battery.update(millis())) {
        battery.printStatus(Serial);
    }
}