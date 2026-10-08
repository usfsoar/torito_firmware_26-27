#include "SDriver.h"

SDriver sol;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    Serial.println("Solenoid Driver Test");
    sol.begin();
}

void loop() {
    // 10 seconds 1 at a time, then 10 seconds all at once
    for (int i = 0; i < 8; i++) {
        Serial.print("Activating solenoid ");
        Serial.println(i);
        sol.DriverActivate(i);
        delay(10000);
        Serial.print("Deactivating solenoid ");
        Serial.println(i);
        sol.DriverDeactivate(i);
    }

    sol.DriverActivate(0);
    sol.DriverActivate(1);
    sol.DriverActivate(2);
    sol.DriverActivate(3);
    sol.DriverActivate(4);
    sol.DriverActivate(5);
    sol.DriverActivate(6);
    sol.DriverActivate(7);

    delay(10000);

    sol.DriverDeactivate(0);
    sol.DriverDeactivate(1);
    sol.DriverDeactivate(2);
    sol.DriverDeactivate(3);
    sol.DriverDeactivate(4);
    sol.DriverDeactivate(5);
    sol.DriverDeactivate(6);
    sol.DriverDeactivate(7);
}