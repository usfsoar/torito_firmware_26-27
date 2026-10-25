#include "SDriver.h"

SDriver::SDriver() {
}

void SDriver::begin() {
    if (!mcp.begin_I2C()) {
        Serial.println("Couldn't find MCP23017 - continuing without solenoid driver.");
        mcpFound = false;
        return;
    }
    mcpFound = true;
    mcp.pinMode(N0, OUTPUT);
    mcp.pinMode(N1, OUTPUT);
    mcp.pinMode(N2, OUTPUT);
    mcp.pinMode(N3, OUTPUT);
    mcp.pinMode(N4, OUTPUT);
    mcp.pinMode(N5, OUTPUT);
    mcp.pinMode(N6, OUTPUT);
    mcp.pinMode(N7, OUTPUT);
    Serial.println("Solenoid driver ready.");
}

void SDriver::DriverActivate(int num) {
    if (!mcpFound) return;
    if (num == 0) mcp.digitalWrite(N0, HIGH);
    else if (num == 1) mcp.digitalWrite(N1, HIGH);
    else if (num == 2) mcp.digitalWrite(N2, HIGH);
    else if (num == 3) mcp.digitalWrite(N3, HIGH);
    else if (num == 4) mcp.digitalWrite(N4, HIGH);
    else if (num == 5) mcp.digitalWrite(N5, HIGH);
    else if (num == 6) mcp.digitalWrite(N6, HIGH);
    else if (num == 7) mcp.digitalWrite(N7, HIGH);
}

void SDriver::DriverDeactivate(int num) {
    if (!mcpFound) return;
    if (num == 0) mcp.digitalWrite(N0, LOW);
    else if (num == 1) mcp.digitalWrite(N1, LOW);
    else if (num == 2) mcp.digitalWrite(N2, LOW);
    else if (num == 3) mcp.digitalWrite(N3, LOW);
    else if (num == 4) mcp.digitalWrite(N4, LOW);
    else if (num == 5) mcp.digitalWrite(N5, LOW);
    else if (num == 6) mcp.digitalWrite(N6, LOW);
    else if (num == 7) mcp.digitalWrite(N7, LOW);
}
