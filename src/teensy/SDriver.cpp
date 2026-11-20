#include "SDriver.h"
#if BEL_ENABLE_OPTIONAL_RELAY_BENCH
#include <Wire.h>

SDriver::SDriver() {}

bool SDriver::begin() {
    mcpFound = false;
    Wire.begin();
    if (!mcp.begin_I2C(BEL_RELAY_I2C_ADDRESS, &Wire)) {
        Serial.println("OPTIONAL MCP23017: not detected; range test continues.");
        return false;
    }
    // Set each output latch to the inactive level BEFORE making it an output.
    // Physical hardware interlocks are still required for any consequential load.
    for (int ch = 0; ch < 8; ++ch) {
        mcp.digitalWrite(ch, inactiveLevel());
        mcp.pinMode(ch, OUTPUT);
    }
    mcpFound = true;
    Serial.println("OPTIONAL MCP23017: eight channels present and set INACTIVE.");
    return true;
}

void SDriver::DriverActivate(int num) {
    if (mcpFound && validChannel(num)) mcp.digitalWrite(num, activeLevel());
}
void SDriver::DriverDeactivate(int num) {
    if (mcpFound && validChannel(num)) mcp.digitalWrite(num, inactiveLevel());
}
void SDriver::deactivateAll() {
    if (!mcpFound) return;
    for (int ch = 0; ch < 8; ++ch) DriverDeactivate(ch);
}
#endif  // BEL_ENABLE_OPTIONAL_RELAY_BENCH
