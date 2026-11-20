#pragma once
#include <Arduino.h>
#include "RelayBenchConfig.h"
#if BEL_ENABLE_OPTIONAL_RELAY_BENCH
#include <Adafruit_MCP23X17.h>

// Optional MCP23017 eight-output interface.
// This class is NEVER used by the automatic recovery simulation.
class SDriver {
public:
    SDriver();
    // Returns false when the MCP23017 is absent. Range telemetry does not depend on it.
    bool begin();
    bool available() const { return mcpFound; }
    void DriverActivate(int num);
    void DriverDeactivate(int num);
    void deactivateAll();
private:
    Adafruit_MCP23X17 mcp;
    bool mcpFound = false;
    bool validChannel(int num) const { return num >= 0 && num < 8; }
    uint8_t inactiveLevel() const { return BEL_RELAY_ACTIVE_LOW == 1 ? HIGH : LOW; }
    uint8_t activeLevel() const { return BEL_RELAY_ACTIVE_LOW == 1 ? LOW : HIGH; }
};
#endif  // BEL_ENABLE_OPTIONAL_RELAY_BENCH
