#pragma once
#include <stdint.h>
class SDLogger;

// Bench software-enable gate: no valid SD checkpoint -> OFF.
// Two checksummed records survive many interrupted writes, but are not
// a certified power-loss-safe recovery system. No EEPROM is consulted.
class AvBayEnableState {
public:
    bool begin(SDLogger &logger);
    bool enabled() const { return isEnabled; }
    bool available() const { return hasStorage; }
    bool setEnabled(bool requested); // true ONLY after verified SD write
    void disableVolatile() { isEnabled = false; } // failsafe on failed OFF write
private:
    bool load(int slot, uint32_t &generation, bool &value) const;
    SDLogger *storage=nullptr;
    bool isEnabled=false;
    bool hasStorage=false;
    int activeSlot=-1;
    uint32_t generation=0;
};
