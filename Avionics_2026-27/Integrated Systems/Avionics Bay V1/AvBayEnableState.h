#pragma once
#include <stdint.h>

// Two validated EEPROM records; a torn write leaves the previous slot usable.
// Defaults DISABLED when EEPROM contains no valid record.
class AvBayEnableState {
public:
    bool begin();
    bool enabled() const { return isEnabled; }
    bool available() const { return hasEeprom; }
    bool setEnabled(bool requested); // succeeds ONLY after persistent readback
private:
    bool load(int slot, uint32_t &generation, bool &value) const;
    bool isEnabled = false;
    bool hasEeprom = false;
    int activeSlot = -1;
    uint32_t generation = 0;
};
