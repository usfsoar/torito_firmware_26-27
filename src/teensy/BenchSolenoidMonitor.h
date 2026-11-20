#pragma once
// Safe bench-only shadow of the MCP23017 solenoid driver.
// Does NOT access I2C, GPIO, SDriver, relays, or solenoid hardware.
// Mirrors simulated pulses to virtual N0/N1 (drogue) and N2/N3 (main).
class BenchSolenoidMonitor {
public:
    // Returns true if any virtual output changed since the previous update.
    bool sync(bool avionicsEnabled, bool droguePulse, bool mainPulse) {
        const bool d = avionicsEnabled && droguePulse;
        const bool m = avionicsEnabled && mainPulse;
        bool changed = (channels[0] != d || channels[1] != d ||
                        channels[2] != m || channels[3] != m);
        channels[0] = channels[1] = d;
        channels[2] = channels[3] = m;
        return changed;
    }
    bool forceOff() { return sync(false, false, false); }
    bool channel(unsigned n) const { return n < 4U ? channels[n] : false; }
private:
    bool channels[4] = {false, false, false, false};
};
