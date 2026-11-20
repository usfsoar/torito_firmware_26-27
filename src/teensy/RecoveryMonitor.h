#pragma once
#include <stdint.h>

// BENCH-ONLY decision monitor. It has no hardware control methods and
// cannot energize MCP23017 outputs. All deployment flags are SIMULATED.
struct RecoveryInput {
    bool valid;
    float altitudeM; // RAW MS5607 height (relative to startup), NOT Kalman
    float verticalVelocityMps; // derived from RAW barometer samples
    float verticalAccelMps2; // unfiltered IMU-derived acceleration
};

struct RecoveryEvents {
    bool drogueCandidate = false;
    bool mainCandidate = false;
    bool droguePulseEnded = false;
    bool mainPulseEnded = false;
};

class RecoveryMonitor {
public:
    void reset();
    void restoreSimulatedFlags(bool drogue, bool main);
    RecoveryEvents step(const RecoveryInput &in, uint32_t nowMs);

    bool drogueSimulated() const { return drogueFlag; }
    bool mainSimulated() const { return mainFlag; }
    bool droguePulseActive() const { return droguePulse; }
    bool mainPulseActive() const { return mainPulse; }
    bool peakValid() const { return havePeak; }
    float peakAltitudeM() const { return peakAltitude; }
    float dropFromPeakM() const { return havePeak ? peakAltitude - lastAltitude : 0.0f; }

private:
    bool drogueFlag = false, mainFlag = false;
    bool droguePulse = false, mainPulse = false;
    bool havePeak = false;
    float peakAltitude = 0.0f, lastAltitude = 0.0f;
    uint32_t drogueStartMs = 0, mainStartMs = 0;
};
