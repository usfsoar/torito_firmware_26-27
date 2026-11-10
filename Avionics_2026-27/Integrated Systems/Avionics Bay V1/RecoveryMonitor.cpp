#include "RecoveryMonitor.h"
#include "Config.h"
#include <math.h>

void RecoveryMonitor::reset() {
    drogueFlag = mainFlag = droguePulse = mainPulse = havePeak = false;
    peakAltitude = lastAltitude = 0.0f;
    drogueStartMs = mainStartMs = 0;
}

void RecoveryMonitor::restoreSimulatedFlags(bool drogue, bool main) {
    drogueFlag = drogue;
    mainFlag = main;
    // NEVER resume a timed pulse on restart, even in simulation.
    droguePulse = mainPulse = false;
}

RecoveryEvents RecoveryMonitor::step(const RecoveryInput &in, uint32_t nowMs) {
    RecoveryEvents events;
    // Pulse deadlines are independent of sensor validity or other deployment state.
    if (droguePulse && (uint32_t)(nowMs - drogueStartMs) >= SIM_PULSE_MS) {
        droguePulse = false;
        events.droguePulseEnded = true;
    }
    if (mainPulse && (uint32_t)(nowMs - mainStartMs) >= SIM_PULSE_MS) {
        mainPulse = false;
        events.mainPulseEnded = true;
    }

    if (!in.valid || !isfinite(in.altitudeM) ||
        !isfinite(in.verticalVelocityMps) || !isfinite(in.verticalAccelMps2))
        return events;

    lastAltitude = in.altitudeM;
    if (!havePeak || in.altitudeM > peakAltitude) {
        peakAltitude = in.altitudeM;
        havePeak = true;
    }
    // SIM ONLY: peak/current are RAW MS5607 altitudes, NOT Kalman.
    // Noise can make this true immediately; NOT suitable for hardware actuation.
    if (!drogueFlag &&
        (peakAltitude - in.altitudeM) > SIM_DROGUE_DROP_M &&
        in.verticalVelocityMps < SIM_DESCENT_VEL_MPS &&
        in.verticalAccelMps2 < 0.0f) {
        drogueFlag = true;  // update flags BEFORE logging/persistence
        droguePulse = true;
        drogueStartMs = nowMs;
        events.drogueCandidate = true;
    }
    // Only a simulated decision -- no physical channels are commanded.
    if (drogueFlag && !mainFlag && !events.drogueCandidate &&
        in.verticalVelocityMps < SIM_DESCENT_VEL_MPS &&
        in.altitudeM <= SIM_MAIN_ALTITUDE_M) {
        mainFlag = true;
        mainPulse = true;
        mainStartMs = nowMs;
        events.mainCandidate = true;
    }
    return events;
}
