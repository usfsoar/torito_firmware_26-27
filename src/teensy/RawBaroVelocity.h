#pragma once
#include <stdint.h>
#include <math.h>

// BENCH ONLY: calculate vertical velocity from BAROMETER HEIGHT SAMPLES ONLY.
// This DOES NOT use a Kalman-filtered altitude or Kalman velocity.
// Two actual MS5607 readings approximately 0.9+ s apart produce a raw slope.
// The estimated velocity is inherently noisy and NOT flight-qualified.
class RawBaroVelocity {
public:
    void reset() { count = 0; next = 0; lastMs = 0; }

    bool add(float altitudeM, uint32_t nowMs, float &velocityMps) {
        velocityMps = 0.0f;
        if (!isfinite(altitudeM)) { reset(); return false; }
        if (count && (uint32_t)(nowMs - lastMs) > 500UL) reset();
        samples[next].ms = nowMs;
        samples[next].altitudeM = altitudeM;
        next = (next + 1) % CAPACITY;
        if (count < CAPACITY) ++count;
        lastMs = nowMs;

        // Prefer the newest historical reading at least ~0.9s old.
        int selected = -1;
        uint32_t elapsed = 0;
        for (unsigned i = 0; i < count; ++i) {
            unsigned idx = (next + CAPACITY - count + i) % CAPACITY;
            uint32_t age = (uint32_t)(nowMs - samples[idx].ms);
            if (age >= 900UL) { selected = (int)idx; elapsed = age; }
            else break; // samples are in chronological order
        }
        if (selected < 0 || elapsed == 0) return false;
        velocityMps = (altitudeM - samples[selected].altitudeM) * 1000.0f / elapsed;
        return isfinite(velocityMps);
    }
private:
    static const unsigned CAPACITY = 16;
    struct Reading { uint32_t ms; float altitudeM; };
    Reading samples[CAPACITY] = {};
    unsigned count = 0, next = 0;
    uint32_t lastMs = 0;
};
