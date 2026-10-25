#pragma once
#include <Arduino.h>

// ============================================================
// SOAR USF - Kalman Filter
// Rewrites last year's generic C implementation as a fixed-size
// embedded C++ class. No dynamic memory allocation (no malloc).
//
// State vector x = [ altitude (m), velocity (m/s), acceleration (m/s^2) ]
//
// Inputs each update:
//   - altitudeMeasured  : barometer reading (m, relative to power-on)
//   - accelMeasured     : vertical linear acceleration from the IMU (m/s^2, 1 g removed)
//   - dt                : time since the last update (seconds)
//
// Outputs:
//   - altitude()        : filtered altitude (m)
//   - velocity()        : estimated vertical velocity (m/s)
//   - acceleration()    : estimated vertical acceleration (m/s^2)
// ============================================================

class KalmanFilter
{
public:
    // Create the filter.
    // altNoise  : barometer measurement noise std-dev (m). Higher = trust baro less.
    //             Typical MS5607 noise is about 0.5 m, start with 1.0.
    // accelNoise: accelerometer measurement noise std-dev (m/s^2).
    //             Typical ICM-20948 noise is about 0.1 m/s^2, start with 0.5.
    // procNoise : process noise std-dev. Accounts for model error (wind, drag changes).
    //             Start with 0.1 and tune up if the filter lags.
    KalmanFilter(float altNoise   = 1.0f,
                 float accelNoise = 0.5f,
                 float procNoise  = 5.0f);   // higher = filter adapts faster to dynamic changes

    // Reset to a known state (call at power-on after zeroing the altimeter).
    void reset(float initialAltitude = 0.0f);

    // Run one predict + update step. Call at your sensor read rate (e.g. 10 Hz).
    void update(float altitudeMeasured, float accelMeasured, float dt);

    float altitude()     const { return x[0]; }
    float velocity()     const { return x[1]; }
    float acceleration() const { return x[2]; }

private:
    // State: [altitude, velocity, acceleration]
    float x[3];

    // Estimate covariance (3x3, stored row-major)
    float P[3][3];

    // Noise parameters
    float sigmaAlt;    // barometer noise
    float sigmaAccel;  // accelerometer noise
    float sigmaProc;   // process noise

    // 3x3 matrix helpers (all operate on flat arrays)
    static void mat3Add(const float A[3][3], const float B[3][3], float C[3][3]);
    static void mat3Sub(const float A[3][3], const float B[3][3], float C[3][3]);
    static void mat3Mul(const float A[3][3], const float B[3][3], float C[3][3]);
    static void mat3Trans(const float A[3][3], float At[3][3]);
    static bool mat3Inv(const float A[3][3], float Ainv[3][3]);
};
