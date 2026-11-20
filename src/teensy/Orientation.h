#pragma once
#include <Arduino.h>

// Roll (phi), pitch (theta) and vertical acceleration from the accelerometer.
struct OrientationState {
    float phi   = 0.0f; // Roll in radians
    float theta = 0.0f; // Pitch in radians
};

struct Orientation {
    float phiDeg;
    float thetaDeg;
    float verticalTotalMps2;      // True vertical accel including 1g gravity baseline
    float verticalLinearAccelMps2; // True vertical linear accel (0 m/s^2 at rest)
};


// Inputs are the accelerometer readings in milli-g.
//
// IMPORTANT: the angles come from the accelerometer alone, so they are only
// trustworthy while the sensor reads about 1 g (on the bench, or slow motion).
// During powered flight or a drag coast they are not reliable. This is the
// open item to settle with the team (gyro fusion or the ICM-20948 DMP).
Orientation computeOrientation(float axMg, float ayMg, float azMg, float gxDps, float gyDps, float gzDps, float dt, OrientationState &state, float alpha = 0.98f);
