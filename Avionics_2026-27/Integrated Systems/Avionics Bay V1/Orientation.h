#pragma once
#include <Arduino.h>

// Roll (phi), pitch (theta) and vertical acceleration from the accelerometer.
struct Orientation
{
    float phiDeg;                     // roll
    float thetaDeg;                   // pitch
    float verticalSpecificForceMg;    // vertical acceleration, gravity included
    float verticalLinearAccelMps2;    // vertical acceleration with 1 g removed
};

// Inputs are the accelerometer readings in milli-g.
//
// IMPORTANT: the angles come from the accelerometer alone, so they are only
// trustworthy while the sensor reads about 1 g (on the bench, or slow motion).
// During powered flight or a drag coast they are not reliable. This is the
// open item to settle with the team (gyro fusion or the ICM-20948 DMP).
Orientation computeOrientation(float axMg, float ayMg, float azMg);
