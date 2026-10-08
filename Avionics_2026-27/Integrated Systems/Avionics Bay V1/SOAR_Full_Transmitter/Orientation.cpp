#pragma once
#include <math.h>
#include "Orientation.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * Updates 6-DOF orientation filter and calculates true vertical acceleration.
 * 
 * @param axMg, ayMg, azMg Raw/calibrated accelerometer data in mg
 * @param gxDps, gyDps, gzDps Gyroscope angular rates in degrees/second
 * @param dt Time step since last update in seconds
 * @param state Persistent filter state (retained across calls)
 * @param alpha Complementary filter ratio (typically 0.95f - 0.98f)
 */
Orientation computeOrientation(
    float axMg, float ayMg, float azMg,
    float gxDps, float gyDps, float gzDps,
    float dt,
    OrientationState &state,
    float alpha)
{
    // 1. Convert gyro rates from deg/s to rad/s
    const float gx = gxDps * (M_PI / 180.0f);
    const float gy = gyDps * (M_PI / 180.0f);
    const float gz = gzDps * (M_PI / 180.0f);

    // 2. Compute Euler angle rates of change from body angular rates
    const float sPhi = sinf(state.phi);
    const float cPhi = cosf(state.phi);
    const float sThe = sinf(state.theta);
    const float cThe = cosf(state.theta);

    // Kinematic transformation from body rates to Euler angle rates
    const float tanThe = (fabsf(cThe) > 1e-4f) ? (sThe / cThe) : 0.0f;
    const float phiDot   = gx + (gy * sPhi + gz * cPhi) * tanThe;
    const float thetaDot = gy * cPhi - gz * sPhi;

    // 3. Integrate gyro rates to predict new orientation (High-Pass)
    const float phiGyro   = state.phi   + phiDot   * dt;
    const float thetaGyro = state.theta + thetaDot * dt;

    // 4. Calculate instantaneous tilt vector from accelerometer (Low-Pass anchor)
    const float phiAcc   = atan2f(ayMg, azMg);
    const float thetaAcc = atan2f(-axMg, sqrtf(ayMg * ayMg + azMg * azMg));

    // 5. Complementary Filter Blend
    // Trust gyro short-term (alpha), correct drift with accel long-term (1 - alpha)
    state.phi   = alpha * phiGyro   + (1.0f - alpha) * phiAcc;
    state.theta = alpha * thetaGyro + (1.0f - alpha) * thetaAcc;

    // 6. Project accelerometer vector onto Earth's true vertical Z-axis
    const float cPhiF = cosf(state.phi);
    const float sPhiF = sinf(state.phi);
    const float cTheF = cosf(state.theta);
    const float sTheF = sinf(state.theta);

    const float verticalMg = -axMg * sTheF
                           + ayMg * sPhiF * cTheF
                           + azMg * cPhiF * cTheF;

    // 7. Output assembly
    Orientation out;
    out.phiDeg   = state.phi   * (180.0f / M_PI);
    out.thetaDeg = state.theta * (180.0f / M_PI);
    
    // Total vertical force including 1 g gravity (reads ~9.81 m/s^2 at rest)
    out.verticalTotalMps2 = verticalMg * 0.00980665f;
    
    // True vertical linear acceleration (reads ~0.0 m/s^2 at rest)
    out.verticalLinearAccelMps2 = (verticalMg - 1000.0f) * 0.00980665f;

    return out;
}