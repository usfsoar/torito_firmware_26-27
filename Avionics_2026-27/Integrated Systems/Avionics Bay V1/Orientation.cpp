#include "Orientation.h"

Orientation computeOrientation(float axMg, float ayMg, float azMg)
{
    // Roll/pitch estimate from the accelerometer vector.
    const float phi   = atan2f(ayMg, azMg);
    const float theta = atan2f(-axMg, sqrtf(ayMg * ayMg + azMg * azMg));

    // Rotate the body-frame accelerometer vector onto the estimated vertical axis.
    // Result stays in mg because ax/ay/az are in mg.
    const float cPhi = cosf(phi);
    const float sPhi = sinf(phi);
    const float cThe = cosf(theta);
    const float sThe = sinf(theta);

    const float verticalMg =
        -axMg * sThe
        + ayMg * sPhi * cThe
        + azMg * cPhi * cThe;

    Orientation out;
    out.phiDeg   = phi   * 180.0f / PI;
    out.thetaDeg = theta * 180.0f / PI;
    out.verticalSpecificForceMg = verticalMg;

    // A stationary sensor with Z up reads about +1000 mg vertically.
    // Subtract 1 g so a stationary test reads about 0 m/s^2.
    out.verticalLinearAccelMps2 = (verticalMg - 1000.0f) * 9.80665f / 1000.0f;

    return out;
}
