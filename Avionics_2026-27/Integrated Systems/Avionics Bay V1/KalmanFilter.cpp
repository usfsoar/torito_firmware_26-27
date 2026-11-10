#include "KalmanFilter.h"
#include <string.h>
#include <math.h>

// ============================================================
// Construction and reset
// ============================================================

KalmanFilter::KalmanFilter(float altNoise, float accelNoise, float procNoise)
    : sigmaAlt(altNoise), sigmaAccel(accelNoise), sigmaProc(procNoise)
{
    reset();
}

void KalmanFilter::reset(float initialAltitude)
{
    x[0] = initialAltitude;
    x[1] = 0.0f;   // velocity
    x[2] = 0.0f;   // acceleration

    // Initial uncertainty: known start position, unknown velocity and acceleration.
    memset(P, 0, sizeof(P));
    P[0][0] = sigmaAlt   * sigmaAlt;   // altitude known at startup (we zeroed it)
    P[1][1] = 100.0f;                   // velocity: moderate initial uncertainty
    P[2][2] = sigmaAccel * sigmaAccel;  // acceleration: sensor noise level
}

// ============================================================
// Main update step
// ============================================================

void KalmanFilter::update(float altMeas, float accelMeas, float dt)
{
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;

    // ----- State transition matrix F -----
    // x(k) = F * x(k-1):
    //   altitude    += velocity*dt + 0.5*accel*dt^2
    //   velocity    += accel*dt
    //   acceleration = acceleration  (assumed constant between steps)
    float F[3][3] = {
        { 1.0f, dt,   0.5f * dt2 },
        { 0.0f, 1.0f, dt         },
        { 0.0f, 0.0f, 1.0f       }
    };

    // ----- Process noise covariance Q -----
    // Continuous white noise model with process noise variance q = sigmaProc^2
    float q = sigmaProc * sigmaProc;
    // Discrete process noise (piecewise white noise model, acceleration-driven)
    float Q[3][3] = {
        { q * dt3 * dt / 4.0f, q * dt3 / 2.0f, q * dt2 / 2.0f },
        { q * dt3 / 2.0f,      q * dt2,         q * dt          },
        { q * dt2 / 2.0f,      q * dt,          q               }
    };

    // ----- Predict -----
    // x_pred = F * x
    float xp[3];
    xp[0] = F[0][0]*x[0] + F[0][1]*x[1] + F[0][2]*x[2];
    xp[1] = F[1][0]*x[0] + F[1][1]*x[1] + F[1][2]*x[2];
    xp[2] = F[2][0]*x[0] + F[2][1]*x[1] + F[2][2]*x[2];

    // P_pred = F * P * F^T + Q
    float Ft[3][3], FP[3][3], Pp[3][3], tmp[3][3];
    mat3Trans(F,  Ft);
    mat3Mul(F,  P,  FP);
    mat3Mul(FP, Ft, tmp);
    mat3Add(tmp, Q, Pp);

    // ----- Observation model -----
    // We observe: z = [ altitude, vertical_acceleration ]
    // H maps state -> observations:
    //   z[0] = altitude     = [1, 0, 0] * x
    //   z[1] = acceleration = [0, 0, 1] * x
    // H is 2x3
    const float H[2][3] = {
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };

    // ----- Innovation: y = z - H * x_pred -----
    float y[2];
    y[0] = altMeas   - (H[0][0]*xp[0] + H[0][1]*xp[1] + H[0][2]*xp[2]);
    y[1] = accelMeas - (H[1][0]*xp[0] + H[1][1]*xp[1] + H[1][2]*xp[2]);

    // ----- Innovation covariance: S = H * P_pred * H^T + R -----
    // R = diag(sigmaAlt^2, sigmaAccel^2)
    // H*Pp is 2x3; then (H*Pp)*H^T is 2x2
    float HPp[2][3];
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++)
        {
            HPp[i][j] = 0;
            for (int k = 0; k < 3; k++)
                HPp[i][j] += H[i][k] * Pp[k][j];
        }

    float S[2][2];
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
        {
            S[i][j] = 0;
            for (int k = 0; k < 3; k++)
                S[i][j] += HPp[i][k] * H[j][k];   // H[j][k] = H^T[k][j]
        }
    S[0][0] += sigmaAlt   * sigmaAlt;
    S[1][1] += sigmaAccel * sigmaAccel;

    // ----- Outlier rejection (Mahalanobis distance) -----
    // Reject if innovation is more than 5 standard deviations from expected
    bool outlier = false;
    for (int i = 0; i < 2; i++)
    {
        if (S[i][i] > 0 && (y[i] * y[i] / S[i][i]) > 25.0f)
        {
            outlier = true;
            break;
        }
    }

    if (outlier)
    {
        // Keep the prediction but don't update with the bad measurement
        x[0] = xp[0]; x[1] = xp[1]; x[2] = xp[2];
        memcpy(P, Pp, sizeof(P));
        return;
    }

    // ----- Kalman gain: K = P_pred * H^T * S^-1 -----
    // K is 3x2
    float Sinv[2][2];
    float detS = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    if (fabsf(detS) < 1e-10f)
    {
        x[0] = xp[0]; x[1] = xp[1]; x[2] = xp[2];
        memcpy(P, Pp, sizeof(P));
        return;
    }
    float invDet = 1.0f / detS;
    Sinv[0][0] =  S[1][1] * invDet;
    Sinv[0][1] = -S[0][1] * invDet;
    Sinv[1][0] = -S[1][0] * invDet;
    Sinv[1][1] =  S[0][0] * invDet;

    // Pp * H^T  is 3x2
    float PpHt[3][2];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 2; j++)
        {
            PpHt[i][j] = 0;
            for (int k = 0; k < 3; k++)
                PpHt[i][j] += Pp[i][k] * H[j][k];   // H[j][k] = H^T[k][j]
        }

    float K[3][2];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 2; j++)
        {
            K[i][j] = 0;
            for (int k = 0; k < 2; k++)
                K[i][j] += PpHt[i][k] * Sinv[k][j];
        }

    // ----- Update state: x = x_pred + K * y -----
    x[0] = xp[0] + K[0][0]*y[0] + K[0][1]*y[1];
    x[1] = xp[1] + K[1][0]*y[0] + K[1][1]*y[1];
    x[2] = xp[2] + K[2][0]*y[0] + K[2][1]*y[1];

    // ----- Update covariance: P = (I - K*H) * P_pred -----
    // Standard form; enforce symmetry to prevent numerical drift.
    float KH[3][3] = {};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 2; k++)
                KH[i][j] += K[i][k] * H[k][j];

    float IKH[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];

    float newP[3][3];
    mat3Mul(IKH, Pp, newP);

    // Enforce symmetry: P = (P + P^T) / 2 and clamp diagonal to > 0
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[i][j] = 0.5f * (newP[i][j] + newP[j][i]);
    for (int i = 0; i < 3; i++)
        if (P[i][i] < 1e-6f) P[i][i] = 1e-6f;
}

// ============================================================
// 3x3 matrix helpers
// ============================================================

void KalmanFilter::mat3Add(const float A[3][3], const float B[3][3], float C[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C[i][j] = A[i][j] + B[i][j];
}

void KalmanFilter::mat3Sub(const float A[3][3], const float B[3][3], float C[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C[i][j] = A[i][j] - B[i][j];
}

void KalmanFilter::mat3Mul(const float A[3][3], const float B[3][3], float C[3][3])
{
    float tmp[3][3] = {};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                tmp[i][j] += A[i][k] * B[k][j];
    memcpy(C, tmp, sizeof(tmp));
}

void KalmanFilter::mat3Trans(const float A[3][3], float At[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            At[i][j] = A[j][i];
}

bool KalmanFilter::mat3Inv(const float A[3][3], float Ainv[3][3])
{
    // Cofactor / adjugate method for 3x3
    float det =
        A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1]) -
        A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0]) +
        A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    if (fabsf(det) < 1e-10f)
        return false;

    float inv = 1.0f / det;

    Ainv[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * inv;
    Ainv[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * inv;
    Ainv[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * inv;
    Ainv[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * inv;
    Ainv[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * inv;
    Ainv[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * inv;
    Ainv[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * inv;
    Ainv[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * inv;
    Ainv[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * inv;

    return true;
}
