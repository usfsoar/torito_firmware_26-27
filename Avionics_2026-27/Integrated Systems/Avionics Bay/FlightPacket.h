#pragma once
#include <Arduino.h>
#include "FlightData.h"

// Radio packet, 24 bytes. MUST be IDENTICAL on transmitter and receiver.
// (Keeping it at 24 bytes leaves room for 25 packets per second at
//  250 kHz bandwidth / SF7. Bigger packets take longer to send.)
//
// A value that is not available is sent as the "no data" marker
// (-32768 for 16-bit fields, -2147483648 for 32-bit fields).
struct __attribute__((packed)) FlightPacket
{
    int32_t lat_e6;          // degrees x 1,000,000
    int32_t lon_e6;          // degrees x 1,000,000
    int16_t gps_alt_m;       // GPS altitude, whole meters
    int16_t speed_dkn;       // GPS speed, knots x 10
    uint8_t sats;
    uint8_t fix;             // 0 = no GPS fix (then lat/lon/gps_alt/speed are not valid)

    int32_t baro_alt_cm;     // barometric altitude above power-on, cm
    int16_t phi_cdeg;        // roll, 0.01 degree
    int16_t theta_cdeg;      // pitch, 0.01 degree
    int16_t vert_acc_cms2;   // vertical linear acceleration, 0.01 m/s^2
};

static_assert(sizeof(FlightPacket) == 24, "FlightPacket must be exactly 24 bytes");

static const int32_t NO_DATA_I32 = INT32_MIN;
static const int16_t NO_DATA_I16 = INT16_MIN;

// Rounds to the nearest integer (works for negative values too)
static inline int32_t roundToInt(double v)
{
    return (int32_t)(v >= 0 ? v + 0.5 : v - 0.5);
}

// Rounds and limits to the 16-bit range (-32768 is kept for "no data")
static inline int16_t toInt16(double v)
{
    int32_t r = roundToInt(v);

    if (r >  32767) r =  32767;
    if (r < -32767) r = -32767;

    return (int16_t)r;
}

static inline void packFlightData(const FlightData &d, FlightPacket &p)
{
    p.lat_e6    = roundToInt((double)d.lat * 1000000.0);
    p.lon_e6    = roundToInt((double)d.lon * 1000000.0);
    p.gps_alt_m = toInt16(d.gpsAltM);
    p.speed_dkn = toInt16((double)d.speedKn * 10.0);
    p.sats      = (uint8_t)d.sats;
    p.fix       = (uint8_t)d.fix;

    p.baro_alt_cm = d.baroValid ? roundToInt((double)d.baroAltM * 100.0) : NO_DATA_I32;

    if (d.imuValid)
    {
        p.phi_cdeg      = toInt16((double)d.phiDeg * 100.0);
        p.theta_cdeg    = toInt16((double)d.thetaDeg * 100.0);
        p.vert_acc_cms2 = toInt16((double)d.vertAccMps2 * 100.0);
    }
    else
    {
        p.phi_cdeg      = NO_DATA_I16;
        p.theta_cdeg    = NO_DATA_I16;
        p.vert_acc_cms2 = NO_DATA_I16;
    }
}

static inline void unpackFlightPacket(const FlightPacket &p, FlightData &d)
{
    d.lat     = p.lat_e6 / 1000000.0f;
    d.lon     = p.lon_e6 / 1000000.0f;
    d.gpsAltM = (float)p.gps_alt_m;
    d.speedKn = p.speed_dkn / 10.0f;
    d.sats    = p.sats;
    d.fix     = p.fix;

    d.baroValid = (p.baro_alt_cm != NO_DATA_I32);
    d.baroAltM  = d.baroValid ? p.baro_alt_cm / 100.0f : 0.0f;

    d.imuValid = (p.phi_cdeg != NO_DATA_I16 && p.theta_cdeg != NO_DATA_I16 &&
                  p.vert_acc_cms2 != NO_DATA_I16);

    d.phiDeg      = d.imuValid ? p.phi_cdeg / 100.0f      : 0.0f;
    d.thetaDeg    = d.imuValid ? p.theta_cdeg / 100.0f    : 0.0f;
    d.vertAccMps2 = d.imuValid ? p.vert_acc_cms2 / 100.0f : 0.0f;
}
