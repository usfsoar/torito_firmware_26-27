#pragma once
#include <Arduino.h>

// Everything the transmitter knows at one moment, in normal units.
// FlightPacket.h turns this into the compact radio packet and back.
struct FlightData
{
    // ----- GPS -----
    int   fix     = 0;        // 0 = no fix, 1 = GPS, 2 = DGPS
    int   sats    = 0;
    float lat     = 0;        // degrees
    float lon     = 0;        // degrees
    float gpsAltM = 0;        // meters above sea level (GPS)
    float speedKn = 0;        // knots

    // ----- Barometer (MS5607) -----
    bool  baroValid = false;
    float baroAltM  = 0;      // meters above the power-on position

    // ----- IMU (ICM-20948), derived from the accelerometer -----
    bool  imuValid    = false;
    float phiDeg      = 0;    // roll
    float thetaDeg    = 0;    // pitch
    float vertAccMps2 = 0;    // vertical linear acceleration, 1 g removed
};

// Raw sensor values for the SD sensor log (transmitter only).
struct SensorSample
{
    bool  baroValid = false;
    float baroAltM = 0, maxAltM = 0, pressureHPa = 0, tempC = 0;

    bool  imuValid = false;
    float axMg = 0, ayMg = 0, azMg = 0;          // accelerometer
    float gxDps = 0, gyDps = 0, gzDps = 0;       // gyroscope
    float phiDeg = 0, thetaDeg = 0, vertAccMps2 = 0;
};
