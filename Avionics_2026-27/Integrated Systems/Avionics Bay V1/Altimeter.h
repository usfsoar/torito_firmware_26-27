#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <MS5607.h>

// Reads the MS5607 barometer and reports altitude RELATIVE to power-on:
// the pressure measured at startup is the 0 m reference, so it does not
// matter that we are not at sea level.
// Also keeps track of the highest relative altitude seen (max altitude).
class Altimeter
{
public:
    Altimeter();

    // Starts the sensor and zeroes the altitude. Returns false if the sensor
    // did not answer (then altitude stays unavailable).
    bool begin();

    // Takes one reading (blocks about 2-3 ms). Call at about 10 Hz.
    // Returns true only when the altitude is valid.
    bool update();

    // Takes a new 0 m reference from the current pressure and clears the max.
    bool zero();

    float temperatureC();
    float pressureHPa();
    float zeroPressureHPa();

    // Relative altitude in meters: the startup position is 0 m.
    float altitudeM();

    // Highest relative altitude measured since zeroing.
    float maxAltitudeM();

    // Forget the max altitude and start tracking again from the current altitude.
    void resetMaxAltitude();

    bool isZeroed();

private:
    MS5607 sensor;

    float temperature   = 0;
    float pressure      = 0;
    float zeroPressure  = 0;
    float altitude      = 0;
    float maxAltitude   = 0;
    bool  zeroInitialized = false;

    // One reading. Returns false if it looks invalid (sensor not answering).
    bool readRaw(float &tempC, float &pressureHPa);
};
