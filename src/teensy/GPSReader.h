#pragma once
#include <Arduino.h>
#include <Adafruit_GPS.h>

// Reads the GPS module on Serial2 and exposes the parsed values.
class GPSReader
{
public:
    GPSReader();

    void begin();
    void update();          // call every loop() to keep parsing GPS data

    bool  hasFix();
    float latitude();
    float longitude();
    float altitude();
    float speed();          // knots
    int   satellites();
    int   fixQuality();

private:
    Adafruit_GPS gps;
};
