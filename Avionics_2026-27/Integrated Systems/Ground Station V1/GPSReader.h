#pragma once
#include <Arduino.h>
#include <Adafruit_GPS.h>

// Reads the receiver's own GPS (Serial2) and measures distance to the rocket.
class GPSReader
{
public:
    GPSReader();

    void begin();
    void update();          // call every loop() to keep parsing GPS data

    bool  hasFix();
    float latitude();
    float longitude();
    int   satellites();

    // Straight-line distance in meters from this GPS to the given point
    float distanceTo(float lat2, float lon2);

private:
    Adafruit_GPS gps;
};
