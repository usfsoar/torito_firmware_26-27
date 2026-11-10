#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_ADS1X15.h>
#include "Config.h"
#include "TelemetryPacket.h"

class SensorReader {
public:
    SensorReader();
    bool begin();
    void readInto(TelemetryPacket &packet, uint32_t sequence);
    static float convertADCToPressure_psi(float voltage);
private:
    Adafruit_MAX31865 rtd;
    Adafruit_ADS1115 ads;
};
