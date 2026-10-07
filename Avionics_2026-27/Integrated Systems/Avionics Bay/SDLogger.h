#pragma once
#include <Arduino.h>
#include <SdFat.h>
#include "FlightData.h"

// Logs to the SD card, buffered in RAM so logging does not slow down the radio.
//   gpslog.csv   - GPS data, one row per transmitted packet (when the GPS has a fix)
//   sensors.csv  - altitude, raw accelerometer/gyro and derived angles, 10 rows per second
class SDLogger
{
public:
    bool begin();

    void logGps(const FlightData &d);
    void logSensors(const SensorSample &s);

    // Call every loop(): flushes the buffers to the card about once a second
    void service();

private:
    static const size_t BUF_SIZE = 4096;

    struct Channel
    {
        FsFile file;
        char   buf[BUF_SIZE];
        size_t idx  = 0;
        bool   open = false;
    };

    SdFs     sd;
    Channel  gpsLog;
    Channel  sensorLog;

    uint32_t lastFlush = 0;
    bool     cardOk    = false;

    bool openChannel(Channel &c, const char *name, const char *header);
    void append(Channel &c, const char *line, int n);
    bool flushChannel(Channel &c);
};
