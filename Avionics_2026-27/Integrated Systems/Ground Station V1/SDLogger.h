#pragma once
#include <Arduino.h>
#include <SdFat.h>
#include "FlightData.h"

// Logs received data to receiver_log.csv on the SD card.
// Writes are buffered in RAM so logging does not slow down the radio.
class SDLogger
{
public:
    bool begin();

    // dist < 0 means "no distance available"
    void log(const FlightData &d, int rssi, float dist);

    // Call every loop(): flushes the buffer to the card about once a second
    void service();

private:
    static const size_t BUF_SIZE = 4096;

    SdFs     sd;
    FsFile   file;

    char     buf[BUF_SIZE];
    size_t   idx       = 0;
    uint32_t lastFlush = 0;
    bool     ok        = false;

    bool flushBuffer();
};
