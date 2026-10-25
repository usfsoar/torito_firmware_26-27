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
    void logLinkPacket(uint32_t session, uint32_t sequence, uint32_t receiverTotal,
                       int rssi);
    bool saveLinkSummary(uint32_t session, uint32_t txAttempts, uint32_t txCompleted,
                         uint32_t txFailed, uint32_t rxUnique, uint32_t duplicates);

private:
    static const size_t BUF_SIZE = 4096;

    SdFs     sd;
    FsFile   file;
    FsFile   linkFile;

    char     buf[BUF_SIZE];
    char     linkBuf[BUF_SIZE];
    size_t   linkIdx=0;
    bool     linkOk=false;
    size_t   idx       = 0;
    uint32_t lastFlush = 0;
    bool     ok        = false;

    bool flushBuffer();
    bool flushLink();
};
