#pragma once
#include <Arduino.h>
#include <SD.h>
#include "Config.h"

// Built-in Teensy 4.1 microSD socket (SDIO). No external SD SPI wiring.
// SD is optional; failure here does not prevent sending or receiving.
class SDLogger {
public:
    bool begin(const char *filename, const char *header);
    void logCsv(const char *line);
    void service();
    bool ready() const { return enabled; }
    void close();
private:
    File file;
    bool enabled = false;
    uint32_t lastFlushMs = 0;
};
