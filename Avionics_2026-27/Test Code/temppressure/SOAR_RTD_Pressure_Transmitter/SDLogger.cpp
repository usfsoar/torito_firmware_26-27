#include "SDLogger.h"

bool SDLogger::begin(const char *filename, const char *header) {
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD unavailable: continuing normally without SD logging.");
        enabled = false;
        return false;
    }
    bool needsHeader = !SD.exists(filename);
    file = SD.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("Could not open SD CSV: continuing without logging.");
        enabled = false;
        return false;
    }
    // New file (or existing 0-byte file) gets exactly one header.
    if (needsHeader || file.size() == 0) {
        file.println(header);
        file.flush();
    }
    enabled = true;
    lastFlushMs = millis();
    Serial.print("SD logging to: ");
    Serial.println(filename);
    return true;
}

void SDLogger::logCsv(const char *line) {
    if (!enabled) return;
    if (!file.println(line)) {
        Serial.println("SD write failed; disabling SD logging (radio continues).");
        enabled = false;
        file.close();
    }
}

void SDLogger::service() {
    if (!enabled) return;
    const uint32_t now = millis();
    if (static_cast<uint32_t>(now - lastFlushMs) >= SD_FLUSH_INTERVAL_MS) {
        file.flush();
        lastFlushMs = now;
    }
}

void SDLogger::close() {
    if (enabled) { file.flush(); file.close(); }
    enabled = false;
}
