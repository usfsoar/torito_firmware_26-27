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
    bool begin(); // Mount SD then start logging; compatibility helper.
    bool mount(); // Call at boot while software is OFF; creates no CSV files.
    bool beginLogging(); // Call after validated 1 or saved enabled-state=1.
    // Persistent software-enable state. Uses the SAME mounted SD filesystem as
    // the logger (not EEPROM and not inferred from CSV log existence).
    bool cardReady() const { return cardOk; }
    bool readEnableRecord(uint8_t slot, uint8_t *out, size_t length);
    bool writeEnableRecord(uint8_t slot, const uint8_t *data, size_t length);

    void logGps(const FlightData &d);
    void logSensors(const SensorSample &s);
    // Additional CSV; preserves the original sensors.csv packet/log fields.
    void logKalman(bool valid, float rawAltitudeM, float rawAccelMps2,
                   float filteredAltitudeM, float velocityMps, float filteredAccelMps2,
                   float peakM, float deltaFromPeakM,
                   bool simDrogue, bool simMain, bool simDroguePulse, bool simMainPulse,
                   bool rawVelocityValid, float rawVelocityMps);
    // SIMULATED flags only. Not sufficient as a flight-safe recovery checkpoint.
    bool loadSimState(bool &drogue, bool &main);
    bool saveSimState(bool drogue, bool main);

    // Call every loop(): flushes the buffers to the card about once a second
    void service();
    void flushNow();
    void logTelemetryAttempt(uint32_t session, uint32_t sequence, bool txDone,
                             uint32_t attempts, uint32_t completed, uint32_t failed);
    bool saveLinkSummary(uint32_t session, uint32_t attempts, uint32_t completed,
                         uint32_t failed);

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
    Channel  kalmanLog;
    Channel  txLinkLog;

    uint32_t lastFlush = 0;
    bool     cardOk    = false;
    bool     loggingStarted = false;

    bool openChannel(Channel &c, const char *name, const char *header);
    void append(Channel &c, const char *line, int n);
    bool flushChannel(Channel &c);
};
