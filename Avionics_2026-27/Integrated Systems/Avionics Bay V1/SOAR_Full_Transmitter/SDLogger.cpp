#include "SDLogger.h"
#include "Config.h"

// Formats milliseconds since power-on as HH:MM:SS.mmm
static void formatTimestamp(char* buf, size_t bufSize)
{
    uint32_t ms      = millis();
    uint32_t hours   = ms / 3600000UL;
    uint32_t minutes = (ms % 3600000UL) / 60000UL;
    uint32_t seconds = (ms % 60000UL) / 1000UL;
    uint32_t milsec  = ms % 1000UL;

    snprintf(buf, bufSize, "%02lu:%02lu:%02lu.%03lu",
             (unsigned long)hours,
             (unsigned long)minutes,
             (unsigned long)seconds,
             (unsigned long)milsec);
}

// Appends one number to a CSV line, or NA when the value is not available.
static void addField(char *line, size_t cap, size_t &n, bool valid, float v, int decimals, bool comma)
{
    if (n >= cap)
        return;

    int w;

    if (valid)
        w = snprintf(line + n, cap - n, "%.*f", decimals, (double)v);
    else
        w = snprintf(line + n, cap - n, "NA");

    if (w > 0)
        n += (size_t)w;

    if (comma && n + 1 < cap)
    {
        line[n++] = ',';
        line[n]   = '\0';
    }
}

bool SDLogger::flushChannel(Channel &c)
{
    if (!c.open)
        return false;

    if (c.idx == 0)
        return true;

    size_t written = c.file.write((const uint8_t *)c.buf, c.idx);

    c.file.sync();

    bool good = (written == c.idx);

    if (!good)
    {
        Serial.println("ERROR: SD write failed!");

        Serial.print("Expected bytes: ");
        Serial.println(c.idx);

        Serial.print("Written bytes: ");
        Serial.println(written);
    }

    c.idx = 0;
    return good;
}

void SDLogger::append(Channel &c, const char *line, int n)
{
    if (!c.open || n <= 0)
        return;

    // Buffer full -> write it out first
    if (c.idx + (size_t)n > BUF_SIZE)
    {
        flushChannel(c);
    }

    memcpy(&c.buf[c.idx], line, n);
    c.idx += n;
}

bool SDLogger::openChannel(Channel &c, const char *name, const char *header)
{
    Serial.print("Opening ");
    Serial.print(name);
    Serial.println("...");

    c.file = sd.open(name, O_RDWR | O_CREAT | O_AT_END);

    if (!c.file)
    {
        Serial.print("ERROR: Could not open ");
        Serial.println(name);

        sd.errorPrint(&Serial);

        c.open = false;
        return false;
    }

    // Add the CSV header if this is a new file
    if (c.file.size() == 0)
    {
        c.file.println(header);
        c.file.sync();
    }

    c.idx  = 0;
    c.open = true;

    Serial.print(name);
    Serial.println(" ready.");

    return true;
}

bool SDLogger::mount()
{
    Serial.println();
    Serial.println("========== SD INITIALIZATION ==========");

    bool success = false;

    if (SD_IS_BUILTIN)
    {
        // ----- Teensy built-in SDIO slot -----
        Serial.println("Using Teensy built-in SDIO slot...");

        success = sd.begin(SdioConfig(FIFO_SDIO));
    }
    else
    {
        // ----- External SD breakout on SPI1 -----
        Serial.println("Using external SD breakout on SPI1.");

        Serial.print("SD CS pin: ");
        Serial.println(SD_CS_PIN);

        Serial.print("SD SPI speed: ");
        Serial.print(SD_SPI_SPEED_MHZ);
        Serial.println(" MHz");

        // Make sure SD is deselected initially
        pinMode(SD_CS_PIN, OUTPUT);
        digitalWrite(SD_CS_PIN, HIGH);

        // Explicitly start SPI1
        SPI1.begin();

        delay(100);

        success = sd.begin(
            SdSpiConfig(
                SD_CS_PIN,
                SHARED_SPI,
                SD_SCK_MHZ(SD_SPI_SPEED_MHZ),
                &SPI1
            )
        );
    }

    if (!success)
    {
        Serial.println();
        Serial.println("!!! SD INITIALIZATION FAILED !!!");

        // Detailed SdFat diagnostic information
        sd.initErrorPrint(&Serial);

        if (sd.card())
        {
            Serial.print("SD card errorCode: 0x");
            Serial.println(sd.card()->errorCode(), HEX);

            Serial.print("SD card errorData: 0x");
            Serial.println(sd.card()->errorData(), HEX);
        }

        Serial.println();
        Serial.println("Check:");
        Serial.println("1. SD card inserted");
        Serial.println("2. CS wiring");
        Serial.println("3. SPI1 MOSI/MISO/SCK wiring");
        Serial.println("4. GND");
        Serial.println("5. Power");
        Serial.println("6. SD card format");
        Serial.println("=======================================");

        cardOk = false;
        return false;
    }

    Serial.println("SD card communication SUCCESS.");
    cardOk = true;

    return true;
}

bool SDLogger::beginLogging() {
    if (!cardOk) return false;
    if (loggingStarted) return true;

    bool gpsOk    = openChannel(gpsLog,    "gpslog.csv",  "timestamp,lat,lon,gps_alt_m,speed_kn,sats,fix");
    bool sensorOk = openChannel(sensorLog, "sensors.csv",
        "timestamp,baro_alt_m,max_alt_m,pressure_hpa,temp_c,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps,phi_deg,theta_deg,vert_acc_mps2,drogue,main");

    bool kalmanOk = openChannel(kalmanLog, "kalman_raw.csv",
        "timestamp,filter_valid,raw_baro_alt_m,raw_imu_accel_mps2,kalman_alt_m,kalman_velocity_mps,kalman_accel_mps2,raw_peak_m,raw_drop_from_peak_m,sim_drogue,sim_main,sim_drogue_pulse,sim_main_pulse,raw_baro_velocity_mps");

    bool linkOk=openChannel(txLinkLog,"tx_link_packets.csv",
        "ms,session_id,tx_sequence,radio_tx_done,tx_attempts_total,tx_done_total,tx_failed_total");
    if (!linkOk) Serial.println("WARNING: tx_link_packets.csv unavailable");
    lastFlush = millis();
    if (!kalmanOk) Serial.println("WARNING: kalman_raw.csv unavailable");

    Serial.println(gpsOk && sensorOk ? "SD LOGGER READY." : "SD LOGGER PARTLY READY (see errors above).");
    Serial.println("=======================================");

    loggingStarted = gpsOk || sensorOk;
    return loggingStarted;
}

bool SDLogger::begin() {
    return mount() && beginLogging();
}

void SDLogger::logGps(const FlightData &d)
{
    if (!gpsLog.open)
        return;

    char ts[16];
    formatTimestamp(ts, sizeof(ts));

    char line[100];

    int n = snprintf(
        line,
        sizeof(line),
        "%s,%.6f,%.6f,%.2f,%.2f,%d,%d\n",
        ts,
        (double)d.lat,
        (double)d.lon,
        (double)d.gpsAltM,
        (double)d.speedKn,
        d.sats,
        d.fix
    );

    if (n <= 0 || n >= (int)sizeof(line))
        return;

    append(gpsLog, line, n);
}

void SDLogger::logSensors(const SensorSample &s)
{
    if (!sensorLog.open)
        return;

    char ts[16];
    formatTimestamp(ts, sizeof(ts));

    char   line[230];
    size_t n = 0;

    n += snprintf(line, sizeof(line), "%s,", ts);

    addField(line, sizeof(line), n, s.baroValid, s.baroAltM,    2, true);
    addField(line, sizeof(line), n, s.baroValid, s.maxAltM,     2, true);
    addField(line, sizeof(line), n, s.baroValid, s.pressureHPa, 2, true);
    addField(line, sizeof(line), n, s.baroValid, s.tempC,       2, true);

    addField(line, sizeof(line), n, s.imuValid, s.axMg,  2, true);
    addField(line, sizeof(line), n, s.imuValid, s.ayMg,  2, true);
    addField(line, sizeof(line), n, s.imuValid, s.azMg,  2, true);
    addField(line, sizeof(line), n, s.imuValid, s.gxDps, 2, true);
    addField(line, sizeof(line), n, s.imuValid, s.gyDps, 2, true);
    addField(line, sizeof(line), n, s.imuValid, s.gzDps, 2, true);

    addField(line, sizeof(line), n, s.imuValid, s.phiDeg,      2, true);
    addField(line, sizeof(line), n, s.imuValid, s.thetaDeg,    2, true);
    addField(line, sizeof(line), n, s.imuValid, s.vertAccMps2, 3, true);

    n += snprintf(line + n, sizeof(line) - n, "%d,%d", s.drogue ? 1 : 0, s.mainDeployed ? 1 : 0);

    if (n + 2 >= sizeof(line))
        return;

    line[n++] = '\n';
    line[n]   = '\0';

    append(sensorLog, line, (int)n);
}

void SDLogger::service()
{
    if (!cardOk)
        return;

    if (millis() - lastFlush >= LOG_FLUSH_MS)
    {
        flushChannel(gpsLog);
        flushChannel(sensorLog);
        flushChannel(kalmanLog);
        flushChannel(txLinkLog);
        lastFlush = millis();
    }
}


void SDLogger::logKalman(bool valid, float rawAltitudeM, float rawAccelMps2,
                         float filteredAltitudeM, float velocityMps, float filteredAccelMps2,
                         float peakM, float deltaFromPeakM,
                         bool simDrogue, bool simMain, bool simDroguePulse, bool simMainPulse,
                         bool rawVelocityValid, float rawVelocityMps)
{
    if (!kalmanLog.open) return;
    char ts[16]; formatTimestamp(ts, sizeof(ts));
    char line[280];
    char rawVel[32];
    if (rawVelocityValid)
        snprintf(rawVel, sizeof(rawVel), "%.3f", (double)rawVelocityMps);
    else
        snprintf(rawVel, sizeof(rawVel), "NA");
    int n;
    if (valid) {
        n = snprintf(line, sizeof(line),
            "%s,1,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%s\n",
            ts, (double)rawAltitudeM, (double)rawAccelMps2,
            (double)filteredAltitudeM, (double)velocityMps, (double)filteredAccelMps2,
            (double)peakM, (double)deltaFromPeakM,
            (unsigned)simDrogue, (unsigned)simMain,
            (unsigned)simDroguePulse, (unsigned)simMainPulse, rawVel);
    } else {
        // Keep raw sensor and simulation state available even if Kalman is invalid.
        n = snprintf(line, sizeof(line),
            "%s,0,%.3f,%.3f,NA,NA,NA,%.3f,%.3f,%u,%u,%u,%u,%s\n",
            ts, (double)rawAltitudeM, (double)rawAccelMps2,
            (double)peakM, (double)deltaFromPeakM,
            (unsigned)simDrogue, (unsigned)simMain,
            (unsigned)simDroguePulse, (unsigned)simMainPulse, rawVel);
    }
    if (n > 0 && n < (int)sizeof(line)) append(kalmanLog, line, n);
}

bool SDLogger::loadSimState(bool &drogue, bool &main)
{
    drogue = main = false;
    if (!cardOk) return false;
    FsFile f = sd.open("sim_raw_state.txt", O_RDONLY);
    if (!f) return false;
    char line[20] = {};
    int n = f.read(line, sizeof(line) - 1);
    f.close();
    if (n <= 0) return false;
    int d = -1, m = -1;
    if (sscanf(line, "SIMD=%d,M=%d", &d, &m) != 2 ||
        (d != 0 && d != 1) || (m != 0 && m != 1)) return false;
    drogue = (d == 1);
    main = (m == 1);
    return true;
}

bool SDLogger::saveSimState(bool drogue, bool main)
{
    if (!cardOk) return false;
    // Not power-loss atomic; simulation/audit only, NEVER suitable for
    // deciding whether to re-energize live recovery hardware on reboot.
    FsFile f = sd.open("sim_raw_state.txt", O_RDWR | O_CREAT | O_TRUNC);
    if (!f) return false;
    char line[20];
    int n = snprintf(line, sizeof(line), "SIMD=%d,M=%d\n", drogue ? 1 : 0, main ? 1 : 0);
    size_t written = f.write((const uint8_t *)line, (size_t)n);
    bool success = written == (size_t)n && f.sync();
    f.close();
    return success;
}

void SDLogger::flushNow() {
    if (!cardOk) return;
    flushChannel(gpsLog);
    flushChannel(sensorLog);
    flushChannel(kalmanLog);
    flushChannel(txLinkLog);
    lastFlush=millis();
}

void SDLogger::logTelemetryAttempt(uint32_t session, uint32_t sequence, bool done,
                                   uint32_t attempts, uint32_t completed, uint32_t failed) {
    char line[128];
    const int n=snprintf(line,sizeof(line),"%lu,%lu,%lu,%u,%lu,%lu,%lu\n",
        (unsigned long)millis(),(unsigned long)session,(unsigned long)sequence,
        done?1U:0U,(unsigned long)attempts,(unsigned long)completed,(unsigned long)failed);
    if (n>0 && n<(int)sizeof(line)) append(txLinkLog,line,n);
}
bool SDLogger::saveLinkSummary(uint32_t session, uint32_t attempts,
                                uint32_t completed, uint32_t failed) {
    if (!cardOk) return false;
    flushNow();
    FsFile out=sd.open("tx_link_summary.csv", O_RDWR | O_CREAT | O_AT_END);
    if (!out) return false;
    if (out.size()==0) out.println("ms,session_id,tx_attempts,tx_done,tx_failed");
    const int n=out.printf("%lu,%lu,%lu,%lu,%lu\n",
        (unsigned long)millis(),(unsigned long)session,(unsigned long)attempts,
        (unsigned long)completed,(unsigned long)failed);
    const bool ok=n>0 && out.sync(); out.close(); return ok;
}

// Store two redundant, checksummed enable-state records. Each write replaces
// the *other* slot, leaving the previous record available after a torn write.
// Files are independent of the CSV logs and never imply physical solenoid arming.
static const char *enableRecordName(uint8_t slot) {
    return slot == 0 ? "AVBAY_A.DAT" : (slot == 1 ? "AVBAY_B.DAT" : nullptr);
}

bool SDLogger::readEnableRecord(uint8_t slot, uint8_t *out, size_t length) {
    if (!cardOk || !out || length != 16U) return false;
    const char *name=enableRecordName(slot);
    if (!name) return false;
    FsFile file=sd.open(name,O_RDONLY);
    if (!file) return false;
    const bool correctSize=(file.size() == length);
    const int n=correctSize ? file.read(out,length) : -1;
    file.close();
    return correctSize && n == (int)length;
}

bool SDLogger::writeEnableRecord(uint8_t slot, const uint8_t *data, size_t length) {
    if (!cardOk || !data || length != 16U) return false;
    const char *name=enableRecordName(slot);
    if (!name) return false;
    FsFile file=sd.open(name,O_WRONLY | O_CREAT | O_TRUNC);
    if (!file) return false;
    const size_t written=file.write(data,length);
    const bool synced = (written == length) && file.sync();
    file.close();
    if (!synced) return false;
    uint8_t check[16];
    if (!readEnableRecord(slot,check,sizeof(check))) return false;
    for (size_t i=0;i<length;++i) if (check[i]!=data[i]) return false;
    return true;
}
