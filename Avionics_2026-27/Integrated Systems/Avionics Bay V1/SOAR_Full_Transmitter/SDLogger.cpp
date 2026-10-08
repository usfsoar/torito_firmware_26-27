#include "SDLogger.h"
#include "Config.h"

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

bool SDLogger::begin()
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

    bool gpsOk    = openChannel(gpsLog,    "gpslog.csv",  "ms,lat,lon,gps_alt_m,speed_kn,sats,fix");
    bool sensorOk = openChannel(sensorLog, "sensors.csv",
        "ms,baro_alt_m,max_alt_m,pressure_hpa,temp_c,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps,phi_deg,theta_deg,vert_acc_mps2");

    lastFlush = millis();

    Serial.println(gpsOk && sensorOk ? "SD LOGGER READY." : "SD LOGGER PARTLY READY (see errors above).");
    Serial.println("=======================================");

    return gpsOk || sensorOk;
}

void SDLogger::logGps(const FlightData &d)
{
    if (!gpsLog.open)
        return;

    char line[96];

    int n = snprintf(
        line,
        sizeof(line),
        "%lu,%.6f,%.6f,%.2f,%.2f,%d,%d\n",
        (unsigned long)millis(),
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

    char   line[220];
    size_t n = 0;

    n += snprintf(line, sizeof(line), "%lu,", (unsigned long)millis());

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
    addField(line, sizeof(line), n, s.imuValid, s.vertAccMps2, 3, false);

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
        lastFlush = millis();
    }
}
