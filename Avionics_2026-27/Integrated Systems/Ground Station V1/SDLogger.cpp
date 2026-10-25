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

bool SDLogger::flushBuffer()
{
    if (!ok)
        return false;

    if (idx == 0)
        return true;

    size_t written = file.write((const uint8_t *)buf, idx);

    file.sync();

    bool good = (written == idx);

    if (!good)
    {
        Serial.println("ERROR: SD write failed!");

        Serial.print("Expected bytes: ");
        Serial.println(idx);

        Serial.print("Written bytes: ");
        Serial.println(written);
    }

    idx = 0;
    lastFlush = millis();

    return good;
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

        ok = false;
        return false;
    }

    Serial.println("SD card communication SUCCESS.");

    // ----- Open the CSV file -----
    Serial.println("Opening receiver_log.csv...");

    file = sd.open("receiver_log.csv", O_RDWR | O_CREAT | O_AT_END);

    if (!file)
    {
        Serial.println("ERROR: Could not open receiver_log.csv!");

        sd.errorPrint(&Serial);

        ok = false;
        return false;
    }

    Serial.println("receiver_log.csv opened successfully.");

    // Add the CSV header if this is a new file
    if (file.size() == 0)
    {
        file.println("ms,fix,lat,lon,gps_alt_m,speed_kn,sats,baro_alt_m,phi_deg,theta_deg,vert_acc_mps2,rssi_dbm,dist_m,drogue,main");
        file.sync();

        Serial.println("CSV header created.");
    }

    linkFile=sd.open("rx_link_packets.csv", O_RDWR | O_CREAT | O_AT_END);
    linkOk=(bool)linkFile;
    if (linkOk && linkFile.size()==0) {
        linkFile.println("ms,session_id,tx_sequence,rx_unique_total,rssi_dbm");
        linkFile.sync();
    }
    if (!linkOk) Serial.println("WARNING: rx_link_packets.csv unavailable");
    lastFlush = millis();
    ok = true;

    Serial.println("SD LOGGER READY.");
    Serial.println("=======================================");

    return true;
}

void SDLogger::log(const FlightData &d, int rssi, float dist)
{
    if (!ok)
        return;

    char   line[220];
    size_t n = 0;

    n += snprintf(line, sizeof(line), "%lu,%d,", (unsigned long)millis(), d.fix);

    const bool gps = (d.fix > 0);

    addField(line, sizeof(line), n, gps, d.lat,     6, true);
    addField(line, sizeof(line), n, gps, d.lon,     6, true);
    addField(line, sizeof(line), n, gps, d.gpsAltM, 0, true);
    addField(line, sizeof(line), n, gps, d.speedKn, 1, true);

    n += snprintf(line + n, sizeof(line) - n, "%d,", d.sats);

    addField(line, sizeof(line), n, d.baroValid, d.baroAltM,    2, true);
    addField(line, sizeof(line), n, d.imuValid,  d.phiDeg,      2, true);
    addField(line, sizeof(line), n, d.imuValid,  d.thetaDeg,    2, true);
    addField(line, sizeof(line), n, d.imuValid,  d.vertAccMps2, 2, true);

    n += snprintf(line + n, sizeof(line) - n, "%d,", rssi);

    addField(line, sizeof(line), n, dist >= 0, dist, 2, true);

    n += snprintf(line + n, sizeof(line) - n, "%d,%d", d.drogue ? 1 : 0, d.mainDeployed ? 1 : 0);

    if (n + 2 >= sizeof(line))
        return;

    line[n++] = '\n';
    line[n]   = '\0';

    // Buffer full -> write it out first
    if (idx + n > BUF_SIZE)
    {
        flushBuffer();
    }

    memcpy(&buf[idx], line, n);
    idx += n;
}

void SDLogger::service()
{
    if (millis() - lastFlush >= LOG_FLUSH_MS) {
        if (ok && idx > 0) flushBuffer();
        if (linkOk && linkIdx > 0) flushLink();
        lastFlush = millis();
    }
}

bool SDLogger::flushLink() {
    if (!linkOk) return false;
    if (!linkIdx) return true;
    const size_t written=linkFile.write(reinterpret_cast<const uint8_t*>(linkBuf),linkIdx);
    const bool saved=written==linkIdx && linkFile.sync();
    if (!saved) Serial.println("WARNING: rx_link_packets.csv write/sync failed");
    linkIdx=0;
    return saved;
}
void SDLogger::logLinkPacket(uint32_t session, uint32_t sequence,
                             uint32_t received, int rssi) {
    if (!linkOk) return;
    char line[112];
    const int n=snprintf(line,sizeof(line),"%lu,%lu,%lu,%lu,%d\n",
        (unsigned long)millis(),(unsigned long)session,
        (unsigned long)sequence,(unsigned long)received,rssi);
    if (n<=0 || n>=(int)sizeof(line)) return;
    if (linkIdx+(size_t)n>BUF_SIZE) flushLink();
    memcpy(linkBuf+linkIdx,line,n); linkIdx+=(size_t)n;
}
bool SDLogger::saveLinkSummary(uint32_t session, uint32_t txAttempts,
                                uint32_t txCompleted, uint32_t txFailed,
                                uint32_t rxUnique, uint32_t duplicates) {
    if (!ok) return false;
    flushBuffer(); flushLink();
    FsFile out=sd.open("rx_link_summary.csv",O_RDWR | O_CREAT | O_AT_END);
    if (!out) return false;
    if (out.size()==0)
        out.println("ms,session_id,tx_attempts,tx_done,tx_failed,rx_unique,lost,duplicates,delivery_percent");
    const uint32_t loss=(txAttempts>=rxUnique ? txAttempts-rxUnique : 0);
    const double rate=txAttempts ? (100.0*(double)rxUnique/txAttempts) : 0.0;
    const int n=out.printf("%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%.3f\n",
        (unsigned long)millis(),(unsigned long)session,(unsigned long)txAttempts,
        (unsigned long)txCompleted,(unsigned long)txFailed,(unsigned long)rxUnique,
        (unsigned long)loss,(unsigned long)duplicates,rate);
    const bool saved=n>0 && out.sync();out.close();return saved;
}
