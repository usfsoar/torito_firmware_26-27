#include "SDLogger.h"
#include "Config.h"

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
        file.println("ms,lat,lon,alt_m,speed_kn,sats,fix,rssi_dbm,dist_m");
        file.sync();

        Serial.println("CSV header created.");
    }

    lastFlush = millis();
    ok = true;

    Serial.println("SD LOGGER READY.");
    Serial.println("=======================================");

    return true;
}

void SDLogger::log(float lat, float lon, float alt, float spd, int sats, int fix, int rssi, float dist)
{
    if (!ok)
        return;

    char line[96];

    int n = snprintf(
        line,
        sizeof(line),
        "%lu,%.6f,%.6f,%.2f,%.2f,%d,%d,%d,%.2f\n",
        (unsigned long)millis(),
        lat,
        lon,
        alt,
        spd,
        sats,
        fix,
        rssi,
        dist
    );

    if (n <= 0 || n >= (int)sizeof(line))
        return;

    // Buffer full -> write it out first
    if (idx + (size_t)n > BUF_SIZE)
    {
        flushBuffer();
    }

    memcpy(&buf[idx], line, n);
    idx += n;
}

void SDLogger::service()
{
    if (ok && idx > 0 && millis() - lastFlush >= LOG_FLUSH_MS)
    {
        flushBuffer();
    }
}
