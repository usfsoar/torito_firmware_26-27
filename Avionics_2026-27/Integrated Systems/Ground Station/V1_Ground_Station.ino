// ============================================================
// SOAR USF - Main Receiver V1 (Teensy 4.1)
// Binary packets + distance to rocket + external SD logger
//
// Files in this sketch folder:
//   Config.h             pins, radio settings, flags
//   GpsPacket.h          the 18-byte radio packet (same as transmitter)
//   GPSReader.h/.cpp     receiver's own GPS + distance calculation
//   RadioReceiver.h/.cpp RFM96W radio
//   SDLogger.h/.cpp      SD card logging
// ============================================================

// Libraries are listed here so the Arduino IDE finds them
// for the .cpp files as well.
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <SdFat.h>

#include "Config.h"
#include "GpsPacket.h"
#include "GPSReader.h"
#include "RadioReceiver.h"
#include "SDLogger.h"

// ============================================================
// GLOBAL OBJECTS
// ============================================================

GPSReader     receiverGPS;
SDLogger      sdLogger;
RadioReceiver receiver;

uint32_t packetCount   = 0;
uint32_t lastRateTime  = 0;
uint32_t lastPrintTime = 0;

// ============================================================
// SERIAL OUTPUT
// ============================================================

void printRocketData(float lat, float lon, float alt, float spd,
                     int sats, int fix, int rssi, float dist)
{
    Serial.println("=== Rocket GPS Data (latest packet) ===");

    Serial.print("Latitude:    ");
    Serial.println(lat, 6);

    Serial.print("Longitude:   ");
    Serial.println(lon, 6);

    Serial.print("Altitude:    ");
    Serial.print(alt);
    Serial.println(" m");

    Serial.print("Speed:       ");
    Serial.print(spd);
    Serial.println(" knots");

    Serial.print("Satellites:  ");
    Serial.println(sats);

    Serial.print("Fix Quality: ");
    Serial.println(fix);

    Serial.print("RSSI:        ");
    Serial.print(rssi);
    Serial.println(" dBm");

    if (dist >= 0)
    {
        Serial.print("Distance:    ");
        Serial.print(dist);
        Serial.println(" m");
    }
    else
    {
        Serial.println("Distance:    Waiting for receiver GPS fix...");
    }

    Serial.print("Receiver GPS: ");
    Serial.print(receiverGPS.latitude(), 6);
    Serial.print(", ");
    Serial.print(receiverGPS.longitude(), 6);
    Serial.print("  sats: ");
    Serial.println(receiverGPS.satellites());

    Serial.println("---");
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // Wait for the Serial Monitor to open
    while (!Serial)
        delay(10);

    Serial.println("SOAR USF - Main Receiver V1 (BINARY)");

    // ----- Receiver's own GPS -----
    receiverGPS.begin();

    // ----- SD card (optional) -----
    if (sdLogger.begin())
        Serial.println("SD card ready, logging to receiver_log.csv");
    else
        Serial.println("WARNING: SD card init failed - continuing WITHOUT logging");

    // ----- Radio -----
    Serial.println("Starting radio setup...");

    if (!receiver.begin())
    {
        Serial.println("ERROR: RFM96W init failed!");

        while (1)
        {
            delay(1000);
        }
    }

    Serial.println("Ready. Waiting for rocket data...");
    Serial.println("---");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // Keep reading the receiver's own GPS
    receiverGPS.update();

    // Flush the SD buffer when needed
    sdLogger.service();

    if (receiver.available())
    {
        float rocketLat = 0, rocketLon = 0, altitude = 0, speed = 0;
        int   satellites = 0, fixQuality = 0;

        if (receiver.receive(rocketLat, rocketLon, altitude, speed, satellites, fixQuality))
        {
            packetCount++;

            int   rssi = receiver.rssi();
            float dist = -1;

            if (receiverGPS.hasFix())
                dist = receiverGPS.distanceTo(rocketLat, rocketLon);

            // Print the full block only once per second so the Serial Monitor can keep up
            if (millis() - lastPrintTime >= PRINT_EVERY_MS)
            {
                lastPrintTime = millis();

                printRocketData(rocketLat, rocketLon, altitude, speed,
                                satellites, fixQuality, rssi, dist);
            }

            sdLogger.log(rocketLat, rocketLon, altitude, speed,
                         satellites, fixQuality, rssi, dist);
        }
        else
        {
            Serial.println("ERROR: Receive failed.");
        }
    }

    // Packets-per-second counter
    if (millis() - lastRateTime >= 1000)
    {
        Serial.print(">>> Packets received in last second: ");
        Serial.println(packetCount);

        packetCount  = 0;
        lastRateTime = millis();
    }
}
