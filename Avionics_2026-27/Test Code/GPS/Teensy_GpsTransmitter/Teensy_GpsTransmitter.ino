// ============================================================
// SOAR USF - Teensy GPS Transmitter
// Binary packet + external SD logger
//
// Files in this sketch folder:
//   Config.h             pins, radio settings, flags
//   GpsPacket.h          the 18-byte radio packet
//   GPSReader.h/.cpp     GPS module
//   RadioTransmitter.h/.cpp   RFM96W radio
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
#include "RadioTransmitter.h"
#include "SDLogger.h"

// ============================================================
// GLOBAL OBJECTS
// ============================================================

SDLogger         sdLogger;
GPSReader        gpsReader;
RadioTransmitter transmitter;

uint32_t lastTransmitTime = 0;
bool     firstWaitMessage = true;
uint32_t lastWaitPrint    = 0;

// ============================================================
// SERIAL OUTPUT
// ============================================================

void printGpsData(bool transmitted)
{
    Serial.println();
    Serial.println("=== GPS DATA ===");

    Serial.print("TX Status:    ");
    Serial.println(transmitted ? "SUCCESS" : "FAILED");

    Serial.print("Latitude:     ");
    Serial.println(gpsReader.latitude(), 6);

    Serial.print("Longitude:    ");
    Serial.println(gpsReader.longitude(), 6);

    Serial.print("Altitude:     ");
    Serial.print(gpsReader.altitude());
    Serial.println(" m");

    Serial.print("Speed:        ");
    Serial.print(gpsReader.speed());
    Serial.println(" knots");

    Serial.print("Satellites:   ");
    Serial.println(gpsReader.satellites());

    Serial.print("Fix Quality:  ");
    Serial.println(gpsReader.fixQuality());

    Serial.println("--------------------------");
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    // Don't wait forever for USB Serial
    delay(2000);

    Serial.println();
    Serial.println();
    Serial.println("==============================================");
    Serial.println("SOAR USF - Teensy GPS Transmitter");
    Serial.println("Binary Packet + External SD Logger");
    Serial.println("==============================================");

    // ----- GPS -----
    gpsReader.begin();

    // ----- Radio -----
    if (!transmitter.begin())
    {
        Serial.println();
        Serial.println("FATAL ERROR: RFM96W initialization failed.");
        Serial.println("Check radio wiring/pins.");

        while (1)
        {
            delay(1000);
        }
    }

    // ----- SD card (optional) -----
    if (sdLogger.begin())
    {
        Serial.println();
        Serial.println("SD card ready - logging to gpslog.csv");
    }
    else
    {
        Serial.println();
        Serial.println("WARNING: SD failed.");
        Serial.println("Radio/GPS will continue WITHOUT SD logging.");
    }

    Serial.println();
    Serial.println("SYSTEM READY.");
    Serial.println("Waiting for GPS fix...");
    Serial.println("----------------------------------------------");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // Keep reading GPS continuously
    gpsReader.update();

    // Flush the SD buffer when needed
    sdLogger.service();

    // Time to send the next packet?
    if (millis() - lastTransmitTime >= TRANSMIT_INTERVAL)
    {
        lastTransmitTime = millis();

        if (gpsReader.hasFix())
        {
            // ----- GPS fix available: send and log -----
            bool transmitted = transmitter.transmit(
                gpsReader.latitude(),
                gpsReader.longitude(),
                gpsReader.altitude(),
                gpsReader.speed(),
                gpsReader.satellites(),
                gpsReader.fixQuality()
            );

            sdLogger.log(
                gpsReader.latitude(),
                gpsReader.longitude(),
                gpsReader.altitude(),
                gpsReader.speed(),
                gpsReader.satellites(),
                gpsReader.fixQuality()
            );

            printGpsData(transmitted);
        }
        else
        {
            // ----- Still waiting for GPS -----
            if (firstWaitMessage)
            {
                delay(2000);
                firstWaitMessage = false;
            }

            if (millis() - lastWaitPrint >= 1000)
            {
                lastWaitPrint = millis();

                Serial.print("Waiting for GPS fix... Satellites: ");
                Serial.println(gpsReader.satellites());
            }
        }
    }
}
