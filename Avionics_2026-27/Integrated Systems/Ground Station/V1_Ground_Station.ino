// ============================================================
// SOAR USF - Ground Station (Teensy 4.1 receiver)
// Receives the 24-byte FlightPacket from the transmitter, shows it,
// measures distance to the rocket, and logs everything to the SD card.
//
// Files in this sketch folder:
//   Config.h               pins, settings, flags
//   FlightData.h           values in normal units
//   FlightPacket.h         the 24-byte radio packet (same file as the transmitter)
//   GPSReader.h/.cpp       receiver's own GPS + distance calculation
//   RadioReceiver.h/.cpp   RFM96W radio
//   SDLogger.h/.cpp        SD card logging (receiver_log.csv)
// ============================================================

// Libraries are listed here so the Arduino IDE finds them
// for the .cpp files as well.
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <SdFat.h>

#include "Config.h"
#include "FlightData.h"
#include "FlightPacket.h"
#include "GPSReader.h"
#include "RadioReceiver.h"
#include "SDLogger.h"

// ============================================================
// GLOBAL OBJECTS
// ============================================================

GPSReader     receiverGPS;
SDLogger      sdLogger;
RadioReceiver receiver;

uint32_t packetCount  = 0;   // packets received since the last printout
uint32_t lastPrint    = 0;

// The newest packet, shown once a second
FlightData latest;
bool       haveLatest  = false;
int        latestRssi  = 0;
float      latestDist  = -1;

// Highest barometric altitude seen in the packets received
bool  haveMaxAlt = false;
float maxBaroAltM = 0;

// ============================================================
// SERIAL OUTPUT
// ============================================================

void printLatest()
{
    Serial.println("=== Rocket data (latest packet) ===");

    if (latest.fix > 0)
    {
        Serial.print("GPS:         fix ");
        Serial.print(latest.fix);
        Serial.print(", ");
        Serial.print(latest.sats);
        Serial.println(" satellites");

        Serial.print("  Lat/Lon:   ");
        Serial.print(latest.lat, 6);
        Serial.print(", ");
        Serial.println(latest.lon, 6);

        Serial.print("  GPS alt:   ");
        Serial.print(latest.gpsAltM, 0);
        Serial.print(" m   speed: ");
        Serial.print(latest.speedKn, 1);
        Serial.println(" knots");
    }
    else
    {
        Serial.print("GPS:         no fix on the rocket (satellites: ");
        Serial.print(latest.sats);
        Serial.println(")");
    }

    if (latest.baroValid)
    {
        Serial.print("Altitude:    ");
        Serial.print(latest.baroAltM, 2);
        Serial.print(" m (above power-on)   max received: ");
        Serial.print(maxBaroAltM, 2);
        Serial.println(" m");
    }
    else
    {
        Serial.println("Altitude:    not available from the rocket");
    }

    if (latest.imuValid)
    {
        Serial.print("Angles:      phi (roll) ");
        Serial.print(latest.phiDeg, 2);
        Serial.print(" deg   theta (pitch) ");
        Serial.print(latest.thetaDeg, 2);
        Serial.println(" deg");

        Serial.print("Vertical:    ");
        Serial.print(latest.vertAccMps2, 2);
        Serial.println(" m/s^2 (1 g removed)");
    }
    else
    {
        Serial.println("IMU:         not available from the rocket");
    }

    Serial.print("RSSI:        ");
    Serial.print(latestRssi);
    Serial.println(" dBm");

    if (latestDist >= 0)
    {
        Serial.print("Distance:    ");
        Serial.print(latestDist);
        Serial.println(" m");
    }
    else
    {
        Serial.println("Distance:    waiting for a GPS fix on both the rocket and this receiver");
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

    Serial.println("SOAR USF - Ground Station");

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
        FlightData data;

        if (receiver.receive(data))
        {
            packetCount++;

            const int rssi = receiver.rssi();
            float dist = -1;

            // Distance needs a real GPS fix on BOTH the rocket and this receiver
            if (data.fix > 0 && receiverGPS.hasFix())
                dist = receiverGPS.distanceTo(data.lat, data.lon);

            if (data.baroValid && (!haveMaxAlt || data.baroAltM > maxBaroAltM))
            {
                maxBaroAltM = data.baroAltM;
                haveMaxAlt  = true;
            }

            latest     = data;
            latestRssi = rssi;
            latestDist = dist;
            haveLatest = true;

            sdLogger.log(data, rssi, dist);
        }
        else
        {
            Serial.println("ERROR: Receive failed (or not a 24-byte FlightPacket).");
        }
    }

    // Once a second: packet rate, then the newest packet
    if (millis() - lastPrint >= PRINT_EVERY_MS)
    {
        lastPrint = millis();

        Serial.print(">>> Packets received in last second: ");
        Serial.println(packetCount);

        packetCount = 0;

        if (haveLatest)
            printLatest();
        else
            Serial.println("No packets yet.");
    }
}
