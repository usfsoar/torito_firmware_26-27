#include "GPSReader.h"
#include "Config.h"

GPSReader::GPSReader()
    : gps(&Serial2)
{
}

void GPSReader::begin()
{
    Serial.println("Starting GPS...");

    gps.begin(GPS_BAUD_RATE);
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Adafruit PA1010D Mini GPS, UART on Teensy Serial2 (GPS TXO->pin 7 RX2,
    // GPS RXI->pin 8 TX2). Keep RMC+GGA for position, speed, altitude, sats.
    // At 9600 baud, 5 Hz RMC+GGA is supported; 10 Hz needs RMC-only,
    // which would lose the GGA altitude/satellite information.
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

    Serial.println("GPS started.");
}

void GPSReader::update()
{
    // Read EVERY character that has arrived, not just one. The main loop can be
    // busy for a few milliseconds (radio send, SD write), and at 9600 baud the
    // serial buffer would otherwise fill up and drop characters.
    while (Serial2.available())
    {
        gps.read();

        if (gps.newNMEAreceived())
        {
            gps.parse(gps.lastNMEA());
        }
    }
}

bool GPSReader::hasFix()
{
    return gps.fix;
}

float GPSReader::latitude()
{
    return gps.latitudeDegrees;
}

float GPSReader::longitude()
{
    return gps.longitudeDegrees;
}

float GPSReader::altitude()
{
    return gps.altitude;
}

float GPSReader::speed()
{
    return gps.speed;
}

int GPSReader::satellites()
{
    return (int)gps.satellites;
}

int GPSReader::fixQuality()
{
    return (int)gps.fixquality;
}
