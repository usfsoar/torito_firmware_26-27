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
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

    Serial.println("GPS started.");
}

void GPSReader::update()
{
    gps.read();

    if (gps.newNMEAreceived())
    {
        gps.parse(gps.lastNMEA());
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
