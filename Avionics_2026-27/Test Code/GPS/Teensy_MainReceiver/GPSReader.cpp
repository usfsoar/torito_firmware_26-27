#include "GPSReader.h"
#include "Config.h"

GPSReader::GPSReader()
    : gps(&Serial2)
{
}

void GPSReader::begin()
{
    gps.begin(GPS_BAUD_RATE);
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
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

int GPSReader::satellites()
{
    return (int)gps.satellites;
}

float GPSReader::distanceTo(float lat2, float lon2)
{
    const float R = 6371000.0;

    float dLat = radians(lat2 - latitude());
    float dLon = radians(lon2 - longitude());

    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(radians(latitude())) * cos(radians(lat2)) *
              sin(dLon / 2) * sin(dLon / 2);

    return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}
