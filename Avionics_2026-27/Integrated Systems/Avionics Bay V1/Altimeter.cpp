#include "Altimeter.h"
#include "Config.h"

Altimeter::Altimeter()
    : sensor(MS5607_CS_PIN, false)
{
}

bool Altimeter::readRaw(float &tempC, float &pressureHPa)
{
    sensor.read();

    float t = sensor.getTemperature() / 100.0f;   // hundredths of a degree C -> C
    float p = sensor.getPressure() / 100.0f;      // Pa -> hPa

    // A real reading is between about 300 and 1200 hPa. Anything else means the
    // sensor is not answering. Reject it so it can never corrupt the zero
    // reference or the max altitude.
    if (p < 300.0f || p > 1200.0f)
        return false;

    tempC       = t;
    pressureHPa = p;
    return true;
}

bool Altimeter::begin()
{
    Serial.println("Starting altimeter (MS5607)...");

    SPI.begin();
    sensor.init();

    Serial.print("Zeroing altitude (averaging ");
    Serial.print(ALTITUDE_ZERO_SAMPLES);
    Serial.println(" readings)...");

    if (!zero())
    {
        Serial.println("WARNING: altimeter did not answer. Altitude will be unavailable.");
        Serial.println("Check: MS5607 CS wire (pin 10), SCK/MOSI/MISO, power, and the PS pin at GND.");
        return false;
    }

    Serial.print("Altitude zeroed. Reference pressure ");
    Serial.print(zeroPressure, 2);
    Serial.println(" hPa = 0.00 m");

    return true;
}

bool Altimeter::zero()
{
    float sum  = 0;
    int   good = 0;

    // Allow some bad readings, but do not wait forever if the sensor is missing
    for (int attempt = 0; attempt < ALTITUDE_ZERO_SAMPLES * 5 && good < ALTITUDE_ZERO_SAMPLES; attempt++)
    {
        float t, p;

        if (readRaw(t, p))
        {
            sum += p;
            good++;
        }

        delay(10);
    }

    if (good < ALTITUDE_ZERO_SAMPLES)
    {
        zeroInitialized = false;
        return false;
    }

    zeroPressure    = sum / good;
    altitude        = 0.0f;
    maxAltitude     = 0.0f;
    zeroInitialized = true;

    return true;
}

bool Altimeter::update()
{
    float t, p;

    if (!readRaw(t, p))
        return false;

    temperature = t;
    pressure    = p;

    if (!zeroInitialized)
        return false;

    // Relative altitude from the pressure ratio to the startup pressure
    altitude = 44330.0f * (1.0f - powf(pressure / zeroPressure, 0.1903f));

    // Max altitude: compare every new reading against the highest one so far
    if (altitude > maxAltitude)
        maxAltitude = altitude;

    return true;
}

float Altimeter::temperatureC()
{
    return temperature;
}

float Altimeter::pressureHPa()
{
    return pressure;
}

float Altimeter::zeroPressureHPa()
{
    return zeroPressure;
}

float Altimeter::altitudeM()
{
    return altitude;
}

float Altimeter::maxAltitudeM()
{
    return maxAltitude;
}

void Altimeter::resetMaxAltitude()
{
    maxAltitude = altitude;
}

bool Altimeter::isZeroed()
{
    return zeroInitialized;
}
