#include "IMUSensor.h"
#include "Config.h"

bool IMUSensor::begin()
{
#if IMU_USE_SPI
    Serial.println("Starting IMU (ICM-20948, SPI)...");

    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWrite(IMU_CS_PIN, HIGH);
    SPI.begin();
#else
    Serial.println("Starting IMU (ICM-20948, I2C)...");

    IMU_WIRE.begin();
    IMU_WIRE.setClock(IMU_I2C_CLOCK);
#endif

    // Try a few times: the IMU can take a moment after power-up
    for (int attempt = 1; attempt <= 5; attempt++)
    {
#if IMU_USE_SPI
        icm.begin(IMU_CS_PIN, SPI, IMU_SPI_HZ);
#else
        icm.begin(IMU_WIRE, IMU_AD0_VAL);
#endif

        if (icm.status == ICM_20948_Stat_Ok)
        {
            // Measurement ranges. The chip's defaults (+-2 g and +-250 deg/s) would
            // clip during a rocket launch, so use the widest ranges.
            ICM_20948_fss_t fss;
            fss.a = gpm16;      // accelerometer +-16 g
            fss.g = dps2000;    // gyroscope +-2000 deg/s

            ICM_20948_Status_e result =
                icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);

            if (result != ICM_20948_Stat_Ok)
            {
                Serial.print("WARNING: could not set IMU ranges: ");
                Serial.println(icm.statusString(result));
            }

            ready = true;
            Serial.println("IMU ready (accelerometer +-16 g, gyroscope +-2000 deg/s).");
            return true;
        }

        Serial.print("IMU not answering (");
        Serial.print(icm.statusString());
        Serial.println("), trying again...");

        delay(300);
    }

    Serial.println("ERROR: IMU not found.");
#if IMU_USE_SPI
    Serial.println("Check: SCK 13, MOSI 11, MISO 12, the CS wire (IMU_CS_PIN in Config.h), 3.3V and GND.");
#else
    Serial.println("Check: I2C wiring (SDA 18, SCL 19), 3.3V, GND, and IMU_AD0_VAL in Config.h.");
#endif

    ready = false;
    return false;
}

bool IMUSensor::update()
{
    if (!ready)
        return false;

    if (icm.dataReady())
    {
        icm.getAGMT();   // values are only refreshed when this is called
        dataSeen = true;
        return true;
    }

    return false;
}

bool IMUSensor::isReady()
{
    return ready;
}

bool IMUSensor::hasData()
{
    return dataSeen;
}

float IMUSensor::accelX_mg()      { return icm.accX(); }
float IMUSensor::accelY_mg()      { return icm.accY(); }
float IMUSensor::accelZ_mg()      { return icm.accZ(); }

float IMUSensor::gyroX_dps()      { return icm.gyrX(); }
float IMUSensor::gyroY_dps()      { return icm.gyrY(); }
float IMUSensor::gyroZ_dps()      { return icm.gyrZ(); }

float IMUSensor::magX_uT()        { return icm.magX(); }
float IMUSensor::magY_uT()        { return icm.magY(); }
float IMUSensor::magZ_uT()        { return icm.magZ(); }

float IMUSensor::temperatureC()   { return icm.temp(); }
