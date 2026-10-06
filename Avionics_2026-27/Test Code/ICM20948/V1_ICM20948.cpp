#include "ICM.h"

#define SPI_PORT SPI     // Your desired SPI port.       Used only when "USE_SPI" is defined
#define SPI_FREQ 5000000 // You can override the default SPI frequency
#define CS_PIN 36        // Which pin you connect CS to. Used only when "USE_SPI" is define

// https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/examples/Arduino/Example1_Basics/Example1_Basics.ino
// https://github.com/UT2UH/Arduino_ICM20948_DMP_Full-Function/blob/main/ICM20948/Arduino-ICM20948.cpp
//https://d17t6iyxenbwp1.cloudfront.net/s3fs-public/2026-03/ds-000189-icm-20948-datasheet.pdf?VersionId=DDZ6UQd2jbFGUCqi1PY4w_nR5jWpqKVN
//Dependency: https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/main
ICM20948::ICM20948(uint8_t csPin, SPIClass &spiPort)
{
    // Constructor implementation
    _csPin = csPin;
    _spiPort = &spiPort;
}
typedef struct
{
    double x;
    double y;
    double z;
} Quat6;

Quat6 Euler;

void ICM20948::begin()
{
    _spiPort->begin();

    // Start communication with the sensor
    _sensor.begin(_csPin, *_spiPort, SPI_FREQ);

    bool DMPloaded = true;
    DMPloaded &= (_sensor.initializeDMP() == ICM_20948_Stat_Ok);
    DMPloaded &= (_sensor.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    DMPloaded &= (_sensor.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok);
    DMPloaded &= (_sensor.enableFIFO() == ICM_20948_Stat_Ok);
    DMPloaded &= (_sensor.resetFIFO() == ICM_20948_Stat_Ok);
    DMPloaded &= (_sensor.enableDMP() == ICM_20948_Stat_Ok);

    if (DMPloaded)
        Serial.println("DMP loaded successfully!");
    else
        Serial.println("DMP loading failed!");

    if (_sensor.status != ICM_20948_Stat_Ok)
    {
        Serial.println("Sensor initialization failed!");
        delay(500);
    }
}

void ICM20948::update()
{
    if (_sensor.dataReady())
    {
        _sensor.resetFIFO(); //reset to prevent overflow and reading corrupted bits 

        sensorData.magneticField.x = _sensor.magX();
        delay(10);
        _sensor.readDMPdataFromFIFO(&DMPdata);

        sensorData.orientation.x = ((double)DMPdata.Quat9.Data.Q1) / 1073741824.0f;
        sensorData.orientation.y = ((double)DMPdata.Quat9.Data.Q2) / 1073741824.0f;
        sensorData.orientation.z = ((double)DMPdata.Quat9.Data.Q3) / 1073741824.0f;
        sensorData.orientation.w = sqrt(1.0 - ((sensorData.orientation.x * sensorData.orientation.x) + (sensorData.orientation.y * sensorData.orientation.y) + (sensorData.orientation.z * sensorData.orientation.z)));

        _sensor.getAGMT();
        sensorData.acceleration.x = _sensor.accX();
        sensorData.acceleration.y = _sensor.accY();
        sensorData.acceleration.z = _sensor.accZ();

        sensorData.gyroscope.x = _sensor.gyrX();
        sensorData.gyroscope.y = _sensor.gyrY();
        sensorData.gyroscope.z = _sensor.gyrZ();

        sensorData.gravity.x = 2 * (sensorData.orientation.x * sensorData.orientation.z - sensorData.orientation.w * sensorData.orientation.y);
        sensorData.gravity.y = 2 * (sensorData.orientation.w * sensorData.orientation.x + sensorData.orientation.y * sensorData.orientation.z);
        sensorData.gravity.z = sensorData.orientation.w * sensorData.orientation.w - sensorData.orientation.x * sensorData.orientation.x - sensorData.orientation.y * sensorData.orientation.y + sensorData.orientation.z * sensorData.orientation.z;

        Euler.x = ((double)DMPdata.Quat6.Data.Q1) / 1073741824.0;
        Euler.y = ((double)DMPdata.Quat6.Data.Q2) / 1073741824.0;
        Euler.z = ((double)DMPdata.Quat6.Data.Q3) / 1073741824.0;

        sensorData.linearAcceleration.x = _sensor.accX() - sensorData.gravity.x;
        sensorData.linearAcceleration.y = _sensor.accY() - sensorData.gravity.y;
        sensorData.linearAcceleration.z = _sensor.accZ() - sensorData.gravity.z;

        // Serial.println("SENSOR UPDATED");
    }
    // digitalWrite(CS_PIN, HIGH);
}
void ICM20948::showSensorData()
{
    ICM20948::update();
    Serial.print("Acceleration: ");
    Serial.print(sensorData.acceleration.x);
    Serial.print(", ");
    Serial.print(sensorData.acceleration.y);
    Serial.print(", ");
    Serial.println(sensorData.acceleration.z);

    Serial.print("Gyroscope: ");
    Serial.print(sensorData.gyroscope.x);
    Serial.print(", ");
    Serial.print(sensorData.gyroscope.y);
    Serial.print(", ");
    Serial.println(sensorData.gyroscope.z);

    Serial.print("Gravity: ");
    Serial.print(sensorData.gravity.x);
    Serial.print(", ");
    Serial.print(sensorData.gravity.y);
    Serial.print(", ");
    Serial.println(sensorData.gravity.z);

    Serial.print("Quaternion: ");
    Serial.print(sensorData.orientation.w);
    Serial.print(", ");
    Serial.print(sensorData.orientation.x);
    Serial.print(", ");
    Serial.print(sensorData.orientation.y);
    Serial.print(", ");
    Serial.println(sensorData.orientation.z);
}

void ICM20948::statusReport()
{
    Serial.print("Sensor status: ");
    Serial.println(_sensor.statusString());
    if (_sensor.statusString() == "Data Underflow")ICM20948::begin();
}
