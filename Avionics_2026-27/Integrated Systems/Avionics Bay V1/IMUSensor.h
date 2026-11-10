#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ICM_20948.h>
#include "Config.h"

// Reads the ICM-20948 9-axis IMU (accelerometer, gyroscope, magnetometer).
// The unit is in each function name.
class IMUSensor
{
public:
    // Starts I2C and the IMU. Returns false if the IMU does not answer.
    bool begin();

    // Call every loop(). Returns true when a fresh reading was taken.
    bool update();

    bool isReady();

    // True once at least one real sample has been read
    bool hasData();

    float accelX_mg();
    float accelY_mg();
    float accelZ_mg();

    float gyroX_dps();
    float gyroY_dps();
    float gyroZ_dps();

    float magX_uT();
    float magY_uT();
    float magZ_uT();

    float temperatureC();

private:
#if IMU_USE_SPI
    ICM_20948_SPI icm;
#else
    ICM_20948_I2C icm;
#endif
    bool ready    = false;
    bool dataSeen = false;
};
