#ifndef ICM20948_H
#define ICM20948_H

#include "ICM_20948.h"
#include<SPI.h>


class ICM20948 {
    private: 
        ICM_20948_SPI _sensor;
        //ArduinoICM20948 _sensorFusion;
        uint8_t _csPin;
        SPIClass* _spiPort;
    public:
        typedef struct {
            float x, y, z;
        } Vector3D_t;

        typedef struct {
            double w, x, y, z;
            float accuracy;
        } Orientation_t;

        typedef struct {
            Vector3D_t acceleration;
            Vector3D_t linearAcceleration;
            Vector3D_t gravity;
            Vector3D_t gyroscope;
            Vector3D_t magneticField;
            Orientation_t orientation;
        } AllSensorData_t;

        AllSensorData_t sensorData;
        icm_20948_DMP_data_t DMPdata;


        ICM20948(uint8_t csPin, SPIClass &spiPort = SPI);
        void begin();
        void update();
        void showSensorData();
        void statusReport();  
};


#endif
