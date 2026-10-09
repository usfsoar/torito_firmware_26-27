// ============================================================
// SOAR USF - Full Teensy 4.1 Transmitter
// GPS + RFM96W + external SD + MS5607 altimeter + ICM-20948 IMU
//
// Sends a 24-byte FlightPacket (GPS + altitude + angles + vertical
// acceleration) 25 times a second, and logs everything to the SD card.
//
// Files in this sketch folder:
//   Config.h                 pins, settings, flags
//   FlightData.h             values in normal units
//   FlightPacket.h           the 24-byte radio packet (same file on the receiver)
//   GPSReader.h/.cpp         GPS module
//   RadioTransmitter.h/.cpp  RFM96W radio
//   SDLogger.h/.cpp          SD card logging (gpslog.csv, sensors.csv)
//   IMUSensor.h/.cpp         ICM-20948 IMU
//   Altimeter.h/.cpp         MS5607 altimeter, zeroed at startup, max altitude
//   Orientation.h/.cpp       phi, theta, vertical acceleration
// ============================================================

// List libraries here so Arduino builds the .cpp modules correctly.
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <SdFat.h>
#include <ICM_20948.h>
#include <MS5607.h>
#include <Watchdog_t4.h>

#include "Config.h"
#include "FlightData.h"
#include "FlightPacket.h"
#include "GPSReader.h"
#include "RadioTransmitter.h"
#include "SDLogger.h"
#include "IMUSensor.h"
#include "Altimeter.h"
#include "Orientation.h"
#include "SDriver.h"

// ============================================================
// GLOBAL OBJECTS
// ============================================================
WDT_T4<WDT3> wdt;
SDLogger         sdLogger;
OrientationState orientState;
GPSReader        gpsReader;
RadioTransmitter transmitter;
IMUSensor        imu;
Altimeter        altimeter;
SDriver          rec;

SensorSample sample;         // latest sensor values (updated 10 times a second)

uint32_t lastTransmitTime = 0;
uint32_t lastSensorRead   = 0;
uint32_t lastDrogueRec = 0;
uint32_t lastMainRec = 0;
uint32_t lastPrint        = 0;
bool DrogueRecoveryFired = false;
bool MainRecoveryFired = false;
bool DisarmDrogueRec = false;
bool DisarmMainRec = false;

uint32_t packetsOk     = 0;  // counted since the last Serial printout
uint32_t packetsFailed = 0;

// ============================================================
// HELPERS
// ============================================================

void deselectAllMainSpiDevices()
{
    for (int i = 0; i < SPI_CS_COUNT; i++)
    {
        pinMode(SPI_CS_PINS[i], OUTPUT);
        digitalWrite(SPI_CS_PINS[i], HIGH);
    }
}

// Reads the altimeter and IMU, works out the angles, and logs one sensor row.
void updateSensors()
{
    // ----- Altimeter -----
    sample.baroValid = altimeter.update();

    if (sample.baroValid)
    {
        sample.baroAltM    = altimeter.altitudeM();
        sample.maxAltM     = altimeter.maxAltitudeM();
        sample.pressureHPa = altimeter.pressureHPa();
        sample.tempC       = altimeter.temperatureC();
    }

    // ----- IMU -----
    sample.imuValid = imu.isReady() && imu.hasData();

    if (sample.imuValid)
    {
        sample.axMg  = imu.accelX_mg();
        sample.ayMg  = imu.accelY_mg();
        sample.azMg  = imu.accelZ_mg();
        sample.gxDps = imu.gyroX_dps();
        sample.gyDps = imu.gyroY_dps();
        sample.gzDps = imu.gyroZ_dps();

        const Orientation o = computeOrientation(sample.axMg, sample.ayMg, sample.azMg, sample.gxDps, sample.gyDps, sample.gzDps, 0.04f, orientState, 0.98f);

        sample.phiDeg      = o.phiDeg;
        sample.thetaDeg    = o.thetaDeg;
        sample.vertAccMps2 = o.verticalLinearAccelMps2;
    }

    sdLogger.logSensors(sample);
}

// Collects everything that goes into one radio packet.
FlightData buildFlightData()
{
    FlightData d;

    d.sats = gpsReader.satellites();

    if (gpsReader.hasFix())
    {
        d.fix     = gpsReader.fixQuality();
        d.lat     = gpsReader.latitude();
        d.lon     = gpsReader.longitude();
        d.gpsAltM = gpsReader.altitude();
        d.speedKn = gpsReader.speed();
    }
    // else: fix stays 0 and the GPS fields stay 0, so no stale position is sent

    d.baroValid = sample.baroValid;
    d.baroAltM  = sample.baroAltM;

    d.imuValid    = sample.imuValid;
    d.phiDeg      = sample.phiDeg;
    d.thetaDeg    = sample.thetaDeg;
    d.vertAccMps2 = sample.vertAccMps2;

    return d;
}

void printStatus()
{
    Serial.println();
    Serial.println("================ STATUS ================");

    Serial.print("Radio:    ");
    Serial.print(packetsOk);
    Serial.print(" sent OK, ");
    Serial.print(packetsFailed);
    Serial.println(" failed (last second)");

    if (gpsReader.hasFix())
    {
        Serial.print("GPS:      fix ");
        Serial.print(gpsReader.fixQuality());
        Serial.print(", ");
        Serial.print(gpsReader.satellites());
        Serial.println(" satellites");

        Serial.print("  Lat/Lon: ");
        Serial.print(gpsReader.latitude(), 6);
        Serial.print(", ");
        Serial.println(gpsReader.longitude(), 6);

        Serial.print("  GPS altitude: ");
        Serial.print(gpsReader.altitude());
        Serial.print(" m   speed: ");
        Serial.print(gpsReader.speed());
        Serial.println(" knots");
    }
    else
    {
        Serial.print("GPS:      no fix (satellites: ");
        Serial.print(gpsReader.satellites());
        Serial.println(")");
    }

    if (sample.baroValid)
    {
        Serial.print("Altitude: ");
        Serial.print(sample.baroAltM, 2);
        Serial.print(" m   max: ");
        Serial.print(sample.maxAltM, 2);
        Serial.print(" m   (pressure ");
        Serial.print(sample.pressureHPa, 2);
        Serial.println(" hPa, relative to power-on)");
    }
    else
    {
        Serial.println("Altitude: UNAVAILABLE (MS5607 not answering or not zeroed)");
    }

    if (sample.imuValid)
    {
        Serial.print("Accel:    X ");
        Serial.print(sample.axMg, 1);
        Serial.print("  Y ");
        Serial.print(sample.ayMg, 1);
        Serial.print("  Z ");
        Serial.print(sample.azMg, 1);
        Serial.println(" mg");

        Serial.print("Angles:   phi (roll) ");
        Serial.print(sample.phiDeg, 2);
        Serial.print(" deg   theta (pitch) ");
        Serial.print(sample.thetaDeg, 2);
        Serial.println(" deg");

        Serial.print("Vertical: ");
        Serial.print(sample.vertAccMps2, 3);
        Serial.println(" m/s^2 linear, 1 g removed (accelerometer-only, bench value)");
    }
    else
    {
        Serial.println("IMU:      NOT READY");
    }

    Serial.println("========================================");

    packetsOk     = 0;
    packetsFailed = 0;
}

float computeAverageDeltaAltitude()
{
    static const int N = 25;  // 25 samples = 1 second at 25 Hz
    static float altitudes[N] = {0};
    static int   idx = 0;

    altitudes[idx] = sample.baroAltM;
    idx = (idx + 1) % N;

    float sumDelta = 0;
    for (int i = 1; i < N; i++)
    {
        sumDelta += altitudes[i] - altitudes[i - 1];
    }

    return sumDelta / (N - 1); // average delta altitude per second I.E. velocity
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("====================================================");
    Serial.println("SOAR USF - FULL TEENSY TRANSMITTER");
    Serial.println("GPS + LoRa + SD + MS5607 + ICM-20948");
    Serial.println("====================================================");

    
    // CRITICAL: all devices sharing the main SPI bus are deselected first.
    deselectAllMainSpiDevices();
    SPI.begin();
    delay(100);

    // ----- Sensors -----
    Serial.println();
    Serial.println("========== SENSOR INITIALIZATION ==========");

    bool imuOk = imu.begin();
    bool altOk = altimeter.begin();   // zeroes the altitude: keep the system still and at the start position

    Serial.print("IMU startup:       ");
    Serial.println(imuOk ? "OK" : "FAIL");
    Serial.print("Altimeter startup: ");
    Serial.println(altOk ? "OK (altitude = 0 m)" : "FAIL");
    Serial.println("===========================================");

    // Return every shared CS HIGH before moving to the next subsystem.
    deselectAllMainSpiDevices();

    // ----- GPS -----
    gpsReader.begin();

    // ----- Radio -----
    // It shares SPI with the IMU and altimeter but has its own CS (40).
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
    // External SD uses SPI1, not the shared main SPI bus.
    if (sdLogger.begin())
    {
        Serial.println();
        Serial.println("SD card ready - logging to gpslog.csv and sensors.csv");
    }
    else
    {
        Serial.println();
        Serial.println("WARNING: SD failed.");
        Serial.println("System will continue WITHOUT SD logging.");
    }
    
    rec.begin();
    WDT_timings_t config;
    config.timeout = 5000; // 5 s
    wdt.begin(config);

    Serial.println();
    Serial.println("SYSTEM READY.");
    Serial.println("----------------------------------------------");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    wdt.feed();
    // GPS parser needs frequent servicing.
    gpsReader.update();

    // Buffered SD writes.
    sdLogger.service();

    // Refresh IMU whenever a fresh sample is available.
    imu.update();

    // Sensors at 25 Hz
    if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL_MS)
    {
        lastSensorRead = millis();
        updateSensors();
        if (!DrogueRecoveryFired && computeAverageDeltaAltitude() < -5 && sample.maxAltM - sample.baroAltM >= 100 && sample.vertAccMps2 < 0) { // use kalman filter altitude instead of sample.baroaAltM  
            Serial.println("Recovery Activated"); // add solenoid driver logic here
            rec.DriverActivate(0); // Activate solenoid driver 0
            rec.DriverActivate(1); // Activate solenoid driver 1
            lastDrogueRec = millis();
            DrogueRecoveryFired = true;
        }
        
        if (millis() - lastDrogueRec >= 10000 && !MainRecoveryFired && !DisarmDrogueRec) { // 10 seconds after drogue recovery, disarm the drogue solenoids
            Serial.println("Disarming Drogue Recovery Solenoids");
            lastDrogueRec = millis();
            rec.DriverDeactivate(0); // Deactivate solenoid driver 0
            rec.DriverDeactivate(1); // Deactivate solenoid driver 1
            DisarmDrogueRec = true;
        }

        if (!MainRecoveryFired && computeAverageDeltaAltitude() < -5 && sample.baroAltM <=1000 && sample.vertAccMps2 < 0) { // use kalman filter altitude instead of sample.baroaAltM  
            Serial.println("Main Recovery Activated"); // add solenoid driver logic here
            rec.DriverActivate(2); // Activate solenoid driver 2
            rec.DriverActivate(3); // Activate solenoid driver 3
            lastMainRec = millis();
            MainRecoveryFired = true;
        }
        if (millis() - lastMainRec >= 10000 && !DisarmMainRec) { // 10 seconds after main recovery, disarm the main solenoids
            Serial.println("Disarming Main Recovery Solenoids");
            lastMainRec = millis();
            rec.DriverDeactivate(2); // Deactivate solenoid driver 2
            rec.DriverDeactivate(3); // Deactivate solenoid driver 3
            DisarmMainRec = true;
        }
    }

    // Radio packets
    if (millis() - lastTransmitTime >= TRANSMIT_INTERVAL)
    {
        lastTransmitTime = millis();

        if (TRANSMIT_WITHOUT_GPS_FIX || gpsReader.hasFix())
        {
            const FlightData data = buildFlightData();

            if (transmitter.transmit(data))
                packetsOk++;
            else
                packetsFailed++;

            // The GPS log only gets rows that have a real position
            if (data.fix > 0)
                sdLogger.logGps(data);
        }
    }

    // Serial Monitor status, once a second
    if (millis() - lastPrint >= PRINT_INTERVAL_MS)
    {
        lastPrint = millis();
        printStatus();
    }
}
