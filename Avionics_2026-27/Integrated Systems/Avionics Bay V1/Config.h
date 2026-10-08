#pragma once
#include <Arduino.h>

// ============================================================
// HARDWARE SETTINGS  (Teensy 4.1)
// ============================================================

// ----- RFM96W LoRa radio -----
// Normal SPI bus: SCK = 13, MOSI = 11, MISO = 12
const int RFM96W_CS_PIN  = 40;
const int RFM96W_RST_PIN = 9;
const int RFM96W_INT_PIN = 2;

// ----- GPS -----
// Serial2 on Teensy 4.1: RX2 = pin 7, TX2 = pin 8
const unsigned int GPS_BAUD_RATE = 9600;

// ----- External Adafruit MicroSD breakout -----
// Uses SPI1, NOT the normal SPI bus.
const bool SD_IS_BUILTIN = false;
const int SD_CS_PIN = 0;
const uint32_t SD_SPI_SPEED_MHZ = 4;
const uint32_t LOG_FLUSH_MS = 1000;

// ============================================================
// ALTIMETER  (Parallax MS5607, SPI mode)
// ============================================================

// Shared normal SPI bus: SCK 13, MOSI 11, MISO 12
const int MS5607_CS_PIN = 10;

// Altitude is measured relative to power-on (0 m at startup), so the
// sea-level pressure does not matter. At startup this many valid pressure
// readings are averaged to set the 0 m reference. 1 = use only the very
// first reading.
const int ALTITUDE_ZERO_SAMPLES = 10;

// ============================================================
// IMU  (ICM-20948)
// ============================================================

// 1 = SPI, 0 = I2C
#define IMU_USE_SPI 1

// SPI: shared SCK 13, MOSI 11, MISO 12
const int      IMU_CS_PIN = 36;
const uint32_t IMU_SPI_HZ = 4000000;

// I2C settings (unused while IMU_USE_SPI == 1)
#define IMU_WIRE Wire
const uint8_t  IMU_AD0_VAL   = 1;
const uint32_t IMU_I2C_CLOCK = 400000;

// ============================================================
// SHARED NORMAL SPI BUS
// ============================================================

// Hold every CS HIGH before starting any SPI device.
// 40 = radio, 36 = IMU, 10 = MS5607
const int SPI_CS_PINS[] = { RFM96W_CS_PIN, IMU_CS_PIN, MS5607_CS_PIN };
const int SPI_CS_COUNT  = sizeof(SPI_CS_PINS) / sizeof(SPI_CS_PINS[0]);

// ============================================================
// RADIO SETTINGS
// ============================================================

const float RF_FREQUENCY = 433.0;
const unsigned int TRANSMIT_INTERVAL = 40; // 25 packets/s target

// true  = send packets even without a GPS fix (the sensor data is still useful;
//         the packet's fix field tells the ground station the GPS is not valid)
// false = send only when the GPS has a fix
const bool TRANSMIT_WITHOUT_GPS_FIX = true;

#define REG_HOP_PERIOD 0x24
#define REG_IRQ_FLAGS  0x12
#define IRQ_TX_DONE    0x08

const bool ENABLE_FHSS = false;

// ============================================================
// LOOP SETTINGS
// ============================================================

const uint32_t SENSOR_READ_INTERVAL_MS = 100;   // 10 Hz sensors
const uint32_t PRINT_INTERVAL_MS       = 1000;  // Serial Monitor status, once a second
