#pragma once
#include <Arduino.h>

// Original side-gig hardware pinout: UNCHANGED.
// MAX31865 / PT1000 on SPI0, CS=10.
static const uint8_t RTD_CS_PIN = 10;
static const float RTD_NOMINAL = 1000.0f;
static const float RREF = 4300.0f;
// Original four-wire RTD configuration.
#define RTD_WIRE_MODE MAX31865_4WIRE

// ADS1115 pressure interface: I2C, address 0x48, A0, gain +/-6.144V.
static const uint8_t ADS_CHANNEL = 0;

// RFM radio on Teensy 4.1 SPI1 (not SPI0).
static const uint8_t RFM_CS_PIN = 9;
static const uint8_t RFM_INT_PIN = 2;
static const uint8_t RFM_RST_PIN = 3;
static const uint8_t SPI1_MISO_PIN = 1;
static const uint8_t SPI1_MOSI_PIN = 26;
static const uint8_t SPI1_SCK_PIN = 27;

// Preserved exactly from original sketch. CHECK module's actual RF band and
// local radio authorization before radiating: 500 MHz is NOT 500 kHz BW.
static const float RF_FREQ_MHZ = 500.0f;
static const int8_t RF_TX_POWER_DBM = 20;
static const uint32_t SERIAL_BAUD_RATE = 115200;

// Added: Teensy 4.1's own microSD socket, separate from SPI0 and SPI1.
// Logging failure NEVER stops telemetry. Flush about once per second.
static const uint32_t SD_FLUSH_INTERVAL_MS = 1000UL;
