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

// ----- Receiver's own GPS -----
// Serial2 on Teensy 4.1: RX2 = pin 7, TX2 = pin 8
const int GPS_BAUD_RATE = 9600;   // PA1616D default

// ----- Serial Monitor -----
const int SERIAL_BAUD_RATE = 115200;

// ----- External Adafruit MicroSD breakout -----
// Uses SPI1, NOT the normal SPI bus used by the radio.
const bool SD_IS_BUILTIN = false;

// IMPORTANT: must match the physical CS wire from the SD breakout.
const int SD_CS_PIN = 0;

// Start slow for reliability. Once everything works, try 10 or 20 MHz.
const uint32_t SD_SPI_SPEED_MHZ = 4;

// How often buffered data is forced onto the card
const uint32_t LOG_FLUSH_MS = 1000;

// ============================================================
// RADIO SETTINGS  (must match the transmitter)
// ============================================================

const float RF_FREQUENCY = 433.0;

#define REG_HOP_PERIOD 0x24

const bool ENABLE_FHSS = false;

// ============================================================
// DISPLAY
// ============================================================

// The Serial Monitor shows the latest packet this often
// (every packet is still counted and logged to the SD card)
const uint32_t PRINT_EVERY_MS = 1000;
