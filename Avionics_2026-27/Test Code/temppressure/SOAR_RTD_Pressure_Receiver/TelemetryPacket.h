#pragma once
#include <Arduino.h>
#include <RH_RF95.h>

// Exact binary layout of the supplied original transmitter sketch.
// Both ends must use this exact header. Teensy 4.1 floats: IEEE 754 32-bit.
struct __attribute__((packed)) TelemetryPacket {
    uint32_t seqNum;
    uint32_t timeMs;
    float rtdTempC;
    uint8_t fault;
    int16_t adcRaw;
    float adcVoltage;
    float pressurePsi;
    float pressureKPa;
};
static_assert(sizeof(float) == 4, "Telemetry requires 32-bit floats");
static_assert(sizeof(TelemetryPacket) == 27, "Original telemetry packet must be 27 bytes");
static_assert(sizeof(TelemetryPacket) <= RH_RF95_MAX_MESSAGE_LEN,
              "Telemetry packet exceeds RFM95 payload limit");
