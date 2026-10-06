#pragma once
#include <Arduino.h>

// Radio packet. Must be IDENTICAL on transmitter and receiver.
struct __attribute__((packed)) GpsPacket
{
    int32_t lat_e6;     // degrees x 1,000,000
    int32_t lon_e6;     // degrees x 1,000,000
    int32_t alt_cm;     // meters x 100
    int32_t speed_cs;   // knots x 100

    uint8_t sats;
    uint8_t fix;
};

static_assert(sizeof(GpsPacket) == 18, "GpsPacket must be exactly 18 bytes");

// Rounds to the nearest integer (works for negative values too)
static inline int32_t roundToInt(double v)
{
    return (int32_t)(v >= 0 ? v + 0.5 : v - 0.5);
}
