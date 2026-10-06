#pragma once
#include <Arduino.h>
#include <RH_RF95.h>

// Sends GpsPacket data over the RFM96W LoRa radio.
class RadioTransmitter
{
public:
    RadioTransmitter();

    bool begin();

    // Builds the 18-byte packet and sends it.
    // Returns true only if the radio reported the transmission finished.
    bool transmit(float lat, float lon, float alt, float spd, int sats, int fix);

private:
    RH_RF95 radio;
};
