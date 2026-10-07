#pragma once
#include <Arduino.h>
#include <RH_RF95.h>
#include "FlightData.h"

// Sends FlightData over the RFM96W LoRa radio as a 24-byte FlightPacket.
class RadioTransmitter
{
public:
    RadioTransmitter();

    bool begin();

    // Packs the data into a FlightPacket and sends it.
    // Returns true only if the radio reported the transmission finished.
    bool transmit(const FlightData &data);

private:
    RH_RF95 radio;
};
