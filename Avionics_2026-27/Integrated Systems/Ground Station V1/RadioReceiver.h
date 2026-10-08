#pragma once
#include <Arduino.h>
#include <RH_RF95.h>
#include "FlightData.h"

// Receives FlightPacket data from the transmitter over the RFM96W LoRa radio.
class RadioReceiver
{
public:
    RadioReceiver();

    bool begin();

    bool available();   // true when a packet is waiting
    int  rssi();        // signal strength of the last packet

    // Reads one packet and converts it to normal units.
    // Returns false if nothing valid was received (or it was not a 24-byte FlightPacket).
    bool receive(FlightData &data);

private:
    RH_RF95 radio;
};
