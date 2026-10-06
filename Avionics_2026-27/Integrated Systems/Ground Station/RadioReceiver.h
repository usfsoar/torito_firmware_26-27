#pragma once
#include <Arduino.h>
#include <RH_RF95.h>

// Receives GpsPacket data from the transmitter over the RFM96W LoRa radio.
class RadioReceiver
{
public:
    RadioReceiver();

    bool begin();

    bool available();   // true when a packet is waiting
    int  rssi();        // signal strength of the last packet

    // Reads one packet and converts it to normal units.
    // Returns false if nothing valid was received.
    bool receive(float &lat, float &lon, float &alt, float &spd, int &sats, int &fix);

private:
    RH_RF95 radio;
};
