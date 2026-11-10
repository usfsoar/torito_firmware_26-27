#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHHardwareSPI1.h>
#include "Config.h"
#include "TelemetryPacket.h"

class LoRaRadio {
public:
    LoRaRadio();
    void prepareBus(); // Call before other SPI sensors when needed.
    bool begin();
    bool send(const TelemetryPacket &packet);
    bool available();
    bool receive(TelemetryPacket &packet, int16_t &rssi);
private:
    RH_RF95 rf95;
    bool busPrepared = false;
};
