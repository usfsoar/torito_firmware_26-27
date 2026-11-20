#pragma once
#include <Arduino.h>
#include <RH_RF95.h>
#include "FlightData.h"
#include "ControlProtocol.h"
#include "LinkStats.h"

class PollingTxRF95 : public RH_RF95 {
public:
    PollingTxRF95(uint8_t cs, uint8_t irq) : RH_RF95(cs, irq) {}
    void servicePendingIrq() { handleInterrupt(); }
};
class RadioTransmitter {
public:
    RadioTransmitter();
    bool begin();
    bool transmit(const FlightData &data, uint32_t session, uint32_t sequence);
    bool reportStats(const LinkStatsPacket &stats);
    bool receiveControl(ControlPacket &control);
    bool acknowledge(const ControlPacket &command, bool actualEnabled, bool saved);
private:
    bool sendRaw(const uint8_t *data, uint8_t len);
    PollingTxRF95 radio;
};
