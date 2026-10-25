#pragma once
#include <Arduino.h>
#include <RH_RF95.h>
#include "FlightData.h"
#include "ControlProtocol.h"
#include "LinkStats.h"

// The normal RadioHead driver requires a DIO0->Teensy interrupt edge to
// populate its receive buffer. Expose the same library handler so we can
// instead service RX_DONE from the radio's IRQ register in loop().
class PollingRF95 : public RH_RF95
{
public:
    PollingRF95(uint8_t cs, uint8_t irq) : RH_RF95(cs, irq) {}
    void servicePendingIrq() { handleInterrupt(); }
};

enum class ReceivedKind : uint8_t { Nothing, Flight, Control, Stats };

class RadioReceiver
{
public:
    RadioReceiver();
    bool begin();
    bool available();
    ReceivedKind receive(FlightData &data, ControlPacket &control,
                         LinkStatsPacket &stats, uint32_t &session, uint32_t &sequence);
    bool sendControl(const ControlPacket &control);
    int rssi();

    uint32_t irqRxDoneCount() const { return rxDoneIrqs; }
    uint32_t irqCrcErrorCount() const { return crcErrors; }
    uint32_t wrongLengthCount() const { return wrongLengths; }
    uint8_t lastIrq() const { return lastIrqFlags; }
    uint8_t opMode() { return radio.spiRead(RH_RF95_REG_01_OP_MODE); }

private:
    PollingRF95 radio;
    uint32_t rxDoneIrqs = 0;
    uint32_t crcErrors = 0;
    uint32_t wrongLengths = 0;
    uint8_t lastIrqFlags = 0;
};
