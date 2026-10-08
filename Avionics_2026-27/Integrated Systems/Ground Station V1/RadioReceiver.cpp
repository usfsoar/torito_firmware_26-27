#include "RadioReceiver.h"
#include "Config.h"
#include "FlightPacket.h"

RadioReceiver::RadioReceiver()
    : radio(RFM96W_CS_PIN, RFM96W_INT_PIN)
{
}

bool RadioReceiver::begin()
{
    pinMode(RFM96W_RST_PIN, OUTPUT);

    // Reset radio
    digitalWrite(RFM96W_RST_PIN, LOW);
    delay(10);
    digitalWrite(RFM96W_RST_PIN, HIGH);
    delay(10);

    Serial.println("Radio: reset done, calling init...");

    if (!radio.init())
        return false;

    Serial.println("Radio: init OK");

    if (!radio.setFrequency(RF_FREQUENCY))
        return false;

    Serial.println("Radio: frequency set");

    radio.setTxPower(17, false);
    radio.setSignalBandwidth(250000);   // 250 kHz (must match transmitter)
    radio.setSpreadingFactor(7);

    if (ENABLE_FHSS)
    {
        uint8_t hopPeriodSymbols = 0x1A;   // same as transmitter
        radio.spiWrite(REG_HOP_PERIOD, hopPeriodSymbols);
    }

    Serial.println("Radio: settings applied");

    return true;
}

bool RadioReceiver::available()
{
    return radio.available();
}

int RadioReceiver::rssi()
{
    return radio.lastRssi();
}

bool RadioReceiver::receive(FlightData &data)
{
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (!radio.recv(buf, &len))
        return false;

    // Not our packet (for example, an old 18-byte GPS-only packet), ignore it
    if (len != sizeof(FlightPacket))
        return false;

    FlightPacket pkt;
    memcpy(&pkt, buf, sizeof(pkt));

    unpackFlightPacket(pkt, data);

    return true;
}
