#include "RadioTransmitter.h"
#include "Config.h"
#include "GpsPacket.h"

RadioTransmitter::RadioTransmitter()
    : radio(RFM96W_CS_PIN, RFM96W_INT_PIN)
{
}

bool RadioTransmitter::begin()
{
    Serial.println();
    Serial.println("========== RADIO INITIALIZATION ==========");

    pinMode(RFM96W_RST_PIN, OUTPUT);

    // Reset radio
    digitalWrite(RFM96W_RST_PIN, LOW);
    delay(10);
    digitalWrite(RFM96W_RST_PIN, HIGH);
    delay(10);

    Serial.println("Radio reset complete.");

    if (!radio.init())
    {
        Serial.println("ERROR: radio.init() failed!");
        return false;
    }

    Serial.println("radio.init() SUCCESS.");

    if (!radio.setFrequency(RF_FREQUENCY))
    {
        Serial.println("ERROR: Could not set radio frequency!");
        return false;
    }

    Serial.print("Frequency set to ");
    Serial.print(RF_FREQUENCY);
    Serial.println(" MHz");

    radio.setTxPower(17, false);
    radio.setSignalBandwidth(250000);
    radio.setSpreadingFactor(7);

    Serial.println("TX power: 17 dBm");
    Serial.println("Bandwidth: 250 kHz");
    Serial.println("Spreading Factor: 7");

    if (ENABLE_FHSS)
    {
        uint8_t hopPeriodSymbols = 0x1A;
        radio.spiWrite(REG_HOP_PERIOD, hopPeriodSymbols);

        Serial.println("FHSS ENABLED.");
    }
    else
    {
        Serial.println("FHSS disabled.");
    }

    // Transmit-only mode: don't depend on the RadioHead interrupt.
    detachInterrupt(digitalPinToInterrupt(RFM96W_INT_PIN));
    pinMode(RFM96W_INT_PIN, INPUT);

    Serial.println("RADIO READY.");
    Serial.println("==========================================");

    return true;
}

bool RadioTransmitter::transmit(float lat, float lon, float alt, float spd, int sats, int fix)
{
    GpsPacket pkt;

    pkt.lat_e6   = roundToInt((double)lat * 1000000.0);
    pkt.lon_e6   = roundToInt((double)lon * 1000000.0);
    pkt.alt_cm   = roundToInt((double)alt * 100.0);
    pkt.speed_cs = roundToInt((double)spd * 100.0);
    pkt.sats     = (uint8_t)sats;
    pkt.fix      = (uint8_t)fix;

    // Clear old IRQ flags
    radio.spiWrite(REG_IRQ_FLAGS, 0xFF);

    bool sent = radio.send((uint8_t *)&pkt, sizeof(pkt));

    bool done = false;
    unsigned long start = millis();

    // Wait maximum 200 ms for the radio's TX done flag
    while (millis() - start < 200)
    {
        uint8_t flags = radio.spiRead(REG_IRQ_FLAGS);

        if (flags & IRQ_TX_DONE)
        {
            done = true;
            break;
        }
    }

    // Clear IRQ flags again
    radio.spiWrite(REG_IRQ_FLAGS, 0xFF);

    radio.setModeIdle();

    if (!sent)
    {
        Serial.println("ERROR: radio.send() returned false");
    }

    if (!done)
    {
        Serial.println("WARNING: Radio TX Done flag timeout");
    }

    return sent && done;
}
