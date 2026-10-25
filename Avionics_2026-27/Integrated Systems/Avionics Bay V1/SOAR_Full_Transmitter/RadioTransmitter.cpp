#include "RadioTransmitter.h"
#include "Config.h"
#include "FlightPacket.h"

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
    radio.setSignalBandwidth(RADIO_BANDWIDTH_HZ);
    radio.setSpreadingFactor(7);
    radio.setCodingRate4(5);
    radio.setPayloadCRC(true);
    radio.setPreambleLength(8);

    if (ENABLE_FHSS)
        radio.spiWrite(REG_HOP_PERIOD, 0x1A);
    else
        radio.spiWrite(REG_HOP_PERIOD, 0x00);

    Serial.print("TX radio version (usually 0x12): 0x");
    Serial.println(radio.spiRead(RH_RF95_REG_42_VERSION), HEX);
    Serial.print("TX radio config [1D/1E/26] = 0x");
    Serial.print(radio.spiRead(RH_RF95_REG_1D_MODEM_CONFIG1), HEX);
    Serial.print(" / 0x");
    Serial.print(radio.spiRead(RH_RF95_REG_1E_MODEM_CONFIG2), HEX);
    Serial.print(" / 0x");
    Serial.println(radio.spiRead(RH_RF95_REG_26_MODEM_CONFIG3), HEX);
    Serial.print("TX: ");
    Serial.print(RF_FREQUENCY, 3);
    Serial.print(" MHz; BW="); Serial.print(RADIO_BANDWIDTH_HZ / 1000U);
    Serial.println("kHz SF=7 CR=4/5 CRC=ON; 32-byte BINARY frames");

    // Transmit-only mode: don't depend on the RadioHead interrupt.
    detachInterrupt(digitalPinToInterrupt(RFM96W_INT_PIN));
    pinMode(RFM96W_INT_PIN, INPUT);

    radio.spiWrite(REG_IRQ_FLAGS, 0xFF);
    radio.setModeRx();  // Must hear ground commands even while bay is OFF.
    Serial.println("RADIO READY: waiting for commands in RX mode.");
    Serial.println("==========================================");

    return true;
}

bool RadioTransmitter::transmit(const FlightData &data, uint32_t session, uint32_t sequence) {
    FlightPacket packet = {};
    packFlightData(data, packet);
    packet.session_id=session;
    packet.tx_sequence=sequence;
    return sendRaw(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

bool RadioTransmitter::sendRaw(const uint8_t *data, uint8_t len) {
    radio.spiWrite(REG_IRQ_FLAGS, 0xFF);
    bool sent = radio.send(data, len);
    bool done = false;
    const uint32_t start=millis();
    while (sent && (uint32_t)(millis()-start) < 250UL) {
        if (radio.spiRead(REG_IRQ_FLAGS) & IRQ_TX_DONE) { done=true; break; }
    }
    radio.spiWrite(REG_IRQ_FLAGS, 0xFF);
    radio.setModeRx();            // Half-duplex: listen between telemetry packets.
    if (!sent || !done) Serial.println("WARNING: LoRa TX failed or TX_DONE timeout");
    return sent && done;
}

bool RadioTransmitter::receiveControl(ControlPacket &out) {
    if (!radio.available()) {
        const uint8_t flags = radio.spiRead(RH_RF95_REG_12_IRQ_FLAGS);
        if (flags & (RH_RF95_RX_DONE | RH_RF95_PAYLOAD_CRC_ERROR | RH_RF95_RX_TIMEOUT))
            radio.servicePendingIrq();
    }
    if (!radio.available()) return false;
    uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len=sizeof(data);
    if (!radio.recv(data,&len)) { radio.setModeRx(); return false; }
    radio.setModeRx();
    if (len != sizeof(ControlPacket)) return false;
    memcpy(&out,data,sizeof(out));
    return soarControlValid(out) && (out.type==SOAR_CONTROL_SET || out.type==SOAR_CONTROL_END);
}

bool RadioTransmitter::acknowledge(const ControlPacket &command, bool actualEnabled, bool saved) {
    const ControlPacket ack=soarControl(SOAR_CONTROL_ACK,command.sequence,actualEnabled,saved);
    return sendRaw(reinterpret_cast<const uint8_t*>(&ack), sizeof(ack));
}

bool RadioTransmitter::reportStats(const LinkStatsPacket &stats) {
    return sendRaw(reinterpret_cast<const uint8_t*>(&stats),sizeof(stats));
}
