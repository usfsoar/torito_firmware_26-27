#include "RadioReceiver.h"
#include "Config.h"
#include "FlightPacket.h"

RadioReceiver::RadioReceiver() : radio(RFM96W_CS_PIN, RFM96W_INT_PIN) {}

bool RadioReceiver::begin()
{
    pinMode(RFM96W_CS_PIN, OUTPUT);
    digitalWrite(RFM96W_CS_PIN, HIGH);
    SPI.begin();

    pinMode(RFM96W_RST_PIN, OUTPUT);
    digitalWrite(RFM96W_RST_PIN, LOW);
    delay(10);
    digitalWrite(RFM96W_RST_PIN, HIGH);
    delay(10);

    if (!radio.init()) {
        Serial.println("Radio RX: init FAILED: check SPI, 3.3V, CS and reset");
        return false;
    }

    // Poll the radio's RX_DONE flag, not the Teensy DIO0 interrupt pin.
    // This isolates a common cause of 'No packets yet': a missing DIO0 edge.
    detachInterrupt(digitalPinToInterrupt(RFM96W_INT_PIN));
    pinMode(RFM96W_INT_PIN, INPUT);

    if (!radio.setFrequency(RF_FREQUENCY)) return false;
    radio.setSignalBandwidth(RADIO_BANDWIDTH_HZ);
    radio.setSpreadingFactor(7);
    radio.setCodingRate4(5);
    radio.setPayloadCRC(true);
    radio.setPreambleLength(8);
    radio.setTxPower(17, false);

    if (ENABLE_FHSS)
        radio.spiWrite(REG_HOP_PERIOD, 0x1A);
    else
        radio.spiWrite(REG_HOP_PERIOD, 0x00);

    radio.spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xFF);
    radio.setModeRx();

    Serial.print("RX radio version (usually 0x12): 0x");
    Serial.println(radio.spiRead(RH_RF95_REG_42_VERSION), HEX);
    Serial.print("RX radio config [1D/1E/26] = 0x");
    Serial.print(radio.spiRead(RH_RF95_REG_1D_MODEM_CONFIG1), HEX);
    Serial.print(" / 0x");
    Serial.print(radio.spiRead(RH_RF95_REG_1E_MODEM_CONFIG2), HEX);
    Serial.print(" / 0x");
    Serial.println(radio.spiRead(RH_RF95_REG_26_MODEM_CONFIG3), HEX);
    Serial.print("RX: listening at ");
    Serial.print(RF_FREQUENCY, 3);
    Serial.print(" MHz; BW="); Serial.print(RADIO_BANDWIDTH_HZ / 1000U);
    Serial.println("kHz SF=7 CR=4/5 CRC=ON; IRQ POLLING; 32-byte BINARY frames");
    Serial.println("RX: DIO0 pin is not needed by this polling version.");
    Serial.print("RX expected frame bytes: Flight="); Serial.print(sizeof(FlightPacket));
    Serial.print(" Control="); Serial.print(sizeof(ControlPacket));
    Serial.print(" Stats="); Serial.println(sizeof(LinkStatsPacket));
    return true;
}

bool RadioReceiver::available()
{
    // Calling available() ensures RadioHead is actually in continuous RX.
    if (radio.available()) return true;

    const uint8_t flags = radio.spiRead(RH_RF95_REG_12_IRQ_FLAGS);
    if (flags & (RH_RF95_RX_DONE | RH_RF95_PAYLOAD_CRC_ERROR | RH_RF95_RX_TIMEOUT)) {
        lastIrqFlags = flags;
        if (flags & RH_RF95_RX_DONE) ++rxDoneIrqs;
        if (flags & RH_RF95_PAYLOAD_CRC_ERROR) ++crcErrors;
        // RadioHead's own handler checks CRC, reads its FIFO including the
        // four RadioHead header bytes, validates it, and clears IRQ flags.
        radio.servicePendingIrq();
    }
    return radio.available();
}

int RadioReceiver::rssi() { return radio.lastRssi(); }

ReceivedKind RadioReceiver::receive(FlightData &data, ControlPacket &control,
                                    LinkStatsPacket &stats, uint32_t &session, uint32_t &sequence)
{
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len=sizeof(buf);
    if (!radio.recv(buf,&len)) { radio.setModeRx(); return ReceivedKind::Nothing; }
    radio.setModeRx();
    if (len==sizeof(FlightPacket)) {
        FlightPacket packet;
        memcpy(&packet,buf,sizeof(packet));
        unpackFlightPacket(packet,data);
        session=packet.session_id; sequence=packet.tx_sequence;
        if (!sequence) { Serial.println("REJECTED Flight frame: sequence=0"); return ReceivedKind::Nothing; }
        return ReceivedKind::Flight;
    }
    if (len==sizeof(ControlPacket)) {
        memcpy(&control,buf,sizeof(control));
        if (soarControlValid(control) && control.type==SOAR_CONTROL_ACK)
            return ReceivedKind::Control;
    }
    if (len==sizeof(LinkStatsPacket)) {
        memcpy(&stats,buf,sizeof(stats));
        if (soarStatsValid(stats)) return ReceivedKind::Stats;
    }
    ++wrongLengths;
    Serial.print("Ignored invalid/unknown frame length="); Serial.print(len);
    Serial.print(" expected Flight/Control/Stats="); Serial.print(sizeof(FlightPacket));
    Serial.print("/"); Serial.print(sizeof(ControlPacket));
    Serial.print("/"); Serial.println(sizeof(LinkStatsPacket));
    return ReceivedKind::Nothing;
}

bool RadioReceiver::sendControl(const ControlPacket &control) {
    // Temporarily transmit on the same RFM96W, then return to RX immediately.
    radio.spiWrite(RH_RF95_REG_12_IRQ_FLAGS,0xFF);
    const bool started=radio.send(reinterpret_cast<const uint8_t*>(&control),sizeof(control));
    bool finished=false;
    const uint32_t start=millis();
    while (started && (uint32_t)(millis()-start)<250UL) {
        if (radio.spiRead(RH_RF95_REG_12_IRQ_FLAGS)&RH_RF95_TX_DONE) {
            finished=true;
            break;
        }
    }
    radio.spiWrite(RH_RF95_REG_12_IRQ_FLAGS,0xFF);
    radio.setModeRx();
    return started && finished;
}
