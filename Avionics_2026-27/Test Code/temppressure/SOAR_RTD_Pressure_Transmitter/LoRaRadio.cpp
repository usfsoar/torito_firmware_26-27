#include "LoRaRadio.h"
#include <string.h>

LoRaRadio::LoRaRadio()
    : rf95(RFM_CS_PIN, digitalPinToInterrupt(RFM_INT_PIN), hardware_spi1) {}

void LoRaRadio::prepareBus() {
    if (busPrepared) return;
    // Park both chip selects HIGH before touching either SPI bus.
    pinMode(RTD_CS_PIN, OUTPUT);
    digitalWrite(RTD_CS_PIN, HIGH);
    pinMode(RFM_CS_PIN, OUTPUT);
    digitalWrite(RFM_CS_PIN, HIGH);

    SPI1.setMISO(SPI1_MISO_PIN);
    SPI1.setMOSI(SPI1_MOSI_PIN);
    SPI1.setSCK(SPI1_SCK_PIN);
    SPI1.begin();
    busPrepared = true;
}

bool LoRaRadio::begin() {
    prepareBus();
    // Hardware reset sequence exactly matches the original sketch.
    pinMode(RFM_RST_PIN, OUTPUT);
    digitalWrite(RFM_RST_PIN, HIGH); delay(10);
    digitalWrite(RFM_RST_PIN, LOW);  delay(10);
    digitalWrite(RFM_RST_PIN, HIGH); delay(10);

    if (!rf95.init()) {
        Serial.println("RFM init failed - check SPI1/CS/IRQ/RST wiring and power");
        return false;
    }
    if (!rf95.setFrequency(RF_FREQ_MHZ)) {
        Serial.println("RFM setFrequency failed");
        return false;
    }
    rf95.setTxPower(RF_TX_POWER_DBM, false);
    return true;
}

bool LoRaRadio::send(const TelemetryPacket &packet) {
    const bool accepted = rf95.send(reinterpret_cast<const uint8_t *>(&packet),
                                    static_cast<uint8_t>(sizeof(packet)));
    if (accepted) rf95.waitPacketSent();
    return accepted;
}

bool LoRaRadio::available() {
    return rf95.available();
}

bool LoRaRadio::receive(TelemetryPacket &packet, int16_t &rssi) {
    uint8_t bytes[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t length = sizeof(bytes);
    if (!rf95.recv(bytes, &length)) return false;
    if (length != sizeof(TelemetryPacket)) {
        Serial.print("Ignored unexpected radio packet length: ");
        Serial.println(length);
        return false;
    }
    memcpy(&packet, bytes, sizeof(packet));
    rssi = rf95.lastRssi();
    return true;
}
