#include <Arduino.h>
#include <stdio.h>
#include "Config.h"
#include "TelemetryPacket.h"
#include "TelemetryCsv.h"
#include "LoRaRadio.h"
#include "SDLogger.h"

LoRaRadio radio;
SDLogger sdLogger;

// Counts gaps in increasing sequence numbers. The first received packet
// establishes a baseline; power cycles restart the counter at 0.
uint32_t receivedPackets = 0;
uint32_t missedPackets = 0;
uint32_t duplicateOrOld = 0;
uint32_t lastSeq = 0;
bool haveSeq = false;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { }

    if (!radio.begin()) {
        while (1) { delay(10); }
    }

    sdLogger.begin("rx_telemetry.csv",
                   "seq,ms,rtd_c,rtd_fault,adc_raw,adc_v,pressure_psi,pressure_kpa,rssi_dbm,missing_total,received_total");
    Serial.println("seq,ms,rtd_c,rtd_fault,adc_raw,adc_v,pressure_psi,pressure_kpa,rssi_dbm,missing_total,received_total");
}

void loop() {
    if (radio.available()) {
        TelemetryPacket packet = {};
        int16_t rssi = 0;
        if (radio.receive(packet, rssi)) {
            ++receivedPackets;
            if (!haveSeq) {
                haveSeq = true;
                lastSeq = packet.seqNum;
            } else if (packet.seqNum > lastSeq) {
                if (packet.seqNum > lastSeq + 1U) {
                    missedPackets += packet.seqNum - lastSeq - 1U;
                }
                lastSeq = packet.seqNum;
            } else {
                ++duplicateOrOld;
            }

            char csv[RH_RF95_MAX_MESSAGE_LEN];
            const int n = formatTelemetryCsv(packet, csv, sizeof(csv));
            if (n >= 0 && n < (int)sizeof(csv)) {
                char line[RH_RF95_MAX_MESSAGE_LEN + 80];
                const int m = snprintf(line, sizeof(line), "%s,%d,%lu,%lu",
                    csv, (int)rssi,
                    (unsigned long)missedPackets,
                    (unsigned long)receivedPackets);
                if (m >= 0 && m < (int)sizeof(line)) {
                    Serial.println(line);
                    sdLogger.logCsv(line);
                }
            }
        }
    }
    sdLogger.service();
}
