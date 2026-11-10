#include <Arduino.h>
#include "Config.h"
#include "TelemetryPacket.h"
#include "TelemetryCsv.h"
#include "SensorReader.h"
#include "LoRaRadio.h"
#include "SDLogger.h"

SensorReader sensors;
LoRaRadio radio;
SDLogger sdLogger;
uint32_t seqNum = 0;

void setup() {
    // Original order: park chip-select pins before any SPI communication.
    radio.prepareBus();
    // Original 115200 baud Serial Monitor behavior.
    Serial.begin(SERIAL_BAUD_RATE);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { }

    // Preserve original order: RTD + ADS before radio initialization.
    if (!sensors.begin()) {
        while (1) { delay(10); } // Original ADS failure behavior.
    }
    if (!radio.begin()) {
        while (1) { delay(10); } // Original radio failure behavior.
    }

    // NEW: on-board SD card logging, optional and non-blocking on failure.
    sdLogger.begin("tx_telemetry.csv", TELEMETRY_CSV_HEADER);
    Serial.println(TELEMETRY_CSV_HEADER);
}

void loop() {
    TelemetryPacket packet = {};
    sensors.readInto(packet, seqNum);

    // Identical human-readable debug CSV columns to original sketch.
    char csv[RH_RF95_MAX_MESSAGE_LEN];
    const int n = formatTelemetryCsv(packet, csv, sizeof(csv));
    if (n >= 0 && n < (int)sizeof(csv)) {
        Serial.println(csv);
        sdLogger.logCsv(csv);
    } else {
        Serial.println("CSV debug output formatting failed");
    }

    // Original binary LoRa packet and blocking waitPacketSent behavior.
    if (!radio.send(packet)) {
        Serial.println("RFM send failed");
    }
    sdLogger.service();
    seqNum++;
}
