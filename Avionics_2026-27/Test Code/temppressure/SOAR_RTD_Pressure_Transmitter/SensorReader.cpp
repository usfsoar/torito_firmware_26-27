#include "SensorReader.h"

SensorReader::SensorReader() : rtd(RTD_CS_PIN) {}

bool SensorReader::begin() {
    // RTD initialization unchanged from original code: SPI0 / 4-wire PT1000.
    rtd.begin(RTD_WIRE_MODE);
    // ADS1115 initialization unchanged: default I2C address 0x48.
    if (!ads.begin()) {
        Serial.println("ADS1115 not found - check wiring/I2C address");
        return false;
    }
    // This sets conversion full-scale, NOT allowable absolute input voltage.
    ads.setGain(GAIN_TWOTHIRDS);
    return true;
}

float SensorReader::convertADCToPressure_psi(float voltage) {
    const float P_MAX = 10000.0f;
    const float V_MIN = 0.0f;  // original code: 0V-4.5V = 0-10000psi
    const float V_MAX = 4.5f;
    float psi = (voltage - V_MIN) * P_MAX / (V_MAX - V_MIN);
    if (psi < 0.0f) psi = 0.0f;
    if (psi > P_MAX) psi = P_MAX;
    return psi;
}

void SensorReader::readInto(TelemetryPacket &packet, uint32_t sequence) {
    // Preserve original sensor reads, fault behavior, units, and order.
    const float rtdTempC = rtd.temperature(RTD_NOMINAL, RREF);
    const uint8_t fault = rtd.readFault();
    if (fault) {
        Serial.print("RTD fault: 0x");
        Serial.println(fault, HEX);
        rtd.clearFault();
    }
    const int16_t adcRaw = ads.readADC_SingleEnded(ADS_CHANNEL);
    const float adcVoltage = ads.computeVolts(adcRaw);
    const float pressurePsi = convertADCToPressure_psi(adcVoltage);
    const float pressureKPa = pressurePsi * 6.894757f;
    packet.seqNum = sequence;
    packet.timeMs = millis();
    packet.rtdTempC = rtdTempC;
    packet.fault = fault;
    packet.adcRaw = adcRaw;
    packet.adcVoltage = adcVoltage;
    packet.pressurePsi = pressurePsi;
    packet.pressureKPa = pressureKPa;
}
