#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_ADS1X15.h>

// ---------------- MAX31865 / PT1000 RTD ----------------
#define RTD_CS_PIN 10   // idk whatever pin change
Adafruit_MAX31865 rtd = Adafruit_MAX31865(RTD_CS_PIN);
// Hardware SPI on Teensy 4.1 default pins: MOSI 11, MISO 12, SCK 13

#define RTD_NOMINAL 1000.0  // PT1000 = 1000 ohm at 0C
#define RREF        4300.0  // resistor on PT1000

// MAX31865_2WIRE, MAX31865_3WIRE, or MAX31865_4WIRE default 4 wire
#define RTD_WIRE_MODE MAX31865_4WIRE

// ---------------- ADS1115 ----------------
Adafruit_ADS1115 ads; // default I2C address 0x48

const uint8_t ADS_CHANNEL = 0;   // which ADS1115 input your analog temp sensor is on

// function for converting the ADC voltage to temperature 
float convertADCToTemp(float voltage) {
  // placeholder 
  return voltage; // currently just passes voltage through, unmodified
}

void setup() {

  Serial.begin(115200);
  uint32_t t0 = millis();

  while (!Serial && millis() - t0 < 3000) { } // if no serial connection, wait 3 seconds before continuing

  // RTD init
  rtd.begin(RTD_WIRE_MODE);

  // ADS1115 init
  if (!ads.begin()) {
    Serial.println("ADS1115 not found — check wiring/I2C address (default 0x48)");
    while (1) { delay(10); }
  }

  ads.setGain(GAIN_ONE); // +/-4.096V 
}

void loop() {
  // ---- RTD temperature ----
  float rtdTempC = rtd.temperature(RTD_NOMINAL, RREF);

  uint8_t fault = rtd.readFault();
  if (fault) {
    Serial.print("RTD fault: 0x");
    Serial.println(fault, HEX);
    rtd.clearFault();
  }

  // ---- ADS1115 analog temperature channel ----
  int16_t adcRaw = ads.readADC_SingleEnded(ADS_CHANNEL);
  float adcVoltage = ads.computeVolts(adcRaw);
  float adcTempC = convertADCToTemp(adcVoltage);

  // ---- Output ----
  Serial.print("RTD Temp (C): ");
  Serial.print(rtdTempC, 2);
  Serial.print("   ADC raw: ");
  Serial.print(adcRaw);
  Serial.print("   ADC Voltage: ");
  Serial.print(adcVoltage, 4);
  Serial.print("   ADC Temp: ");
  Serial.println(adcTempC, 2);

  delay(500); // adjust to your desired sample rate
}
