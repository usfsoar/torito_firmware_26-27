#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_ADS1X15.h>
#include <RH_RF95.h>
#include <RHHardwareSPI1.h>   // RadioHead SPI1 driver for Teensy 4.x

// ---------------- MAX31865 / PT1000 RTD ----------------
#define RTD_CS_PIN 10
Adafruit_MAX31865 rtd = Adafruit_MAX31865(RTD_CS_PIN);

#define RTD_NOMINAL 1000.0
#define RREF        4300.0
#define RTD_WIRE_MODE MAX31865_4WIRE   // change to MAX31865_3WIRE if needed

// ---------------- ADS1115 / PRESSURE ----------------
Adafruit_ADS1115 ads;              // default I2C address 0x48
const uint8_t ADS_CHANNEL = 0;     // pressure signal on A0

// ---------------- RFM95W on SPI1 ----------------
#define RFM_CS_PIN   9
#define RFM_INT_PIN  2
#define RFM_RST_PIN  3
#define RFM_INT_NUM  digitalPinToInterrupt(RFM_INT_PIN)

#define RF_FREQ_MHZ     915.0
#define RF_TX_POWER_DBM 13

// Standard Teensy 4.1 SPI1 pins:
// MISO1 = 1, MOSI1 = 26, SCK1 = 27
static const int SPI1_MISO_PIN = 1;
static const int SPI1_MOSI_PIN = 26;
static const int SPI1_SCK_PIN  = 27;

RH_RF95 rf95(RFM_CS_PIN, RFM_INT_NUM, hardware_spi1);

uint32_t seqNum = 0;

// Pressure sensor: linear, 0-4.5 V = 0-10000 psi
float convertADCToPressure_psi(float voltage) {
  const float P_MAX = 10000.0;
  const float V_MIN = 0.0;    // use 0.5 if your sensor is 0.5-4.5 V ratiometric
  const float V_MAX = 4.5;

  float psi = (voltage - V_MIN) * P_MAX / (V_MAX - V_MIN);

  if (psi < 0.0)   psi = 0.0;
  if (psi > P_MAX) psi = P_MAX;

  return psi;
}

void setup() {
  // Park both chip selects high before touching the SPI buses
  pinMode(RTD_CS_PIN, OUTPUT); digitalWrite(RTD_CS_PIN, HIGH);
  pinMode(RFM_CS_PIN, OUTPUT); digitalWrite(RFM_CS_PIN, HIGH);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }

  // Configure SPI1 pins before RFM init
  SPI1.setMISO(SPI1_MISO_PIN);
  SPI1.setMOSI(SPI1_MOSI_PIN);
  SPI1.setSCK(SPI1_SCK_PIN);
  SPI1.begin();

  // RTD init on SPI0
  rtd.begin(RTD_WIRE_MODE);

  // ADS1115 init on I2C
  if (!ads.begin()) {
    Serial.println("ADS1115 not found - check wiring/I2C address");
    while (1) { delay(10); }
  }

  // 0-4.5 V pressure sensor needs +/-6.144 V range
  ads.setGain(GAIN_TWOTHIRDS);

  // RFM hardware reset
  pinMode(RFM_RST_PIN, OUTPUT);
  digitalWrite(RFM_RST_PIN, HIGH); delay(10);
  digitalWrite(RFM_RST_PIN, LOW);  delay(10);
  digitalWrite(RFM_RST_PIN, HIGH); delay(10);

  if (!rf95.init()) {
    Serial.println("RFM init failed - check SPI1/CS/IRQ/RST wiring and power");
    while (1) { delay(10); }
  }

  if (!rf95.setFrequency(RF_FREQ_MHZ)) {
    Serial.println("RFM setFrequency failed");
    while (1) { delay(10); }
  }

  rf95.setTxPower(RF_TX_POWER_DBM, false);

  Serial.println("seq,ms,rtd_c,rtd_fault,adc_raw,adc_v,pressure_psi,pressure_kpa");
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

  // ---- ADS1115 pressure channel ----
  int16_t adcRaw = ads.readADC_SingleEnded(ADS_CHANNEL);
  float adcVoltage = ads.computeVolts(adcRaw);

  float pressurePsi = convertADCToPressure_psi(adcVoltage);
  float pressureKPa = pressurePsi * 6.894757;

  // ---- Build CSV packet ----
  char pkt[RH_RF95_MAX_MESSAGE_LEN];
  int n = snprintf(pkt, sizeof(pkt),
                   "%lu,%lu,%.2f,%u,%d,%.4f,%.1f,%.1f",
                   (unsigned long)seqNum,
                   (unsigned long)millis(),
                   rtdTempC,
                   (unsigned)fault,
                   (int)adcRaw,
                   adcVoltage,
                   pressurePsi,
                   pressureKPa);

  if (n > 0 && n < (int)sizeof(pkt)) {
    Serial.println(pkt);

    if (rf95.send((uint8_t*)pkt, (uint8_t)n)) {
      rf95.waitPacketSent();
    } else {
      Serial.println("RFM send failed");
    }
  }

  seqNum++;
  delay(500);
}