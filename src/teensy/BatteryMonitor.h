#pragma once
// ============================================================================
// Reads everything
//   bus (pack) voltage, shunt voltage, current, power, accumulated energy,
//   accumulated charge, die temperature, and the DIAG_ALRT diagnostic flags.
// Also tracks min/max/peak 
// outputs:
//   1) data BatteryData    engineering units, for local use
//   2) a packet  BatteryPacket   for radio transmission (44 bytes, CRC-16/CCITT-FALSE),
//      unless BATTERY_SERIAL_ONLY is defined
//   3) CSV line       for Serial / SD / ground-station display
//
// SIGN CONVENTION
//   INA228 reports positive current when it flows Vin+ -> Vin-.
//
// If you want for the transmitter to send battery telemetry, you need to add a few lines to the
//   transmitter and receiver code.  For example:
// It kinda autocompleted this
//   Transmitter, RadioTransmitter.h/.cpp:
//     bool sendBattery(const BatteryPacket &p) {
//         return sendRaw(reinterpret_cast<const uint8_t*>(&p), sizeof(p));
//     }
//   Transmitter, loop(), ~1 Hz and outside the command-listen window:
//     battery.update(now);
//     if (!telemetryFinished && !commandListenWindow && now - lastBattTx >= 1000) {
//         lastBattTx = now;
//         BatteryPacket bp = battery.makePacket();
//         transmitter.sendBattery(bp);
//     }
//   Receiver, RadioReceiver.cpp receive(), before the wrong-length fallthrough:
//     if (len == sizeof(BatteryPacket)) {
//         BatteryData b;
//         if (batteryDecode(buf, len, b)) { /* print batteryFormatCsv(b, ...) */ }
//     }
//
//   The receiver only needs the packet/CSV parts. To compile it without the
//   INA228 library, define BATTERY_PACKET_ONLY (PlatformIO:
//   build_flags = -DBATTERY_PACKET_ONLY) before including this header.
//   For serial/CSV output only, define BATTERY_SERIAL_ONLY (PlatformIO:
//   build_flags = -DBATTERY_SERIAL_ONLY). Packet support is enabled by default;
//   BATTERY_PACKET_ONLY still enables packet helpers.
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

#ifndef BATTERY_PACKET_ONLY
#include <Wire.h>
#include <Adafruit_INA228.h>   // PlatformIO: adafruit/Adafruit INA228 Library + adafruit/Adafruit BusIO
#endif

// ---------------------------------------------------------------------------
// Status flags (BatteryData::flags / BatteryPacket::flags)
// ---------------------------------------------------------------------------
// damn hitting tab really autocompletes all of this
static constexpr uint16_t BATT_FLAG_VALID            = 0x0001; // sample read OK
static constexpr uint16_t BATT_FLAG_LOW_VOLTAGE      = 0x0002; // bus V below cfg.warnLowV
static constexpr uint16_t BATT_FLAG_OVER_CURRENT     = 0x0004; // |I| above cfg.warnOverA
static constexpr uint16_t BATT_FLAG_OVER_TEMP        = 0x0008; // die temp above cfg.warnOverTempC
static constexpr uint16_t BATT_FLAG_MATH_OVERFLOW    = 0x0010; // INA228 MATHOF
static constexpr uint16_t BATT_FLAG_ACCUM_OVERFLOW   = 0x0020; // INA228 CHARGEOF or ENERGYOF
static constexpr uint16_t BATT_FLAG_MEMORY_ERROR     = 0x0040; // INA228 MEMSTAT = 0 (trim checksum error)
static constexpr uint16_t BATT_FLAG_I2C_ERROR        = 0x0080; // INA228 did not answer
static constexpr uint16_t BATT_FLAG_SHUNT_SATURATED  = 0x0100; // shunt ADC near full scale; current is clipped

// INA228 DIAG_ALRT register bits (raw copy goes out in BatteryData::diag)
// The INA228 datasheet calls these "alert flags" but they are really diagnostic flags. The alert pin can be configured to trigger on any combination of these bits.
// also autocompleted
static constexpr uint16_t INA228_DIAG_MEMSTAT  = 1u << 0;
static constexpr uint16_t INA228_DIAG_CNVRF    = 1u << 1;
static constexpr uint16_t INA228_DIAG_POL      = 1u << 2;
static constexpr uint16_t INA228_DIAG_BUSUL    = 1u << 3;
static constexpr uint16_t INA228_DIAG_BUSOL    = 1u << 4;
static constexpr uint16_t INA228_DIAG_SHNTUL   = 1u << 5;
static constexpr uint16_t INA228_DIAG_SHNTOL   = 1u << 6;
static constexpr uint16_t INA228_DIAG_TMPOL    = 1u << 7;
static constexpr uint16_t INA228_DIAG_MATHOF   = 1u << 9;
static constexpr uint16_t INA228_DIAG_CHARGEOF = 1u << 10;
static constexpr uint16_t INA228_DIAG_ENERGYOF = 1u << 11;

static constexpr uint8_t  BATTERY_SOC_UNKNOWN    = 255;
#if !defined(BATTERY_SERIAL_ONLY) || defined(BATTERY_PACKET_ONLY)
static constexpr uint32_t BATTERY_PACKET_MAGIC   = 0x42415454UL; // "BATT"
static constexpr uint8_t  BATTERY_PACKET_VERSION = 1;
#endif

// ---------------------------------------------------------------------------
// Engineering-unit snapshot
// ---------------------------------------------------------------------------
struct BatteryData {
    bool     valid       = false;
    uint32_t uptimeMs    = 0;     // millis() when sampled (decoded packets: TX uptime)
    uint16_t sequence    = 0;     // packet counter (set by makePacket / batteryDecode)

    float    busV        = 0;     // pack voltage, V
    float    shuntMv     = 0;     // voltage across the shunt, mV
    float    currentMa   = 0;     // mA, sign per cfg.invertCurrent
    float    powerMw     = 0;     // mW, always >= 0
    float    energyJ     = 0;     // accumulated since reset, J
    float    energyWh    = 0;     // same, Wh
    float    chargeC     = 0;     // accumulated since reset, C (signed)
    float    chargeMah   = 0;     // same, mAh (signed)
    float    dieTempC    = 0;     // INA228 die temperature, degC
    uint8_t  socPct      = BATTERY_SOC_UNKNOWN; // 0..100, 255 = unknown (needs cfg.capacityMah)
    uint16_t flags       = 0;     // BATT_FLAG_*
    uint16_t diag        = 0;     // raw DIAG_ALRT bits 0..11

    // Local running statistics (not sent over the air)
    float    minBusV       = 0;
    float    maxBusV       = 0;
    float    peakCurrentMa = 0;   // largest |I| seen
    float    peakPowerMw   = 0;
    uint32_t samples       = 0;
    uint32_t i2cErrors     = 0;
};

// ---------------------------------------------------------------------------
// Binary frame for the radio. 44 bytes. CRC-16/CCITT-FALSE (poly 0x1021,
// init 0xFFFF), identical to soarCrc16() in ControlProtocol.h.
// ---------------------------------------------------------------------------
#if !defined(BATTERY_SERIAL_ONLY) || defined(BATTERY_PACKET_ONLY)
struct __attribute__((packed)) BatteryPacket {
    uint32_t magic;        // BATTERY_PACKET_MAGIC
    uint8_t  version;      // BATTERY_PACKET_VERSION
    uint8_t  soc_pct;      // 0..100, 255 = unknown
    uint16_t flags;        // BATT_FLAG_*
    uint16_t sequence;     // wraps at 65535
    uint32_t uptime_ms;    // transmitter millis()
    int32_t  bus_mV;       // pack voltage, mV
    int32_t  shunt_uV;     // shunt voltage, uV
    int32_t  current_mA;   // signed
    int32_t  power_mW;
    uint32_t energy_mWh;   // accumulated since reset
    int32_t  charge_mAh;   // accumulated since reset, signed
    int16_t  temp_cC;      // die temperature, 0.01 degC
    uint16_t diag;         // raw DIAG_ALRT bits 0..11
    uint16_t crc;          // over every byte above
};
static_assert(sizeof(BatteryPacket) == 44, "BatteryPacket must be 44 bytes");

// Packet / CSV helpers (no INA228 library needed)
uint16_t    batteryCrc16(const uint8_t *bytes, size_t count);
bool        batteryPacketValid(const uint8_t *buf, size_t len);              // length + magic + version + CRC
bool        batteryDecode(const uint8_t *buf, size_t len, BatteryData &out); // false if invalid
#endif
const char *batteryCsvHeader();
size_t      batteryFormatCsv(const BatteryData &d, char *buf, size_t len);   // one line, no newline; returns chars written

#ifndef BATTERY_PACKET_ONLY
// ---------------------------------------------------------------------------
// Configuration. Defaults match the Adafruit INA228 breakout (15 mOhm shunt,
// 10 A) at I2C address 0x41 (A0 jumper closed).
// ---------------------------------------------------------------------------
struct BatteryMonitorConfig {
    uint8_t  i2cAddr       = 0x41;
    float    shuntOhms     = 0.015f;   // external shunt? put its value here
    float    maxCurrentA   = 10.0f;    // largest current you expect; sets resolution
    bool     invertCurrent = false;    // true if discharge reads negative

    INA2XX_AveragingCount averaging   = INA2XX_COUNT_16;      // ~50 ms per averaged result
    INA2XX_ConversionTime voltageTime = INA2XX_TIME_1052_us;
    INA2XX_ConversionTime currentTime = INA2XX_TIME_1052_us;
    INA2XX_ConversionTime tempTime    = INA2XX_TIME_1052_us;

    uint32_t sampleIntervalMs = 100;   // update() rate limit (10 Hz)

    float    capacityMah   = 0.0f;     // pack capacity; 0 = no state-of-charge estimate
    float    startSocPct   = 100.0f;   // assumed charge level at power-on / resetAccumulators()

    float    warnLowV      = 0.0f;     // 0 = disabled
    float    warnOverA     = 0.0f;     // 0 = disabled
    float    warnOverTempC = 0.0f;     // 0 = disabled
};

class BatteryMonitor {
public:
    // Returns false if the INA228 does not answer at cfg.i2cAddr.
    bool begin(const BatteryMonitorConfig &cfg = BatteryMonitorConfig(), TwoWire &wire = Wire);

    // Call every loop. Samples at most every cfg.sampleIntervalMs.
    // Returns true when a NEW valid sample was taken.
    bool update(uint32_t nowMs);
    bool sampleNow(uint32_t nowMs);             // ignore the rate limit

    bool               ready() const { return initialized_; }
    const BatteryData &data()  const { return data_; }

#if !defined(BATTERY_SERIAL_ONLY) || defined(BATTERY_PACKET_ONLY)
    BatteryPacket makePacket();                 // snapshot of data(), CRC filled, sequence++
#endif
    size_t        formatCsv(char *buf, size_t len) const { return batteryFormatCsv(data_, buf, len); }
    void          printStatus(Print &out) const;

    // Zero the INA228 energy/charge accumulators and the local min/max/peak stats.
    void resetAccumulators();
    void resetStats();

private:
    bool probe();

    Adafruit_INA228      ina_;
    TwoWire             *wire_        = nullptr;
    BatteryMonitorConfig cfg_;
    BatteryData          data_;
    bool                 initialized_ = false;
    bool                 haveSample_  = false;
    bool                 lowRange_    = false;   // true = +/-40.96 mV ADC range
    bool                 firstStats_  = true;
    uint32_t             lastSampleMs_ = 0;
    uint16_t             seq_         = 0;
};
#endif // BATTERY_PACKET_ONLY
