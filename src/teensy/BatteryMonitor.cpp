#include "BatteryMonitor.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// ============================================================================
// Packet helpers (compiles even with BATTERY_PACKET_ONLY)
// ============================================================================

#if !defined(BATTERY_SERIAL_ONLY) || defined(BATTERY_PACKET_ONLY)
uint16_t batteryCrc16(const uint8_t *bytes, size_t count) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < count; ++i) {
        crc ^= static_cast<uint16_t>(bytes[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

static int32_t satI32(float v) {
    if (!(v == v)) return 0;                       // NaN
    if (v >= 2147483520.0f)  return INT32_MAX;
    if (v <= -2147483648.0f) return INT32_MIN;
    return static_cast<int32_t>(lroundf(v));
}
static uint32_t satU32(float v) {
    if (!(v == v) || v <= 0.0f) return 0;
    if (v >= 4294967040.0f) return UINT32_MAX;
    return static_cast<uint32_t>(lroundf(v));
}
static int16_t satI16(float v) {
    if (!(v == v)) return 0;
    if (v >= 32767.0f)  return INT16_MAX;
    if (v <= -32768.0f) return INT16_MIN;
    return static_cast<int16_t>(lroundf(v));
}

bool batteryPacketValid(const uint8_t *buf, size_t len) {
    if (!buf || len != sizeof(BatteryPacket)) return false;
    BatteryPacket p;
    memcpy(&p, buf, sizeof(p));
    if (p.magic != BATTERY_PACKET_MAGIC || p.version != BATTERY_PACKET_VERSION) return false;
    return p.crc == batteryCrc16(buf, offsetof(BatteryPacket, crc));
}

bool batteryDecode(const uint8_t *buf, size_t len, BatteryData &out) {
    if (!batteryPacketValid(buf, len)) return false;
    BatteryPacket p;
    memcpy(&p, buf, sizeof(p));
    out = BatteryData();
    out.valid     = (p.flags & BATT_FLAG_VALID) != 0;
    out.uptimeMs  = p.uptime_ms;
    out.sequence  = p.sequence;
    out.busV      = p.bus_mV / 1000.0f;
    out.shuntMv   = p.shunt_uV / 1000.0f;
    out.currentMa = static_cast<float>(p.current_mA);
    out.powerMw   = static_cast<float>(p.power_mW);
    out.energyWh  = p.energy_mWh / 1000.0f;
    out.energyJ   = out.energyWh * 3600.0f;
    out.chargeMah = static_cast<float>(p.charge_mAh);
    out.chargeC   = out.chargeMah * 3.6f;
    out.dieTempC  = p.temp_cC / 100.0f;
    out.socPct    = p.soc_pct;
    out.flags     = p.flags;
    out.diag      = p.diag;
    return true;
}
#endif

const char *batteryCsvHeader() {
    return "ms,seq,valid,bus_V,shunt_mV,current_mA,power_mW,energy_Wh,charge_mAh,die_C,soc_pct,flags,diag";
}

size_t batteryFormatCsv(const BatteryData &d, char *buf, size_t len) {
    if (!buf || len == 0) return 0;
    const int n = snprintf(buf, len, "%lu,%u,%u,%.3f,%.4f,%.1f,%.1f,%.4f,%.2f,%.2f,%d,0x%04X,0x%03X",
                           static_cast<unsigned long>(d.uptimeMs),
                           static_cast<unsigned>(d.sequence),
                           (d.flags & BATT_FLAG_VALID) ? 1u : 0u,
                           d.busV, d.shuntMv, d.currentMa, d.powerMw,
                           d.energyWh, d.chargeMah, d.dieTempC,
                           (d.socPct == BATTERY_SOC_UNKNOWN) ? -1 : static_cast<int>(d.socPct),
                           static_cast<unsigned>(d.flags), static_cast<unsigned>(d.diag));
    if (n < 0) { buf[0] = 0; return 0; }
    return (static_cast<size_t>(n) < len) ? static_cast<size_t>(n) : len - 1;
}

// ============================================================================
// INA228 monitor (not compiled with BATTERY_PACKET_ONLY)
// ============================================================================
#ifndef BATTERY_PACKET_ONLY

bool BatteryMonitor::probe() {
    if (!wire_) return false;
    wire_->beginTransmission(cfg_.i2cAddr);
    return wire_->endTransmission() == 0;
}

bool BatteryMonitor::begin(const BatteryMonitorConfig &cfg, TwoWire &wire) {
    cfg_         = cfg;
    wire_        = &wire;
    data_        = BatteryData();
    initialized_ = false;
    haveSample_  = false;
    firstStats_  = true;
    seq_         = 0;

    if (!probe()) { data_.flags = BATT_FLAG_I2C_ERROR; return false; }

    // begin() checks the TI manufacturer ID and INA228 device ID, then resets the chip.
    if (!ina_.begin(cfg_.i2cAddr, &wire)) { data_.flags = BATT_FLAG_I2C_ERROR; return false; }

    // The reset above cleared the calibration, so (re)apply it.
    ina_.setShunt(cfg_.shuntOhms, cfg_.maxCurrentA);

    // Use the finer +/-40.96 mV ADC range when the expected shunt drop fits with margin.
    const float fullScaleV = cfg_.shuntOhms * cfg_.maxCurrentA;
    lowRange_ = (fullScaleV <= 0.040f);
    ina_.setADCRange(lowRange_ ? 1 : 0);    // also refreshes SHUNT_CAL

    ina_.setMode(INA2XX_MODE_CONTINUOUS);
    ina_.setAveragingCount(cfg_.averaging);
    ina_.setVoltageConversionTime(cfg_.voltageTime);
    ina_.setCurrentConversionTime(cfg_.currentTime);
    ina_.setTemperatureConversionTime(cfg_.tempTime);

    initialized_ = true;
    return sampleNow(millis());
}

bool BatteryMonitor::update(uint32_t nowMs) {
    if (!initialized_) return false;
    if (haveSample_ && static_cast<uint32_t>(nowMs - lastSampleMs_) < cfg_.sampleIntervalMs) return false;
    return sampleNow(nowMs);
}

bool BatteryMonitor::sampleNow(uint32_t nowMs) {
    lastSampleMs_ = nowMs;
    haveSample_   = true;
    data_.uptimeMs = nowMs;

    if (!initialized_ || !probe()) {
        data_.valid = false;
        data_.flags = static_cast<uint16_t>((data_.flags & ~BATT_FLAG_VALID) | BATT_FLAG_I2C_ERROR);
        ++data_.i2cErrors;
        return false;
    }

    // Units straight from the Adafruit library: V, mV, mA, mW, J, C, degC.
    const float busV      = ina_.readBusVoltage();
    float       shuntMv   = ina_.readShuntVoltage();
    float       currentMa = ina_.readCurrent();
    const float powerMw   = ina_.readPower();
    const float energyJ   = ina_.readEnergy();
    float       chargeC   = ina_.readCharge();
    const float dieC      = ina_.readDieTemp();
    const uint16_t diag   = ina_.alertFunctionFlags();   // DIAG_ALRT bits 0..11

    if (cfg_.invertCurrent) { shuntMv = -shuntMv; currentMa = -currentMa; chargeC = -chargeC; }

    const bool plausible = isfinite(busV) && isfinite(shuntMv) && isfinite(currentMa) &&
                           isfinite(powerMw) && isfinite(energyJ) && isfinite(chargeC) &&
                           isfinite(dieC) && busV >= 0.0f && busV <= 85.0f;   // INA228 bus limit is 85 V
    if (!plausible) {
        data_.valid = false;
        data_.flags = static_cast<uint16_t>(data_.flags & ~BATT_FLAG_VALID);
        ++data_.i2cErrors;
        return false;
    }

    data_.busV      = busV;
    data_.shuntMv   = shuntMv;
    data_.currentMa = currentMa;
    data_.powerMw   = powerMw;
    data_.energyJ   = energyJ;
    data_.energyWh  = energyJ / 3600.0f;
    data_.chargeC   = chargeC;
    data_.chargeMah = chargeC / 3.6f;          // 1 mAh = 3.6 C
    data_.dieTempC  = dieC;
    data_.diag      = diag;

    // State of charge by coulomb counting (assumes discharge-positive current).
    if (cfg_.capacityMah > 0.0f) {
        float soc = cfg_.startSocPct - 100.0f * data_.chargeMah / cfg_.capacityMah;
        if (soc < 0.0f)   soc = 0.0f;
        if (soc > 100.0f) soc = 100.0f;
        data_.socPct = static_cast<uint8_t>(lroundf(soc));
    } else {
        data_.socPct = BATTERY_SOC_UNKNOWN;
    }

    // Running statistics
    const float absI = fabsf(currentMa);
    if (firstStats_) {
        data_.minBusV = data_.maxBusV = busV;
        data_.peakCurrentMa = absI;
        data_.peakPowerMw   = powerMw;
        firstStats_ = false;
    } else {
        if (busV < data_.minBusV) data_.minBusV = busV;
        if (busV > data_.maxBusV) data_.maxBusV = busV;
        if (absI > data_.peakCurrentMa) data_.peakCurrentMa = absI;
        if (powerMw > data_.peakPowerMw) data_.peakPowerMw = powerMw;
    }
    ++data_.samples;

    // Flags
    const float rangeMv = lowRange_ ? 40.96f : 163.84f;
    uint16_t f = BATT_FLAG_VALID;
    if (cfg_.warnLowV > 0.0f      && busV < cfg_.warnLowV)                   f |= BATT_FLAG_LOW_VOLTAGE;
    if (cfg_.warnOverA > 0.0f     && absI > cfg_.warnOverA * 1000.0f)        f |= BATT_FLAG_OVER_CURRENT;
    if (cfg_.warnOverTempC > 0.0f && dieC > cfg_.warnOverTempC)              f |= BATT_FLAG_OVER_TEMP;
    if (diag & INA228_DIAG_MATHOF)                                           f |= BATT_FLAG_MATH_OVERFLOW;
    if (diag & (INA228_DIAG_CHARGEOF | INA228_DIAG_ENERGYOF))                f |= BATT_FLAG_ACCUM_OVERFLOW;
    if (!(diag & INA228_DIAG_MEMSTAT))                                       f |= BATT_FLAG_MEMORY_ERROR;
    if (fabsf(shuntMv) >= 0.98f * rangeMv)                                   f |= BATT_FLAG_SHUNT_SATURATED;
    data_.flags = f;
    data_.valid = true;
    return true;
}

#if !defined(BATTERY_SERIAL_ONLY) || defined(BATTERY_PACKET_ONLY)
BatteryPacket BatteryMonitor::makePacket() {
    BatteryPacket p;
    memset(&p, 0, sizeof(p));
    p.magic      = BATTERY_PACKET_MAGIC;
    p.version    = BATTERY_PACKET_VERSION;
    p.soc_pct    = data_.socPct;
    p.flags      = data_.flags;
    p.sequence   = seq_;
    p.uptime_ms  = data_.uptimeMs;
    p.bus_mV     = satI32(data_.busV * 1000.0f);
    p.shunt_uV   = satI32(data_.shuntMv * 1000.0f);
    p.current_mA = satI32(data_.currentMa);
    p.power_mW   = satI32(data_.powerMw);
    p.energy_mWh = satU32(data_.energyWh * 1000.0f);
    p.charge_mAh = satI32(data_.chargeMah);
    p.temp_cC    = satI16(data_.dieTempC * 100.0f);
    p.diag       = data_.diag;
    p.crc        = batteryCrc16(reinterpret_cast<const uint8_t *>(&p), offsetof(BatteryPacket, crc));
    data_.sequence = seq_;
    ++seq_;
    return p;
}
#endif

void BatteryMonitor::resetStats() {
    firstStats_ = true;
    data_.minBusV = data_.maxBusV = data_.peakCurrentMa = data_.peakPowerMw = 0;
    data_.samples = 0;
    data_.i2cErrors = 0;
}

void BatteryMonitor::resetAccumulators() {
    if (!initialized_) return;
    ina_.resetAccumulators();
    // Defensive: if RSTACC (CONFIG bit 14) is not self-clearing on your silicon,
    // leaving it set would hold ENERGY/CHARGE at zero forever. Writing 0 is harmless.
    Adafruit_I2CRegisterBits rstacc(ina_.Config, 1, 14);
    rstacc.write(0);
    data_.energyJ = data_.energyWh = data_.chargeC = data_.chargeMah = 0;
    resetStats();
}

void BatteryMonitor::printStatus(Print &out) const {
    out.println("=== Battery (INA228) ===");
    if (!(data_.flags & BATT_FLAG_VALID)) {
        out.println((data_.flags & BATT_FLAG_I2C_ERROR) ? "INA228 not responding on I2C" : "No valid sample yet");
        return;
    }
    out.print("Voltage:  "); out.print(data_.busV, 3);       out.println(" V");
    out.print("Current:  "); out.print(data_.currentMa, 1);  out.println(" mA");
    out.print("Power:    "); out.print(data_.powerMw, 1);    out.println(" mW");
    out.print("Shunt:    "); out.print(data_.shuntMv, 3);    out.println(" mV");
    out.print("Energy:   "); out.print(data_.energyWh, 4);   out.print(" Wh  ("); out.print(data_.energyJ, 1); out.println(" J)");
    out.print("Charge:   "); out.print(data_.chargeMah, 2);  out.print(" mAh ("); out.print(data_.chargeC, 2); out.println(" C)");
    out.print("Die temp: "); out.print(data_.dieTempC, 2);   out.println(" C");
    if (data_.socPct != BATTERY_SOC_UNKNOWN) { out.print("SOC est:  "); out.print((int)data_.socPct); out.println(" %"); }
    out.print("Min/Max V: "); out.print(data_.minBusV, 3); out.print(" / "); out.println(data_.maxBusV, 3);
    out.print("Peak I/P:  "); out.print(data_.peakCurrentMa, 1); out.print(" mA / "); out.print(data_.peakPowerMw, 1); out.println(" mW");
    out.print("Flags: 0x"); out.print(data_.flags, HEX); out.print("  DIAG_ALRT: 0x"); out.println(data_.diag, HEX);
}

#endif // BATTERY_PACKET_ONLY
