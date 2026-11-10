#pragma once
#include "TelemetryPacket.h"
#include <stdio.h>

// Human-readable CSV preserved from the supplied transmitter code.
// Radio packets are always binary TelemetryPacket, not CSV text.
static inline int formatTelemetryCsv(const TelemetryPacket &packet,
                                     char *csv, size_t capacity) {
    return snprintf(csv, capacity,
                    "%lu,%lu,%.2f,%u,%d,%.4f,%.1f,%.1f",
                    (unsigned long)packet.seqNum,
                    (unsigned long)packet.timeMs,
                    packet.rtdTempC,
                    (unsigned)packet.fault,
                    (int)packet.adcRaw,
                    packet.adcVoltage,
                    packet.pressurePsi,
                    packet.pressureKPa);
}
static const char *const TELEMETRY_CSV_HEADER =
    "seq,ms,rtd_c,rtd_fault,adc_raw,adc_v,pressure_psi,pressure_kpa";
