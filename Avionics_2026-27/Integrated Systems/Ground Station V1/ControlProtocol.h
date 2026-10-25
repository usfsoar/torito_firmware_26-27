#pragma once
#include <stdint.h>
#include <stddef.h>

// BENCH ONLY. LoRa CRC is used by the modem; this additional CRC guards
// serialization and malformed control messages. NOT authentication.
// The Arduino IDE Serial Monitor sends ASCII '1'/'0' to the GROUND receiver.
// The receiver sends binary ControlPacket messages to the powered avionics bay.
static constexpr uint32_t SOAR_CONTROL_MAGIC = 0x534F4152UL;
static constexpr uint8_t SOAR_CONTROL_VERSION = 1;
static constexpr uint8_t SOAR_CONTROL_SET = 1;
static constexpr uint8_t SOAR_CONTROL_ACK = 2;
static constexpr uint8_t SOAR_CONTROL_END = 3; // STOP telemetry + report counters
struct __attribute__((packed)) ControlPacket {
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t sequence;
    uint8_t enabled;    // SET: requested 0/1; ACK: actual 0/1
    uint8_t persisted;  // ACK: 1 only after successful SD save/readback
    uint16_t crc;
};
static_assert(sizeof(ControlPacket) == 12, "Control packet size mismatch");

static inline uint16_t soarCrc16(const uint8_t *bytes, size_t count) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < count; ++i) {
        crc ^= static_cast<uint16_t>(bytes[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}
static inline ControlPacket soarControl(uint8_t type, uint16_t seq, bool enabled, bool saved) {
    ControlPacket p = {};
    p.magic = SOAR_CONTROL_MAGIC;
    p.version = SOAR_CONTROL_VERSION;
    p.type = type;
    p.sequence = seq;
    p.enabled = enabled ? 1 : 0;
    p.persisted = saved ? 1 : 0;
    p.crc = soarCrc16(reinterpret_cast<const uint8_t*>(&p), offsetof(ControlPacket, crc));
    return p;
}
static inline bool soarControlValid(const ControlPacket &p) {
    return p.magic == SOAR_CONTROL_MAGIC && p.version == SOAR_CONTROL_VERSION &&
           (p.type == SOAR_CONTROL_SET || p.type == SOAR_CONTROL_ACK || p.type == SOAR_CONTROL_END) &&
           p.enabled <= 1 && p.persisted <= 1 &&
           p.crc == soarCrc16(reinterpret_cast<const uint8_t*>(&p), offsetof(ControlPacket, crc));
}
