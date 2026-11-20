#pragma once
#include <stdint.h>
#include <stddef.h>
#include "ControlProtocol.h"
// Engineering telemetry link statistics. CRC detects accidental corruption,
// but is NOT a security mechanism. Bench range testing only.
static constexpr uint8_t SOAR_STATS_REPLY = 4;
static constexpr uint32_t SOAR_STATS_MAGIC = 0x53544154UL; // STAT
struct __attribute__((packed)) LinkStatsPacket {
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t request_sequence;
    uint32_t session_id;
    uint32_t tx_attempts;    // attempts incl failures
    uint32_t tx_completed;   // local radio TX_DONE (not guaranteed RX)
    uint32_t tx_failed;      // send() failure / TX_DONE timeout
    uint16_t crc;
};
static_assert(sizeof(LinkStatsPacket) == 26, "LinkStatsPacket must be 26 bytes");
static inline LinkStatsPacket soarStats(uint16_t requestSeq, uint32_t session,
                                         uint32_t attempts, uint32_t completed, uint32_t failed) {
    LinkStatsPacket p = {};
    p.magic=SOAR_STATS_MAGIC; p.version=SOAR_CONTROL_VERSION; p.type=SOAR_STATS_REPLY;
    p.request_sequence=requestSeq; p.session_id=session;
    p.tx_attempts=attempts; p.tx_completed=completed; p.tx_failed=failed;
    p.crc=soarCrc16(reinterpret_cast<const uint8_t*>(&p),offsetof(LinkStatsPacket,crc));
    return p;
}
static inline bool soarStatsValid(const LinkStatsPacket& p) {
    return p.magic==SOAR_STATS_MAGIC && p.version==SOAR_CONTROL_VERSION &&
        p.type==SOAR_STATS_REPLY && p.tx_completed<=p.tx_attempts &&
        p.tx_failed<=p.tx_attempts && p.tx_completed+p.tx_failed==p.tx_attempts &&
        p.crc==soarCrc16(reinterpret_cast<const uint8_t*>(&p),offsetof(LinkStatsPacket,crc));
}
// Incoming RX telemetry is sequential on one LoRa link. Ignore duplicate or
// reordered/old packets for PDR; report count separately. 32-bit overflow of
// sequence IDs is beyond intended bench duration. Reset on new session_id.
class LinkReceiveCounter {
public:
    void reset() { sessionId_=0; received_=0; latestSeq_=0; duplicates_=0; seen_=false; }
    bool accept(uint32_t session, uint32_t seq) {
        if (seq==0) return false;
        if (!seen_ || session!=sessionId_) {
            reset(); sessionId_=session; seen_=true;
        }
        if (seq<=latestSeq_) { ++duplicates_; return false; }
        latestSeq_=seq; ++received_; return true;
    }
    bool seen() const { return seen_; }
    uint32_t sessionId() const { return sessionId_; }
    uint32_t received() const { return received_; }
    uint32_t latestSequence() const { return latestSeq_; }
    uint32_t duplicates() const { return duplicates_; }
    uint32_t missedThroughLatest() const { return latestSeq_ >= received_ ? latestSeq_ - received_ : 0; }
    float provisionalPct() const { return latestSeq_ ? 100.0f*received_/latestSeq_ : 0.0f; }
private:
    uint32_t sessionId_=0, received_=0, latestSeq_=0, duplicates_=0;
    bool seen_=false;
};
