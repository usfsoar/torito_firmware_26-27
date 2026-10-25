#include "AvBayEnableState.h"
#include "ControlProtocol.h"
#include "SDLogger.h"

namespace {
constexpr uint32_t MAGIC = 0x534F4154UL;   // SOAR state
constexpr uint32_t COMMIT = 0xA65C2D91UL;
constexpr int SLOT_SIZE = 16;
static void store32(uint8_t *dst, uint32_t v) {
    for (int i=0; i<4; ++i) dst[i] = static_cast<uint8_t>(v >> (8*i));
}
static uint32_t read32(const uint8_t *src) {
    uint32_t v=0;
    for (int i=0; i<4; ++i) v |= static_cast<uint32_t>(src[i]) << (8*i);
    return v;
}
static void format(uint8_t *p, uint32_t seq, bool enabled) {
    for (int i=0; i<SLOT_SIZE; ++i) p[i]=0;
    store32(p+0, MAGIC);
    store32(p+4, seq);
    p[8] = enabled ? 1 : 0;
    p[9] = static_cast<uint8_t>(~p[8]);
    const uint16_t crc = soarCrc16(p, 10);
    p[10] = static_cast<uint8_t>(crc);
    p[11] = static_cast<uint8_t>(crc >> 8);
    store32(p+12, COMMIT);
}
}

bool AvBayEnableState::load(int slot, uint32_t &seq, bool &enabled) const {
    uint8_t p[SLOT_SIZE];
    if (!storage || !storage->readEnableRecord((uint8_t)slot,p,sizeof(p))) return false;
    if (read32(p+0) != MAGIC || read32(p+12) != COMMIT || p[8]>1 ||
        p[9] != static_cast<uint8_t>(~p[8])) return false;
    const uint16_t stored = static_cast<uint16_t>(p[10]) |
                            static_cast<uint16_t>(p[11]) << 8;
    if (stored != soarCrc16(p,10)) return false;
    seq = read32(p+4);
    enabled = p[8] != 0;
    return true;
}

bool AvBayEnableState::begin(SDLogger &logger) {
    storage=&logger;
    isEnabled = false;
    activeSlot = -1;
    generation = 0;
    hasStorage = logger.cardReady();
    if (!hasStorage) return false;
    uint32_t g0=0,g1=0;
    bool e0=false,e1=false;
    const bool ok0=load(0,g0,e0), ok1=load(1,g1,e1);
    if (ok0 && (!ok1 || static_cast<int32_t>(g0-g1)>=0)) {
        activeSlot=0; generation=g0; isEnabled=e0;
    } else if (ok1) {
        activeSlot=1; generation=g1; isEnabled=e1;
    }
    return true;
}

bool AvBayEnableState::setEnabled(bool requested) {
    if (!hasStorage || !storage) return false; // Never enable if SD unavailable
    if (activeSlot>=0 && isEnabled==requested) return true; // avoid unnecessary SD writes
    const int target = (activeSlot==0) ? 1 : 0;
    uint8_t p[SLOT_SIZE];
    format(p, generation+1, requested);
    // Never overwrite the last valid slot; verify the full new file on SD.
    if (!storage->writeEnableRecord((uint8_t)target,p,sizeof(p))) return false;
    uint32_t verifiedGeneration=0;
    bool verifiedEnabled=false;
    if (!load(target, verifiedGeneration, verifiedEnabled) ||
        verifiedGeneration!=generation+1 || verifiedEnabled!=requested) return false;
    activeSlot=target;
    generation=verifiedGeneration;
    isEnabled=requested;
    return true;
}
