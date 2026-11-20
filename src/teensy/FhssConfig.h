#pragma once
// EXPERIMENTAL bench-test FHSS: SX1276/77/78/79 packet-internal frequency hopping.
// NOT field-validated. Check local radio authorization before any over-the-air test.
// TX and RX must use identical channels and hop period.
// Both boards must service the FHSS_CHANGE_CHANNEL IRQ promptly. This build polls
// the SPI IRQ register, because the existing BEL schematic does not wire DIO2.
// Blocking SD/Serial operations can miss deadlines and lose packets.
#include <Arduino.h>
#include "Config.h"

static const float BEL_HOP_CHANNEL_MHZ[] = {433.30f, 433.90f, 434.50f};
static const uint8_t BEL_HOP_CHANNEL_COUNT = sizeof(BEL_HOP_CHANNEL_MHZ) / sizeof(BEL_HOP_CHANNEL_MHZ[0]);
// At SF7 / BW500k, one symbol = 0.256 ms. 20 symbols = ~5.12 ms.
// A typical 32-byte telemetry packet (+ 4 RadioHead headers) lasts about 19 ms;
// a 48-symbol period would not exercise all three configured channels.
static const uint8_t BEL_HOP_PERIOD_SYMBOLS = 20;
static const uint8_t BEL_REG_HOP_CHANNEL = 0x1C;
static const uint8_t BEL_IRQ_FHSS_CHANGE_CHANNEL = 0x02;

// This is not a general frequency planner or a declaration of regulatory legality.
// It is an experimental 3-channel bench setup only. Keep antennas disconnected
// from deployment/ignition equipment; use an RF-safe, authorized test setup.
template <typename RadioT>
static inline bool belFhssService(RadioT &radio) {
    if (!ENABLE_FHSS) return false;
    const uint8_t flags = radio.spiRead(REG_IRQ_FLAGS);
    if (!(flags & BEL_IRQ_FHSS_CHANGE_CHANNEL)) return false;
    const uint8_t hop = radio.spiRead(BEL_REG_HOP_CHANNEL) & 0x3F;
    const float f = BEL_HOP_CHANNEL_MHZ[hop % BEL_HOP_CHANNEL_COUNT];
    const bool changed = radio.setFrequency(f);
    // Clear only FHSS flag, not RX_DONE or TX_DONE.
    radio.spiWrite(REG_IRQ_FLAGS, BEL_IRQ_FHSS_CHANGE_CHANNEL);
    return changed;
}

template <typename RadioT>
static inline void belFhssResetToStart(RadioT &radio) {
    if (ENABLE_FHSS) radio.setFrequency(BEL_HOP_CHANNEL_MHZ[0]);
}
