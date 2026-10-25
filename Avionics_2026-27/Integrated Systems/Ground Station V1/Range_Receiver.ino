// ============================================================
// SOAR USF - Ground Station (Teensy 4.1 receiver)
// Receives the 32-byte FlightPacket from the transmitter, shows it,
// measures distance to the rocket, and logs everything to the SD card.
//
// Files in this sketch folder:
//   Config.h               pins, settings, flags
//   FlightData.h           values in normal units
//   FlightPacket.h         the 32-byte binary packet (same file as transmitter)
//   GPSReader.h/.cpp       receiver's own GPS + distance calculation
//   RadioReceiver.h/.cpp   RFM96W radio
//   SDLogger.h/.cpp        SD card logging (receiver_log.csv)
// ============================================================

// Libraries are listed here so the Arduino IDE finds them
// for the .cpp files as well.
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <SdFat.h>

#include "Config.h"
#include "FlightData.h"
#include "FlightPacket.h"
#include "LinkStats.h"
#include "GPSReader.h"
#include "RadioReceiver.h"
#include "SDLogger.h"

// ============================================================
// GLOBAL OBJECTS
// ============================================================

GPSReader     receiverGPS;
SDLogger      sdLogger;
RadioReceiver receiver;

LinkReceiveCounter linkCount;
uint32_t receivedThisSecond=0;
uint32_t lastPrint=0;
// Only a received END reply supplies the true attempted-transmit denominator.
bool lastReportValid=false;
LinkStatsPacket lastReport={};

// ASCII '1'/'0' typed in Arduino IDE Serial Monitor are sent as LoRa commands.
// ACK must match both sequence and requested state AND confirm SD save.
uint16_t commandSequence=0;
bool commandPending=false;
uint8_t requestedBayState=0;
bool pendingEnd=false;
uint8_t commandAttempts=0;
uint32_t lastCommandTxMs=0;
static const uint8_t MAX_COMMAND_ATTEMPTS=10;
static const uint32_t COMMAND_RETRY_MS=700;

void sendSerialHelp() {
    Serial.println("Ground commands: 1 = ENABLE/RESUME, 0 = PAUSE, E or END = END TEST and SAVE stats, ? = help");
    Serial.println("Choose ANY Serial Monitor line ending, including No line ending.");
    Serial.println("Power must already be supplied to the avionics Teensy/radio.");
}
void serviceUsbCommands() {
    while (Serial.available()) {
        const char c=static_cast<char>(Serial.read());
        if (c=='1' || c=='0' || c=='E' || c=='e') {
            pendingEnd=(c=='E' || c=='e');
            if (!pendingEnd) requestedBayState=static_cast<uint8_t>(c-'0');
            ++commandSequence;
            commandPending=true;
            commandAttempts=0;
            lastCommandTxMs=0;
            if (pendingEnd) Serial.println("GROUND REQUEST: END TEST; freeze TX and return exact attempts");
            else { Serial.print("GROUND REQUEST: AVBAY="); Serial.println(requestedBayState); }
        } else if (c=='?') sendSerialHelp();
        // Ignore CR/LF and any other characters.
    }
}
void serviceCommandRetries() {
    if (!commandPending) return;
    const uint32_t now=millis();
    if (commandAttempts>=MAX_COMMAND_ATTEMPTS) {
        if ((uint32_t)(now-lastCommandTxMs)>=COMMAND_RETRY_MS) {
            Serial.println("NO ACK after retries. AVBAY state UNKNOWN; check both radios.");
            commandPending=false;
        }
        return;
    }
    if (commandAttempts>0 && (uint32_t)(now-lastCommandTxMs)<COMMAND_RETRY_MS) return;
    const ControlPacket cmd=soarControl(pendingEnd ? SOAR_CONTROL_END : SOAR_CONTROL_SET,
                                        commandSequence,requestedBayState != 0,false);
    lastCommandTxMs=millis();
    ++commandAttempts;
    const bool sent=receiver.sendControl(cmd);
    Serial.print("Ground command TX attempt "); Serial.print(commandAttempts);
    Serial.println(sent ? " (radio TX done; awaiting ACK)" : " (radio TX failure)");
}
void handleControlAck(const ControlPacket &ack) {
    if (!commandPending || pendingEnd || ack.sequence!=commandSequence) {
        Serial.println("Ignored unexpected/old control ACK");
        return;
    }
    if (!ack.persisted || ack.enabled!=requestedBayState) {
        Serial.print("ACK received, but bay state/persistence differs. Bay reports state=");
        Serial.print(ack.enabled); Serial.print(" saved="); Serial.println(ack.persisted);
        commandPending=false;
        return;
    }
    Serial.print("CONFIRMED: AVBAY="); Serial.print(ack.enabled);
    Serial.println(" (saved on bay SD card; survives restart)");
    commandPending=false;
    if (ack.enabled && lastReportValid)
        Serial.println("Note: prior END report belongs to previous test; new session starts at TX.");
}

void handleStatsReply(const LinkStatsPacket &stats) {
    if (!commandPending || !pendingEnd || stats.request_sequence!=commandSequence) {
        Serial.println("Ignored old/unrequested END report"); return;
    }
    const uint32_t rx=(linkCount.seen() && linkCount.sessionId()==stats.session_id)
                        ? linkCount.received() : 0;
    const uint32_t dup=(linkCount.seen() && linkCount.sessionId()==stats.session_id)
                        ? linkCount.duplicates() : 0;
    const uint32_t lost=stats.tx_attempts>=rx ? stats.tx_attempts-rx : 0;
    const float percent=stats.tx_attempts ? 100.0f*(float)rx/stats.tx_attempts : 0.0f;
    Serial.println("======== FINAL LINK TEST RESULT ========");
    Serial.print("Session ID: "); Serial.println(stats.session_id);
    Serial.print("TX attempted: "); Serial.println(stats.tx_attempts);
    Serial.print("TX radio DONE: "); Serial.println(stats.tx_completed);
    Serial.print("TX failed: "); Serial.println(stats.tx_failed);
    Serial.print("RX unique packets: "); Serial.println(rx);
    Serial.print("Estimated lost: "); Serial.println(lost);
    Serial.print("Delivery percentage: "); Serial.print(percent,2); Serial.println("%");
    Serial.print("Duplicates/old: "); Serial.println(dup);
    Serial.println("TX telemetry FROZEN. Type 1 to resume with NEW counters.");
    Serial.println("========================================");
    Serial.println(sdLogger.saveLinkSummary(stats.session_id,stats.tx_attempts,
        stats.tx_completed,stats.tx_failed,rx,dup)
        ? "RX final stats saved to rx_link_summary.csv" : "RX final SD save FAILED/absent");
    lastReport=stats; lastReportValid=true;
    pendingEnd=false; commandPending=false;
}

bool drogueReported = false;
bool mainReported   = false;

// The newest packet, shown once a second
FlightData latest;
bool       haveLatest  = false;
int        latestRssi  = 0;
float      latestDist  = -1;

// Highest barometric altitude seen in the packets received
bool  haveMaxAlt = false;
float maxBaroAltM = 0;

// ============================================================
// SERIAL OUTPUT
// ============================================================

void printLatest()
{
    Serial.println("=== Rocket data (latest packet) ===");

    if (latest.fix > 0)
    {
        Serial.print("GPS:         fix ");
        Serial.print(latest.fix);
        Serial.print(", ");
        Serial.print(latest.sats);
        Serial.println(" satellites");

        Serial.print("  Lat/Lon:   ");
        Serial.print(latest.lat, 6);
        Serial.print(", ");
        Serial.println(latest.lon, 6);

        Serial.print("  GPS alt:   ");
        Serial.print(latest.gpsAltM, 0);
        Serial.print(" m   speed: ");
        Serial.print(latest.speedKn, 1);
        Serial.println(" knots");
    }
    else
    {
        Serial.print("GPS:         NO FIX on the rocket (satellites: ");
        Serial.print(latest.sats);
        Serial.println(") - position not valid, everything else below is still live");
    }

    if (latest.baroValid)
    {
        Serial.print("Altitude:    ");
        Serial.print(latest.baroAltM, 2);
        Serial.print(" m (above power-on)   max received: ");
        Serial.print(maxBaroAltM, 2);
        Serial.println(" m");
    }
    else
    {
        Serial.println("Altitude:    not available from the rocket");
    }

    if (latest.imuValid)
    {
        Serial.print("Angles:      phi (roll) ");
        Serial.print(latest.phiDeg, 2);
        Serial.print(" deg   theta (pitch) ");
        Serial.print(latest.thetaDeg, 2);
        Serial.println(" deg");

        Serial.print("Vertical:    ");
        Serial.print(latest.vertAccMps2, 2);
        Serial.println(" m/s^2 (1 g removed)");
    }
    else
    {
        Serial.println("IMU:         not available from the rocket");
    }

    Serial.print("Drogue:      ");
    Serial.println(latest.drogue ? "DEPLOYED (1)" : "not fired (0)");
    Serial.print("Main:        ");
    Serial.println(latest.mainDeployed ? "DEPLOYED (1)" : "not fired (0)");

    Serial.print("RSSI:        ");
    Serial.print(latestRssi);
    Serial.println(" dBm");

    if (latestDist >= 0)
    {
        Serial.print("Distance:    ");
        Serial.print(latestDist);
        Serial.println(" m");
    }
    else
    {
        Serial.println("Distance:    waiting for a GPS fix on both the rocket and this receiver");
    }

    Serial.print("Receiver GPS: ");
    Serial.print(receiverGPS.latitude(), 6);
    Serial.print(", ");
    Serial.print(receiverGPS.longitude(), 6);
    Serial.print("  sats: ");
    Serial.println(receiverGPS.satellites());

    Serial.println("---");
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // Wait for the Serial Monitor to open
    const uint32_t serialStart = millis();
    while (!Serial && millis() - serialStart < 2000)
        delay(10);

    Serial.println("SOAR USF - Range Test Receiver / 50 Hz TEST V4");
    Serial.print("RF: "); Serial.print(RF_FREQUENCY,1); Serial.print(" MHz / ");
    Serial.print(RADIO_BANDWIDTH_HZ/1000U); Serial.print(" kHz / SF7 / target ");
    Serial.print(SOAR_TARGET_HZ); Serial.println(" Hz (actual RX Hz below)");

    // ----- Receiver's own GPS -----
    receiverGPS.begin();

    // ----- SD card (optional) -----
    if (sdLogger.begin())
        Serial.println("SD card ready, logging to receiver_log.csv");
    else
        Serial.println("WARNING: SD card init failed - continuing WITHOUT logging");

    // ----- Radio -----
    Serial.println("Starting radio setup...");

    if (!receiver.begin())
    {
        Serial.println("ERROR: RFM96W init failed!");

        while (1)
        {
            delay(1000);
        }
    }

    lastPrint    = millis();

    Serial.println("Ready. Bay may be OFF; enter 1 in this Serial Monitor to enable it.");
    sendSerialHelp();
    Serial.println("---");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    serviceUsbCommands();

    // Keep reading the receiver's own GPS
    receiverGPS.update();

    // Flush the SD buffer when needed
    sdLogger.service();

    if (receiver.available())
    {
        FlightData data;

        ControlPacket control={};
        LinkStatsPacket stats={};
        uint32_t session=0, sequence=0;
        const ReceivedKind kind=receiver.receive(data,control,stats,session,sequence);
        if (kind==ReceivedKind::Control) {
            handleControlAck(control);
        } else if (kind==ReceivedKind::Stats) {
            handleStatsReply(stats);
        } else if (kind==ReceivedKind::Flight)
        {
            const bool previousSession=linkCount.seen();
            const uint32_t previousId=linkCount.sessionId();
            const bool unique=linkCount.accept(session,sequence);
            if (!unique) return; // never double-count duplicate frames
            if (!previousSession || previousId!=session) {
                receivedThisSecond=0;
                Serial.print("LINK session started: "); Serial.println(session);
            }
            ++receivedThisSecond;
            const int rssi = receiver.rssi();
            sdLogger.logLinkPacket(session,sequence,linkCount.received(),rssi);
            float dist = -1;

            // Distance needs a real GPS fix on BOTH the rocket and this receiver
            if (data.fix > 0 && receiverGPS.hasFix())
                dist = receiverGPS.distanceTo(data.lat, data.lon);

            if (data.baroValid && (!haveMaxAlt || data.baroAltM > maxBaroAltM))
            {
                maxBaroAltM = data.baroAltM;
                haveMaxAlt  = true;
            }

            if (data.baroValid && data.drogue && !drogueReported)
            {
                Serial.println(">>> DROGUE DEPLOYED (received)");
                drogueReported = true;
            }

            if (data.mainDeployed && !mainReported)
            {
                Serial.println(">>> MAIN DEPLOYED (received)");
                mainReported = true;
            }

            latest     = data;
            latestRssi = rssi;
            latestDist = dist;
            haveLatest = true;

            sdLogger.log(data, rssi, dist);
        }
    }

    // Commands use the same half-duplex radio. Retry when no ACK arrives.
    serviceCommandRetries();

    // Live count is cumulative. The provisional denominator is the LAST SEEN
    // sequence, not exact transmissions; use E/END for the final percentage.
    const uint32_t printNow=millis();
    if ((uint32_t)(printNow-lastPrint)>=PRINT_EVERY_MS) {
        const uint32_t elapsedMs=(uint32_t)(printNow-lastPrint);
        lastPrint=printNow;
        const float measuredRxHz=(elapsedMs > 0) ?
            ((float)receivedThisSecond * 1000.0f / (float)elapsedMs) : 0.0f;
        Serial.print("RX MEASURED: "); Serial.print(measuredRxHz,1);
        Serial.print(" Hz  | Target TX: "); Serial.print(SOAR_TARGET_HZ);
        Serial.print(" Hz | interval "); Serial.print(elapsedMs);
        Serial.println(" ms (valid UNIQUE packets only)");
        Serial.print("LINK: RX window packets="); Serial.print(receivedThisSecond);
        Serial.print(" cumulative="); Serial.print(linkCount.received());
        Serial.print(" last TX seq="); Serial.print(linkCount.latestSequence());
        Serial.print(" missing through last seq="); Serial.print(linkCount.missedThroughLatest());
        Serial.print(" observed delivery (provisional)=");
        Serial.print(linkCount.provisionalPct(),1); Serial.println("%");
        Serial.println("Type E (or END) for EXACT TX/RX percentage + SD summary.");
        receivedThisSecond=0;
        Serial.print("Radio diagnostics: RX_DONE="); Serial.print(receiver.irqRxDoneCount());
        Serial.print(" CRC_ERRORS="); Serial.print(receiver.irqCrcErrorCount());
        Serial.print(" WRONG_LENGTH="); Serial.println(receiver.wrongLengthCount());
        if (haveLatest) printLatest();
        else Serial.println("No telemetry yet (bay may be OFF). Type 1 to enable.");
    }
}
