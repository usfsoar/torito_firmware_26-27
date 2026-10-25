// ============================================================
// SOAR BEL: RAW-ALTITUDE bench recovery + Kalman telemetry + LoRa
// No recovery hardware is initialized, enabled or commanded by this build.
// This link test uses matching 32-byte telemetry packets on BOTH boards.
// ============================================================
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <SdFat.h>
#include <ICM_20948.h>
#include <MS5607.h>
#include <Watchdog_t4.h>
#include <math.h>

#include "Config.h"
#include "FlightData.h"
#include "FlightPacket.h"
#include "LinkStats.h"
#include "GPSReader.h"
#include "RadioTransmitter.h"
#include "SDLogger.h"
#include "IMUSensor.h"
#include "Altimeter.h"
#include "Orientation.h"
#include "KalmanFilter.h"
#include "RecoveryMonitor.h"
#include "RawBaroVelocity.h"
#include "AvBayEnableState.h"
#include "BenchSolenoidMonitor.h"
// Hardware SDriver remains deliberately uninstantiated.
// All output changes below are virtual; MCP23017 pins never change.

WDT_T4<WDT3> wdt;
SDLogger sdLogger;
OrientationState orientState;
GPSReader gpsReader;
RadioTransmitter transmitter;
IMUSensor imu;
Altimeter altimeter;
KalmanFilter kalman;
RecoveryMonitor recovery;
RawBaroVelocity rawBaroVelocity;
BenchSolenoidMonitor solenoidBench;
SensorSample sample;
AvBayEnableState avBayState;
bool servicesStarted=false;

uint32_t lastTransmitTime = 0, lastSensorRead = 0, lastPrint = 0;
uint32_t lastFreshImuMs = 0, lastOrientMs = 0, lastKalmanMs = 0;
uint32_t packetsSentOk = 0, packetsFailed = 0; // rate counters, reset every print
uint32_t txAttemptsTotal=0, txCompletedTotal=0, txFailedTotal=0;
uint32_t txSessionId=0;
bool telemetryFinished=false;
uint16_t lastEndedRequest=0;
bool haveEndRequest=false;

void beginNewLinkSession() {
    // Session ID is a test label, NOT a cryptographic random ID.
    txSessionId=micros() ^ (millis()<<16) ^ (uint32_t)0xA53C7D21UL;
    if (!txSessionId) txSessionId=1;
    txAttemptsTotal=txCompletedTotal=txFailedTotal=0;
    telemetryFinished=false;
    haveEndRequest=false;
    Serial.print("LINK NEW SESSION id="); Serial.println(txSessionId);
}

bool kalmanInitialized = false, kalmanFresh = false, haveFreshImu = false;
bool rawVelocityValid = false;
float rawVelocityMps = 0.0f;

void deselectAllMainSpiDevices() {
    for (int i = 0; i < SPI_CS_COUNT; ++i) {
        pinMode(SPI_CS_PINS[i], OUTPUT);
        digitalWrite(SPI_CS_PINS[i], HIGH);
    }
}

void printBenchChannels() {
    Serial.print("VIRTUAL SOLENOID outputs (NO MCP23017 IO): ");
    Serial.print("N0/N1 [drogue] = ");
    Serial.print(solenoidBench.channel(0) ? "HIGH" : "LOW");
    Serial.print("; N2/N3 [main] = ");
    Serial.println(solenoidBench.channel(2) ? "HIGH" : "LOW");
}

void updateSensors(uint32_t now) {
    sample.baroValid = altimeter.update();
    if (sample.baroValid) {
        sample.baroAltM = altimeter.altitudeM();
        sample.maxAltM = altimeter.maxAltitudeM();
        sample.pressureHPa = altimeter.pressureHPa();
        sample.tempC = altimeter.temperatureC();
    }

    // Guard against stale dataReady() values. Existing original loop serviced IMU
    // independently from this 10 Hz sensor snapshot.
    sample.imuValid = imu.isReady() && haveFreshImu &&
                      (uint32_t)(now - lastFreshImuMs) <= IMU_MAX_SAMPLE_AGE_MS;
    if (sample.imuValid) {
        sample.axMg = imu.accelX_mg(); sample.ayMg = imu.accelY_mg(); sample.azMg = imu.accelZ_mg();
        sample.gxDps = imu.gyroX_dps(); sample.gyDps = imu.gyroY_dps(); sample.gzDps = imu.gyroZ_dps();
        float dt = lastOrientMs ? (uint32_t)(now - lastOrientMs) * 0.001f :
                                SENSOR_READ_INTERVAL_MS * 0.001f;
        if (dt < 0.01f || dt > 0.5f) {
            // Gap too long to trust attitude estimate as continuous.
            orientState = OrientationState();
            dt = SENSOR_READ_INTERVAL_MS * 0.001f;
        }
        const Orientation o = computeOrientation(sample.axMg, sample.ayMg, sample.azMg,
            sample.gxDps, sample.gyDps, sample.gzDps, dt, orientState, 0.98f);
        lastOrientMs = now;
        sample.phiDeg = o.phiDeg;
        sample.thetaDeg = o.thetaDeg;
        sample.vertAccMps2 = o.verticalLinearAccelMps2;
    }
    // Real flight-deployment flags: false because physical outputs are DISABLED.
    sample.drogue = false;
    sample.mainDeployed = false;
    sdLogger.logSensors(sample);

    kalmanFresh = false;
    if (sample.baroValid && sample.imuValid &&
        isfinite(sample.baroAltM) && isfinite(sample.vertAccMps2)) {
        if (!kalmanInitialized || (uint32_t)(now - lastKalmanMs) > 500UL) {
            // Initialize using the CURRENT measured altitude; never propagate a
            // seconds-long sensor gap as one prediction step.
            kalman.reset(sample.baroAltM);
            kalmanInitialized = true;
            kalmanFresh = true;
        } else {
            const float dt = (uint32_t)(now - lastKalmanMs) * 0.001f;
            if (dt >= 0.01f && dt <= 0.5f) {
                kalman.update(sample.baroAltM, sample.vertAccMps2, dt);
                kalmanFresh = isfinite(kalman.altitude()) &&
                              isfinite(kalman.velocity()) && isfinite(kalman.acceleration());
            }
        }
        lastKalmanMs = now;
    }

    // RECOVERY SIMULATION ONLY: use the MS5607 RAW altitude and its raw
    // ~1-second finite-difference velocity. Do not use any Kalman output
    // in simulated drogue/main decisions. IMU accel is unfiltered as well.
    if (sample.baroValid) {
        rawVelocityValid = rawBaroVelocity.add(sample.baroAltM, now, rawVelocityMps);
    } else {
        rawBaroVelocity.reset();
        rawVelocityValid = false;
        rawVelocityMps = 0.0f;
    }
    const bool rawRecoveryValid = sample.baroValid && sample.imuValid &&
        rawVelocityValid && isfinite(sample.vertAccMps2);
    RecoveryInput input = {rawRecoveryValid, sample.baroAltM,
                           rawVelocityMps, sample.vertAccMps2};
    RecoveryEvents ev = recovery.step(input, now);
    if (ev.drogueCandidate || ev.mainCandidate) {
        if (ev.drogueCandidate) Serial.println("*** SIMULATED DROGUE CANDIDATE -- NO SOLENOID OUTPUT ***");
        if (ev.mainCandidate) Serial.println("*** SIMULATED MAIN CANDIDATE -- NO SOLENOID OUTPUT ***");
        const bool saved = sdLogger.saveSimState(recovery.drogueSimulated(), recovery.mainSimulated());
        Serial.println(saved ? "Simulated flags saved to SD." : "WARNING: simulation state NOT saved to SD.");
    }
    if (ev.droguePulseEnded) Serial.println("SIM drogue 10s pulse ended (NO HARDWARE OUTPUT)");
    if (ev.mainPulseEnded) Serial.println("SIM main 10s pulse ended (NO HARDWARE OUTPUT)");

    // The simulation drives only these four virtual channel states.
    // This function never addresses the MCP23017 or solenoid driver.
    if (solenoidBench.sync(avBayState.enabled(),
                           recovery.droguePulseActive(), recovery.mainPulseActive())) {
        printBenchChannels();
    }

    sdLogger.logKalman(kalmanFresh, sample.baroAltM, sample.vertAccMps2,
        kalman.altitude(), kalman.velocity(), kalman.acceleration(),
        recovery.peakAltitudeM(), recovery.dropFromPeakM(),
        recovery.drogueSimulated(), recovery.mainSimulated(),
        recovery.droguePulseActive(), recovery.mainPulseActive(),
        rawVelocityValid, rawVelocityMps);
}

FlightData buildFlightData()
{
    FlightData d;

    d.sats = gpsReader.satellites();

    if (gpsReader.hasFix())
    {
        d.fix     = gpsReader.fixQuality();
        d.lat     = gpsReader.latitude();
        d.lon     = gpsReader.longitude();
        d.gpsAltM = gpsReader.altitude();
        d.speedKn = gpsReader.speed();
    }
    // else: fix stays 0 and the GPS fields stay 0, so no stale position is sent

    d.baroValid = sample.baroValid;
    d.baroAltM  = sample.baroAltM;

    d.drogue       = false; // No physical deployment in bench build
    d.mainDeployed = false; // No physical deployment in bench build

    d.imuValid    = sample.imuValid;
    d.phiDeg      = sample.phiDeg;
    d.thetaDeg    = sample.thetaDeg;
    d.vertAccMps2 = sample.vertAccMps2;

    return d;
}

void printStatus(uint32_t rateWindowMs)
{
    Serial.println();
    Serial.println("================ STATUS ================");

    const float rateFactor=rateWindowMs ? (1000.0f/(float)rateWindowMs) : 0.0f;
    Serial.print("TX MEASURED: "); Serial.print((packetsSentOk+packetsFailed)*rateFactor,1);
    Serial.print(" Hz attempted; "); Serial.print(packetsSentOk*rateFactor,1);
    Serial.print(" Hz TX_DONE; target "); Serial.print(SOAR_TARGET_HZ);
    Serial.println(" Hz (scheduled, not guaranteed)");
    Serial.print("Radio:    ");
    Serial.print(packetsSentOk);
    Serial.print(" sent OK, ");
    Serial.print(packetsFailed);
    Serial.println(" failed (last second)");

    Serial.print("Packets sent OK: ");
    Serial.print(packetsSentOk);
    Serial.print("  Failed: ");
    Serial.println(packetsFailed);

    Serial.print("LINK cumulative: attempted="); Serial.print(txAttemptsTotal);
    Serial.print(" radio TX_DONE="); Serial.print(txCompletedTotal);
    Serial.print(" failed="); Serial.print(txFailedTotal);
    Serial.print(" session="); Serial.print(txSessionId);
    Serial.print(" finished="); Serial.println(telemetryFinished ? "YES" : "NO");

    if (gpsReader.hasFix())
    {
        Serial.print("GPS:      fix ");
        Serial.print(gpsReader.fixQuality());
        Serial.print(", ");
        Serial.print(gpsReader.satellites());
        Serial.println(" satellites");

        Serial.print("  Lat/Lon: ");
        Serial.print(gpsReader.latitude(), 6);
        Serial.print(", ");
        Serial.println(gpsReader.longitude(), 6);

        Serial.print("  GPS altitude: ");
        Serial.print(gpsReader.altitude());
        Serial.print(" m   speed: ");
        Serial.print(gpsReader.speed());
        Serial.println(" knots");
    }
    else
    {
        Serial.print("GPS:      no fix (satellites: ");
        Serial.print(gpsReader.satellites());
        Serial.println(")");
    }

    if (sample.baroValid)
    {
        Serial.print("Altitude: ");
        Serial.print(sample.baroAltM, 2);
        Serial.print(" m   max: ");
        Serial.print(sample.maxAltM, 2);
        Serial.print(" m   (pressure ");
        Serial.print(sample.pressureHPa, 2);
        Serial.println(" hPa, relative to power-on)");
    }
    else
    {
        Serial.println("Altitude: UNAVAILABLE (MS5607 not answering or not zeroed)");
    }

    if (sample.imuValid)
    {
        Serial.print("Accel:    X ");
        Serial.print(sample.axMg, 1);
        Serial.print("  Y ");
        Serial.print(sample.ayMg, 1);
        Serial.print("  Z ");
        Serial.print(sample.azMg, 1);
        Serial.println(" mg");

        Serial.print("Angles:   phi (roll) ");
        Serial.print(sample.phiDeg, 2);
        Serial.print(" deg   theta (pitch) ");
        Serial.print(sample.thetaDeg, 2);
        Serial.println(" deg");

        Serial.print("Vertical: ");
        Serial.print(sample.vertAccMps2, 3);
        Serial.println(" m/s^2 linear, 1 g removed (accelerometer-only, bench value)");
    }
    else
    {
        Serial.println("IMU:      NOT READY");
    }

    Serial.print("Kalman:   ");
    if (kalmanFresh) {
        Serial.print(kalman.altitude(), 2); Serial.print(" m; v=");
        Serial.print(kalman.velocity(), 2); Serial.print(" m/s; a=");
        Serial.print(kalman.acceleration(), 2); Serial.println(" m/s^2");
    } else Serial.println("unavailable (requires fresh IMU and barometer)");
    Serial.print("SIM RAW MS5607: alt=");
    if (sample.baroValid) Serial.print(sample.baroAltM, 2);
    else Serial.print("NA");
    Serial.print(" m; velocity=");
    if (rawVelocityValid) Serial.print(rawVelocityMps, 2);
    else Serial.print("warming up / unavailable");
    Serial.println(" m/s (barometer difference, NO Kalman)");
    Serial.print("SIM RAW: peak="); Serial.print(recovery.peakAltitudeM(), 2);
    Serial.print(" m; peak-drop="); Serial.print(recovery.dropFromPeakM(), 2);
    Serial.println(" m (RAW barometer, virtual decisions only)");
    Serial.print("SIM decisions: drogue="); Serial.print(recovery.drogueSimulated());
    Serial.print(" main="); Serial.print(recovery.mainSimulated());
    Serial.print(" pulses d/m="); Serial.print(recovery.droguePulseActive());
    Serial.print("/"); Serial.println(recovery.mainPulseActive());
    printBenchChannels();
    Serial.println("PHYSICAL SOLENOID OUTPUTS DISABLED -- VIRTUAL CHANNELS ONLY");
    Serial.println("========================================");

    packetsSentOk     = 0;
    packetsFailed = 0;
}


void handleUsbBenchCommands() {
    // Intentionally USB SERIAL ONLY. Unauthenticated over-the-air RF reconfiguration
    // is NOT implemented; a separate acknowledged command protocol is required.
    static char cmd[24];
    static uint8_t len = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            cmd[len] = 0;
            if (strcmp(cmd, "RESET_SIM") == 0) {
                recovery.reset();
                rawBaroVelocity.reset(); rawVelocityValid = false; rawVelocityMps = 0.0f;
                if (solenoidBench.forceOff()) printBenchChannels();
                const bool ok = sdLogger.saveSimState(false, false);
                Serial.println(ok ? "Sim flags reset and saved." : "Sim reset in RAM; no SD persistence!");
            } else if (strcmp(cmd, "HELP") == 0) {
                Serial.println("USB bench commands: HELP, RESET_SIM, RF?, STATE?, CHANNELS?");
            } else if (strcmp(cmd, "CHANNELS?") == 0) {
                printBenchChannels();
            } else if (strcmp(cmd, "STATE?") == 0) {
                Serial.print("AVBAY state = "); Serial.println(avBayState.enabled() ? 1 : 0);
            } else if (strcmp(cmd, "RF?") == 0) {
                Serial.print("RF FIXED: 433 MHz BW "); Serial.print(RADIO_BANDWIDTH_HZ / 1000U);
                Serial.print(" kHz SF7 CR4/5 CRC ON; target "); Serial.print(SOAR_TARGET_HZ);
                Serial.println(" Hz, 17 dBm");
            } else if (len) {
                Serial.println("Unknown USB command; type HELP.");
            }
            len = 0;
        } else if (len < sizeof(cmd)-1) cmd[len++] = c;
        else len = 0;
    }
}

// Only start sensors, filter and SD logging after software enable is latched.
// This does NOT switch physical power or enable any recovery hardware.
void startActiveServices() {
    if (servicesStarted) return;
    Serial.println("AVBAY: starting sensors / Kalman / SD services");
    bool imuOk=imu.begin();
    bool altOk=altimeter.begin();
    Serial.print("IMU: "); Serial.println(imuOk ? "OK" : "FAILED");
    Serial.print("MS5607: "); Serial.println(altOk ? "OK" : "FAILED");
    deselectAllMainSpiDevices();
    gpsReader.begin();
    const bool logsReady=sdLogger.beginLogging();
    Serial.println(logsReady ? "SD logging started" : "WARNING: SD log channels unavailable");
    bool d=false,m=false;
    if (sdLogger.loadSimState(d,m)) {
        recovery.restoreSimulatedFlags(d,m);
        Serial.println("BENCH SIM recovery flags restored (NO ACTUATORS)");
    }
    if (altOk) kalman.reset(0.0f);
    kalmanInitialized=false;
    rawBaroVelocity.reset(); rawVelocityValid = false; rawVelocityMps = 0.0f;
    lastSensorRead=lastTransmitTime=lastPrint=millis();
    servicesStarted=true;
}

void handleAirCommands() {
    ControlPacket cmd;
    if (!transmitter.receiveControl(cmd)) return;
    if (cmd.type==SOAR_CONTROL_END) {
        // Freeze counters BEFORE sending the report. Retry of the same END
        // request returns the same snapshot; no telemetry resumes automatically.
        if (!telemetryFinished) {
            telemetryFinished=true;
            Serial.print("LINK END: attempts="); Serial.print(txAttemptsTotal);
            Serial.print(" done="); Serial.print(txCompletedTotal);
            Serial.print(" failed="); Serial.println(txFailedTotal);
            Serial.println(sdLogger.saveLinkSummary(txSessionId,txAttemptsTotal,
                txCompletedTotal,txFailedTotal) ? "TX stats saved to SD" : "TX stats SD unavailable");
        }
        const LinkStatsPacket stats=soarStats(cmd.sequence,txSessionId,
            txAttemptsTotal,txCompletedTotal,txFailedTotal);
        if (!transmitter.reportStats(stats)) Serial.println("WARNING: END stats reply TX failed");
        return;
    }
    if (cmd.type != SOAR_CONTROL_SET) return;
    const bool request=cmd.enabled != 0;
    const bool saved=avBayState.setEnabled(request);
    // A failed write of OFF should still pause the currently running software.
    // The ACK truthfully says persisted=NO; old SD state may resume on reboot.
    if (!saved && !request) avBayState.disableVolatile();
    const bool actual=avBayState.enabled();
    Serial.print("AIR COMMAND "); Serial.print(request ? 1 : 0);
    Serial.print(" seq="); Serial.print(cmd.sequence);
    Serial.print(" persisted="); Serial.println(saved ? "YES" : "NO");
    if (saved && actual && telemetryFinished) beginNewLinkSession();
    // ACK persisted state using existing control protocol.
    if (!transmitter.acknowledge(cmd,actual,saved))
        Serial.println("WARNING: command ACK radio TX failed");
    if (actual && !servicesStarted) startActiveServices();
    if (!actual) {
        if (solenoidBench.forceOff()) printBenchChannels();
    }
    if (!actual && servicesStarted) {
        sdLogger.flushNow();
        Serial.println("AVBAY 0: paused telemetry and filtering. Radio keeps listening.");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("SOAR BEL -- 50 Hz TARGET V4 / SD ENABLE RESUME / BENCH ONLY");
    Serial.println("50 Hz target (20ms schedule), 500 kHz BW. Receiver must match.");
    Serial.println("Recovery solenoid hardware outputs are NOT controlled by this build.");
    solenoidBench.forceOff();
    printBenchChannels();
    deselectAllMainSpiDevices();
    SPI.begin();
    delay(100);
    if (!transmitter.begin()) {
        Serial.println("FATAL: RFM96W initialization failed. Check 3.3V SPI CS IRQ and reset.");
        while (1) delay(1000);
    }
    // Mount once at boot, even when software services are disabled, so the
    // next 1/0 command can be checkpointed and a reboot can restore it.
    // The logger itself shares this mounted SdFs instance with the state store.
    sdLogger.mount();
    if (!sdLogger.cardReady())
        Serial.println("AVBAY: SD unavailable: start DISABLED; command 1 will be rejected.");

    if (!avBayState.begin(sdLogger))
        Serial.println("AVBAY: persistent SD state unavailable. Remain DISABLED.");
    WDT_timings_t conf;
    conf.timeout=5000;
    wdt.begin(conf);
    Serial.print("AVBAY restored from SD = ");
    Serial.println(avBayState.enabled() ? "1 (ENABLED)" : "0 (DISABLED)");
    Serial.println("NO SD state record = 0; validated SD record 1 = auto resume.");
    Serial.println("To reset first-start behavior, remove AVBAY_A.DAT and AVBAY_B.DAT while powered OFF.");
    Serial.println("Await ground Serial Monitor command 1 to enable, 0 to pause.");
    beginNewLinkSession();
    if (avBayState.enabled()) startActiveServices();
}

void loop() {
    wdt.feed();
    handleUsbBenchCommands();
    handleAirCommands();
    if (!avBayState.enabled()) {
        // Radio is fully powered and in RX, but sensor/telemetry processing is OFF.
        if (servicesStarted) sdLogger.service();
        static uint32_t lastOffPrint=0;
        if (millis()-lastOffPrint >= 5000) {
            lastOffPrint=millis();
            Serial.println("AVBAY=0; radio listening for the ground station command 1");
        }
        return;
    }
    if (!servicesStarted) startActiveServices();
    gpsReader.update();
    sdLogger.service();
    if (imu.update()) {
        lastFreshImuMs=millis();
        haveFreshImu=true;
    }
    const uint32_t now=millis();
    if ((uint32_t)(now-lastSensorRead)>=SENSOR_READ_INTERVAL_MS) {
        lastSensorRead=now;
        updateSensors(now);
    }
    // Reserve 120ms per second for 1/0/END, because LoRa is HALF DUPLEX.
    // Receiver resends commands at 700ms intervals; leaving RX time helps.
    // This intentionally lowers sustainable rate below the nominal 50 Hz.
    const bool commandListenWindow=(now % 1000UL)<120UL;
    if (!telemetryFinished && !commandListenWindow &&
        (uint32_t)(now-lastTransmitTime)>=TRANSMIT_INTERVAL) {
        lastTransmitTime=now;
        if (TRANSMIT_WITHOUT_GPS_FIX || gpsReader.hasFix()) {
            FlightData data=buildFlightData();
            const uint32_t sequence=++txAttemptsTotal;  // increment on EVERY attempt
            const bool done=transmitter.transmit(data,txSessionId,sequence);
            if (done) { ++packetsSentOk; ++txCompletedTotal; }
            else { ++packetsFailed; ++txFailedTotal; }
            sdLogger.logTelemetryAttempt(txSessionId,sequence,done,
                txAttemptsTotal,txCompletedTotal,txFailedTotal);
            if (data.fix>0) sdLogger.logGps(data);
        }
    }
    if ((uint32_t)(now-lastPrint)>=PRINT_INTERVAL_MS) {
        const uint32_t elapsedMs=(uint32_t)(now-lastPrint);
        lastPrint=now;
        printStatus(elapsedMs);
    }
}
