#include "BatteryMonitor.h"

namespace {

BatteryMonitor battery;
uint32_t lastInitAttemptMs = 0;
uint32_t lastErrorMessageMs = 0;
bool csvHeaderPrinted = false;
bool errorReported = false;

void printSample() {
    errorReported = false;
    if (!csvHeaderPrinted) {
        Serial.println(batteryCsvHeader());
        csvHeaderPrinted = true;
    }

    char line[192];
    if (battery.formatCsv(line, sizeof(line)) > 0) {
        Serial.println(line);
    }
}

void reportError(const char *message, uint32_t nowMs) {
    if (!errorReported || static_cast<uint32_t>(nowMs - lastErrorMessageMs) >= 1000) {
        Serial.println(message);
        lastErrorMessageMs = nowMs;
        errorReported = true;
    }
}

} // namespace

void setup() {
    Serial.begin(115200);
    Serial.println("# INA228 battery monitor");

    lastInitAttemptMs = millis();
    if (battery.begin()) {
        printSample();
    } else {
        reportError("# INA228 initialization/sample failed; retrying", lastInitAttemptMs);
    }
}

void loop() {
    const uint32_t nowMs = millis();

    if (!battery.ready()) {
        if (static_cast<uint32_t>(nowMs - lastInitAttemptMs) < 1000) return;

        lastInitAttemptMs = nowMs;
        if (battery.begin()) {
            printSample();
        } else {
            reportError("# INA228 initialization/sample failed; retrying", nowMs);
        }
        return;
    }

    if (battery.update(nowMs)) {
        printSample();
    } else if (!battery.data().valid) {
        reportError("# INA228 sample failed; retrying", nowMs);
    }
}
