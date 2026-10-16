// Firmware E10: Shared I2C bus Line Sensor & NFC Reader at 100 Hz.
// Based on E6 (Line Sensor reliability) and E9B (NFC FM17550 async monitor).
// Both sensors share Wire on pins D4 (SDA) and D5 (SCL) at 400 kHz.
// Main control loop runs at 100 Hz (10 ms period).
// Serial status reports every 0.1 s (10 Hz) with fixed-length padded output:
//   (1) Latest Line sensor position (8-bit binary mask, bit 7 to bit 0, active-low)
//   (2) Latest Detected NFC UID (11 chars, or space padding if no tag in past 10s)
//   (3) Age of last NFC detection in seconds with 1 decimal (max 10.0s)
//   (4) Main control loop actual rate in Hz
// Non-drained serial guard prevents stalling when USB is closed or slow.
// Any reading errors are appended to the end of the 0.1s line rather than on new lines.

#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <stdarg.h>
#include "../e8_nfc_fm17550_i2c/NfcAsync.h"

namespace
{
    // =========================================================================
    // HARDWARE & TIMING CONSTANTS
    // =========================================================================

    // Shared I2C configuration: D4=SDA, D5=SCL, 400 kHz clock, 1 ms timeout
    constexpr uint8_t kI2cSdaPin = D4;
    constexpr uint8_t kI2cSclPin = D5;
    constexpr uint32_t kI2cClockHz = 400000;
    constexpr uint16_t kI2cTimeoutMs = 1;

    // Line sensor I2C address and data register
    constexpr uint8_t kLineSensorAddress = 0x12;
    constexpr uint8_t kLineSensorRegister = 0x30;

    // Loop periods in microseconds
    constexpr uint64_t kControlPeriodUs = 10000;    // 100 Hz control loop = 10 ms
    constexpr uint64_t kStatusReportPeriodUs = 50000; // 10 Hz status print = 0.1 s
    constexpr uint64_t kNfcTagTimeoutUs = 10000000; // 10 seconds max age for detected tag

    // Fixed width of base status message (excluding newline and optional appended error)
    // "POS: bbbbbbbb | UID: uuuuuuuuuuu | AGE:  x.xs | RATE: xxx.x Hz" -> 65 characters
    constexpr size_t kBaseMessageLength = 65;

    // =========================================================================
    // STATE VARIABLES
    // =========================================================================

    // Asynchronous NFC reader instance
    NfcAsync nfcReader;
    bool nfcRunning = false;

    // Latest Line sensor reading state
    uint8_t latestLineMask = 0xFF; // Active-low: 0=black line, 1=white background
    bool lineReadValid = false;
    uint8_t lineTransmitError = 0;
    bool lineShortRead = false;

    // Latest NFC reading state
    uint8_t latestUid[4] = {};
    bool hasDetectedUid = false;
    uint64_t lastUidDetectedAtUs = 0; // Timestamp of the most recent successful UID read
    NfcAsync::Result lastNfcError = NfcAsync::Result::None;

    // Accumulated reading errors within the current 0.1s reporting window
    uint32_t windowLineTxErrors = 0;
    uint32_t windowLineShortReads = 0;
    uint32_t windowNfcErrors = 0;
    uint8_t windowLastTxError = 0;
    NfcAsync::Result windowLastNfcError = NfcAsync::Result::None;

    // Control loop rate tracking
    uint64_t nextControlSlotUs = 0;
    uint64_t windowStartedAtUs = 0;
    uint32_t controlCyclesInWindow = 0;
    float measuredControlRateHz = 0.0f;
    uint64_t lastStatusPrintUs = 0;

    // Serial backpressure tracking (guarded non-blocking output)
    uint32_t serialDroppedLines = 0;
    uint32_t serialShortWrites = 0;

    // =========================================================================
    // NON-DRAINED SERIAL GUARD
    // =========================================================================

    // Drop entire lines if USB serial is disconnected or buffer lacks capacity.
    // This prevents blocking or stalling the 100 Hz control loop.
    void serialGuardedWrite(const char *buffer, size_t length)
    {
        if (!Serial)
        {
            ++serialDroppedLines;
            return;
        }

        // Verify that the hardware outgoing buffer has space for the entire line
        if (Serial.availableForWrite() < static_cast<int>(length))
        {
            ++serialDroppedLines;
            return;
        }

        const size_t written = Serial.write(reinterpret_cast<const uint8_t *>(buffer), length);
        if (written != length)
        {
            ++serialShortWrites;
        }
    }

    // =========================================================================
    // LINE SENSOR READ (Address 0x12)
    // =========================================================================

    // Read the 8-channel line sensor once using repeated-START (atomic ~250 us).
    // The sensor register 0x30 returns a single active-low byte (bit 7=X1, bit 0=X8).
    void readLineSensor()
    {
        Wire.beginTransmission(kLineSensorAddress);
        Wire.write(kLineSensorRegister);
        // Repeated-START write: endTransmission(false) holds the bus until requestFrom
        const uint8_t txError = Wire.endTransmission(false);
        if (txError != 0)
        {
            lineTransmitError = txError;
            ++windowLineTxErrors;
            windowLastTxError = txError;
            return;
        }

        const size_t received = Wire.requestFrom(static_cast<uint8_t>(kLineSensorAddress), static_cast<size_t>(1), true);
        if (received == 1 && Wire.available() == 1)
        {
            latestLineMask = static_cast<uint8_t>(Wire.read());
            lineReadValid = true;
            lineTransmitError = 0;
            lineShortRead = false;
        }
        else
        {
            lineShortRead = true;
            ++windowLineShortReads;
            while (Wire.available())
            {
                Wire.read(); // Discard partial feedback
            }
        }
    }

    // =========================================================================
    // NFC READER INITIALIZATION & SERVICE
    // =========================================================================

    // Initialize FM17550 over shared Wire bus with receiver gain 43 dB.
    // Disables pointer caching so the driver explicitly addresses registers
    // even when intermixed with line sensor reads on the same bus.
    void initializeNfc()
    {
        NfcAsync::Options options;
        options.clockHz = kI2cClockHz;
        options.gain = 6;              // 43 dB recommended baseline
        options.cachePointer = false;  // Ensure pointer is always asserted on shared bus
        options.logInitialization = false; // Quiet initialization; E10 prints its own debug banner
        options.rfTimeoutUs = 1000;
        options.pollUs = 710;

        nfcRunning = nfcReader.begin(options);
        if (nfcRunning)
        {
            Serial.println("[DEBUG] NFC FM17550 initialized successfully (gain=43dB, cachePointer=false).");
        }
        else
        {
            const bool rfOff = nfcReader.end();
            Serial.printf("[DEBUG] NFC FM17550 initialization failed! (RF off=%s)\n", rfOff ? "yes" : "unconfirmed");
        }
    }

    // Service the asynchronous NFC state machine. Call frequently in loop().
    void serviceNfc(uint64_t nowUs)
    {
        if (!nfcRunning)
            return;

        nfcReader.service(nowUs);

        uint8_t uid[4];
        const NfcAsync::Result result = nfcReader.takeResult(uid);
        if (result != NfcAsync::Result::None)
        {
            if (result == NfcAsync::Result::Uid)
            {
                memcpy(latestUid, uid, 4);
                hasDetectedUid = true;
                lastUidDetectedAtUs = nowUs;
                digitalWrite(LED_BUILTIN, LOW); // LED ON (active-low) on detection
            }
            else if (result == NfcAsync::Result::NoTag)
            {
                digitalWrite(LED_BUILTIN, HIGH); // LED OFF when field is empty
            }
            else // ProtocolError, BusError, Cancelled
            {
                digitalWrite(LED_BUILTIN, HIGH);
                lastNfcError = result;
                ++windowNfcErrors;
                windowLastNfcError = result;

                // On bus fault, attempt a clean restart of the NFC peripheral
                if (result == NfcAsync::Result::BusError)
                {
                    nfcReader.end();
                    NfcAsync::Options options;
                    options.clockHz = kI2cClockHz;
                    options.gain = 6;
                    options.cachePointer = false;
                    options.logInitialization = false;
                    nfcRunning = nfcReader.begin(options);
                }
            }
        }
    }

    // =========================================================================
    // PERIODIC STATUS REPORT (10 Hz / every 0.1 s)
    // =========================================================================

    // Format and output the status line every 0.1 seconds.
    // Output fields:
    // (1) Latest Line sensor position (8-bit binary mask, e.g. 00111100, or --------)
    // (2) Latest Detected NFC UID (11 chars, e.g. 11:E5:99:23, or 11 spaces if no tag in past 10s)
    // (3) Age of last NFC detection in seconds with 1 decimal (e.g.  0.5s, max 10.0s)
    // (4) Main control loop actual rate in Hz (e.g. 100.0 Hz)
    // Padded to ensure constant length. Reading errors, if any, are appended at the end.
    void printStatusReport(uint64_t nowUs)
    {
        // 1. Line sensor 8-bit binary mask (bit 7 = X1 to bit 0 = X8)
        char lineStr[9];
        if (lineReadValid)
        {
            for (int i = 7; i >= 0; --i)
            {
                lineStr[7 - i] = (latestLineMask & (1 << i)) ? '1' : '0';
            }
            lineStr[8] = '\0';
        }
        else
        {
            memcpy(lineStr, "--------", 9);
        }

        // 2 & 3. NFC UID and detection age
        char uidStr[12];
        char ageStr[6];
        bool tagWithinTenSeconds = false;
        float ageSeconds = 10.0f;

        if (hasDetectedUid && lastUidDetectedAtUs > 0)
        {
            const uint64_t elapsedSinceTagUs = nowUs - lastUidDetectedAtUs;
            if (elapsedSinceTagUs < kNfcTagTimeoutUs)
            {
                tagWithinTenSeconds = true;
                ageSeconds = elapsedSinceTagUs / 1000000.0f;
                if (ageSeconds > 10.0f)
                    ageSeconds = 10.0f;
            }
        }

        if (tagWithinTenSeconds)
        {
            snprintf(uidStr, sizeof(uidStr), "%02X:%02X:%02X:%02X",
                     latestUid[0], latestUid[1], latestUid[2], latestUid[3]);
            snprintf(ageStr, sizeof(ageStr), "%4.1fs", ageSeconds);
        }
        else
        {
            // Pad with exactly 11 spaces if no tag detected in the past 10s
            memcpy(uidStr, "           ", 12);
            memcpy(ageStr, "10.0s", 6);
        }

        // 4. Actual control loop rate in Hz
        char rateStr[9];
        snprintf(rateStr, sizeof(rateStr), "%5.1f Hz", measuredControlRateHz);

        // Build the base status message
        char messageBuffer[160];
        const int baseWritten = snprintf(messageBuffer, sizeof(messageBuffer),
                                         "POS: %s | UID: %s | AGE: %s | RATE: %s",
                                         lineStr, uidStr, ageStr, rateStr);

        // Append any reading errors that occurred during the 0.1s window to the end of the line
        int totalWritten = baseWritten;
        if (windowLineTxErrors > 0 || windowLineShortReads > 0 || windowNfcErrors > 0)
        {
            int errOffset = totalWritten;
            totalWritten += snprintf(messageBuffer + totalWritten, sizeof(messageBuffer) - totalWritten, " | ERR:");
            if (windowLineTxErrors > 0)
            {
                totalWritten += snprintf(messageBuffer + totalWritten, sizeof(messageBuffer) - totalWritten,
                                         " LINE_TX=%u(%lu)", windowLastTxError, (unsigned long)windowLineTxErrors);
            }
            if (windowLineShortReads > 0)
            {
                totalWritten += snprintf(messageBuffer + totalWritten, sizeof(messageBuffer) - totalWritten,
                                         " LINE_SHORT(%lu)", (unsigned long)windowLineShortReads);
            }
            if (windowNfcErrors > 0)
            {
                const char *errType = (windowLastNfcError == NfcAsync::Result::BusError) ? "BUS" :
                                      (windowLastNfcError == NfcAsync::Result::ProtocolError) ? "PROTO" : "OTHER";
                totalWritten += snprintf(messageBuffer + totalWritten, sizeof(messageBuffer) - totalWritten,
                                         " NFC_%s(%lu)", errType, (unsigned long)windowNfcErrors);
            }
        }

        // Reset the window error counters for the next 0.1s interval
        windowLineTxErrors = 0;
        windowLineShortReads = 0;
        windowNfcErrors = 0;
        windowLastTxError = 0;
        windowLastNfcError = NfcAsync::Result::None;

        // Append newline and send through guarded serial output
        if (totalWritten < static_cast<int>(sizeof(messageBuffer)) - 1)
        {
            messageBuffer[totalWritten++] = '\n';
            messageBuffer[totalWritten] = '\0';
            serialGuardedWrite(messageBuffer, totalWritten);
        }
    }
} // namespace

// =============================================================================
// SETUP
// =============================================================================

// Boot configuration: initialize Serial, I2C bus, sensors, and print debug messages.
void setup()
{
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0); // Zero timeout prevents blocking on full USB buffer
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Off initially

    delay(200); // Allow serial monitor to connect if already open

    Serial.println("==========================================================");
    Serial.println("  E10 Firmware: Shared I2C Line Sensor & NFC Monitor");
    Serial.println("==========================================================");
    Serial.printf("[DEBUG] Framework: Arduino %s, ESP-IDF %s\n", ESP_ARDUINO_VERSION_STR, ESP.getSdkVersion());
    Serial.printf("[DEBUG] Target Loop Rate: 100 Hz (10 ms period), Status Rate: 10 Hz (0.1 s period)\n");

    // Initialize shared I2C bus on D4 (SDA) and D5 (SCL) at 400 kHz
    if (!Wire.begin(kI2cSdaPin, kI2cSclPin, kI2cClockHz))
    {
        Serial.println("[DEBUG] ERROR: Failed to initialize Wire on D4/D5!");
    }
    else
    {
        Wire.setTimeOut(kI2cTimeoutMs);
        Serial.printf("[DEBUG] Shared Wire I2C bus initialized: SDA=D4, SCL=D5, Clock=%lu Hz, Timeout=%u ms\n",
                      (unsigned long)kI2cClockHz, kI2cTimeoutMs);
    }

    // Initialize Line sensor
    Serial.println("[DEBUG] Initializing Line Sensor at address 0x12...");
    readLineSensor();
    if (lineReadValid)
    {
        Serial.printf("[DEBUG] Line Sensor detected, initial mask: 0x%02X\n", latestLineMask);
    }
    else
    {
        Serial.printf("[DEBUG] Line Sensor read failed (tx_error=%u, short_read=%d)\n",
                      lineTransmitError, lineShortRead);
    }

    // Initialize NFC reader
    Serial.println("[DEBUG] Initializing NFC FM17550 at address 0x28...");
    initializeNfc();

    // Prepare fixed-rate control schedule
    const uint64_t now = esp_timer_get_time();
    nextControlSlotUs = now;
    windowStartedAtUs = now;
    lastStatusPrintUs = now;
    controlCyclesInWindow = 0;

    Serial.println("[DEBUG] Initialization complete. Entering 100 Hz main loop.\n");
}

// =============================================================================
// MAIN LOOP
// =============================================================================

// Main loop coordinates the 100 Hz control cycle, cooperative NFC servicing,
// and 10 Hz status serial reporting.
void loop()
{
    const uint64_t nowUs = esp_timer_get_time();

    // -------------------------------------------------------------------------
    // 1. 100 Hz Main Control Loop Tick
    // -------------------------------------------------------------------------
    if (nowUs >= nextControlSlotUs)
    {
        // Anchor schedule to avoid timing drift
        const uint64_t missedSlots = (nowUs - nextControlSlotUs) / kControlPeriodUs;
        nextControlSlotUs += (missedSlots + 1) * kControlPeriodUs;
        ++controlCyclesInWindow;

        // Read Line Sensor synchronously at the start of each 100 Hz cycle (~250 us)
        readLineSensor();

        // Trigger NFC read if the async reader is currently idle
        if (nfcRunning && !nfcReader.busy())
        {
            nfcReader.startRead();
        }
    }

    // -------------------------------------------------------------------------
    // 2. Cooperative NFC Async State Machine Service
    // -------------------------------------------------------------------------
    // Between 100 Hz ticks, allow NFC state machine to advance its non-blocking steps
    serviceNfc(nowUs);

    // -------------------------------------------------------------------------
    // 3. 10 Hz Status Reporting (Every 0.1 second)
    // -------------------------------------------------------------------------
    if (nowUs - lastStatusPrintUs >= kStatusReportPeriodUs)
    {
        const uint64_t elapsedWindowUs = nowUs - windowStartedAtUs;
        if (elapsedWindowUs > 0)
        {
            measuredControlRateHz = (controlCyclesInWindow * 1000000.0f) / elapsedWindowUs;
        }
        controlCyclesInWindow = 0;
        windowStartedAtUs = nowUs;
        lastStatusPrintUs = nowUs;

        printStatusReport(nowUs);
    }
}
