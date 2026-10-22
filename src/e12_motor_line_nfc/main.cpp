// main.cpp — Main entry point for E12 Dual BLDC Motor, Line Sensor & NFC Reader.
//
// Combines:
//   1. 100 Hz main control loop (10 ms period):
//      - Reads 8-channel line sensor over I2C (address 0x12) synchronously (~250 us).
//      - Calculates line centroid error (-3.5..+3.5) with safety stop on no line (0xFF or 0x00).
//      - Computes differential steering velocity commands based on detected line.
//      - Slows down forward speed on sharp corners / near out-of-bounds line positions.
//      - Calculates instantaneous motor speeds (steps/sec) from CAP encoder interrupts.
//      - Filters speed with Exponential Moving Average (EMA, alpha = 0.24).
//      - Runs dual closed-loop Feedforward + PI speed regulators with anti-windup.
//      - Initiates asynchronous NFC read cycle when idle.
//   2. Cooperative asynchronous NFC servicing:
//      - Advances FM17550 state machine between 100 Hz control ticks.
//      - Tracks last scanned tag UID and age in seconds.
//   3. Non-blocking USB Serial command parser:
//      - S<mm/s> : Sets base closed-loop line following speed in mm/sec (-120.0..+120.0, clamped).
//      - P<val>  : Sets open-loop PWM for both motors (-255..+255).
//      - L<val>  : Sets open-loop PWM for Left motor only (-255..+255).
//      - R<val>  : Sets open-loop PWM for Right motor only (-255..+255).
//      - STOP    : Immediately stops motors and resets speed controller state.
//      - STATUS  : Prints diagnostic overview.
//      - HELP    : Prints command summary.
//   4. 0.1 s (10 Hz) periodic telemetry output:
//      - Includes PWM value, Left speed (mm/s, integer), Right speed (mm/s, integer),
//        Line 8-bit binary mask (01010101), NFC tag UID (last scanned within 1s or NONE),
//        last tag age in seconds with 1 decimal point, and main control loop rate in Hz.

#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include "motor_control.h"
#include "../e8_nfc_fm17550_i2c/NfcAsync.h"

namespace
{
    // ── Timing State & Loop Rate Tracking ────────────────────────────
    static uint64_t nextControlSlotUs      = 0; // Timestamp of next 100 Hz control loop tick
    static uint64_t lastReportUs            = 0; // Timestamp of previous 0.1s status print
    static uint64_t windowStartedAtUs       = 0; // Start timestamp for rate calculation window
    static uint32_t controlCyclesInWindow   = 0; // Number of control loop cycles executed in window
    static float measuredControlRateHz      = 0.0f; // Measured main loop rate in Hz

    // ── Line Sensor & Steering State ─────────────────────────────────
    static uint8_t latestLineMask = 0xFF;  // Active-low: 0=black line, 1=white background
    static uint64_t lastLineDetectedAtMs = 0; // Timestamp of the most recent time a line was detected
    static const uint64_t lostLineTimeoutMs = 800; // Time to retain last valid line reading after losing the line
    static uint8_t previousLineMask = 0xFF;
    static bool lineReadValid     = false;
    static uint8_t lineTxError    = 0;
    static bool lineShortRead     = false;

    static float commandedBaseSpeedMmS = 0.0f; // Desired base forward speed commanded via 'S'
    static float prevNormError         = 0.0f; // Error history for derivative steering term
    static bool lineFollowerActive     = false; // True when car is actively line-following

    // ── Asynchronous NFC State ───────────────────────────────────────
    static NfcAsync nfcReader;
    static bool nfcRunning = false;
    static uint8_t latestNfcUid[4] = {};
    static bool hasScannedUid = false;
    static uint64_t lastUidScannedAtUs = 0; // Timestamp of the most recent successful UID read

    // ── Serial Command Input Buffer ──────────────────────────────────
    static const size_t CMD_BUF_CAPACITY = 64;
    static char cmdBuffer[CMD_BUF_CAPACITY];
    static size_t cmdLength = 0;

    // ── Non-Drained Serial Output Guard ──────────────────────────────
    static uint32_t serialDroppedLines = 0;

    /**
     * @brief Writes buffer to Serial only if USB is connected and buffer has space.
     * Prevents stalling or blocking the 100 Hz control loop when serial port is closed.
     */
    void serialGuardedWrite(const char *buffer, size_t length)
    {
        if (!Serial)
        {
            ++serialDroppedLines;
            return;
        }

        if (Serial.availableForWrite() < static_cast<int>(length))
        {
            ++serialDroppedLines;
            return;
        }

        Serial.write(reinterpret_cast<const uint8_t *>(buffer), length);
    }

    // ── Synchronous Line Sensor Reader ───────────────────────────────

    /**
     * @brief Reads 8-channel line sensor register 0x30 over I2C address 0x12.
     * Takes ~250 us with repeated-START at 400 kHz clock.
     */
    void readLineSensor()
    {
        Wire.beginTransmission(LINE_SENSOR_I2C_ADDR);
        Wire.write(LINE_SENSOR_REG_DATA);
        const uint8_t txErr = Wire.endTransmission(false);
        if (txErr != 0)
        {
            lineTxError = txErr;
            lineReadValid = false;
            return;
        }

        const size_t received = Wire.requestFrom(static_cast<uint8_t>(LINE_SENSOR_I2C_ADDR), static_cast<size_t>(1), true);
        if (received == 1 && Wire.available() == 1)
        {
            latestLineMask = static_cast<uint8_t>(Wire.read());
            lineReadValid = true;
            lineTxError = 0;
            lineShortRead = false;
        }
        else
        {
            lineShortRead = true;
            lineReadValid = false;
            while (Wire.available())
            {
                Wire.read();
            }
        }
    }

    // ── Line Following Controller ────────────────────────────────────

    /**
     * @brief Updates differential steering velocities based on line sensor reading.
     * 
     * Line Sensor Bit Convention (Active-low, 0 = black line detected):
     *   Bit 7 = Leftmost sensor (X1)   -> Weight +3.5
     *   Bit 6 = Sensor X2              -> Weight +2.5
     *   Bit 5 = Sensor X3              -> Weight +1.5
     *   Bit 4 = Center-left sensor X4  -> Weight +0.5
     *   Bit 3 = Center-right sensor X5 -> Weight -0.5
     *   Bit 2 = Sensor X6              -> Weight -1.5
     *   Bit 1 = Sensor X7              -> Weight -2.5
     *   Bit 0 = Rightmost sensor (X8)  -> Weight -3.5
     * 
     * Coordinate conventions:
     *   Line Centered: 11100111 -> Error = 0.0
     *   Veered Right:  10011111 (black line under left sensors, bit 6/5)
     *                  Car is too far right -> normError > 0 -> steer LEFT:
     *                  Outer wheel = Right (vRight = vBase > 0)
     *                  Inner wheel = Left  (vLeft = vBase * (1 - steerMag) < 0 in sharp turns)
     *   Veered Left:   11111001 (black line under right sensors, bit 2/1)
     *                  Car is too far left -> normError < 0 -> steer RIGHT:
     *                  Outer wheel = Left  (vLeft = vBase > 0)
     *                  Inner wheel = Right (vRight = vBase * (1 - steerMag) < 0 in sharp turns)
     * 
     * Inner Wheel Reverse Limit Constraint:
     *   The inner wheel is allowed to drive in reverse for fast pivoting, but its reverse
     *   speed must not exceed the other wheel's forward speed (vInner >= -vOuter).
     *   This bounds minimum net forward speed to zero ((vOuter + vInner) / 2 >= 0) and
     *   produces an in-place zero-radius spin turn on extreme curves without moving backwards.
     * 
     * Safety Stop:
     *   If mask is all 1s (0xFF, line lost) or all 0s (0x00, off surface/error),
     *   motors are immediately stopped.
     */
    void updateLineFollowing(float dt)
    {
        if (!lineFollowerActive || fabsf(commandedBaseSpeedMmS) < 0.1f)
        {
            // Line follower inactive or zero speed commanded
            return;
        }
        // Safety Stop: No line detected (all 1s) or invalid read / off surface (all 0s)
        if (!lineReadValid || latestLineMask == 0xFF || latestLineMask == 0x00)
        {
            // Retain a lost line for a small period of time to allow recovery from brief occlusions
            if ((millis() - lastLineDetectedAtMs) > lostLineTimeoutMs)
            {
                setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
                prevNormError = 0.0f;
                return;
            }
            latestLineMask = previousLineMask; // Use last valid line reading for steering
        }else
        {
            lastLineDetectedAtMs = millis();
            previousLineMask = latestLineMask;
        }

        // Calculate weighted line centroid
        // Weights: bit 7 = +3.5, bit 6 = +2.5, bit 5 = +1.5, bit 4 = +0.5,
        //          bit 3 = -0.5, bit 2 = -1.5, bit 1 = -2.5, bit 0 = -3.5
        static const float SENSOR_WEIGHTS[8] = {
            -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f
        };

        float weightedSum = 0.0f;
        int activeSensorCount = 0;

        for (int i = 0; i < 8; i++)
        {
            // Active-low: 0 means black line detected
            if ((latestLineMask & (1 << i)) == 0)
            {
                weightedSum += SENSOR_WEIGHTS[i];
                activeSensorCount++;
            }
        }

        if (activeSensorCount == 0)
        {
            // Safety fallback: no active black sensors
            setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
            prevNormError = 0.0f;
            return;
        }

        // Normalized line error in [-1.0, +1.0]:
        // Error > 0: line is to the left (car veered right) -> steer LEFT
        // Error < 0: line is to the right (car veered left) -> steer RIGHT
        const float rawError  = weightedSum / (float)activeSensorCount;
        const float normError = rawError / 3.5f;

        // Derivative term: rate of change of normalized line error
        const float dError = (dt > 0.0001f) ? (normError - prevNormError) / dt : 0.0f;
        prevNormError = normError;

        // Steering demand signal (positive = steer LEFT, negative = steer RIGHT)
        const float steerSignal = (LINE_STEER_KP * normError) + (LINE_STEER_KD * dError);

        float vLeft  = commandedBaseSpeedMmS;
        float vRight = commandedBaseSpeedMmS;

        if (steerSignal > 0.001f)
        {
            // Steer LEFT: Outer wheel is Right (+), Inner wheel is Left
            const float steerMag = steerSignal;
            const float vOuter   = commandedBaseSpeedMmS;
            float vInner         = commandedBaseSpeedMmS * (1.0f - steerMag);

            // Inner wheel reverse constraint: allow inner wheel to reverse for aggressive
            // pivoting, but clamp so it does not exceed the negative of outer wheel speed
            if (commandedBaseSpeedMmS > 0.0f)
            {
                if (vInner < -vOuter)
                {
                    vInner = -vOuter;
                }
            }
            else
            {
                if (vInner > -vOuter)
                {
                    vInner = -vOuter;
                }
            }

            vLeft  = vInner;
            vRight = vOuter;
        }
        else if (steerSignal < -0.001f)
        {
            // Steer RIGHT: Outer wheel is Left (+), Inner wheel is Right
            const float steerMag = -steerSignal;
            const float vOuter   = commandedBaseSpeedMmS;
            float vInner         = commandedBaseSpeedMmS * (1.0f - steerMag);

            // Inner wheel reverse constraint: allow inner wheel to reverse for aggressive
            // pivoting, but clamp so it does not exceed the negative of outer wheel speed
            if (commandedBaseSpeedMmS > 0.0f)
            {
                if (vInner < -vOuter)
                {
                    vInner = -vOuter;
                }
            }
            else
            {
                if (vInner > -vOuter)
                {
                    vInner = -vOuter;
                }
            }

            vLeft  = vOuter;
            vRight = vInner;
        }

        // Apply individual speed targets to closed-loop PI controllers
        setIndividualTargetSpeedsMmPerSec(vLeft, vRight);
    }

    // ── Asynchronous NFC Initializer & Service ───────────────────────

    /**
     * @brief Initializes FM17550 NFC chip over shared I2C bus.
     */
    void initializeNfc()
    {
        NfcAsync::Options options;
        options.clockHz = I2C_CLOCK_HZ;
        options.gain = 3;                  // 23 dB receiver gain
        options.cachePointer = false;      // Ensure pointer asserted on shared bus with line sensor
        options.logInitialization = false;
        options.rfTimeoutUs = 1000;
        options.pollUs = 710;

        nfcRunning = nfcReader.begin(options);
        if (nfcRunning)
        {
            Serial.println("# [DEBUG] NFC FM17550 initialized successfully (gain=23dB).");
        }
        else
        {
            nfcReader.end();
            Serial.println("# [DEBUG] NFC FM17550 initialization failed or not responding.");
        }
    }

    /**
     * @brief Services the asynchronous NFC state machine. Called continuously in loop().
     */
    void serviceNfc(uint64_t nowUs)
    {
        if (!nfcRunning) return;

        nfcReader.service(nowUs);

        uint8_t uid[4];
        const NfcAsync::Result res = nfcReader.takeResult(uid);
        if (res != NfcAsync::Result::None)
        {
            if (res == NfcAsync::Result::Uid)
            {
                memcpy(latestNfcUid, uid, 4);
                hasScannedUid = true;
                lastUidScannedAtUs = nowUs;
            }
            else if (res == NfcAsync::Result::BusError)
            {
                // On I2C bus error, reinitialize NFC cleanly
                nfcReader.end();
                NfcAsync::Options options;
                options.clockHz = I2C_CLOCK_HZ;
                options.gain = 3;
                options.cachePointer = false;
                options.logInitialization = false;
                nfcRunning = nfcReader.begin(options);
            }
        }
    }

    // ── Periodic Telemetry Report (10 Hz / every 0.1 s) ───────────────

    /**
     * @brief Formats and emits telemetry every 0.1 seconds (10 Hz).
     * Output fields:
     *   - Commanded/active PWM (integer)
     *   - Left Speed in mm/s (no decimal, rounded integer)
     *   - Right Speed in mm/s (no decimal, rounded integer)
     *   - Line 8-bit binary mask (01010101, bit 7 to bit 0)
     *   - NFC Tag UID last scanned (within 1 sec, e.g. 11:E5:99:23, or NONE if older/no tag)
     *   - Last tag age in seconds with 1 decimal point (e.g. 0.3s, or 99.9s if no tag yet)
     * 
     * Example format:
     *   P=150,L=98,R=97,LINE=11111111,NFC=11:E5:99:23,AGE=0.2s
     */
    void printTelemetry(uint64_t nowUs)
    {
        // 1. Commanded / average PWM
        int pwmVal = getCommandedPWM();

        // 2 & 3. Left and Right speed in mm/sec rounded to nearest integer (no decimal)
        int leftSpeedMmS  = (int)roundf(getLeftMotorSpeedMmS());
        int rightSpeedMmS = (int)roundf(getRightMotorSpeedMmS());

        // 4. Line sensor 8-bit binary mask (bit 7 = X1 down to bit 0 = X8)
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

        // 5 & 6. NFC Tag UID (last scanned within 1 sec) and age in seconds (1 decimal)
        char nfcStr[16];
        char ageStr[10];

        if (hasScannedUid && lastUidScannedAtUs > 0)
        {
            uint64_t elapsedUs = nowUs - lastUidScannedAtUs;
            float ageSec = (float)elapsedUs / 1000000.0f;

            if (elapsedUs <= NFC_TAG_ACTIVE_TIMEOUT_US)
            {
                // Active tag within 1.0 second
                snprintf(nfcStr, sizeof(nfcStr), "%02X:%02X:%02X:%02X",
                         latestNfcUid[0], latestNfcUid[1], latestNfcUid[2], latestNfcUid[3]);
            }
            else
            {
                // Tag was scanned previously, but older than 1.0 second
                snprintf(nfcStr, sizeof(nfcStr), "NONE");
            }

            // Cap age display at 99.9s
            if (ageSec > 99.9f) ageSec = 99.9f;
            snprintf(ageStr, sizeof(ageStr), "%.1fs", ageSec);
        }
        else
        {
            // No tag scanned yet
            snprintf(nfcStr, sizeof(nfcStr), "NONE");
            snprintf(ageStr, sizeof(ageStr), "--.-s");
        }

        // Format telemetry line
        char outBuf[160];
        int len = snprintf(outBuf, sizeof(outBuf),
                           "P=%d,L=%d,R=%d,LINE=%s,NFC=%s,AGE=%s,RATE=%.1fHz\n",
                           pwmVal, leftSpeedMmS, rightSpeedMmS, lineStr, nfcStr, ageStr, measuredControlRateHz);

        if (len > 0 && len < static_cast<int>(sizeof(outBuf)))
        {
            serialGuardedWrite(outBuf, (size_t)len);
        }
    }

    // ── Serial Command Handler ───────────────────────────────────────

    /**
     * @brief Parses incoming serial command strings.
     */
    void processCommand(char *line)
    {
        while (*line == ' ' || *line == '\t') line++;
        if (*line == '\0') return;

        // Check for 'P' command: P<val> (sets PWM for both motors)
        if (line[0] == 'P' || line[0] == 'p')
        {
            char *valStr = line + 1;
            while (*valStr == ' ' || *valStr == '\t') valStr++;
            int val = atoi(valStr);
            lineFollowerActive = false;
            setBothMotorsPWM(val);
            Serial.printf("# OK P=%d\n", getCommandedPWM());
            return;
        }

        // Check for 'L' command: L<val> (sets Left motor)
        if ((line[0] == 'L' || line[0] == 'l') && line[1] != 'I' && line[1] != 'i' && line[1] != 'O' && line[1] != 'o')
        {
            char *valStr = line + 1;
            while (*valStr == ' ' || *valStr == '\t') valStr++;
            int val = atoi(valStr);
            lineFollowerActive = false;
            setSingleMotorPWM(MOTOR_LEFT, val);
            Serial.printf("# OK L=%d\n", motorStates[MOTOR_LEFT].pwmValue);
            return;
        }

        // Check for 'R' command: R<val> (sets Right motor)
        if (line[0] == 'R' || line[0] == 'r')
        {
            char *valStr = line + 1;
            while (*valStr == ' ' || *valStr == '\t') valStr++;
            int val = atoi(valStr);
            lineFollowerActive = false;
            setSingleMotorPWM(MOTOR_RIGHT, val);
            Serial.printf("# OK R=%d\n", motorStates[MOTOR_RIGHT].pwmValue);
            return;
        }

        // Check for STOP command
        if (strcasecmp(line, "STOP") == 0)
        {
            lineFollowerActive     = false;
            commandedBaseSpeedMmS  = 0.0f;
            stopAllMotors();
            Serial.println("# OK STOPPED");
            return;
        }

        // Diagnostic STATUS command
        if (strcasecmp(line, "STATUS") == 0)
        {
            noInterrupts();
            unsigned long lPulses = leftPulseCount;
            unsigned long rPulses = rightPulseCount;
            interrupts();
            Serial.printf("# STATUS: Mode=%s, LineFollower=%s, BaseSpeed=%.1f mm/s, TargetSpeed=%.1f mm/s, P=%d, "
                          "LeftPWM=%d, RightPWM=%d, L_spd=%.1f stp/s (%d mm/s), R_spd=%.1f stp/s (%d mm/s), "
                          "LineMask=0x%02X (%s), NFC=%s, Rate=%.1fHz, LeftPulses=%lu, RightPulses=%lu\n",
                          (getControlMode() == MODE_CLOSED_LOOP_SPEED) ? "SPEED" : "PWM",
                          lineFollowerActive ? "ACTIVE" : "INACTIVE",
                          commandedBaseSpeedMmS,
                          getTargetSpeedMmPerSec(),
                          getCommandedPWM(),
                          motorStates[MOTOR_LEFT].pwmValue,
                          motorStates[MOTOR_RIGHT].pwmValue,
                          getLeftMotorSpeed(), (int)roundf(getLeftMotorSpeedMmS()),
                          getRightMotorSpeed(), (int)roundf(getRightMotorSpeedMmS()),
                          latestLineMask, lineReadValid ? "VALID" : "INVALID",
                          nfcRunning ? "RUNNING" : "STOPPED",
                          measuredControlRateHz,
                          lPulses, rPulses);
            return;
        }

        // Check for 'S' speed command: S<mm/s> (e.g. S15, S30, S100, S-50, S0)
        // Engages closed-loop line following at base speed targetMmS.
        // Out-of-range values are clamped to [-120.0, +120.0] mm/s.
        // A bare 'S' or 's' without parameters acts as STOP.
        if (line[0] == 'S' || line[0] == 's')
        {
            char *valStr = line + 1;
            while (*valStr == ' ' || *valStr == '\t') valStr++;
            if (*valStr == '\0')
            {
                lineFollowerActive    = false;
                commandedBaseSpeedMmS = 0.0f;
                stopAllMotors();
                Serial.println("# OK STOPPED");
                return;
            }

            float targetMmS = atof(valStr);
            if (targetMmS > SPEED_CMD_MAX_MM_S) targetMmS = SPEED_CMD_MAX_MM_S;
            if (targetMmS < SPEED_CMD_MIN_MM_S) targetMmS = SPEED_CMD_MIN_MM_S;

            if (fabsf(targetMmS) < 0.1f)
            {
                lineFollowerActive    = false;
                commandedBaseSpeedMmS = 0.0f;
                stopAllMotors();
                Serial.println("# OK S=0.0 mm/s (STOPPED)");
            }
            else
            {
                commandedBaseSpeedMmS = targetMmS;
                lineFollowerActive    = true;
                prevNormError         = 0.0f;
                // Pre-seed closed loop speed targets with base speed
                setIndividualTargetSpeedsMmPerSec(targetMmS, targetMmS);
                Serial.printf("# OK S=%.1f mm/s (Line Following Active)\n", targetMmS);
            }
            return;
        }

        // Help command
        if (strcasecmp(line, "HELP") == 0)
        {
            Serial.println("# Commands: S<mm/s> (speed -120..+120), P<val> (PWM -255..+255), L<val>, R<val>, STOP, STATUS, HELP");
            return;
        }

        Serial.printf("# ERR Unknown command: '%s'\n", line);
    }

    /**
     * @brief Reads non-blocking characters from USB Serial.
     */
    void pollSerialCommands()
    {
        while (Serial.available())
        {
            char c = (char)Serial.read();
            if (c == '\r' || c == '\n')
            {
                if (cmdLength > 0)
                {
                    cmdBuffer[cmdLength] = '\0';
                    processCommand(cmdBuffer);
                    cmdLength = 0;
                }
            }
            else if (cmdLength < CMD_BUF_CAPACITY - 1)
            {
                cmdBuffer[cmdLength++] = c;
            }
        }
    }
} // namespace

// ── Arduino Setup ────────────────────────────────────────────────────

void setup()
{
    // Initialize USB CDC Serial at 115200 baud
    Serial.begin(115200);
    delay(1000); // 400 min to run properly, 1000 delay to see Serial on boot
    Serial.setTxTimeoutMs(0); // Zero timeout prevents blocking on full buffer

    Serial.println();
    Serial.println("==========================================================");
    Serial.println("  E12 Firmware: Dual BLDC Motors, Line Sensor & NFC Reader");
    Serial.println("==========================================================");

    // Initialize motor hardware (LEDC PWM, GPIO Direction, CAP Interrupts)
    setupMotors();

    // Initialize shared I2C bus on D4 (SDA) and D5 (SCL) at 400 kHz
    if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ))
    {
        Serial.println("# [ERROR] Failed to initialize Wire I2C bus on D4/D5!");
    }
    else
    {
        Wire.setTimeOut(I2C_TIMEOUT_MS);
        Serial.printf("# [DEBUG] Shared Wire I2C bus initialized (400 kHz, timeout %u ms).\n", I2C_TIMEOUT_MS);
    }

    // Initialize Line Sensor
    Serial.println("# [DEBUG] Probing Line Sensor at 0x12...");
    readLineSensor();
    if (lineReadValid)
    {
        Serial.printf("# [DEBUG] Line Sensor detected (initial mask: 0x%02X).\n", latestLineMask);
    }
    else
    {
        Serial.printf("# [DEBUG] Line Sensor not responding (txErr=%u, shortRead=%d).\n", lineTxError, lineShortRead);
    }

    // Initialize NFC Reader
    Serial.println("# [DEBUG] Initializing NFC FM17550 at 0x28...");
    initializeNfc();

    // Initialize 100 Hz control loop and 0.1s reporting schedule
    const uint64_t nowUs = esp_timer_get_time();
    nextControlSlotUs    = nowUs + CONTROL_INTERVAL_US;
    lastReportUs         = nowUs;
    windowStartedAtUs    = nowUs;
    controlCyclesInWindow = 0;
    measuredControlRateHz = 100.0f;

    Serial.println("# Commands: S<mm/s> (speed), P<val> (PWM), STOP, STATUS, HELP");
    Serial.println("# Telemetry: P=<pwm>,L=<mm/s>,R=<mm/s>,LINE=<bits>,NFC=<uid|NONE>,AGE=<sec>,RATE=<Hz> @ 10 Hz");
    Serial.println("# Ready");

    // Initialize robot to run at default speed
    lineFollowerActive    = true;
    commandedBaseSpeedMmS = 100.0f;
    Serial.printf("# Initialized Speed at S=100.0 mm/s (Line Following Active)\n");
}

// ── Arduino Main Loop ────────────────────────────────────────────────

void loop()
{
    const uint64_t nowUs = esp_timer_get_time();

    // ── 1. Poll incoming serial commands ─────────────────────────────
    pollSerialCommands();

    // ── 2. Service 100 Hz Control Loop (every 10 ms) ─────────────────
    if (nowUs >= nextControlSlotUs)
    {
        uint64_t missedSlots = (nowUs - nextControlSlotUs) / CONTROL_INTERVAL_US;
        nextControlSlotUs += (missedSlots + 1) * CONTROL_INTERVAL_US;
        ++controlCyclesInWindow;

        // Synchronous line sensor read at the start of each 10 ms tick (~250 us)
        readLineSensor();

        // Compute differential line following steering commands
        updateLineFollowing(0.010f);

        // Service motor control (pulse snapshot, EMA filter, PI+FF speed regulation)
        updateMotorControlLoop(nowUs);

        // Trigger NFC read if async reader is currently idle
        if (nfcRunning && !nfcReader.busy())
        {
            nfcReader.startRead();
        }
    }

    // ── 3. Cooperative Asynchronous NFC State Machine Service ─────────
    serviceNfc(nowUs);

    // ── 4. Telemetry Output Every 0.1 Seconds (10 Hz) ─────────────────
    if (nowUs - lastReportUs >= REPORT_INTERVAL_US)
    {
        const uint64_t elapsedWindowUs = nowUs - windowStartedAtUs;
        if (elapsedWindowUs > 0)
        {
            measuredControlRateHz = ((float)controlCyclesInWindow * 1000000.0f) / (float)elapsedWindowUs;
        }
        controlCyclesInWindow = 0;
        windowStartedAtUs     = nowUs;
        lastReportUs          = nowUs;

        printTelemetry(nowUs);
    }
}
