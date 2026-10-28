// main.cpp — E13 BLE-enabled dual BLDC motor, line sensor, and NFC controller.
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
//      - S<mm/s> : Selects AUTO and sets its speed target (-120.0..+120.0).
//      - S / STOP: Selects STOPPED and zeros both wheel targets.
//      - STATUS  : Prints diagnostic overview.
//      - HELP    : Prints command summary.
//   4. BLE World State reception and 20 Hz Robot State telemetry.
//   5. 0.1 s (10 Hz) development telemetry output:
//      - Includes PWM value, Left speed (mm/s, integer), Right speed (mm/s, integer),
//        Line 8-bit binary mask (01010101), NFC tag UID (last scanned within 1s or NONE),
//        last tag age in seconds with 1 decimal point, and main control loop rate in Hz.

#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <cstring>
#include "ble_telemetry.h"
#include "motor_control.h"
#include "../../e8_nfc_fm17550_i2c/NfcAsync.h"
#include <ble14_watchdog.h>

#ifndef BLE_CAR_ID
#define BLE_CAR_ID 1
#endif

namespace
{
    constexpr uint8_t PIN_BATTERY_MONITOR = D10;
    constexpr uint32_t kBleTelemetryPeriodMs = 50;
    constexpr uint32_t kWorldStateTimeoutMs = 500;
    constexpr float kAutoRunSpeedMmS = 100.0f;
    constexpr float kManualSpeedMmS = 50.0f;
    constexpr float kManualTurnMagnitude = 0.8f;
    constexpr float kBatteryEmaAlpha = 0.02f;
    constexpr float kLowBatteryEnterMv = 9500.0f;
    constexpr float kLowBatteryRecoverMv = 10000.0f;
    constexpr uint32_t kBatteryDwellMs = 5000;
    constexpr uint32_t kLineLostGraceMs = 1500;
    constexpr bool kAutoStartOnBoot = false;
    // false: black line on white background; true: white line on black background.
    constexpr bool kLineIsWhiteOnBlack = true;

    enum ControllerState : uint8_t
    {
        AUTO = 0,
        MANUAL = 1,
        LOW_BATT = 14,
        STOPPED = 15
    };

    enum LineFollowerState : uint8_t
    {
        FOLLOW_LINE = 0,
        BRANCHING_CENTER = 4,
        BRANCHING_LEFT = 5,
        BRANCHING_RIGHT = 6,
        LINE_LOST = 10
    };

    // Edge-following error tables, indexed by sensor position (0 = leftmost, 7 = rightmost)
    // of the outermost line edge. Positive steers LEFT, negative steers RIGHT, zero goes straight.
    // Left-edge tables are indexed by the first line sensor from the left; right-edge tables
    // by the last line sensor. Edit freely to tune non-linear turning.
    constexpr float kLeftEdgeErrorBySensor[8]  = {+5.0f, +3.0f, +1.0f, 0.0f, -1.0f, -2.0f, -4.0f, -5.0f};
    constexpr float kRightEdgeErrorBySensor[8] = {+5.0f, +4.0f, +2.0f, +1.0f, 0.0f, -1.0f, -3.0f, -5.0f};

    static BleTelemetry bleTelemetry;
    static ControllerState controllerState = STOPPED;
    static LineFollowerState lineFollowerState = FOLLOW_LINE;
    static LineFollowerState savedLineFollowerState = FOLLOW_LINE;
    static float autoRunTargetMmS = 0.0f;
    static uint8_t latestTurnIndicators = 0;
    static float batteryFilteredMv = 0.0f;
    static bool batteryFilterInitialized = false;
    static uint32_t batteryBelowSinceMs = 0;
    static uint32_t batteryRecoveredSinceMs = 0;
    static uint32_t lastWorldStateMs = 0;
    static bool worldStateTimeoutApplied = false;
    static bool worldCommandSource = true;

    // ── Timing State & Loop Rate Tracking ────────────────────────────
    static uint64_t nextControlSlotUs      = 0; // Timestamp of next 100 Hz control loop tick
    static uint64_t lastReportUs            = 0; // Timestamp of previous 0.1s status print
    static uint64_t windowStartedAtUs       = 0; // Start timestamp for rate calculation window
    static uint32_t controlCyclesInWindow   = 0; // Number of control loop cycles executed in window
    static float measuredControlRateHz      = 0.0f; // Measured main loop rate in Hz

    // ── Line Sensor & Steering State ─────────────────────────────────
    static uint8_t latestLineMask = 0xFF;  // Active-low: 0=black line, 1=white background
    static uint64_t lastLineDetectedAtMs = 0; // Timestamp of the most recent time a line was detected
    static uint8_t previousLineMask = 0xFF;
    static bool lineReadValid     = false;
    static uint8_t lineTxError    = 0;
    static bool lineShortRead     = false;

    static float commandedBaseSpeedMmS = 0.0f; // Desired base forward speed commanded via 'S'
    static float prevNormError         = 0.0f; // Error history for derivative steering term
    static float manualLeftTargetMmS = 0.0f;
    static float manualRightTargetMmS = 0.0f;

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
            // Flip raw bits for white line on black background so the rest of
            // the logic keeps its active-low (0 = line) convention.
            if (kLineIsWhiteOnBlack) latestLineMask = ~latestLineMask;
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

    // Samples the battery divider once per control tick and applies the configured EMA.
    void sampleBatteryVoltage()
    {
        const uint16_t adc = analogRead(PIN_BATTERY_MONITOR);
        const float dividerMv = (float(adc) * 3300.0f) / 4095.0f;
        const float batteryMv = dividerMv * (122.0f / 22.0f);
        if (!batteryFilterInitialized)
        {
            batteryFilteredMv = batteryMv;
            batteryFilterInitialized = true;
        }
        else
        {
            batteryFilteredMv += kBatteryEmaAlpha * (batteryMv - batteryFilteredMv);
        }
    }

    // Applies battery thresholds and dwell times, overriding movement when unsafe.
    void updateBatterySafety(uint32_t nowMs)
    {
        if (controllerState != LOW_BATT)
        {
            batteryRecoveredSinceMs = 0;
            if (batteryFilteredMv < kLowBatteryEnterMv)
            {
                if (batteryBelowSinceMs == 0) batteryBelowSinceMs = nowMs;
                if (nowMs-batteryBelowSinceMs >= kBatteryDwellMs)
                {
                    controllerState = LOW_BATT;
                    autoRunTargetMmS = 0.0f;
                    manualLeftTargetMmS = 0.0f;
                    manualRightTargetMmS = 0.0f;
                    setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
                }
            }
            else
            {
                batteryBelowSinceMs = 0;
            }
            return;
        }

        if (batteryFilteredMv > kLowBatteryRecoverMv)
        {
            if (batteryRecoveredSinceMs == 0) batteryRecoveredSinceMs = nowMs;
            if (nowMs-batteryRecoveredSinceMs >= kBatteryDwellMs)
            {
                controllerState = AUTO;
                autoRunTargetMmS = 0.0f;
                commandedBaseSpeedMmS = 0.0f;
                manualLeftTargetMmS = 0.0f;
                manualRightTargetMmS = 0.0f;
                worldCommandSource = false;
                worldStateTimeoutApplied = true;
                setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
                batteryBelowSinceMs = 0;
                batteryRecoveredSinceMs = 0;
            }
        }
        else
        {
            batteryRecoveredSinceMs = 0;
        }
    }

    // Selects the line-following mode from the turn bits: none = FOLLOW_LINE, left only =
    // BRANCHING_LEFT, right only = BRANCHING_RIGHT, both = BRANCHING_CENTER.
    // While LINE_LOST, only the saved state changes so recovery resumes the newest selection.
    void selectLineFollowerStateFromTurnBits(uint8_t turnBits)
    {
        const bool left = (turnBits & 0x08) != 0;
        const bool right = (turnBits & 0x10) != 0;
        const LineFollowerState selected = (left && right) ? BRANCHING_CENTER
            : left ? BRANCHING_LEFT : right ? BRANCHING_RIGHT : FOLLOW_LINE;
        LineFollowerState &target = (lineFollowerState == LINE_LOST) ? savedLineFollowerState : lineFollowerState;
        if (target != selected)
        {
            target = selected;
            prevNormError = 0.0f; // avoid a derivative spike when the error definition changes
        }
    }

    // Converts a movement byte into controller mode and the corresponding wheel targets.
    void applyWorldMovement(uint8_t movement)
    {
        latestTurnIndicators = movement & 0x18;
        worldCommandSource = true;
        worldStateTimeoutApplied = false;
        if (controllerState == LOW_BATT) return;

        if (movement & 0x01)
        {
            controllerState = AUTO;
            selectLineFollowerStateFromTurnBits(movement);
            autoRunTargetMmS = (movement & 0x02) ? kAutoRunSpeedMmS : 0.0f;
            commandedBaseSpeedMmS = autoRunTargetMmS;
            manualLeftTargetMmS = 0.0f;
            manualRightTargetMmS = 0.0f;
            if (lineFollowerState == LINE_LOST || autoRunTargetMmS == 0.0f)
                setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
            return;
        }

        controllerState = MANUAL;
        commandedBaseSpeedMmS = 0.0f;
        const bool forward = (movement & 0x02) != 0;
        const bool backward = (movement & 0x04) != 0;
        const bool left = (movement & 0x08) != 0;
        const bool right = (movement & 0x10) != 0;
        const float direction = forward == backward ? 0.0f : (forward ? 1.0f : -1.0f);

        if (forward && backward)
        {
            manualLeftTargetMmS = 0.0f;
            manualRightTargetMmS = 0.0f;
        }
        else if (direction != 0.0f && (left || right))
        {
            const bool turnRight = right;
            const float inner = kManualSpeedMmS * (1.0f-kManualTurnMagnitude) * direction;
            const float outer = kManualSpeedMmS * direction;
            manualLeftTargetMmS = turnRight ? outer : inner;
            manualRightTargetMmS = turnRight ? inner : outer;
        }
        else if (direction != 0.0f)
        {
            manualLeftTargetMmS = kManualSpeedMmS * direction;
            manualRightTargetMmS = kManualSpeedMmS * direction;
        }
        else if (left || right)
        {
            manualLeftTargetMmS = right ? kManualSpeedMmS : -kManualSpeedMmS;
            manualRightTargetMmS = -manualLeftTargetMmS;
        }
        else
        {
            manualLeftTargetMmS = 0.0f;
            manualRightTargetMmS = 0.0f;
        }
        setIndividualTargetSpeedsMmPerSec(manualLeftTargetMmS, manualRightTargetMmS);
    }

    // Applies the 500 ms World State failsafe without changing the selected mode.
    void applyWorldTimeout(uint32_t nowMs)
    {
        if (!worldCommandSource || worldStateTimeoutApplied ||
            nowMs-lastWorldStateMs < kWorldStateTimeoutMs || controllerState == LOW_BATT)
            return;
        autoRunTargetMmS = 0.0f;
        commandedBaseSpeedMmS = 0.0f;
        manualLeftTargetMmS = 0.0f;
        manualRightTargetMmS = 0.0f;
        latestTurnIndicators = 0;
        if (controllerState == AUTO) selectLineFollowerStateFromTurnBits(0);
        worldStateTimeoutApplied = true;
        setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
    }

    // Consumes the newest BLE command and enforces the command-age failsafe.
    void serviceWorldState(uint32_t nowMs)
    {
        Ble14::World world{};
        if (bleTelemetry.takeWorldState(world))
        {
            lastWorldStateMs = nowMs;
            applyWorldMovement(world.movement[BLE_CAR_ID-1]);
        }
        applyWorldTimeout(nowMs);
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
        const uint32_t nowMs = millis();
        const bool lineDetected = lineReadValid && latestLineMask != 0xFF && latestLineMask != 0x00;
        if (lineDetected)
        {
            lastLineDetectedAtMs = nowMs;
            previousLineMask = latestLineMask;
            if (lineFollowerState == LINE_LOST)
                lineFollowerState = savedLineFollowerState;
        }
        else if (nowMs-lastLineDetectedAtMs > kLineLostGraceMs)
        {
            if (lineFollowerState != LINE_LOST)
            {
                savedLineFollowerState = lineFollowerState;
                lineFollowerState = LINE_LOST;
            }
            if (controllerState == AUTO)
                setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
            prevNormError = 0.0f;
            return;
        }

        if (controllerState != AUTO || lineFollowerState == LINE_LOST ||
            fabsf(commandedBaseSpeedMmS) < 0.1f)
        {
            if (controllerState == AUTO && fabsf(commandedBaseSpeedMmS) < 0.1f)
                setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
            return;
        }

        if (!lineDetected)
        {
            latestLineMask = previousLineMask;
        }

        // Calculate weighted line centroid
        // Weights: bit 7 = +3.5, bit 6 = +2.5, bit 5 = +1.5, bit 4 = +0.5,
        //          bit 3 = -0.5, bit 2 = -1.5, bit 1 = -2.5, bit 0 = -3.5
        static const float SENSOR_WEIGHTS[8] = {
            -5.0f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 5.0f
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
        float rawError = weightedSum / (float)activeSensorCount;
        if (lineFollowerState == BRANCHING_LEFT || lineFollowerState == BRANCHING_RIGHT)
        {
            // Edge following: only the outermost line/ground transition matters.
            // Active-low mask, bit 7 = leftmost sensor, so sensor index = 7 - bit.
            const uint8_t lineBits = static_cast<uint8_t>(~latestLineMask);
            if (lineFollowerState == BRANCHING_LEFT)
            {
                const int highestBit = 31 - __builtin_clz(static_cast<unsigned>(lineBits));
                rawError = kLeftEdgeErrorBySensor[7 - highestBit];
            }
            else
            {
                const int lowestBit = __builtin_ctz(static_cast<unsigned>(lineBits));
                rawError = kRightEdgeErrorBySensor[7 - lowestBit];
            }
        }
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

        if (strcasecmp(line, "STOP") == 0)
        {
            controllerState = STOPPED;
            autoRunTargetMmS = 0.0f;
            commandedBaseSpeedMmS = 0.0f;
            manualLeftTargetMmS = 0.0f;
            manualRightTargetMmS = 0.0f;
            worldCommandSource = false;
            setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
            Serial.println("# OK STOPPED");
            return;
        }

        if (strcasecmp(line, "STATUS") == 0)
        {
            noInterrupts();
            unsigned long lPulses = leftPulseCount;
            unsigned long rPulses = rightPulseCount;
            interrupts();
            Serial.printf("# STATUS: Controller=%u, LineState=%u, BaseSpeed=%.1f mm/s, "
                          "LeftTarget=%.1f, RightTarget=%.1f, Battery=%.0f mV, "
                          "LineMask=0x%02X (%s), Rate=%.1fHz, LeftPulses=%lu, RightPulses=%lu\n",
                          static_cast<unsigned>(controllerState),
                          static_cast<unsigned>(lineFollowerState),
                          commandedBaseSpeedMmS, getTargetSpeedMmPerSec(),
                          motorStates[MOTOR_RIGHT].targetSpeedMmS, batteryFilteredMv,
                          latestLineMask, lineReadValid ? "VALID" : "INVALID",
                          measuredControlRateHz, lPulses, rPulses);
            return;
        }

        if (line[0] == 'S' || line[0] == 's')
        {
            char *valStr = line + 1;
            while (*valStr == ' ' || *valStr == '\t') valStr++;
            if (*valStr == '\0')
            {
                controllerState = STOPPED;
                autoRunTargetMmS = 0.0f;
                commandedBaseSpeedMmS = 0.0f;
                manualLeftTargetMmS = 0.0f;
                manualRightTargetMmS = 0.0f;
                worldCommandSource = false;
                setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
                Serial.println("# OK STOPPED");
                return;
            }

            char *end = nullptr;
            float targetMmS = strtof(valStr, &end);
            if (end == valStr || *end != '\0' || !isfinite(targetMmS))
            {
                Serial.println("# ERR Invalid speed; use S<mm/s>");
                return;
            }
            if (targetMmS > SPEED_CMD_MAX_MM_S) targetMmS = SPEED_CMD_MAX_MM_S;
            if (targetMmS < SPEED_CMD_MIN_MM_S) targetMmS = SPEED_CMD_MIN_MM_S;
            if (controllerState == LOW_BATT)
            {
                Serial.println("# ERR LOW_BATT safety override active");
                return;
            }
            controllerState = AUTO;
            selectLineFollowerStateFromTurnBits(0);
            autoRunTargetMmS = targetMmS;
            commandedBaseSpeedMmS = targetMmS;
            manualLeftTargetMmS = 0.0f;
            manualRightTargetMmS = 0.0f;
            worldCommandSource = false;
            prevNormError = 0.0f;
            setIndividualTargetSpeedsMmPerSec(targetMmS, targetMmS);
            Serial.printf("# OK AUTO S=%.1f mm/s\n", targetMmS);
            return;
        }

        if (strcasecmp(line, "HELP") == 0)
        {
            Serial.println("# Commands: S<mm/s> (AUTO speed -120..+120), S or STOP, STATUS, HELP");
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

// Initializes motor, sensor, battery, BLE, and watchdog services with motion disabled.
void setup()
{
    // Initialize USB CDC Serial at 115200 baud
    Serial.begin(115200);
    delay(1000); // 400 min to run properly, 1000 delay to see Serial on boot
    Serial.setTxTimeoutMs(0); // Zero timeout prevents blocking on full buffer
    initializeWatchdog();

    Serial.println();
    Serial.println("==========================================================");
    Serial.println("  E14 Firmware: BLE, Line Sensor edge following & NFC Reader");
    Serial.println("==========================================================");

    // Initialize motor hardware (LEDC PWM, GPIO Direction, CAP Interrupts)
    setupMotors();

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_BATTERY_MONITOR, ADC_11db);
    pinMode(PIN_BATTERY_MONITOR, INPUT);
    sampleBatteryVoltage();

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

    Serial.println("# Commands: S<mm/s> (AUTO speed), S or STOP, STATUS, HELP");
    Serial.println("# Telemetry: P=<pwm>,L=<mm/s>,R=<mm/s>,LINE=<bits>,NFC=<uid|NONE>,AGE=<sec>,RATE=<Hz> @ 10 Hz");
    Serial.println("# Ready");

    if (!bleTelemetry.begin(BLE_CAR_ID))
        Serial.println("# [ERROR] BLE telemetry initialization failed.");

    lastWorldStateMs = millis();
    controllerState = kAutoStartOnBoot ? AUTO : STOPPED;
    autoRunTargetMmS = kAutoStartOnBoot ? kAutoRunSpeedMmS : 0.0f;
    commandedBaseSpeedMmS = autoRunTargetMmS;
    setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
    Serial.printf("# E13 ready: car=%u, controller=%u, battery=%.0f mV, startup_motion=%s\n",
                  static_cast<unsigned>(BLE_CAR_ID),
                  static_cast<unsigned>(controllerState),
                  batteryFilteredMv,
                  kAutoStartOnBoot ? "enabled" : "disabled");
}

// ── Arduino Main Loop ────────────────────────────────────────────────

// Services World State between 100 Hz motor ticks and publishes a 20 Hz snapshot.
void loop()
{
    const uint64_t nowUs = esp_timer_get_time();
    const uint32_t nowMs = millis();

    // ── 1. Poll incoming serial commands ─────────────────────────────
    pollSerialCommands();
    bleTelemetry.service(nowMs);
    serviceWorldState(nowMs);

    // ── 2. Service 100 Hz Control Loop (every 10 ms) ─────────────────
    if (nowUs >= nextControlSlotUs)
    {
        uint64_t missedSlots = (nowUs - nextControlSlotUs) / CONTROL_INTERVAL_US;
        nextControlSlotUs += (missedSlots + 1) * CONTROL_INTERVAL_US;
        ++controlCyclesInWindow;

        // Synchronous line sensor read at the start of each 10 ms tick (~250 us)
        readLineSensor();

        sampleBatteryVoltage();
        updateBatterySafety(millis());

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

    // BLE callbacks only publish snapshots; controller and motor state stay in this loop.
    RobotTelemetrySnapshot snapshot{};
    memcpy(snapshot.nfcUid, latestNfcUid, sizeof(snapshot.nfcUid));
    if (hasScannedUid)
    {
        const uint64_t ageMs = (nowUs-lastUidScannedAtUs)/1000;
        snapshot.nfcAgeMs = ageMs > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(ageMs);
    }
    snapshot.controllerState = static_cast<uint8_t>(controllerState);
    snapshot.lineFollowerState = static_cast<uint8_t>(lineFollowerState);
    snapshot.leftSpeedMmS = static_cast<int16_t>(lroundf(getLeftMotorSpeedMmS()));
    snapshot.rightSpeedMmS = static_cast<int16_t>(lroundf(getRightMotorSpeedMmS()));
    snapshot.batteryMv = batteryFilteredMv > UINT16_MAX
        ? UINT16_MAX : static_cast<uint16_t>(lroundf(batteryFilteredMv));
    bleTelemetry.publishIfDue(nowMs, kBleTelemetryPeriodMs, snapshot);

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

    feedWatchdog();
}
