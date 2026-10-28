// main.cpp — E14 BLE-enabled line-following car: dual BLDC motors, 8-channel line sensor, NFC.
//
// Structure of this file:
//   1. Settings        - every tunable value, grouped by topic (edit these first).
//   2. Types and state - controller/line-follower states and the file-local runtime state.
//   3. Helpers         - small, single-purpose functions, grouped by subsystem.
//   4. setup()/loop()  - initialization and the cooperative main loop.
//
// Runtime overview:
//   - A 100 Hz control tick reads the line sensor and battery, steers, and runs the
//     closed-loop wheel speed controllers (motor_control.cpp).
//   - The NFC reader is serviced cooperatively between control ticks.
//   - A BLE World State selects AUTO/MANUAL, speed and turn intent; a 20 Hz Robot State
//     goes back to the modem. See docs/remote_control_payload.md and docs/states.md.
//   - USB serial commands (S<mm/s>, STOP, STATUS, HELP) and a 10 Hz text report are
//     available for bench work.
//
// Line-sensor convention used throughout: the mask is ACTIVE-LOW (0 = line under the
// sensor), bit 7 is the leftmost sensor. Sensor INDEX 0..7 runs left to right
// (index = 7 - bit). kLineIsWhiteOnBlack is applied once, where the sensor is read.

#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <cstring>
#include "ble_telemetry.h"
#include "motor_control.h"
#include "../../e8_nfc_fm17550_i2c/NfcAsync.h"
#include <ble14_watchdog.h>

#ifndef BLE_CAR_ID
#define BLE_CAR_ID 1 // Normally set per PlatformIO environment (-D BLE_CAR_ID=n)
#endif
static_assert(BLE_CAR_ID >= 1 && BLE_CAR_ID <= 4, "BLE_CAR_ID must be 1..4");

namespace
{
    // ══════════════════════════════════════════════════════════════════
    // 1. SETTINGS
    // Wheel/PI gains, line-steering gains (LINE_STEER_KP/KD), pins and
    // physical constants live in motor_config.h.
    // ══════════════════════════════════════════════════════════════════

    // ── Startup ──────────────────────────────────────────────────────
    constexpr bool kAutoStartOnBoot = false;        // true: boot into AUTO at kAutoRunSpeedMmS
    constexpr uint32_t kUsbSerialBootDelayMs = 1000; // Lets the USB CDC port enumerate before logging

    // ── Speeds ───────────────────────────────────────────────────────
    constexpr float kAutoRunSpeedMmS = 100.0f;       // AUTO target while World State "forward" is set
    constexpr float kManualSpeedMmS = 50.0f;         // MANUAL forward/backward wheel speed
    constexpr float kManualTurnMagnitude = 0.8f;     // MANUAL gentle turn: inner wheel = speed * (1 - this)

    // ── BLE / World State ────────────────────────────────────────────
    constexpr uint32_t kBleTelemetryPeriodMs = 50;   // Robot State notification period (20 Hz)
    constexpr uint32_t kWorldStateTimeoutMs = 500;   // No World State this long -> zero targets, keep mode

    // ── Line sensor ──────────────────────────────────────────────────
    constexpr bool kLineIsWhiteOnBlack = true;       // false: black line on white ground; true: the opposite
    constexpr uint32_t kLineLostGraceMs = 1500;      // No line this long -> LINE_LOST, AUTO wheels stop
    constexpr float kLineErrorNormalization = 3.5f;  // Error table units -> normalized error for the steering gains

    // Per-sensor error, indexed by sensor position (0 = leftmost ... 7 = rightmost).
    // Positive steers LEFT, negative steers RIGHT, zero drives straight.
    // Centre following: weights of the sensors that see the line, averaged.
    constexpr float kCentreErrorBySensor[8] = {+5.0f, +2.5f, +1.5f, +0.5f, -0.5f, -1.5f, -2.5f, -5.0f};
    // Edge following: error looked up by the outermost line sensor only (line width is ignored).
    // Left edge uses the leftmost line sensor, right edge uses the rightmost one.
    constexpr float kLeftEdgeErrorBySensor[8]  = {+5.0f, +3.0f, +1.0f, 0.0f, -1.0f, -2.0f, -4.0f, -5.0f};
    constexpr float kRightEdgeErrorBySensor[8] = {+5.0f, +4.0f, +2.0f, +1.0f, 0.0f, -1.0f, -3.0f, -5.0f};

    // ── Battery ──────────────────────────────────────────────────────
    constexpr uint8_t PIN_BATTERY_MONITOR = D10;
    constexpr float kBatteryAdcReferenceMv = 3300.0f;      // Full-scale input at 11 dB attenuation
    constexpr float kBatteryAdcMaxCount = 4095.0f;         // 12-bit ADC
    constexpr float kBatteryDividerRatio = 122.0f / 22.0f; // 100k + 22k over 22k, see docs/hardware.md
    constexpr float kBatteryEmaAlpha = 0.02f;              // Filter coefficient, applied every 100 Hz tick
    constexpr float kLowBatteryEnterMv = 9500.0f;          // Below this for the dwell time -> LOW_BATT
    constexpr float kLowBatteryRecoverMv = 10000.0f;       // Above this for the dwell time -> back to AUTO
    constexpr uint32_t kBatteryDwellMs = 5000;

    // ── NFC reader and activity LED ──────────────────────────────────
    constexpr uint8_t kNfcReceiverGain = 4;          // FM17550 RxGain field 0..7 (higher = more sensitive); 3 = 23 dB, library default is 6
    constexpr uint8_t PIN_NFC_LED = 21;
    constexpr bool kNfcLedActiveLow = true;          // XIAO ESP32S3 user LED lights when the pin is LOW
    constexpr uint32_t kNfcLedOnWithinUs = 70000;    // LED on while the last NFC read is younger than this

    // ── Serial command port ──────────────────────────────────────────
    constexpr size_t kCommandBufferSize = 64;

    // ══════════════════════════════════════════════════════════════════
    // 2. TYPES AND STATE
    // ══════════════════════════════════════════════════════════════════

    // World State movement byte layout (per car), see docs/remote_control_payload.md.
    constexpr uint8_t kMovementAutoBit = 0x01;       // 1 = AUTO, 0 = MANUAL
    constexpr uint8_t kMovementForwardBit = 0x02;
    constexpr uint8_t kMovementBackwardBit = 0x04;
    constexpr uint8_t kMovementLeftBit = 0x08;
    constexpr uint8_t kMovementRightBit = 0x10;

    // Values are the controller_state IDs sent in the Robot State.
    enum ControllerState : uint8_t
    {
        AUTO = 0,     // Line following at the commanded base speed
        MANUAL = 1,   // Direct wheel control from World State movement bits
        LOW_BATT = 14, // Battery protection; wheels stopped
        STOPPED = 15  // Stopped by serial command or boot default
    };

    // Values are the line_follower_state IDs sent in the Robot State.
    enum LineFollowerState : uint8_t
    {
        FOLLOW_LINE = 0,      // Follow the line centre
        BRANCHING_CENTER = 4, // Follow the line centre through a branch
        BRANCHING_LEFT = 5,   // Follow the left edge of the line
        BRANCHING_RIGHT = 6,  // Follow the right edge of the line
        LINE_LOST = 10        // No line; AUTO wheels stopped, previous state kept for recovery
    };

    BleTelemetry bleTelemetry;

    // ── Controller and line-follower state ───────────────────────────
    ControllerState controllerState = STOPPED;
    LineFollowerState lineFollowerState = FOLLOW_LINE;
    LineFollowerState savedLineFollowerState = FOLLOW_LINE; // Restored when the line returns after LINE_LOST
    float autoBaseSpeedMmS = 0.0f;       // Forward speed used by AUTO line following
    float previousNormalizedError = 0.0f; // For the steering derivative term

    // ── World State timeout ──────────────────────────────────────────
    uint32_t lastWorldStateMs = 0;
    bool worldTimeoutArmed = true;       // False after a timeout or while serial commands are in control

    // ── Battery ──────────────────────────────────────────────────────
    float batteryFilteredMv = 0.0f;
    bool batteryFilterInitialized = false;
    uint32_t batteryBelowSinceMs = 0;     // 0 = not currently below the enter threshold
    uint32_t batteryRecoveredSinceMs = 0; // 0 = not currently above the recover threshold

    // ── Line sensor ──────────────────────────────────────────────────
    uint8_t latestLineMask = 0xFF;       // Active-low mask of the latest valid read
    uint8_t lastVisibleLineMask = 0xFF;  // Latest mask that actually showed a line
    uint32_t lastLineVisibleAtMs = 0;
    bool lineReadValid = false;
    uint8_t lineTxError = 0;
    bool lineShortRead = false;

    // ── NFC ──────────────────────────────────────────────────────────
    NfcAsync nfcReader;
    bool nfcRunning = false;
    uint8_t latestNfcUid[4] = {};
    bool hasScannedUid = false;
    uint64_t lastUidScannedAtUs = 0;

    // ── Loop timing and 10 Hz report ─────────────────────────────────
    constexpr float kControlPeriodSeconds = CONTROL_INTERVAL_US / 1000000.0f;
    uint64_t nextControlSlotUs = 0;
    uint64_t lastReportUs = 0;
    uint64_t rateWindowStartUs = 0;
    uint32_t controlCyclesInRateWindow = 0;
    float measuredControlRateHz = 0.0f;

    // ── Serial command input ─────────────────────────────────────────
    char commandBuffer[kCommandBufferSize];
    size_t commandLength = 0;

    // ══════════════════════════════════════════════════════════════════
    // 3. HELPERS
    // ══════════════════════════════════════════════════════════════════

    // ── Movement control ─────────────────────────────────────────────

    // Zeroes the AUTO base speed and both wheel targets.
    void stopAllMovement()
    {
        autoBaseSpeedMmS = 0.0f;
        setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
    }

    // Returns the inner-wheel speed for a turn of the given magnitude. The inner wheel may
    // reverse for a tighter pivot but never faster than the outer wheel drives forward, so
    // the car never drives backwards on a sharp turn.
    float limitedInnerWheelSpeed(float baseSpeedMmS, float steerMagnitude)
    {
        const float innerSpeed = baseSpeedMmS * (1.0f - steerMagnitude);
        const float reverseLimit = -baseSpeedMmS;
        return baseSpeedMmS > 0.0f ? fmaxf(innerSpeed, reverseLimit) : fminf(innerSpeed, reverseLimit);
    }

    // Writes MANUAL wheel speeds for the movement bits. Forward+backward cancel to a stop;
    // a turn bit alone pivots in place; with a direction it makes a gentle turn. Right wins
    // when both turn bits are set.
    void setManualWheelTargets(bool forward, bool backward, bool left, bool right)
    {
        float leftTargetMmS = 0.0f;
        float rightTargetMmS = 0.0f;
        const bool turning = left || right;

        if (forward != backward)
        {
            const float baseSpeedMmS = forward ? kManualSpeedMmS : -kManualSpeedMmS;
            const float innerSpeedMmS = baseSpeedMmS * (1.0f - kManualTurnMagnitude);
            leftTargetMmS = (turning && !right) ? innerSpeedMmS : baseSpeedMmS;
            rightTargetMmS = (turning && right) ? innerSpeedMmS : baseSpeedMmS;
        }
        else if (!forward && turning)
        {
            leftTargetMmS = right ? kManualSpeedMmS : -kManualSpeedMmS;
            rightTargetMmS = -leftTargetMmS;
        }
        setIndividualTargetSpeedsMmPerSec(leftTargetMmS, rightTargetMmS);
    }

    // ── Line sensor ──────────────────────────────────────────────────

    // Reads the 8-channel line sensor over I2C (about 250 us at 400 kHz) into latestLineMask.
    void readLineSensor()
    {
        Wire.beginTransmission(LINE_SENSOR_I2C_ADDR);
        Wire.write(LINE_SENSOR_REG_DATA);
        const uint8_t transmitError = Wire.endTransmission(false);
        if (transmitError != 0)
        {
            lineTxError = transmitError;
            lineReadValid = false;
            return;
        }

        const size_t received = Wire.requestFrom(static_cast<uint8_t>(LINE_SENSOR_I2C_ADDR), static_cast<size_t>(1), true);
        if (received == 1 && Wire.available() == 1)
        {
            latestLineMask = static_cast<uint8_t>(Wire.read());
            // Flip for a white line on black ground so the rest of the code keeps 0 = line.
            if (kLineIsWhiteOnBlack) latestLineMask = ~latestLineMask;
            lineReadValid = true;
            lineTxError = 0;
            lineShortRead = false;
        }
        else
        {
            lineShortRead = true;
            lineReadValid = false;
            while (Wire.available()) Wire.read(); // Discard any partial data
        }
    }

    // True when the sensor under the given left-to-right index (0..7) sees the line.
    bool isLineUnderSensor(uint8_t activeLowMask, int sensorIndex)
    {
        return (activeLowMask & (1 << (7 - sensorIndex))) == 0;
    }

    // Computes the steering error (positive = steer left) for the current line-follower state.
    // FOLLOW_LINE and BRANCHING_CENTER average the sensors seeing the line; the edge states use
    // only the outermost line sensor. Returns false when no sensor sees the line.
    bool computeLineError(uint8_t activeLowMask, float &errorOut)
    {
        int leftmostSensor = -1;
        int rightmostSensor = -1;
        int sensorsOnLine = 0;
        float centreErrorSum = 0.0f;

        for (int sensor = 0; sensor < 8; ++sensor)
        {
            if (!isLineUnderSensor(activeLowMask, sensor)) continue;
            if (leftmostSensor < 0) leftmostSensor = sensor;
            rightmostSensor = sensor;
            centreErrorSum += kCentreErrorBySensor[sensor];
            ++sensorsOnLine;
        }

        if (sensorsOnLine == 0) return false;

        switch (lineFollowerState)
        {
        case BRANCHING_LEFT:
            errorOut = kLeftEdgeErrorBySensor[leftmostSensor];
            break;
        case BRANCHING_RIGHT:
            errorOut = kRightEdgeErrorBySensor[rightmostSensor];
            break;
        default:
            errorOut = centreErrorSum / static_cast<float>(sensorsOnLine);
            break;
        }
        return true;
    }

    // Selects the line-follower state from the World State turn bits: none = FOLLOW_LINE,
    // left only = BRANCHING_LEFT, right only = BRANCHING_RIGHT, both = BRANCHING_CENTER.
    // While LINE_LOST only the saved state changes, so recovery resumes the newest selection.
    void selectLineFollowerStateFromTurnBits(uint8_t movement)
    {
        const bool left = (movement & kMovementLeftBit) != 0;
        const bool right = (movement & kMovementRightBit) != 0;
        const LineFollowerState selected = (left && right) ? BRANCHING_CENTER
            : left ? BRANCHING_LEFT
            : right ? BRANCHING_RIGHT : FOLLOW_LINE;

        LineFollowerState &stateToUpdate = (lineFollowerState == LINE_LOST) ? savedLineFollowerState : lineFollowerState;
        if (stateToUpdate != selected)
        {
            stateToUpdate = selected;
            previousNormalizedError = 0.0f; // The error definition changed; avoid a derivative spike
        }
    }

    // Tracks line visibility: enters LINE_LOST after the grace period and restores the saved
    // state when the line returns. Returns true when AUTO steering should be skipped this tick.
    bool updateLineLossState(uint32_t nowMs, bool lineVisible)
    {
        if (lineVisible)
        {
            lastLineVisibleAtMs = nowMs;
            lastVisibleLineMask = latestLineMask;
            if (lineFollowerState == LINE_LOST) lineFollowerState = savedLineFollowerState;
            return false;
        }

        if (nowMs - lastLineVisibleAtMs <= kLineLostGraceMs) return false; // Short gap: keep last mask

        if (lineFollowerState != LINE_LOST)
        {
            savedLineFollowerState = lineFollowerState;
            lineFollowerState = LINE_LOST;
        }
        if (controllerState == AUTO) setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
        previousNormalizedError = 0.0f;
        return true;
    }

    // Runs every 100 Hz tick: tracks line loss and, in AUTO, steers the wheels toward the line.
    // Positive error means the line is to the left, so the car steers left. PD steering sets
    // how much slower the inner wheel runs.
    void updateLineFollowing()
    {
        const uint32_t nowMs = millis();
        const bool lineVisible = lineReadValid && latestLineMask != 0xFF && latestLineMask != 0x00;
        if (updateLineLossState(nowMs, lineVisible)) return;

        if (controllerState != AUTO || lineFollowerState == LINE_LOST) return;
        if (fabsf(autoBaseSpeedMmS) < 0.1f)
        {
            setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
            return;
        }

        // During a short dropout steer from the last mask that showed the line.
        const uint8_t steeringMask = lineVisible ? latestLineMask : lastVisibleLineMask;
        float lineError = 0.0f;
        if (!computeLineError(steeringMask, lineError))
        {
            setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f); // Safety: nothing to follow
            previousNormalizedError = 0.0f;
            return;
        }

        const float normalizedError = lineError / kLineErrorNormalization;
        const float errorRate = (normalizedError - previousNormalizedError) / kControlPeriodSeconds;
        previousNormalizedError = normalizedError;
        const float steerSignal = (LINE_STEER_KP * normalizedError) + (LINE_STEER_KD * errorRate);

        float leftSpeedMmS = autoBaseSpeedMmS;
        float rightSpeedMmS = autoBaseSpeedMmS;
        if (steerSignal > 0.001f) // Steer left: left wheel is the inner wheel
            leftSpeedMmS = limitedInnerWheelSpeed(autoBaseSpeedMmS, steerSignal);
        else if (steerSignal < -0.001f) // Steer right: right wheel is the inner wheel
            rightSpeedMmS = limitedInnerWheelSpeed(autoBaseSpeedMmS, -steerSignal);

        setIndividualTargetSpeedsMmPerSec(leftSpeedMmS, rightSpeedMmS);
    }

    // ── Battery ──────────────────────────────────────────────────────

    // Samples the battery divider once per control tick and applies the EMA filter.
    void sampleBatteryVoltage()
    {
        const float adcCount = static_cast<float>(analogRead(PIN_BATTERY_MONITOR));
        const float batteryMv = (adcCount * kBatteryAdcReferenceMv / kBatteryAdcMaxCount) * kBatteryDividerRatio;
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

    // Enters LOW_BATT after the voltage stays low for the dwell time and returns to a stopped
    // AUTO after it stays recovered for the dwell time.
    void updateBatteryState(uint32_t nowMs)
    {
        if (controllerState != LOW_BATT)
        {
            batteryRecoveredSinceMs = 0;
            if (batteryFilteredMv >= kLowBatteryEnterMv)
            {
                batteryBelowSinceMs = 0;
                return;
            }
            if (batteryBelowSinceMs == 0) batteryBelowSinceMs = nowMs;
            if (nowMs - batteryBelowSinceMs >= kBatteryDwellMs)
            {
                controllerState = LOW_BATT;
                stopAllMovement();
            }
            return;
        }

        if (batteryFilteredMv <= kLowBatteryRecoverMv)
        {
            batteryRecoveredSinceMs = 0;
            return;
        }
        if (batteryRecoveredSinceMs == 0) batteryRecoveredSinceMs = nowMs;
        if (nowMs - batteryRecoveredSinceMs >= kBatteryDwellMs)
        {
            controllerState = AUTO;
            stopAllMovement();
            worldTimeoutArmed = false; // Wait for a fresh World State before arming the timeout
            batteryBelowSinceMs = 0;
            batteryRecoveredSinceMs = 0;
        }
    }

    // ── World State (BLE commands) ───────────────────────────────────

    // Applies one car's movement byte: AUTO sets the line-following speed and turn state,
    // MANUAL drives the wheels directly. Ignored while LOW_BATT.
    void applyWorldMovementCommand(uint8_t movement)
    {
        worldTimeoutArmed = true;
        if (controllerState == LOW_BATT) return;

        if (movement & kMovementAutoBit)
        {
            controllerState = AUTO;
            selectLineFollowerStateFromTurnBits(movement);
            autoBaseSpeedMmS = (movement & kMovementForwardBit) ? kAutoRunSpeedMmS : 0.0f;
            if (lineFollowerState == LINE_LOST || autoBaseSpeedMmS == 0.0f)
                setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
            return;
        }

        controllerState = MANUAL;
        autoBaseSpeedMmS = 0.0f;
        setManualWheelTargets((movement & kMovementForwardBit) != 0, (movement & kMovementBackwardBit) != 0,
                              (movement & kMovementLeftBit) != 0, (movement & kMovementRightBit) != 0);
    }

    // Once World State has been silent for kWorldStateTimeoutMs, zeroes all targets and turn
    // selection but keeps the AUTO/MANUAL mode. The next World State resumes motion.
    void enforceWorldStateTimeout(uint32_t nowMs)
    {
        if (!worldTimeoutArmed || controllerState == LOW_BATT) return;
        if (nowMs - lastWorldStateMs < kWorldStateTimeoutMs) return;

        stopAllMovement();
        if (controllerState == AUTO) selectLineFollowerStateFromTurnBits(0);
        worldTimeoutArmed = false;
    }

    // Applies the newest BLE World State, then enforces the command-age failsafe.
    void serviceWorldState(uint32_t nowMs)
    {
        Ble14::World world{};
        if (bleTelemetry.takeWorldState(world))
        {
            lastWorldStateMs = nowMs;
            applyWorldMovementCommand(world.movement[BLE_CAR_ID - 1]);
        }
        enforceWorldStateTimeout(nowMs);
    }

    // ── NFC ──────────────────────────────────────────────────────────

    // Starts (or restarts) the FM17550 reader on the shared I2C bus. Returns true when running.
    bool startNfcReader()
    {
        NfcAsync::Options options;
        options.clockHz = I2C_CLOCK_HZ;
        options.gain = kNfcReceiverGain;
        options.cachePointer = false;     // Line sensor shares the bus, so re-assert the pointer each time
        options.logInitialization = false;
        options.rfTimeoutUs = 1000;
        options.pollUs = 710;
        return nfcReader.begin(options);
    }

    // Initializes the NFC reader at boot and reports the result.
    void initializeNfc()
    {
        nfcRunning = startNfcReader();
        if (nfcRunning)
        {
            Serial.printf("# [DEBUG] NFC FM17550 initialized successfully (gain=%u).\n", static_cast<unsigned>(kNfcReceiverGain));
        }
        else
        {
            nfcReader.end();
            Serial.println("# [DEBUG] NFC FM17550 initialization failed or not responding.");
        }
    }

    // Age of the last successful NFC read in microseconds (UINT64_MAX if none since boot).
    uint64_t nfcAgeUs(uint64_t nowUs)
    {
        return hasScannedUid ? nowUs - lastUidScannedAtUs : UINT64_MAX;
    }

    // Advances the NFC state machine, records new UIDs and recovers from I2C bus errors.
    void serviceNfc(uint64_t nowUs)
    {
        if (!nfcRunning) return;

        nfcReader.service(nowUs);

        uint8_t uid[sizeof(latestNfcUid)];
        const NfcAsync::Result result = nfcReader.takeResult(uid);
        if (result == NfcAsync::Result::Uid)
        {
            memcpy(latestNfcUid, uid, sizeof(latestNfcUid));
            hasScannedUid = true;
            lastUidScannedAtUs = nowUs;
        }
        else if (result == NfcAsync::Result::BusError)
        {
            nfcReader.end();
            nfcRunning = startNfcReader();
        }
    }

    // Lights the onboard LED while the last NFC read is younger than kNfcLedOnWithinUs.
    void updateNfcLed(uint64_t nowUs)
    {
        const bool recentRead = nfcAgeUs(nowUs) < kNfcLedOnWithinUs;
        digitalWrite(PIN_NFC_LED, (recentRead != kNfcLedActiveLow) ? HIGH : LOW);
    }

    // ── Telemetry ────────────────────────────────────────────────────

    // Builds the Robot State snapshot sent over BLE from the current car state.
    RobotTelemetrySnapshot buildTelemetrySnapshot(uint64_t nowUs)
    {
        RobotTelemetrySnapshot snapshot{};
        memcpy(snapshot.nfcUid, latestNfcUid, sizeof(snapshot.nfcUid));
        const uint64_t ageMs = nfcAgeUs(nowUs) / 1000;
        snapshot.nfcAgeMs = ageMs > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(ageMs);
        // Active-low internal mask -> 1 = line; an invalid read reports no line.
        snapshot.lineSensorMask = lineReadValid ? static_cast<uint8_t>(~latestLineMask) : 0;
        snapshot.controllerState = static_cast<uint8_t>(controllerState);
        snapshot.lineFollowerState = static_cast<uint8_t>(lineFollowerState);
        snapshot.leftSpeedMmS = static_cast<int16_t>(lroundf(getLeftMotorSpeedMmS()));
        snapshot.rightSpeedMmS = static_cast<int16_t>(lroundf(getRightMotorSpeedMmS()));
        snapshot.batteryMv = batteryFilteredMv > UINT16_MAX
            ? UINT16_MAX : static_cast<uint16_t>(lroundf(batteryFilteredMv));
        return snapshot;
    }

    // Writes to USB serial only if a host is connected and the buffer has room, so a closed
    // or undrained port can never stall the control loop.
    void writeSerialIfPossible(const char *text, size_t length)
    {
        if (!Serial) return;
        if (Serial.availableForWrite() < static_cast<int>(length)) return;
        Serial.write(reinterpret_cast<const uint8_t *>(text), length);
    }

    // Prints the 10 Hz development line, e.g.
    //   P=150,L=98,R=97,LINE=11100111,NFC=11:E5:99:23,AGE=0.2s,RATE=100.0Hz
    // LINE is bit 7 (leftmost) to bit 0; NFC shows the UID only while the last read is under
    // NFC_TAG_ACTIVE_TIMEOUT_US old; AGE is the last read's age (--.-s before the first read).
    void printDevelopmentReport(uint64_t nowUs)
    {
        char lineText[9];
        if (lineReadValid)
        {
            for (int bit = 7; bit >= 0; --bit)
                lineText[7 - bit] = (latestLineMask & (1 << bit)) ? '1' : '0';
            lineText[8] = '\0';
        }
        else
        {
            memcpy(lineText, "--------", sizeof(lineText));
        }

        char nfcText[16] = "NONE";
        char ageText[10] = "--.-s";
        const uint64_t ageUs = nfcAgeUs(nowUs);
        if (hasScannedUid)
        {
            if (ageUs <= NFC_TAG_ACTIVE_TIMEOUT_US)
                snprintf(nfcText, sizeof(nfcText), "%02X:%02X:%02X:%02X",
                         latestNfcUid[0], latestNfcUid[1], latestNfcUid[2], latestNfcUid[3]);
            const float ageSeconds = fminf(static_cast<float>(ageUs) / 1000000.0f, 99.9f);
            snprintf(ageText, sizeof(ageText), "%.1fs", ageSeconds);
        }

        char report[160];
        const int length = snprintf(report, sizeof(report),
                                    "P=%d,L=%d,R=%d,LINE=%s,NFC=%s,AGE=%s,RATE=%.1fHz\n",
                                    getCommandedPWM(),
                                    static_cast<int>(roundf(getLeftMotorSpeedMmS())),
                                    static_cast<int>(roundf(getRightMotorSpeedMmS())),
                                    lineText, nfcText, ageText, measuredControlRateHz);
        if (length > 0 && length < static_cast<int>(sizeof(report)))
            writeSerialIfPossible(report, static_cast<size_t>(length));
    }

    // Emits the development report every REPORT_INTERVAL_US and refreshes the measured loop rate.
    void serviceDevelopmentReport(uint64_t nowUs)
    {
        if (nowUs - lastReportUs < REPORT_INTERVAL_US) return;

        const uint64_t windowUs = nowUs - rateWindowStartUs;
        if (windowUs > 0)
            measuredControlRateHz = (static_cast<float>(controlCyclesInRateWindow) * 1000000.0f) / static_cast<float>(windowUs);
        controlCyclesInRateWindow = 0;
        rateWindowStartUs = nowUs;
        lastReportUs = nowUs;

        printDevelopmentReport(nowUs);
    }

    // ── Serial commands ──────────────────────────────────────────────

    void printSerialHelp()
    {
        Serial.println("# Commands: S<mm/s> (AUTO speed -120..+120), S or STOP, STATUS, HELP");
    }

    // Prints controller, line state, targets, battery, line mask and encoder pulse counts.
    void printStatus()
    {
        noInterrupts();
        const unsigned long leftPulses = leftPulseCount;
        const unsigned long rightPulses = rightPulseCount;
        interrupts();
        Serial.printf("# STATUS: Controller=%u, LineState=%u, BaseSpeed=%.1f mm/s, "
                      "LeftTarget=%.1f, RightTarget=%.1f, Battery=%.0f mV, "
                      "LineMask=0x%02X (%s), Rate=%.1fHz, LeftPulses=%lu, RightPulses=%lu\n",
                      static_cast<unsigned>(controllerState), static_cast<unsigned>(lineFollowerState),
                      autoBaseSpeedMmS, getTargetSpeedMmPerSec(),
                      motorStates[MOTOR_RIGHT].targetSpeedMmS, batteryFilteredMv,
                      latestLineMask, lineReadValid ? "VALID" : "INVALID",
                      measuredControlRateHz, leftPulses, rightPulses);
    }

    // Serial control takes over from BLE: stop everything and disarm the World State timeout.
    void stopFromSerialCommand()
    {
        controllerState = STOPPED;
        stopAllMovement();
        worldTimeoutArmed = false;
        Serial.println("# OK STOPPED");
    }

    // Handles "S<mm/s>": selects AUTO with no turn bits at the requested base speed.
    void handleSerialSpeedCommand(const char *speedText)
    {
        char *end = nullptr;
        float targetMmS = strtof(speedText, &end);
        if (end == speedText || *end != '\0' || !isfinite(targetMmS))
        {
            Serial.println("# ERR Invalid speed; use S<mm/s>");
            return;
        }
        if (controllerState == LOW_BATT)
        {
            Serial.println("# ERR LOW_BATT safety override active");
            return;
        }

        targetMmS = constrain(targetMmS, SPEED_CMD_MIN_MM_S, SPEED_CMD_MAX_MM_S);
        controllerState = AUTO;
        selectLineFollowerStateFromTurnBits(0);
        autoBaseSpeedMmS = targetMmS;
        worldTimeoutArmed = false;
        previousNormalizedError = 0.0f;
        setIndividualTargetSpeedsMmPerSec(targetMmS, targetMmS);
        Serial.printf("# OK AUTO S=%.1f mm/s\n", targetMmS);
    }

    // Parses one complete serial command line.
    void handleSerialCommand(char *line)
    {
        while (*line == ' ' || *line == '\t') ++line;
        if (*line == '\0') return;

        if (strcasecmp(line, "STOP") == 0)
        {
            stopFromSerialCommand();
        }
        else if (strcasecmp(line, "STATUS") == 0)
        {
            printStatus();
        }
        else if (strcasecmp(line, "HELP") == 0)
        {
            printSerialHelp();
        }
        else if (line[0] == 'S' || line[0] == 's')
        {
            const char *speedText = line + 1;
            while (*speedText == ' ' || *speedText == '\t') ++speedText;
            if (*speedText == '\0') stopFromSerialCommand(); // Bare "S" stops
            else handleSerialSpeedCommand(speedText);
        }
        else
        {
            Serial.printf("# ERR Unknown command: '%s'\n", line);
        }
    }

    // Collects non-blocking USB serial characters and runs each completed line.
    void serviceSerialCommands()
    {
        while (Serial.available())
        {
            const char character = static_cast<char>(Serial.read());
            if (character == '\r' || character == '\n')
            {
                if (commandLength > 0)
                {
                    commandBuffer[commandLength] = '\0';
                    handleSerialCommand(commandBuffer);
                    commandLength = 0;
                }
            }
            else if (commandLength < kCommandBufferSize - 1)
            {
                commandBuffer[commandLength++] = character;
            }
        }
    }

    // ── Control tick ─────────────────────────────────────────────────

    // One 100 Hz tick: sensors, battery, steering, wheel speed control, and NFC read start.
    void runControlTick(uint64_t nowUs)
    {
        readLineSensor();
        sampleBatteryVoltage();
        updateBatteryState(millis());
        updateLineFollowing();
        updateMotorControlLoop(nowUs); // Pulse snapshot, EMA filter, PI + feedforward

        if (nfcRunning && !nfcReader.busy()) nfcReader.startRead();
    }
} // namespace

// ══════════════════════════════════════════════════════════════════════
// 4. SETUP AND LOOP
// ══════════════════════════════════════════════════════════════════════

// Initializes hardware and services. Wheels stay stopped unless kAutoStartOnBoot is set.
void setup()
{
    Serial.begin(115200);
    delay(kUsbSerialBootDelayMs);
    Serial.setTxTimeoutMs(0); // Never block the control loop on a full USB buffer
    initializeWatchdog();

    Serial.println();
    Serial.println("==========================================================");
    Serial.println("  E14 Firmware: BLE, line edge following & NFC reader");
    Serial.println("==========================================================");

    setupMotors();

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_BATTERY_MONITOR, ADC_11db);
    pinMode(PIN_BATTERY_MONITOR, INPUT);
    sampleBatteryVoltage();

    if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ))
    {
        Serial.println("# [ERROR] Failed to initialize Wire I2C bus on D4/D5!");
    }
    else
    {
        Wire.setTimeOut(I2C_TIMEOUT_MS);
        Serial.printf("# [DEBUG] Shared Wire I2C bus initialized (400 kHz, timeout %u ms).\n", I2C_TIMEOUT_MS);
    }

    readLineSensor();
    if (lineReadValid)
        Serial.printf("# [DEBUG] Line Sensor detected (initial mask: 0x%02X).\n", latestLineMask);
    else
        Serial.printf("# [DEBUG] Line Sensor not responding (txErr=%u, shortRead=%d).\n", lineTxError, lineShortRead);

    pinMode(PIN_NFC_LED, OUTPUT);
    updateNfcLed(esp_timer_get_time());
    initializeNfc();

    const uint64_t nowUs = esp_timer_get_time();
    nextControlSlotUs = nowUs + CONTROL_INTERVAL_US;
    lastReportUs = nowUs;
    rateWindowStartUs = nowUs;
    measuredControlRateHz = 1000000.0f / CONTROL_INTERVAL_US;

    printSerialHelp();
    Serial.println("# Telemetry: P=<pwm>,L=<mm/s>,R=<mm/s>,LINE=<bits>,NFC=<uid|NONE>,AGE=<sec>,RATE=<Hz> @ 10 Hz");
    Serial.println("# Ready");
    if (!bleTelemetry.begin(BLE_CAR_ID))
        Serial.println("# [ERROR] BLE telemetry initialization failed.");

    lastWorldStateMs = millis();
    controllerState = kAutoStartOnBoot ? AUTO : STOPPED;
    autoBaseSpeedMmS = kAutoStartOnBoot ? kAutoRunSpeedMmS : 0.0f;
    setIndividualTargetSpeedsMmPerSec(0.0f, 0.0f);
    Serial.printf("# E14 ready: car=%u, controller=%u, battery=%.0f mV, startup_motion=%s\n",
                  static_cast<unsigned>(BLE_CAR_ID), static_cast<unsigned>(controllerState),
                  batteryFilteredMv, kAutoStartOnBoot ? "enabled" : "disabled");
}

// Cooperative loop: commands and NFC run continuously, the control tick runs every 10 ms.
void loop()
{
    const uint64_t nowUs = esp_timer_get_time();
    const uint32_t nowMs = millis();

    serviceSerialCommands();
    bleTelemetry.service(nowMs);
    serviceWorldState(nowMs);

    if (nowUs >= nextControlSlotUs)
    {
        // Skip any slots missed during a long iteration instead of bursting to catch up.
        const uint64_t missedSlots = (nowUs - nextControlSlotUs) / CONTROL_INTERVAL_US;
        nextControlSlotUs += (missedSlots + 1) * CONTROL_INTERVAL_US;
        ++controlCyclesInRateWindow;
        runControlTick(nowUs);
    }

    serviceNfc(nowUs);
    updateNfcLed(nowUs);

    // BLE callbacks only publish a snapshot; controller and motor state stay in this loop.
    bleTelemetry.publishIfDue(nowMs, kBleTelemetryPeriodMs, buildTelemetrySnapshot(nowUs));

    serviceDevelopmentReport(nowUs);
    feedWatchdog();
}
