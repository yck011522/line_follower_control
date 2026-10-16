// main.cpp — Main entry point for E11 Dual BLDC Motor Speed & PWM Test Firmware.
//
// Manages:
//   1. USB Serial command parsing:
//      - P<val> : Sets PWM for both motors (-255 to +255).
//      - L<val> : Sets PWM for Left motor only (-255 to +255).
//      - R<val> : Sets PWM for Right motor only (-255 to +255).
//      - STOP   : Immediately stops both motors (PWM = 0).
//   2. 100 Hz main control loop (10 ms period):
//      - Calculates instantaneous steps/sec from CAP feedback interrupts.
//      - Smooths velocity using an Exponential Moving Average filter (alpha = 0.24).
//   3. 0.2 s periodic telemetry reporting:
//      - Emits format "P=%d,L=%.1f,R=%.1f\n" (e.g. "P=150,L=50.5,R=50.5").

#include <Arduino.h>
#include <esp_timer.h>
#include "motor_control.h"

// ── Timing State ─────────────────────────────────────────────────────
static uint64_t nextControlSlotUs = 0; // Timestamp of next 100 Hz control loop tick
static uint64_t lastReportUs       = 0; // Timestamp of previous 0.2s status print

// ── Serial Command Input Buffer ──────────────────────────────────────
static const size_t CMD_BUF_CAPACITY = 64;
static char cmdBuffer[CMD_BUF_CAPACITY];
static size_t cmdLength = 0;

// ── Serial Command Handler ───────────────────────────────────────────

/**
 * @brief Parses and executes a single null-terminated command string from Serial.
 * 
 * Supports:
 *   - 'P' / 'p' followed by integer: sets PWM for both motors (-255..255).
 *   - 'L' / 'l' followed by integer: sets Left motor PWM (-255..255).
 *   - 'R' / 'r' followed by integer: sets Right motor PWM (-255..255).
 *   - "STOP" or 'S' / 's': immediately zeroes both motors.
 *   - "STATUS": prints full diagnostic pulse counts and speeds.
 *   - "HELP": prints list of supported commands.
 * 
 * @param line Null-terminated ASCII command string.
 */
static void processCommand(char *line)
{
    // Trim leading whitespace
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    // Check for 'P' command: P<val> or P <val> (sets both motors)
    if (line[0] == 'P' || line[0] == 'p')
    {
        char *valStr = line + 1;
        while (*valStr == ' ' || *valStr == '\t') valStr++;
        int val = atoi(valStr);
        setBothMotorsPWM(val);
        Serial.printf("# OK P=%d\n", getCommandedPWM());
        return;
    }

    // Check for 'L' command: L<val> (sets Left motor)
    if ((line[0] == 'L' || line[0] == 'l') && line[1] != 'O' && line[1] != 'o')
    {
        char *valStr = line + 1;
        while (*valStr == ' ' || *valStr == '\t') valStr++;
        int val = atoi(valStr);
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
        setSingleMotorPWM(MOTOR_RIGHT, val);
        Serial.printf("# OK R=%d\n", motorStates[MOTOR_RIGHT].pwmValue);
        return;
    }

    // Check for STOP command: "STOP", "stop"
    if (strcasecmp(line, "STOP") == 0)
    {
        stopAllMotors();
        Serial.println("# OK STOPPED");
        return;
    }

    // Diagnostic STATUS command (must check before generic 'S' prefix)
    if (strcasecmp(line, "STATUS") == 0)
    {
        noInterrupts();
        unsigned long lPulses = leftPulseCount;
        unsigned long rPulses = rightPulseCount;
        interrupts();
        Serial.printf("# STATUS: Mode=%s, TargetSpeed=%.1f mm/s, P=%d, LeftPWM=%d, RightPWM=%d, "
                      "L_spd=%.1f stp/s (%.1f mm/s), R_spd=%.1f stp/s (%.1f mm/s), "
                      "L_I=%.1f, R_I=%.1f, LeftPulses=%lu, RightPulses=%lu\n",
                      (getControlMode() == MODE_CLOSED_LOOP_SPEED) ? "SPEED" : "PWM",
                      getTargetSpeedMmPerSec(),
                      getCommandedPWM(),
                      motorStates[MOTOR_LEFT].pwmValue,
                      motorStates[MOTOR_RIGHT].pwmValue,
                      getLeftMotorSpeed(), getLeftMotorSpeedMmS(),
                      getRightMotorSpeed(), getRightMotorSpeedMmS(),
                      motorStates[MOTOR_LEFT].integralError,
                      motorStates[MOTOR_RIGHT].integralError,
                      lPulses, rPulses);
        return;
    }

    // Check for 'S' speed command: S<mm/s> (e.g. S100, S-50, S120, S0)
    // Out-of-range values are clamped to [-120.0, +120.0] mm/s.
    // A bare 'S' or 's' without parameters acts as STOP.
    if (line[0] == 'S' || line[0] == 's')
    {
        char *valStr = line + 1;
        while (*valStr == ' ' || *valStr == '\t') valStr++;
        if (*valStr == '\0')
        {
            // Bare 'S' without argument is STOP
            stopAllMotors();
            Serial.println("# OK STOPPED");
            return;
        }

        float targetMmS = atof(valStr);
        // Clamp to [-120.0, +120.0] mm/s envelope per specification
        if (targetMmS > SPEED_CMD_MAX_MM_S) targetMmS = SPEED_CMD_MAX_MM_S;
        if (targetMmS < SPEED_CMD_MIN_MM_S) targetMmS = SPEED_CMD_MIN_MM_S;

        if (fabsf(targetMmS) < 0.1f)
        {
            stopAllMotors();
            Serial.println("# OK S=0.0 mm/s (STOPPED)");
        }
        else
        {
            setTargetSpeedMmPerSec(targetMmS);
            Serial.printf("# OK S=%.1f mm/s\n", targetMmS);
        }
        return;
    }

    // Help command
    if (strcasecmp(line, "HELP") == 0)
    {
        Serial.println("# Commands: S<mm/s> (closed-loop speed -120..+120), P<val> (open-loop PWM -255..+255), L<val>, R<val>, STOP, STATUS, HELP");
        return;
    }

    Serial.printf("# ERR Unknown command: '%s'\n", line);
}

/**
 * @brief Reads incoming characters from USB Serial and triggers processCommand on newline.
 * 
 * Discards excessive characters beyond buffer capacity to prevent overflow.
 */
static void pollSerialCommands()
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
        // Excess characters silently ignored until newline reset
    }
}

// ── Arduino Setup ────────────────────────────────────────────────────

/**
 * @brief System hardware setup routine.
 * 
 * Initializes USB CDC Serial, sets up motor PWM / direction / encoder pins,
 * primes the 100 Hz control loop timer, and prints startup diagnostics.
 */
void setup()
{
    // Initialize USB CDC Serial at 115200 baud
    Serial.begin(115200);

    // Short settling delay for native USB CDC enumeration
    delay(500);

    // Initialize motor hardware (LEDC PWM, GPIO Direction, CAP Interrupts)
    setupMotors();

    // Initialize 100 Hz control loop and report deadlines
    const uint64_t nowUs = esp_timer_get_time();
    nextControlSlotUs = nowUs + CONTROL_INTERVAL_US;
    lastReportUs = nowUs;

    Serial.println();
    Serial.println("=== E11 BLDC Motor Speed & PWM Controller ===");
    Serial.println("Commands: S-120..S120 (closed-loop speed mm/s), P-255..P255 (open-loop PWM), STOP, STATUS, HELP");
    Serial.println("Telemetry: P=<pwm>,L=<left_steps/s>,R=<right_steps/s> @ 5 Hz");
    Serial.println("# Ready");
}

// ── Arduino Main Loop ────────────────────────────────────────────────

/**
 * @brief Main execution loop.
 * 
 * Coordinates:
 *   1. Non-blocking serial command polling.
 *   2. Fixed-cadence 100 Hz motor speed estimation & EMA filtering.
 *   3. 0.2 s (5 Hz) telemetry output to Serial.
 */
void loop()
{
    const uint64_t nowUs = esp_timer_get_time();

    // ── 1. Poll and process incoming serial commands ─────────────────
    pollSerialCommands();

    // ── 2. Service 100 Hz Control Loop (every 10 ms) ─────────────────
    if (nowUs >= nextControlSlotUs)
    {
        // Anchor schedule to prevent drift over time
        uint64_t missedSlots = (nowUs - nextControlSlotUs) / CONTROL_INTERVAL_US;
        nextControlSlotUs += (missedSlots + 1) * CONTROL_INTERVAL_US;

        updateMotorControlLoop(nowUs);
    }

    // ── 3. Emit Telemetry Every 0.2 Seconds (5 Hz) ───────────────────
    if (nowUs - lastReportUs >= REPORT_INTERVAL_US)
    {
        lastReportUs = nowUs;

        // Telemetry format: "P=150,L=50.5,R=50.5"
        // Uses commanded PWM, LeftMotorStepPerSec, RightMotorStepPerSec
        Serial.printf("P=%d,L=%.1f,R=%.1f\n",
                      getCommandedPWM(),
                      getLeftMotorSpeed(),
                      getRightMotorSpeed());
    }
}
