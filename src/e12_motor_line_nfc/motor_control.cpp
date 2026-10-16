// motor_control.cpp — Implementation of dual BLDC motor control and speed calculation.
//
// Manages hardware pins (PWM via LEDC, DIR GPIO, CAP pulse interrupt), applies
// direction logic, executes the 100 Hz velocity calculation loop, filters
// instantaneous pulse rates using an Exponential Moving Average (EMA, alpha=0.24),
// and implements a closed-loop Feedforward + PI speed regulator with anti-windup.

#include "motor_control.h"

// ── Pulse Counters (one per motor) ───────────────────────────────────
// Incremented by encoder ISRs on the RISING edge of the CAP feedback pin.
// Marked volatile and accessed atomically in the control loop via noInterrupts().
volatile unsigned long leftPulseCount  = 0;
volatile unsigned long rightPulseCount = 0;

// ── Hardware Configuration ───────────────────────────────────────────
// Pin mappings and polarity settings for Left and Right drive motors.
const MotorConfig motorConfigs[NUM_MOTORS] = {
    // Left Motor (Motor 0)
    {
        /* pinPWM          */ PIN_LEFT_PWM,
        /* pinDIR          */ PIN_LEFT_DIR,
        /* pinCAP          */ PIN_LEFT_CAP,
        /* invertDir       */ INVERT_LEFT_DIR,
        /* pulseCountPtr   */ &leftPulseCount,
    },
    // Right Motor (Motor 1)
    {
        /* pinPWM          */ PIN_RIGHT_PWM,
        /* pinDIR          */ PIN_RIGHT_DIR,
        /* pinCAP          */ PIN_RIGHT_CAP,
        /* invertDir       */ INVERT_RIGHT_DIR,
        /* pulseCountPtr   */ &rightPulseCount,
    },
};

// ── Runtime State ────────────────────────────────────────────────────
// Holds the live PWM duty, previous pulse snapshot, instantaneous and filtered speeds.
MotorState motorStates[NUM_MOTORS] = {};

// Timestamp of the previous control loop execution in microseconds
static uint64_t lastControlTimeUs = 0;

// Shared commanded PWM value applied across both motors (or average in speed mode)
static int sharedPwmValue = 0;

// Active control mode: open-loop PWM (P command) or closed-loop speed regulation (S command)
static ControlMode currentControlMode = MODE_OPEN_LOOP_PWM;

// Target velocity in mm/sec for closed-loop speed regulation mode
static float targetSpeedMmS = 0.0f;

// ── Interrupt Service Routines ───────────────────────────────────────

/**
 * @brief Left motor CAP interrupt handler.
 * Executed in IRAM on every RISING edge of D6. Increments the raw pulse count.
 * Kept minimal to avoid ISR latency.
 */
void IRAM_ATTR onLeftPulseISR()
{
    leftPulseCount = leftPulseCount + 1;
}

/**
 * @brief Right motor CAP interrupt handler.
 * Executed in IRAM on every RISING edge of D7. Increments the raw pulse count.
 * Kept minimal to avoid ISR latency.
 */
void IRAM_ATTR onRightPulseISR()
{
    rightPulseCount = rightPulseCount + 1;
}

// ── Hardware Initialization ──────────────────────────────────────────

/**
 * @brief Configures GPIO pins, LEDC PWM generators, and attaches encoder interrupts.
 * 
 * Sets direction pins to OUTPUT with default level.
 * Attaches LEDC timers at 20 kHz with 8-bit resolution (0..255 duty cycle) to
 * ensure silent motor operation. Sets CAP pins as inputs with pull-ups and binds
 * the rising-edge interrupts to count FG pulses.
 */
void setupMotors()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        const MotorConfig &cfg = motorConfigs[i];
        MotorState &state = motorStates[i];

        // Reset runtime state values to zero
        state.pwmValue = 0;
        state.lastPulseSnap = 0;
        state.instSpeed = 0.0f;
        state.filteredSpeed = 0.0f;
        state.speedMmPerSec = 0.0f;
        state.integralError = 0.0f;

        // Configure direction pin
        pinMode(cfg.pinDIR, OUTPUT);
        digitalWrite(cfg.pinDIR, cfg.invertDir ? HIGH : LOW);

        // Attach LEDC PWM to the motor PWM pin (ESP32 Arduino 3.x API).
        // 20 kHz frequency keeps PWM switching outside human audible range.
        // NOTE: JGY2430 motor PWM input is inverted:
        //   duty 255 (3.3V HIGH) = STOP (0% speed)
        //   duty 0   (0V LOW)    = FULL SPEED (100% speed)
        ledcAttach(cfg.pinPWM, PWM_FREQ_HZ, PWM_RESOLUTION);
        ledcWrite(cfg.pinPWM, 255); // Initialize to STOP (duty 255)

        // Configure CAP pulse input pin with internal pull-up to ensure stable logic level
        pinMode(cfg.pinCAP, INPUT_PULLUP);
    }

    // Attach hardware interrupts on RISING edge of CAP feedback pins
    attachInterrupt(digitalPinToInterrupt(motorConfigs[MOTOR_LEFT].pinCAP), onLeftPulseISR, RISING);
    attachInterrupt(digitalPinToInterrupt(motorConfigs[MOTOR_RIGHT].pinCAP), onRightPulseISR, RISING);

    // Snapshot initial pulse counts and timestamp
    noInterrupts();
    motorStates[MOTOR_LEFT].lastPulseSnap = leftPulseCount;
    motorStates[MOTOR_RIGHT].lastPulseSnap = rightPulseCount;
    interrupts();

    lastControlTimeUs = esp_timer_get_time();
}

// ── Motor Control Functions ──────────────────────────────────────────

/**
 * @brief Sets PWM speed and direction for an individual motor.
 * 
 * Clamps the target PWM within [-255, +255].
 * Sets the direction pin level according to sign and inversion settings:
 *   - Positive (>0): forward direction
 *   - Negative (<0): reverse direction
 *   - Zero (0): zero speed, duty set to 0
 * 
 * @param motorIdx 0 for Left motor, 1 for Right motor.
 * @param targetPwm Target PWM speed (-255..+255).
 */
void setSingleMotorPWM(int motorIdx, int targetPwm)
{
    if (motorIdx < 0 || motorIdx >= NUM_MOTORS)
    {
        return;
    }

    // Clamp input command to hardware bounds
    if (targetPwm > PWM_MAX) targetPwm = PWM_MAX;
    if (targetPwm < PWM_MIN) targetPwm = PWM_MIN;

    const MotorConfig &cfg = motorConfigs[motorIdx];
    MotorState &state = motorStates[motorIdx];
    state.pwmValue = targetPwm;

    // Determine direction pin level and PWM magnitude.
    // JGY2430 PWM input is inverted: duty 255 = STOP, duty 0 = FULL SPEED.
    uint32_t dutyMagnitude = (targetPwm < 0) ? (uint32_t)(-targetPwm) : (uint32_t)targetPwm;
    if (dutyMagnitude > PWM_MAX) dutyMagnitude = PWM_MAX;
    uint32_t ledcDuty = 255 - dutyMagnitude;

    uint8_t dirLevel = LOW;

    if (targetPwm > 0)
    {
        // Normal forward is LOW; if inverted, forward is HIGH
        dirLevel = cfg.invertDir ? HIGH : LOW;
    }
    else if (targetPwm < 0)
    {
        // Normal reverse is HIGH; if inverted, reverse is LOW
        dirLevel = cfg.invertDir ? LOW : HIGH;
    }
    else
    {
        // Zero speed: full stop
        dirLevel = cfg.invertDir ? HIGH : LOW;
    }

    // Apply direction before writing PWM duty to prevent brief reverse jolts
    digitalWrite(cfg.pinDIR, dirLevel);
    ledcWrite(cfg.pinPWM, ledcDuty);
}

/**
 * @brief Sets PWM speed and direction for both motors simultaneously.
 * 
 * Switches system mode to open-loop PWM (MODE_OPEN_LOOP_PWM), resets
 * target speed, zeroes integral accumulation, and updates both motors.
 * 
 * @param targetPwm Target PWM speed (-255..+255).
 */
void setBothMotorsPWM(int targetPwm)
{
    // Clamp to valid envelope
    if (targetPwm > PWM_MAX) targetPwm = PWM_MAX;
    if (targetPwm < PWM_MIN) targetPwm = PWM_MIN;

    // Direct PWM commands switch mode to open loop and clear speed controller state
    currentControlMode = MODE_OPEN_LOOP_PWM;
    targetSpeedMmS     = 0.0f;
    motorStates[MOTOR_LEFT].integralError  = 0.0f;
    motorStates[MOTOR_RIGHT].integralError = 0.0f;

    sharedPwmValue = targetPwm;

    setSingleMotorPWM(MOTOR_LEFT, targetPwm);
    setSingleMotorPWM(MOTOR_RIGHT, targetPwm);
}

/**
 * @brief Stops both motors immediately, clears target speed, and resets controller states.
 */
void stopAllMotors()
{
    currentControlMode = MODE_OPEN_LOOP_PWM;
    targetSpeedMmS     = 0.0f;
    motorStates[MOTOR_LEFT].targetSpeedMmS  = 0.0f;
    motorStates[MOTOR_RIGHT].targetSpeedMmS = 0.0f;
    motorStates[MOTOR_LEFT].integralError   = 0.0f;
    motorStates[MOTOR_RIGHT].integralError  = 0.0f;

    setBothMotorsPWM(0);
}

/**
 * @brief Sets closed-loop target velocity in mm/sec.
 * 
 * Clamps input to the safe operational envelope [-120.0, +120.0] mm/s.
 * Switches controlMode to MODE_CLOSED_LOOP_SPEED and resets integral error
 * to ensure a smooth, bumpless transition into closed-loop regulation.
 * If target speed is near zero (< 0.1 mm/s), motors are immediately stopped.
 * 
 * @param targetMmS Target forward (+) or reverse (-) speed in mm/sec.
 */
void setTargetSpeedMmPerSec(float targetMmS)
{
    // Clamp to hardware and vehicle safety envelope [-120, +120] mm/s
    if (targetMmS > SPEED_CMD_MAX_MM_S) targetMmS = SPEED_CMD_MAX_MM_S;
    if (targetMmS < SPEED_CMD_MIN_MM_S) targetMmS = SPEED_CMD_MIN_MM_S;

    if (fabsf(targetMmS) < 0.1f)
    {
        stopAllMotors();
        return;
    }

    targetSpeedMmS = targetMmS;
    motorStates[MOTOR_LEFT].targetSpeedMmS  = targetMmS;
    motorStates[MOTOR_RIGHT].targetSpeedMmS = targetMmS;
    currentControlMode = MODE_CLOSED_LOOP_SPEED;

    // Reset integral terms on target speed transition to prevent overshoot
    motorStates[MOTOR_LEFT].integralError  = 0.0f;
    motorStates[MOTOR_RIGHT].integralError = 0.0f;
}

/**
 * @brief Sets individual closed-loop target velocities for Left and Right motors in mm/sec.
 * 
 * Enables differential steering while maintaining closed-loop speed regulation.
 * 
 * @param leftTargetMmS Left wheel target speed in mm/sec (clamped to [-120, +120]).
 * @param rightTargetMmS Right wheel target speed in mm/sec (clamped to [-120, +120]).
 */
void setIndividualTargetSpeedsMmPerSec(float leftTargetMmS, float rightTargetMmS)
{
    if (leftTargetMmS > SPEED_CMD_MAX_MM_S)  leftTargetMmS = SPEED_CMD_MAX_MM_S;
    if (leftTargetMmS < SPEED_CMD_MIN_MM_S)  leftTargetMmS = SPEED_CMD_MIN_MM_S;
    if (rightTargetMmS > SPEED_CMD_MAX_MM_S) rightTargetMmS = SPEED_CMD_MAX_MM_S;
    if (rightTargetMmS < SPEED_CMD_MIN_MM_S) rightTargetMmS = SPEED_CMD_MIN_MM_S;

    if (fabsf(leftTargetMmS) < 0.1f && fabsf(rightTargetMmS) < 0.1f)
    {
        stopAllMotors();
        return;
    }

    // Reset integral error when a motor reverses target direction
    // to prevent accumulated forward windup from delaying reverse response
    if (leftTargetMmS * motorStates[MOTOR_LEFT].targetSpeedMmS < 0.0f)
    {
        motorStates[MOTOR_LEFT].integralError = 0.0f;
    }
    if (rightTargetMmS * motorStates[MOTOR_RIGHT].targetSpeedMmS < 0.0f)
    {
        motorStates[MOTOR_RIGHT].integralError = 0.0f;
    }

    targetSpeedMmS = (leftTargetMmS + rightTargetMmS) * 0.5f;
    motorStates[MOTOR_LEFT].targetSpeedMmS  = leftTargetMmS;
    motorStates[MOTOR_RIGHT].targetSpeedMmS = rightTargetMmS;
    currentControlMode = MODE_CLOSED_LOOP_SPEED;
}

// ── 100 Hz Control Loop & Filter Execution ───────────────────────────

/**
 * @brief Main 100 Hz control loop step for speed calculation, filtering, and regulation.
 * 
 * Executed every 10 ms (10,000 us).
 * 1. Atomically captures pulse counters to avoid ISR race conditions.
 * 2. Computes elapsed time dt in seconds for precise rate scaling.
 * 3. Derives instantaneous speed = delta_pulses / dt (steps/sec).
 * 4. Signs instantaneous speed based on active direction (negative for reverse).
 * 5. Applies Exponential Moving Average (EMA) filter with alpha = 0.24.
 * 6. Converts filtered steps/sec to physical wheel speed in mm/sec.
 * 7. In closed-loop speed mode:
 *    - Calculates speed error in mm/sec.
 *    - Applies feedforward baseline (K_ff * target_speed).
 *    - Applies proportional correction (Kp * error).
 *    - Applies anti-windup integration (conditional integration + hard integral clamping).
 *    - Writes resulting PWM to Left and Right motors.
 * 
 * @param nowUs Current system timestamp in microseconds from esp_timer_get_time().
 */
void updateMotorControlLoop(uint64_t nowUs)
{
    // Compute elapsed time in seconds since previous control cycle
    float dt = (float)(nowUs - lastControlTimeUs) / 1000000.0f;
    lastControlTimeUs = nowUs;

    // Guard against anomalous non-positive dt values
    if (dt <= 0.0001f)
    {
        dt = (float)CONTROL_INTERVAL_US / 1000000.0f; // 0.010 s fallback
    }

    // Atomically snapshot pulse counters to ensure consistency
    noInterrupts();
    unsigned long snapLeft = leftPulseCount;
    unsigned long snapRight = rightPulseCount;
    interrupts();

    // ── Update Left Motor Speed ──────────────────────────────────────
    {
        MotorState &state = motorStates[MOTOR_LEFT];
        unsigned long deltaPulses = snapLeft - state.lastPulseSnap;
        state.lastPulseSnap = snapLeft;

        // Calculate raw instantaneous step rate (steps/sec)
        float instRate = (float)deltaPulses / dt;

        // Assign directional sign: negative when commanded motion is reverse
        bool isReverse = false;
        if (currentControlMode == MODE_CLOSED_LOOP_SPEED)
        {
            isReverse = (state.targetSpeedMmS < 0.0f) || (state.pwmValue < 0);
        }
        else
        {
            isReverse = (state.pwmValue < 0);
        }

        if (isReverse)
        {
            instRate = -instRate;
        }

        state.instSpeed = instRate;

        // Exponential Moving Average filter: EMA = alpha * inst + (1 - alpha) * prev_EMA
        state.filteredSpeed = EMA_ALPHA * instRate + (1.0f - EMA_ALPHA) * state.filteredSpeed;

        // Convert steps/sec to mm/sec using wheel and gearbox geometry
        state.speedMmPerSec = state.filteredSpeed * MM_PER_STEP;

        // Clean deadband cutoff: snap residual floating point values to 0 when stopped
        if (state.pwmValue == 0 && fabsf(state.filteredSpeed) < 0.05f)
        {
            state.filteredSpeed = 0.0f;
            state.speedMmPerSec = 0.0f;
        }
    }

    // ── Update Right Motor Speed ─────────────────────────────────────
    {
        MotorState &state = motorStates[MOTOR_RIGHT];
        unsigned long deltaPulses = snapRight - state.lastPulseSnap;
        state.lastPulseSnap = snapRight;

        // Calculate raw instantaneous step rate (steps/sec)
        float instRate = (float)deltaPulses / dt;

        // Assign directional sign: negative when commanded motion is reverse
        bool isReverse = false;
        if (currentControlMode == MODE_CLOSED_LOOP_SPEED)
        {
            isReverse = (state.targetSpeedMmS < 0.0f) || (state.pwmValue < 0);
        }
        else
        {
            isReverse = (state.pwmValue < 0);
        }

        if (isReverse)
        {
            instRate = -instRate;
        }

        state.instSpeed = instRate;

        // Exponential Moving Average filter: EMA = alpha * inst + (1 - alpha) * prev_EMA
        state.filteredSpeed = EMA_ALPHA * instRate + (1.0f - EMA_ALPHA) * state.filteredSpeed;

        // Convert steps/sec to mm/sec using wheel and gearbox geometry
        state.speedMmPerSec = state.filteredSpeed * MM_PER_STEP;

        // Clean deadband cutoff: snap residual floating point values to 0 when stopped
        if (state.pwmValue == 0 && fabsf(state.filteredSpeed) < 0.05f)
        {
            state.filteredSpeed = 0.0f;
            state.speedMmPerSec = 0.0f;
        }
    }

    // ── Closed-Loop Speed Controller (Feedforward + PI) ───────────────
    if (currentControlMode == MODE_CLOSED_LOOP_SPEED)
    {
        if (fabsf(motorStates[MOTOR_LEFT].targetSpeedMmS) < 0.1f && fabsf(motorStates[MOTOR_RIGHT].targetSpeedMmS) < 0.1f)
        {
            // Zero target speed: full stop and reset integrals
            setSingleMotorPWM(MOTOR_LEFT, 0);
            setSingleMotorPWM(MOTOR_RIGHT, 0);
            sharedPwmValue = 0;
            motorStates[MOTOR_LEFT].integralError  = 0.0f;
            motorStates[MOTOR_RIGHT].integralError = 0.0f;
        }
        else
        {
            for (int i = 0; i < NUM_MOTORS; i++)
            {
                MotorState &state = motorStates[i];

                if (fabsf(state.targetSpeedMmS) < 0.1f)
                {
                    setSingleMotorPWM(i, 0);
                    state.integralError = 0.0f;
                    continue;
                }

                // Feedforward baseline derived from calibrated hardware speed sweep:
                // Accounts for ~95% of required PWM, leaving PI to handle load and battery drift
                const float ffPwm = SPEED_FF_GAIN * state.targetSpeedMmS;

                // Velocity tracking error in physical mm/sec: e = target - measured
                const float errorMmS = state.targetSpeedMmS - state.speedMmPerSec;

                // Proportional term: P = Kp * error
                const float propPwm = SPEED_KP * errorMmS;

                // Anti-windup conditional integration:
                // If output would saturate at +/-PWM_MAX and error pushes deeper into saturation,
                // do not integrate. This prevents integral blowup if battery voltage is low
                // and the target speed is physically unachievable.
                const float tentativePwm = ffPwm + propPwm + state.integralError;
                const bool saturatedHigh = (tentativePwm >= (float)PWM_MAX) && (errorMmS > 0.0f);
                const bool saturatedLow  = (tentativePwm <= (float)PWM_MIN) && (errorMmS < 0.0f);

                if (!saturatedHigh && !saturatedLow)
                {
                    // Accumulate integral error: I = I + Ki * error * dt
                    state.integralError += SPEED_KI * errorMmS * dt;

                    // Hard clamp on integral contribution to protect against sustained windup
                    if (state.integralError > SPEED_INTEGRAL_MAX)
                    {
                        state.integralError = SPEED_INTEGRAL_MAX;
                    }
                    else if (state.integralError < -SPEED_INTEGRAL_MAX)
                    {
                        state.integralError = -SPEED_INTEGRAL_MAX;
                    }
                }

                // Total controller command: u = FF + P + I
                const float totalOutput = ffPwm + propPwm + state.integralError;
                int outPwm = (int)roundf(totalOutput);

                // Clamp to physical hardware PWM envelope [-255, +255]
                if (outPwm > PWM_MAX) outPwm = PWM_MAX;
                if (outPwm < PWM_MIN) outPwm = PWM_MIN;

                setSingleMotorPWM(i, outPwm);
            }

            // In speed mode, report average active motor PWM for telemetry
            sharedPwmValue = (motorStates[MOTOR_LEFT].pwmValue + motorStates[MOTOR_RIGHT].pwmValue) / 2;
        }
    }
}

// ── Telemetry Accessors ──────────────────────────────────────────────

/**
 * @brief Returns the shared commanded PWM value.
 */
int getCommandedPWM()
{
    return sharedPwmValue;
}

/**
 * @brief Returns the EMA-filtered speed of the Left motor in steps/sec.
 */
float getLeftMotorSpeed()
{
    return motorStates[MOTOR_LEFT].filteredSpeed;
}

/**
 * @brief Returns the EMA-filtered speed of the Right motor in steps/sec.
 */
float getRightMotorSpeed()
{
    return motorStates[MOTOR_RIGHT].filteredSpeed;
}

/**
 * @brief Returns the EMA-filtered speed of the Left motor in mm/sec.
 */
float getLeftMotorSpeedMmS()
{
    return motorStates[MOTOR_LEFT].speedMmPerSec;
}

/**
 * @brief Returns the EMA-filtered speed of the Right motor in mm/sec.
 */
float getRightMotorSpeedMmS()
{
    return motorStates[MOTOR_RIGHT].speedMmPerSec;
}

/**
 * @brief Returns the current closed-loop target speed in mm/sec.
 */
float getTargetSpeedMmPerSec()
{
    return targetSpeedMmS;
}

/**
 * @brief Returns the current active control mode.
 */
ControlMode getControlMode()
{
    return currentControlMode;
}

/**
 * @brief Explicitly switches the control mode.
 */
void setControlMode(ControlMode mode)
{
    currentControlMode = mode;
}

/**
 * @brief Converts pulse rate in steps/sec to linear speed in mm/sec.
 */
float stepsPerSecToMmPerSec(float stepsPerSec)
{
    return stepsPerSec * MM_PER_STEP;
}

/**
 * @brief Converts linear speed in mm/sec to pulse rate in steps/sec.
 */
float mmPerSecToStepsPerSec(float mmPerSec)
{
    return mmPerSec * STEPS_PER_MM;
}
