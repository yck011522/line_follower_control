// motor_config.h — Configuration constants and data structures for E12 dual BLDC motor, line sensor, and NFC.
//
// Defines hardware pin mappings for Seeed Studio XIAO ESP32-S3, PWM parameters,
// physical wheel/gearbox constants, control loop rates, and runtime state structures.

#pragma once

#include <Arduino.h>

// Number of drive motors (Left = 0, Right = 1).
constexpr int NUM_MOTORS  = 2;
constexpr int MOTOR_LEFT  = 0;
constexpr int MOTOR_RIGHT = 1;

// ── Hardware Pin Assignments (Seeed XIAO ESP32-S3) ───────────────────
// Motor & Feedback Pin mappings per docs/hardware.md:
//   Left Motor  : PWM=D0 (GPIO1), CW=D2 (GPIO3), CAP=D6 (GPIO43)
//   Right Motor : PWM=D1 (GPIO2), CW=D3 (GPIO4), CAP=D7 (GPIO44)
constexpr uint8_t PIN_LEFT_PWM  = D0;
constexpr uint8_t PIN_LEFT_DIR  = D2;
constexpr uint8_t PIN_LEFT_CAP  = D6;

constexpr uint8_t PIN_RIGHT_PWM = D1;
constexpr uint8_t PIN_RIGHT_DIR = D3;
constexpr uint8_t PIN_RIGHT_CAP = D7;

// Shared I2C bus pins: D4 (SDA), D5 (SCL)
constexpr uint8_t PIN_I2C_SDA   = D4;
constexpr uint8_t PIN_I2C_SCL   = D5;
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint16_t I2C_TIMEOUT_MS = 1;

// Line Sensor I2C Address (0x12) & register (0x30)
constexpr uint8_t LINE_SENSOR_I2C_ADDR = 0x12;
constexpr uint8_t LINE_SENSOR_REG_DATA = 0x30;

// NFC FM17550 I2C Address (0x28)
constexpr uint8_t NFC_I2C_ADDR         = 0x28;

// Direction pin active levels:
// Left motor physical mounting is inverted relative to Right motor:
// Left motor requires HIGH for forward (CW), Right motor requires LOW for forward (CCW).
constexpr bool INVERT_LEFT_DIR  = true;
constexpr bool INVERT_RIGHT_DIR = false;

// ── PWM Hardware Parameters ──────────────────────────────────────────
// 20 kHz PWM frequency keeps switching noise outside audible human range.
// 8-bit resolution matches the 0..255 duty cycle range.
constexpr uint32_t PWM_FREQ_HZ    = 20000;
constexpr uint8_t  PWM_RESOLUTION = 8;
constexpr int      PWM_MAX        = 255;
constexpr int      PWM_MIN        = -255;

// ── Control Loop & Reporting Timing ──────────────────────────────────
// Main control loop runs at 100 Hz (10,000 microseconds / 10 ms).
constexpr uint64_t CONTROL_INTERVAL_US = 10000;

// Telemetry output period: 0.1 seconds (100,000 microseconds / 10 Hz).
constexpr uint64_t REPORT_INTERVAL_US  = 100000;

// NFC tag detection timeout window for telemetry: 1.0 second (1,000,000 us).
constexpr uint64_t NFC_TAG_ACTIVE_TIMEOUT_US = 1000000;

// Exponential Moving Average (EMA) smoothing factor:
// EMA_t = alpha * Instantaneous + (1 - alpha) * EMA_{t-1}
// Configured with alpha = 0.24 at 100 Hz control loop to smooth out discrete pulse steps.
constexpr float EMA_ALPHA = 0.24f;

// ── Control Modes ────────────────────────────────────────────────────
enum ControlMode : uint8_t
{
    MODE_OPEN_LOOP_PWM     = 0, // Direct PWM control (P, L, R commands)
    MODE_CLOSED_LOOP_SPEED = 1  // Closed-loop velocity control (S command)
};

// ── Physical Wheel & Gearbox Parameters ──────────────────────────────
// Motor & Wheel geometry per docs/hardware.md:
//   - JGY2430 12V Brushless Worm Gear Motor: 150:1 gear ratio
//   - Motor rotor encoder: 9 pulses / rotor rev -> 1350 pulses / wheel rev
//   - Drive wheel diameter: 66.5 mm -> Circumference = pi * 66.5 = 208.9159 mm
//   - Conversion:
//       steps_per_mm = 1350 / (pi * 66.5) = 6.461947 steps/mm
//       mm_per_step  = (pi * 66.5) / 1350 = 0.1547525 mm/step
constexpr float WHEEL_DIAMETER_MM      = 66.5f;
constexpr float GEAR_RATIO             = 150.0f;
constexpr float ENCODER_PPR            = 9.0f;
constexpr float STEPS_PER_WHEEL_REV    = GEAR_RATIO * ENCODER_PPR; // 1350.0f
constexpr float WHEEL_CIRCUMFERENCE_MM = 3.14159265358979323846f * WHEEL_DIAMETER_MM;
constexpr float STEPS_PER_MM           = STEPS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM; // ~6.461947f
constexpr float MM_PER_STEP            = WHEEL_CIRCUMFERENCE_MM / STEPS_PER_WHEEL_REV; // ~0.1547525f

// Command envelope for target speed in mm/sec
constexpr float SPEED_CMD_MIN_MM_S     = -120.0f;
constexpr float SPEED_CMD_MAX_MM_S     =  120.0f;

// ── PI + Feedforward Speed Controller Gains ──────────────────────────
// Feedforward gain (K_ff): PWM units per (mm/sec).
// Hardware sweep: 100 mm/s (~646 steps/s) requires ~198 PWM -> K_ff = 1.98 PWM/(mm/s).
constexpr float SPEED_FF_GAIN          = 1.98f;

// Proportional gain (Kp): PWM correction per (mm/sec) speed error.
constexpr float SPEED_KP               = 0.60f;

// Integral gain (Ki): PWM accumulation per (mm/sec * sec) integrated error.
constexpr float SPEED_KI               = 1.50f;

// Anti-windup integral clamp: maximum absolute PWM contribution from the I term.
// Protects against integral blowup if battery voltage sags or load prevents reaching target.
constexpr float SPEED_INTEGRAL_MAX     = 50.0f;

// ── Line Following Steering Parameters ───────────────────────────────
// Line steering proportional gain: multiplier on normalized line error [-1.0..+1.0].
// At steerMag = 1.0, the inner wheel slows to zero (pivot around inner wheel).
// At steerMag >= 2.0, the inner wheel reaches full reverse (-vOuter) for in-place spin turn.
constexpr float LINE_STEER_KP          = 2.4f;

// Line steering derivative gain: damping on rate of change of normalized line error.
constexpr float LINE_STEER_KD          = 0.05f;

// ── Per-Motor Configuration ──────────────────────────────────────────
struct MotorConfig
{
    uint8_t pinPWM;                        // LEDC PWM output pin
    uint8_t pinDIR;                        // Direction output pin (CW/CCW)
    uint8_t pinCAP;                        // Feedback pulse input pin (RISING edge IRQ)
    bool invertDir;                        // True if physical mounting reverses forward motion
    volatile unsigned long *pulseCountPtr; // Pointer to ISR-incremented pulse counter
};

// ── Per-Motor Runtime State ──────────────────────────────────────────
struct MotorState
{
    int pwmValue;                  // Current commanded PWM duty (-255..+255)
    unsigned long lastPulseSnap;   // Pulse counter snapshot from previous control cycle
    float instSpeed;               // Instantaneous speed (steps/sec) in latest cycle
    float filteredSpeed;           // EMA-filtered speed (steps/sec)
    float speedMmPerSec;           // Filtered speed in mm/sec
    float targetSpeedMmS;          // Active target velocity in mm/sec for closed-loop regulation
    float integralError;           // Accumulated integral error for PI speed controller
};
