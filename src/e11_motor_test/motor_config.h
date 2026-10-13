// motor_config.h — Configuration constants and data structures for dual BLDC motor test.
//
// Defines hardware pin mappings for Seeed Studio XIAO ESP32-S3, PWM parameters,
// control loop rates, and runtime state structures for each motor.

#pragma once

#include <Arduino.h>

// Number of drive motors (Left = 0, Right = 1).
constexpr int NUM_MOTORS  = 2;
constexpr int MOTOR_LEFT  = 0;
constexpr int MOTOR_RIGHT = 1;

// ── Hardware Pin Assignments (Seeed XIAO ESP32-S3) ───────────────────
// Hardware pin mappings per docs/hardware.md:
//   Left Motor  : PWM=D0 (GPIO1), CW=D2 (GPIO3), CAP=D6 (GPIO43)
//   Right Motor : PWM=D1 (GPIO2), CW=D3 (GPIO4), CAP=D7 (GPIO44)
constexpr uint8_t PIN_LEFT_PWM  = D0;
constexpr uint8_t PIN_LEFT_DIR  = D2;
constexpr uint8_t PIN_LEFT_CAP  = D6;

constexpr uint8_t PIN_RIGHT_PWM = D1;
constexpr uint8_t PIN_RIGHT_DIR = D3;
constexpr uint8_t PIN_RIGHT_CAP = D7;

// ── PWM Hardware Parameters ──────────────────────────────────────────
// 20 kHz PWM frequency keeps switching noise outside audible human range.
// 8-bit resolution matches the 0..255 duty cycle range.
constexpr uint32_t PWM_FREQ_HZ    = 20000;
constexpr uint8_t  PWM_RESOLUTION = 8;
constexpr int      PWM_MAX        = 255;
constexpr int      PWM_MIN        = -255;

// ── Control Loop & Filter Timing ─────────────────────────────────────
// Main control loop runs at 100 Hz (10,000 microseconds / 10 ms).
constexpr uint64_t CONTROL_INTERVAL_US = 10000;

// Exponential Moving Average (EMA) smoothing factor:
// EMA_t = alpha * Instantaneous + (1 - alpha) * EMA_{t-1}
// Configured with alpha = 0.24 at 100 Hz control loop to smooth out discrete pulse steps.
constexpr float EMA_ALPHA = 0.24f;

// Telemetry output period: 0.2 seconds (200,000 microseconds / 5 Hz).
constexpr uint64_t REPORT_INTERVAL_US = 200000;

// Direction pin active levels:
// LOW = forward (CW), HIGH = reverse (CCW).
// If motor mounting orientation requires inverting a motor, toggle these flags.
constexpr bool INVERT_LEFT_DIR  = false;
constexpr bool INVERT_RIGHT_DIR = false;

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
};
