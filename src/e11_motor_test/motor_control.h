// motor_control.h — Public interface for dual BLDC motor control and speed estimation.
//
// Provides functions to initialize motor pins/LEDC, set motor PWM, service the
// 100 Hz speed calculation control loop with EMA filtering, and read speed telemetry.

#pragma once

#include "motor_config.h"

// External pulse counters incremented inside CAP pin ISRs
extern volatile unsigned long leftPulseCount;
extern volatile unsigned long rightPulseCount;

// External per-motor configuration and runtime state arrays
extern const MotorConfig motorConfigs[NUM_MOTORS];
extern MotorState motorStates[NUM_MOTORS];

// Hardware ISR handler for Left motor CAP pulse edge detection
void onLeftPulseISR();

// Hardware ISR handler for Right motor CAP pulse edge detection
void onRightPulseISR();

// Initializes motor GPIO pins, configures LEDC PWM channels, and attaches interrupts
void setupMotors();

// Sets PWM speed and direction for a specific motor (-255..255)
void setSingleMotorPWM(int motorIdx, int targetPwm);

// Sets PWM speed and direction for both motors simultaneously (-255..255)
void setBothMotorsPWM(int targetPwm);

// Stops both motors immediately (PWM = 0)
void stopAllMotors();

// Executes one tick of the 100 Hz control loop:
// computes instantaneous steps/sec and updates the EMA filter
void updateMotorControlLoop(uint64_t nowUs);

// Returns the current commanded PWM value for both motors
int getCommandedPWM();

// Returns the EMA filtered speed (steps/sec) for Left motor
float getLeftMotorSpeed();

// Returns the EMA filtered speed (steps/sec) for Right motor
float getRightMotorSpeed();
