#pragma once
#include <stdint.h>

// Project baseline: docs/motor-settings.md. Applications reapply these over I2C.
namespace MotorSettings {
constexpr uint8_t type = 1;
constexpr uint16_t deadZone = 1650, pulseLine = 2000, pulsePhase = 23;
constexpr float diameterMm = 65.0f;
// No documented I2C PID access: these gains are assumed stored, not written/read.
constexpr float storedP = 3.0f, storedI = 0.375f, storedD = 0.5f;
}
