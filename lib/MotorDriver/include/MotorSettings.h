#pragma once
#include <stdint.h>

// Adopted baseline: docs/motor-settings.md. Reapplied over I2C at each boot.
namespace MotorSettings
{
  constexpr uint8_t motorType = 1; // Driver's 520-motor preset.
  constexpr uint16_t pwmDeadZone = 1650;
  // Pulses per MOTOR revolution, before the driver's x4 quadrature decoding.
  constexpr uint16_t encoderPulsesPerMotorRevolution = 500;
  constexpr uint16_t gearRatio = 23; // Integer setting written to the driver.
  constexpr float wheelDiameterMm = 65.0f;

  // Independent E2 feedback scale: one hand-turned WHEEL revolution measured
  // 44998 quadrature counts. Do not replace with 500 * 4 * configured ratio 23.
  constexpr uint32_t measuredEncoderCountsPerWheelRevolution = 44998;
  constexpr float pi = 3.14159265358979323846f;
  constexpr float millimetersPerEncoderCount =
      pi * wheelDiameterMm / measuredEncoderCountsPerWheelRevolution;

  // No I2C PID registers are documented. These are saved/assumed, not verified.
  constexpr float storedP = 3.0f;
  constexpr float storedI = 0.375f;
  constexpr float storedD = 0.5f;
}
