#pragma once

#include <Arduino.h>
#include <Wire.h>

// Robot-facing motor operations. The application initializes Wire, chooses the
// update schedule, and requires successful initialize() before commanding motion.
class MotorDriver
{
public:
  // Physical board assignments. These names do not imply the command sign
  // is vehicle-forward: mounted wheel directions have not yet been mapped.
  enum class Wheel : uint8_t { Left = 2, Right = 4 };

  // Attach an already-owned I2C bus; hardware configuration happens explicitly
  // in initialize(), after the application's Wire.begin() in setup().
  explicit MotorDriver(TwoWire &bus) : bus_(bus) {}

  // Reapply baseline settings at each boot, or reconfigure while stopped.
  // Releases outputs before/after; about 500 ms of save waits. PID stays stored.
  bool initialize();

  // Closed-loop wheel targets in mm/s, as supported by E2. Driver range is
  // -1000..1000; the caller supplies values in range. M1/M3 are always zero.
  bool setWheelSpeedsMmPerSecond(int16_t leftMmPerSecond, int16_t rightMmPerSecond);

  // Estimate speeds from signed encoder counts over the driver's last 10 ms.
  // Uses nominal wheel diameter and measured counts/revolution from E2, not
  // the driver's configured integer gear ratio. Outputs change only on success.
  bool readWheelSpeedsMmPerSecond(float &leftMmPerSecond, float &rightMmPerSecond);

  // Set all PWM outputs to zero, leaving speed PID mode. This differs from
  // a zero-speed command, which keeps PID holding the wheels. ACK is not stop proof.
  bool releaseMotorOutputs();

  // Signed accumulated encoder position in counts, for E2 and later odometry.
  // Uses high/low/high reads; roughly 0.5 ms per wheel on the earlier bench.
  bool readEncoderPosition(Wheel wheel, int32_t &encoderCounts);

  // Raw feedback used by E2's existing CSV schema: counts during the last 10 ms.
  // Integration normally uses readWheelSpeedsMmPerSecond() instead.
  bool readEncoderCountLast10Ms(Wheel wheel, int16_t &encoderCounts);

  // Wire transmission code, or 0x80 for short read / 0x81 for incoherent position.
  uint8_t lastCommunicationError() const { return communicationError_; }

private:
  // Write register address followed by payload bytes to the motor board.
  bool writeRegisterBytes(uint8_t registerAddress, const uint8_t *payload, size_t payloadSize);

  // Decode a two-byte register with the verified repeated-START and big-endian order.
  bool readRegister16BigEndian(uint8_t registerAddress, uint16_t &value);

  // Assemble the high/low position registers, retrying once if the high word changes.
  bool readEncoderCounter32(uint8_t highWordRegister, uint32_t &encoderCounts);

  // Encode the four-channel command packet; this vehicle drives only M2/M4.
  bool writeFourMotorCommands(uint8_t registerAddress, int16_t leftCommand, int16_t rightCommand);

  // Encode one 16-bit configuration parameter in the verified big-endian order.
  bool writeConfiguration16(uint8_t registerAddress, uint16_t value);

  // Encode wheel diameter as the protocol's little-endian 32-bit float.
  bool writeWheelDiameterMm(float diameterMm);

  TwoWire &bus_;
  uint8_t communicationError_ = 0;
  static constexpr uint8_t kI2cAddress = 0x26;
  static constexpr uint8_t kMotorTypeRegister = 0x01;
  static constexpr uint8_t kPwmDeadZoneRegister = 0x02;
  static constexpr uint8_t kEncoderPulsesRegister = 0x03;
  static constexpr uint8_t kGearRatioRegister = 0x04;
  static constexpr uint8_t kWheelDiameterRegister = 0x05;
  static constexpr uint8_t kWheelSpeedRegister = 0x06;
  static constexpr uint8_t kMotorPwmRegister = 0x07;
  static constexpr uint8_t kEncoder10MsRegisterBase = 0x10;
  static constexpr uint8_t kEncoderPositionRegisterBase = 0x20;
  static constexpr uint8_t kShortReadError = 0x80;
  static constexpr uint8_t kIncoherentPositionError = 0x81;
  static constexpr uint8_t kMotorChannels = 4;
  static constexpr uint8_t kBytesPerWord = 2;
  static constexpr uint8_t kBitsPerByte = 8;
  static constexpr uint8_t kBitsPerWord = 16;
  static constexpr uint8_t kPositionReadAttempts = 2;
  static constexpr uint32_t kConfigurationSaveDelayMs = 100;
  static constexpr float kEncoderWindowSeconds = 0.010f;
};
