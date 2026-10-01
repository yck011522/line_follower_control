#include "MotorDriver.h"
#include "MotorSettings.h"
#include <cstring>

// Send [register][payload...] as one transaction to the motor board.
bool MotorDriver::writeRegisterBytes(uint8_t registerAddress, const uint8_t *payload, size_t payloadSize)
{
  bus_.beginTransmission(kI2cAddress);
  bus_.write(registerAddress);
  bus_.write(payload, payloadSize);
  communicationError_ = bus_.endTransmission();
  return communicationError_ == 0;
}

// Select one register and decode its two-byte value. Bench reads require a
// repeated START: inserting a STOP before requestFrom() returned zero values.
bool MotorDriver::readRegister16BigEndian(uint8_t registerAddress, uint16_t &value)
{
  bus_.beginTransmission(kI2cAddress);
  bus_.write(registerAddress);
  communicationError_ = bus_.endTransmission(false);
  if (communicationError_ != 0) return false;
  if (bus_.requestFrom(kI2cAddress, static_cast<size_t>(kBytesPerWord), true) != kBytesPerWord)
  {
    while (bus_.available()) bus_.read(); // Discard partial feedback before the next request.
    communicationError_ = kShortReadError;
    return false;
  }
  const uint8_t highByte = bus_.read();
  const uint8_t lowByte = bus_.read();
  value = (static_cast<uint16_t>(highByte) << kBitsPerByte) | lowByte;
  return true;
}

// Read high/low/high so a low-word rollover cannot combine mismatched words.
// Retry once when high changes. Three transactions per attempt make this more
// expensive than the 10 ms feedback; retain it for measurements and odometry.
bool MotorDriver::readEncoderCounter32(uint8_t highWordRegister, uint32_t &encoderCounts)
{
  for (uint8_t attempt = 0; attempt < kPositionReadAttempts; ++attempt)
  {
    uint16_t highWord, lowWord, highWordCheck;
    if (!readRegister16BigEndian(highWordRegister, highWord) ||
        !readRegister16BigEndian(highWordRegister + 1, lowWord) ||
        !readRegister16BigEndian(highWordRegister, highWordCheck)) return false;
    if (highWord == highWordCheck)
    {
      encoderCounts = (static_cast<uint32_t>(highWord) << kBitsPerWord) | lowWord;
      return true;
    }
  }
  communicationError_ = kIncoherentPositionError;
  return false;
}

// Encode signed commands into M1..M4 big-endian words. Zero-fill unused M1/M3;
// left/right map to the board's physical M2/M4 channels.
bool MotorDriver::writeFourMotorCommands(uint8_t registerAddress, int16_t leftCommand, int16_t rightCommand)
{
  uint8_t payload[kMotorChannels * kBytesPerWord] = {};
  const uint16_t leftBits = static_cast<uint16_t>(leftCommand);
  const uint16_t rightBits = static_cast<uint16_t>(rightCommand);
  const size_t leftOffset = (static_cast<uint8_t>(Wheel::Left) - 1) * kBytesPerWord;
  const size_t rightOffset = (static_cast<uint8_t>(Wheel::Right) - 1) * kBytesPerWord;
  payload[leftOffset] = static_cast<uint8_t>(leftBits >> kBitsPerByte);
  payload[leftOffset + 1] = static_cast<uint8_t>(leftBits);
  payload[rightOffset] = static_cast<uint8_t>(rightBits >> kBitsPerByte);
  payload[rightOffset + 1] = static_cast<uint8_t>(rightBits);
  return writeRegisterBytes(registerAddress, payload, sizeof(payload));
}

// Command closed-loop targets in E2-established mm/s units, not raw PWM.
bool MotorDriver::setWheelSpeedsMmPerSecond(int16_t leftMmPerSecond, int16_t rightMmPerSecond)
{
  return writeFourMotorCommands(kWheelSpeedRegister, leftMmPerSecond, rightMmPerSecond);
}

// Zero PWM releases speed-PID control; zero SPEED instead keeps PID active.
bool MotorDriver::releaseMotorOutputs()
{
  return writeFourMotorCommands(kMotorPwmRegister, 0, 0);
}

// Encode integer configuration values using the verified big-endian order.
bool MotorDriver::writeConfiguration16(uint8_t registerAddress, uint16_t value)
{
  const uint8_t payload[] = {static_cast<uint8_t>(value >> kBitsPerByte), static_cast<uint8_t>(value)};
  return writeRegisterBytes(registerAddress, payload, sizeof(payload));
}

// Diameter is the protocol exception: an IEEE-754 float in little-endian order.
bool MotorDriver::writeWheelDiameterMm(float diameterMm)
{
  static_assert(sizeof(float) == 4, "Driver requires 32-bit float");
  uint32_t floatBits;
  std::memcpy(&floatBits, &diameterMm, sizeof(floatBits)); // Avoid pointer aliasing.
  uint8_t payload[sizeof(floatBits)];
  for (size_t byte = 0; byte < sizeof(payload); ++byte)
    payload[byte] = static_cast<uint8_t>(floatBits >> (byte * kBitsPerByte));
  return writeRegisterBytes(kWheelDiameterRegister, payload, sizeof(payload));
}

// Reapply five I2C settings at boot or while stopped. Preserve existing save
// waits and first-error reporting. PID is explicitly left stored/unverified.
bool MotorDriver::initialize()
{
  bool acknowledged = releaseMotorOutputs();
  if (acknowledged)
  {
    const uint8_t motorType = MotorSettings::motorType;
    acknowledged = writeRegisterBytes(kMotorTypeRegister, &motorType, sizeof(motorType));
    delay(kConfigurationSaveDelayMs);
  }
  if (acknowledged)
  {
    acknowledged = writeConfiguration16(kPwmDeadZoneRegister, MotorSettings::pwmDeadZone);
    delay(kConfigurationSaveDelayMs);
  }
  if (acknowledged)
  {
    acknowledged = writeConfiguration16(kEncoderPulsesRegister, MotorSettings::encoderPulsesPerMotorRevolution);
    delay(kConfigurationSaveDelayMs);
  }
  if (acknowledged)
  {
    acknowledged = writeConfiguration16(kGearRatioRegister, MotorSettings::gearRatio);
    delay(kConfigurationSaveDelayMs);
  }
  if (acknowledged)
  {
    acknowledged = writeWheelDiameterMm(MotorSettings::wheelDiameterMm);
    delay(kConfigurationSaveDelayMs);
  }
  const uint8_t configurationError = communicationError_;
  const bool outputsReleased = releaseMotorOutputs(); // Attempt cleanup even after a failed setting.
  if (!acknowledged) communicationError_ = configurationError;
  return acknowledged && outputsReleased;
}

// Decode signed encoder counts in the driver's last 10 ms window for a wheel.
bool MotorDriver::readEncoderCountLast10Ms(Wheel wheel, int16_t &encoderCounts)
{
  const uint8_t motorChannel = static_cast<uint8_t>(wheel);
  if (motorChannel < 1 || motorChannel > kMotorChannels) return false;
  uint16_t rawCounts;
  if (!readRegister16BigEndian(kEncoder10MsRegisterBase + motorChannel - 1, rawCounts)) return false;
  encoderCounts = static_cast<int16_t>(rawCounts);
  return true;
}

// Convert the 10 ms counts into encoder-derived wheel mm/s. M2/M4 are read
// sequentially, not as an atomic pair. Keep both caller outputs unchanged if
// either transaction fails; this does not read the driver's serial speed value.
bool MotorDriver::readWheelSpeedsMmPerSecond(float &leftMmPerSecond, float &rightMmPerSecond)
{
  int16_t leftCounts, rightCounts;
  if (!readEncoderCountLast10Ms(Wheel::Left, leftCounts) ||
      !readEncoderCountLast10Ms(Wheel::Right, rightCounts)) return false;
  const float speedPerCount = MotorSettings::millimetersPerEncoderCount / kEncoderWindowSeconds;
  leftMmPerSecond = leftCounts * speedPerCount;
  rightMmPerSecond = rightCounts * speedPerCount;
  return true;
}

// Read signed encoder position from the high/low register pair for this wheel.
bool MotorDriver::readEncoderPosition(Wheel wheel, int32_t &encoderCounts)
{
  const uint8_t motorChannel = static_cast<uint8_t>(wheel);
  if (motorChannel < 1 || motorChannel > kMotorChannels) return false;
  uint32_t rawCounts;
  const uint8_t highWordRegister = kEncoderPositionRegisterBase + kBytesPerWord * (motorChannel - 1);
  if (!readEncoderCounter32(highWordRegister, rawCounts)) return false;
  encoderCounts = static_cast<int32_t>(rawCounts);
  return true;
}
