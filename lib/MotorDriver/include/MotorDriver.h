#pragma once

#include <Arduino.h>
#include <Wire.h>

// Register interface only; application owns scheduling and motion limits.
class MotorDriver
{
public:
  explicit MotorDriver(TwoWire &bus) : bus_(bus) {}
  uint8_t probe();
  bool readWord(uint8_t reg, uint16_t &value);
  bool readCumulative(uint8_t highReg, uint32_t &value); // Costly: about 0.5 ms per call; see MotorDriver.cpp.
  bool setSpeeds(int16_t m2, int16_t m4);
  bool setPwm(int16_t m2, int16_t m4); // Open-loop PWM, documented range +-3600.
  bool release();
  bool setType(uint8_t type);
  bool setParameter(uint8_t reg, uint16_t value, bool littleEndian);
  bool setDiameter(float mm);
  // Writes every MotorSettings value (100 ms apart) between two releases; true only if all were acknowledged.
  bool applySettings();
  // Motor numbers are 1..4 as printed on the board.
  bool readRecent(uint8_t motor, int16_t &counts); // Counts in the last 10 ms.
  bool readTotal(uint8_t motor, int32_t &counts);  // Cumulative signed count; costly (three I2C reads, about 0.5 ms).
  uint8_t error() const { return error_; }
  void repeatedStart(bool enabled) { repeatedStart_ = enabled; }

private:
  bool write(uint8_t reg, const uint8_t *data, size_t size);
  bool writeMotors(uint8_t reg, int16_t m2, int16_t m4);
  TwoWire &bus_;
  uint8_t error_ = 0;
  bool repeatedStart_ = true; // Bench verified: STOP-separated reads return zeros.
  static constexpr uint8_t kAddress = 0x26;
};
