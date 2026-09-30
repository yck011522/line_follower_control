#include "MotorDriver.h"
#include <cstring>

uint8_t MotorDriver::probe()
{
  bus_.beginTransmission(kAddress);
  error_ = bus_.endTransmission();
  return error_;
}

bool MotorDriver::write(uint8_t reg, const uint8_t *data, size_t size)
{
  bus_.beginTransmission(kAddress);
  bus_.write(reg);
  bus_.write(data, size);
  error_ = bus_.endTransmission();
  return error_ == 0;
}

bool MotorDriver::readWord(uint8_t reg, uint16_t &value)
{
  bus_.beginTransmission(kAddress);
  bus_.write(reg);
  error_ = bus_.endTransmission(!repeatedStart_);
  if (error_ != 0)
    return false;
  if (bus_.requestFrom(kAddress, static_cast<size_t>(2), true) != 2)
  {
    while (bus_.available())
      bus_.read();
    error_ = 0x80; // Short read, distinct from Wire endTransmission codes.
    return false;
  }
  const uint8_t hi = bus_.read();
  const uint8_t lo = bus_.read();
  value = (static_cast<uint16_t>(hi) << 8) | lo;
  return true;
}

bool MotorDriver::readCumulative(uint8_t highReg, uint32_t &value)
{
  // Bounded high-low-high read avoids mixing words across a low-word rollover.
  for (uint8_t attempt = 0; attempt < 2; ++attempt)
  {
    uint16_t hi, lo, check;
    if (!readWord(highReg, hi) || !readWord(highReg + 1, lo) ||
        !readWord(highReg, check))
      return false;
    if (hi == check)
    {
      value = (static_cast<uint32_t>(hi) << 16) | lo;
      return true;
    }
  }
  error_ = 0x81; // Could not get a coherent snapshot.
  return false;
}

bool MotorDriver::setSpeeds(int16_t m2, int16_t m4)
{
  const uint16_t a = static_cast<uint16_t>(m2);
  const uint16_t b = static_cast<uint16_t>(m4);
  const uint8_t bytes[] = {0, 0, static_cast<uint8_t>(a >> 8),
                           static_cast<uint8_t>(a), 0, 0,
                           static_cast<uint8_t>(b >> 8), static_cast<uint8_t>(b)};
  return write(0x06, bytes, sizeof(bytes));
}

bool MotorDriver::setPwm(int16_t m2, int16_t m4)
{
  const uint16_t a = static_cast<uint16_t>(m2);
  const uint16_t b = static_cast<uint16_t>(m4);
  const uint8_t bytes[] = {0, 0, static_cast<uint8_t>(a >> 8),
                           static_cast<uint8_t>(a), 0, 0,
                           static_cast<uint8_t>(b >> 8), static_cast<uint8_t>(b)};
  return write(0x07, bytes, sizeof(bytes));
}

bool MotorDriver::release()
{
  const uint8_t zeros[8] = {};
  return write(0x07, zeros, sizeof(zeros));
}

bool MotorDriver::setType(uint8_t type) { return write(0x01, &type, 1); }

bool MotorDriver::setParameter(uint8_t reg, uint16_t value, bool littleEndian)
{
  uint8_t bytes[] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
  if (littleEndian)
  {
    const uint8_t tmp = bytes[0];
    bytes[0] = bytes[1];
    bytes[1] = tmp;
  }
  return write(reg, bytes, sizeof(bytes));
}

bool MotorDriver::setDiameter(float mm)
{
  static_assert(sizeof(float) == 4, "Driver requires 32-bit float");
  uint32_t bits;
  std::memcpy(&bits, &mm, sizeof(bits));
  const uint8_t bytes[] = {static_cast<uint8_t>(bits), static_cast<uint8_t>(bits >> 8),
                           static_cast<uint8_t>(bits >> 16), static_cast<uint8_t>(bits >> 24)};
  return write(0x05, bytes, sizeof(bytes));
}
