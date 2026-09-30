#include "MotorDriver.h"
#include <cstring>

// Address-only transmission: returns 0 if the driver acknowledges its I2C address.
uint8_t MotorDriver::probe()
{
  bus_.beginTransmission(kAddress);
  error_ = bus_.endTransmission();
  return error_;
}

// Sends [register][data...] in one I2C transaction; every driver write uses this.
bool MotorDriver::write(uint8_t reg, const uint8_t *data, size_t size)
{
  bus_.beginTransmission(kAddress);
  bus_.write(reg);
  bus_.write(data, size);
  error_ = bus_.endTransmission();
  return error_ == 0;
}

// Reads one big-endian 16-bit register. Sends the register address, then requests 2 bytes.
bool MotorDriver::readWord(uint8_t reg, uint16_t &value)
{
  bus_.beginTransmission(kAddress);
  bus_.write(reg);
  // With repeatedStart_ the STOP is skipped; the driver returns zeros if a STOP separates the write and read.
  error_ = bus_.endTransmission(!repeatedStart_);
  if (error_ != 0)
    return false;
  if (bus_.requestFrom(kAddress, static_cast<size_t>(2), true) != 2)
  {
    // Discard any partial bytes so they cannot corrupt the next read.
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

// Reads a 32-bit encoder total stored as two 16-bit registers (high word at highReg, low word at highReg + 1).
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
      // Callers cast to int32_t: counts go negative when the wheel turns in reverse.
      value = (static_cast<uint32_t>(hi) << 16) | lo;
      return true;
    }
  }
  error_ = 0x81; // Could not get a coherent snapshot.
  return false;
}

// Closed-loop speed targets (register 0x06, documented range -1000..1000, not checked here).
// The frame is four big-endian int16 values M1..M4; M1 and M3 are sent as 0.
bool MotorDriver::setSpeeds(int16_t m2, int16_t m4)
{
  const uint16_t a = static_cast<uint16_t>(m2);
  const uint16_t b = static_cast<uint16_t>(m4);
  const uint8_t bytes[] = {0, 0, static_cast<uint8_t>(a >> 8),
                           static_cast<uint8_t>(a), 0, 0,
                           static_cast<uint8_t>(b >> 8), static_cast<uint8_t>(b)};
  return write(0x06, bytes, sizeof(bytes));
}

// Open-loop PWM (register 0x07, documented range -3600..3600, not checked here). Same frame layout as setSpeeds.
bool MotorDriver::setPwm(int16_t m2, int16_t m4)
{
  const uint16_t a = static_cast<uint16_t>(m2);
  const uint16_t b = static_cast<uint16_t>(m4);
  const uint8_t bytes[] = {0, 0, static_cast<uint8_t>(a >> 8),
                           static_cast<uint8_t>(a), 0, 0,
                           static_cast<uint8_t>(b >> 8), static_cast<uint8_t>(b)};
  return write(0x07, bytes, sizeof(bytes));
}

// Sets all four PWM outputs to 0 so the wheels can turn freely (speed 0 would keep the PID holding them).
bool MotorDriver::release()
{
  const uint8_t zeros[8] = {};
  return write(0x07, zeros, sizeof(zeros));
}

// Register 0x01: 1=520, 2=310, 3=TT with encoder, 4=TT without encoder.
bool MotorDriver::setType(uint8_t type) { return write(0x01, &type, 1); }

// Writes a 16-bit parameter (dead zone 0x02, encoder lines 0x03, gear ratio 0x04); big-endian unless littleEndian.
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

// Register 0x05: wheel diameter in mm as a little-endian IEEE-754 float (unlike the big-endian integers).
bool MotorDriver::setDiameter(float mm)
{
  static_assert(sizeof(float) == 4, "Driver requires 32-bit float");
  uint32_t bits;
  std::memcpy(&bits, &mm, sizeof(bits)); // memcpy avoids aliasing issues; shifts below make the byte order little-endian on any host
  const uint8_t bytes[] = {static_cast<uint8_t>(bits), static_cast<uint8_t>(bits >> 8),
                           static_cast<uint8_t>(bits >> 16), static_cast<uint8_t>(bits >> 24)};
  return write(0x05, bytes, sizeof(bytes));
}
