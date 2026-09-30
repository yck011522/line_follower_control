#include "MotorDriver.h"
#include "MotorSettings.h"
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
//
// Implementation: each attempt reads high, low, then high again (three separate readWord transactions).
// If the two high reads match, the low word cannot have rolled over in between, so the pair is coherent.
// If they differ, the whole sequence is retried once; after two failed attempts error() is 0x81.
//
// WARNING - COSTLY: about 3 x 176 us = roughly 0.53 ms of blocking I2C time per motor at 400 kHz
// (176 us per word, measured on the bench). Reading two motors' totals is about 1.05 ms, and the E2 test's
// full 50 Hz tick (two totals, two recent counts and one speed write) costs roughly 1.7 ms of every 20 ms.
// Keep this out of high-rate control loops; use it only for measurement and calibration.
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

// Sends one four-motor frame of big-endian int16 values M1..M4; M1 and M3 are sent as 0.
bool MotorDriver::writeMotors(uint8_t reg, int16_t m2, int16_t m4)
{
  const uint16_t a = static_cast<uint16_t>(m2);
  const uint16_t b = static_cast<uint16_t>(m4);
  const uint8_t bytes[] = {0, 0, static_cast<uint8_t>(a >> 8),
                           static_cast<uint8_t>(a), 0, 0,
                           static_cast<uint8_t>(b >> 8), static_cast<uint8_t>(b)};
  return write(reg, bytes, sizeof(bytes));
}

// Closed-loop speed targets (register 0x06, documented range -1000..1000, not checked here).
bool MotorDriver::setSpeeds(int16_t m2, int16_t m4) { return writeMotors(0x06, m2, m4); }

// Open-loop PWM (register 0x07, documented range -3600..3600, not checked here).
bool MotorDriver::setPwm(int16_t m2, int16_t m4) { return writeMotors(0x07, m2, m4); }

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

// Boot configuration from MotorSettings.h. The driver needs time to save each value, hence the delays.
bool MotorDriver::applySettings()
{
  bool ok = release();
  if (ok)
  {
    ok = setType(MotorSettings::type);
    delay(100);
  }
  if (ok)
  {
    ok = setParameter(0x02, MotorSettings::deadZone, false);
    delay(100);
  }
  if (ok)
  {
    ok = setParameter(0x03, MotorSettings::pulseLine, false);
    delay(100);
  }
  if (ok)
  {
    ok = setParameter(0x04, MotorSettings::pulsePhase, false);
    delay(100);
  }
  if (ok)
  {
    ok = setDiameter(MotorSettings::diameterMm);
    delay(100);
  }
  const uint8_t settingsError = error_;
  const bool released = release();
  if (!ok)
    error_ = settingsError; // Keep the first failure's code rather than the cleanup's.
  return ok && released;
}

// Recent-count registers are 0x10..0x13 for M1..M4.
bool MotorDriver::readRecent(uint8_t motor, int16_t &counts)
{
  if (motor < 1 || motor > 4)
    return false;
  uint16_t raw;
  if (!readWord(0x10 + motor - 1, raw))
    return false;
  counts = static_cast<int16_t>(raw);
  return true;
}

// Total-count registers are 0x20/0x21 (M1), 0x22/0x23 (M2), 0x24/0x25 (M3), 0x26/0x27 (M4).
bool MotorDriver::readTotal(uint8_t motor, int32_t &counts)
{
  if (motor < 1 || motor > 4)
    return false;
  uint32_t raw;
  if (!readCumulative(0x20 + 2 * (motor - 1), raw))
    return false;
  counts = static_cast<int32_t>(raw);
  return true;
}
