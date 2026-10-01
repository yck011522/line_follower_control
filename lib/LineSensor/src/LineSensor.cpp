#include <LineSensor.h>
#include <esp_timer.h>

// Configure hardware at object creation. Construct this object from setup() so
// Wire initialization runs after the ESP32/Arduino runtime is ready.
LineSensor::LineSensor(const Config &config) : bus_(config.busNumber), config_(config)
{
  if (config_.busNumber > 1 || config_.requestHz > 1000000 ||
      config_.maxAttempts == 0 || config_.clockHz == 0 || config_.timeoutMs == 0 ||
      config_.address > 0x7F)
    return;
  periodUs_ = config_.requestHz ? 1000000ULL / config_.requestHz : 0;
  ready_ = bus_.begin(config_.sda, config_.scl, config_.clockHz);
  bus_.setTimeOut(config_.timeoutMs);
  resetSchedule();
}

// Time one physical request, including register selection and reading the byte.
// Failed reads discard partial data and never replace the last valid reading.
bool LineSensor::readOnce(TickResult &result, uint8_t &raw)
{
  const uint64_t start = esp_timer_get_time();
  bool success = false;
  bus_.beginTransmission(config_.address);
  bus_.write(0x30);
  const uint8_t error = bus_.endTransmission(false); // Repeated START is required for the read.
  if (error != 0)
  {
    ++result.transmitErrors;
    result.lastTransmitError = error;
  }
  else
  {
    const size_t received = bus_.requestFrom(config_.address, static_cast<size_t>(1), true);
    if (received == 1 && bus_.available() == 1)
    {
      raw = static_cast<uint8_t>(bus_.read());
      success = true;
    }
    else
    {
      ++result.shortReads;
      while (bus_.available()) bus_.read(); // Discard incomplete feedback before retrying.
    }
  }
  const uint64_t duration = esp_timer_get_time() - start;
  ++result.attempts;
  result.requestTotalUs += duration;
  if (duration < result.minimumRequestUs) result.minimumRequestUs = duration;
  if (duration > result.maximumRequestUs) result.maximumRequestUs = duration;
  if (success) result.successfulRequestUs = duration;
  return success;
}

// Service at most one scheduled poll per tick. Retry synchronously up to the
// configured limit. Wire's timeout is requested per operation, but underlying
// driver failures can block longer than that setting (observed on hardware).
LineSensor::TickResult LineSensor::tick()
{
  TickResult result;
  const uint64_t start = esp_timer_get_time();
  if (!ready_ || (periodUs_ && start < nextDueUs_)) return result;
  result.polled = true;
  uint8_t raw = 0;
  for (uint16_t attempt = 0; attempt < config_.maxAttempts; ++attempt)
  {
    if (readOnce(result, raw))
    {
      latest_.valid = true;
      latest_.rawMask = raw;
      latest_.receivedAtUs = esp_timer_get_time();
      result.updated = true;
      break; // Stop at the first valid result; failed polls leave latest_ intact.
    }
  }
  const uint64_t end = esp_timer_get_time();
  result.pollDurationUs = end - start;
  if (periodUs_)
  {
    // Preserve the requested cadence but skip expired slots instead of allowing
    // a burst of catch-up polls after a long gap or unsuccessful retry batch.
    nextDueUs_ += periodUs_;
    if (end > nextDueUs_)
    {
      result.skippedSlots = (end - nextDueUs_ + periodUs_ - 1) / periodUs_;
      nextDueUs_ += result.skippedSlots * periodUs_;
    }
  }
  return result;
}

// Calculate age on demand, including when every recent poll has failed. Using
// the 64-bit monotonic timer avoids the short rollover interval of micros().
LineSensor::Reading LineSensor::reading() const
{
  Reading result = latest_;
  if (result.valid) result.ageUs = esp_timer_get_time() - result.receivedAtUs;
  return result;
}

// Apply a new request frequency and make the next tick immediately due.
bool LineSensor::setRequestFrequency(uint32_t hz)
{
  if (hz > 1000000) return false;
  config_.requestHz = hz;
  periodUs_ = hz ? 1000000ULL / hz : 0;
  resetSchedule();
  return true;
}

// Apply a nonzero total request limit for the next due poll.
bool LineSensor::setMaxAttempts(uint8_t attempts)
{
  if (attempts == 0) return false;
  config_.maxAttempts = attempts;
  return true;
}

// Begin a new polling schedule without discarding the last valid sensor byte.
void LineSensor::resetSchedule()
{
  nextDueUs_ = esp_timer_get_time();
}
