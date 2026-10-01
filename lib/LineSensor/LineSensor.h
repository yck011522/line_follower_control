#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>

// Owns I2C controller 1 for the line sensor. Construct inside setup(), after
// Arduino startup, and call tick() from one task. No serial logging is performed.
class LineSensor
{
public:
  enum class Status { Idle, Success, Failure };

  // The original sensor byte is active-low: bit 7 is X1, bit 0 is X8.
  // Before the first successful request, valid is false and ageUs is UINT64_MAX.
  struct Reading
  {
    bool valid = false;
    uint8_t rawMask = 0;
    uint64_t ageUs = UINT64_MAX;
  };

  // Timing of the most recent physical request, for the hardware benchmark.
  // Read this only after tick() returns Success or Failure, not after Idle.
  struct Request
  {
    uint64_t startedAtUs = 0;
    uint64_t durationUs = 0;
    uint8_t transmitError = 0;
    uint8_t attempt = 0; // One-based position within the current retry batch.
  };

  // Automatically configure the supplied SDA/SCL pins, 1 MHz clock and 1 ms
  // Wire timeout. requestHz=0 means the 1000 Hz maximum; higher rates are capped.
  // maxAttempts includes the initial request; use a nonzero value (default five).
  LineSensor(int sda, int scl, uint32_t requestHz = 1000, uint8_t maxAttempts = 5)
      : bus_(1), maxAttempts_(maxAttempts)
  {
    const uint32_t hz = requestHz == 0 || requestHz > 1000 ? 1000 : requestHz;
    // Round up so an integer period never exceeds the requested rate.
    periodUs_ = (1000000ULL + hz - 1) / hz;
    ready_ = bus_.begin(sda, scl, 1000000);
    bus_.setTimeOut(1);
  }

  // A hardware-owning object must not be copied into a second bus owner.
  LineSensor(const LineSensor &) = delete;
  LineSensor &operator=(const LineSensor &) = delete;

  // Perform at most one request when due. Waiting for the next request/retry
  // never sleeps here; callers keep ticking. The I2C operation itself blocks.
  Status tick()
  {
    const uint64_t now = esp_timer_get_time();
    if (!ready_ || now < nextRequestUs_) return Status::Idle;

    // Anchor each normal poll to its actual start, with no catch-up requests.
    // Retries retain this poll's normal next-due time until the batch finishes.
    const uint64_t start = esp_timer_get_time();
    if (attempts_ == 0) nextPollUs_ = start + periodUs_;
    ++attempts_;
    request_.startedAtUs = start;
    request_.attempt = attempts_;
    bool success = false;

    // Refactored directly from E3: select register 0x30 and read one byte from
    // address 0x12 with a repeated START, without releasing the bus between them.
    bus_.beginTransmission(0x12);
    bus_.write(0x30);
    request_.transmitError = bus_.endTransmission(false);
    if (request_.transmitError == 0)
    {
      const size_t count = bus_.requestFrom(static_cast<uint8_t>(0x12), static_cast<size_t>(1), true);
      if (count == 1 && bus_.available() == 1)
      {
        latest_.rawMask = static_cast<uint8_t>(bus_.read());
        latest_.valid = true;
        receivedAtUs_ = esp_timer_get_time();
        success = true;
      }
      else
      {
        // Discard partial feedback as E3 does. Failed requests never overwrite
        // the last successful byte or its timestamp, so its age keeps growing.
        while (bus_.available()) bus_.read();
      }
    }
    request_.durationUs = esp_timer_get_time() - start;

    // Every physical request, including retries, has at least 1000 us between
    // recorded starts. A late tick cannot accumulate a burst of overdue reads.
    nextRequestUs_ = start + 1000;
    if (success || attempts_ >= maxAttempts_)
    {
      attempts_ = 0;
      if (nextPollUs_ > nextRequestUs_) nextRequestUs_ = nextPollUs_;
    }
    return success ? Status::Success : Status::Failure;
  }

  // Calculate age when requested, even if tick() has not been called recently
  // or every recent transaction failed. This is communication age, not sample age.
  Reading reading() const
  {
    Reading result = latest_;
    if (result.valid) result.ageUs = esp_timer_get_time() - receivedAtUs_;
    return result;
  }

  // Report whether the constructor successfully initialized the I2C bus.
  bool ready() const { return ready_; }

  // Expose the last request's timing/error information without extra bus traffic.
  const Request &lastRequest() const { return request_; }

private:
  TwoWire bus_;
  bool ready_ = false;
  uint8_t maxAttempts_ = 5, attempts_ = 0;
  uint64_t periodUs_ = 1000, nextPollUs_ = 0, nextRequestUs_ = 0;
  uint64_t receivedAtUs_ = 0;
  Reading latest_;
  Request request_;
};
