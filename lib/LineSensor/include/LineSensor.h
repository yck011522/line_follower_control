#pragma once

#include <Arduino.h>
#include <Wire.h>

// Owns the line sensor's I2C controller. Construct in setup(), after Arduino has
// initialized the runtime, rather than as a global hardware-initializing object.
class LineSensor
{
public:
  struct Config
  {
    uint8_t busNumber = 1;
    int sda = D6;
    int scl = D7;
    uint32_t clockHz = 1000000;
    uint16_t timeoutMs = 1;
    uint32_t requestHz = 50; // 0 means a poll on every tick().
    uint8_t maxAttempts = 5; // Includes the first request: five total, not five extra retries.
    uint8_t address = 0x12;
  };

  // Snapshot of the last valid byte. An unseen sensor has valid=false and an
  // age of UINT64_MAX, so startup can never masquerade as fresh information.
  struct Reading
  {
    bool valid = false;
    uint8_t rawMask = 0;
    uint64_t receivedAtUs = 0;
    uint64_t ageUs = UINT64_MAX;

    // Convert the supplied active-low status byte into an on-black mask.
    uint8_t blackMask() const { return static_cast<uint8_t>(~rawMask); }
  };

  // Describes this tick only. Quiet diagnostics let experiments count physical
  // bus requests separately from successful polls recovered through retries.
  struct TickResult
  {
    bool polled = false;
    bool updated = false;
    uint8_t attempts = 0;
    uint8_t transmitErrors = 0;
    uint8_t shortReads = 0;
    uint8_t lastTransmitError = 0;
    uint64_t requestTotalUs = 0;
    uint64_t successfulRequestUs = 0;
    uint64_t minimumRequestUs = UINT64_MAX;
    uint64_t maximumRequestUs = 0;
    uint64_t pollDurationUs = 0;
    uint64_t skippedSlots = 0;
  };

  // Configure the owned bus immediately; ready() exposes initialization failure.
  explicit LineSensor(const Config &config);
  LineSensor(const LineSensor &) = delete;
  LineSensor &operator=(const LineSensor &) = delete;

  // Poll only when due, trying at most maxAttempts until the first valid byte.
  TickResult tick();

  // Return a fresh age calculation without performing any I2C communication.
  Reading reading() const;

  // Report whether construction initialized the bus and validated the settings.
  bool ready() const { return ready_; }

  // Change polling rate without reconfiguring the bus; 0 means maximum rate.
  bool setRequestFrequency(uint32_t hz);

  // Change the total attempt limit; reject zero instead of silently skipping reads.
  bool setMaxAttempts(uint8_t attempts);

  // Make the next tick immediately due, useful when beginning an experiment.
  void resetSchedule();

private:
  // Read register 0x30 once, recording latency and error counts without printing.
  bool readOnce(TickResult &result, uint8_t &raw);

  TwoWire bus_;
  Config config_;
  bool ready_ = false;
  Reading latest_;
  uint64_t periodUs_ = 0;
  uint64_t nextDueUs_ = 0;
};
