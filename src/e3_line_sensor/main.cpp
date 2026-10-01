#include <Arduino.h>
#include <LineSensor.h>
#include <esp_timer.h>

namespace
{
  LineSensor *sensor = nullptr; // The hardware-owning object is constructed inside setup().

  // Edit these settings and upload E3. A "request" means one complete register
  // selection plus one-byte read, not a multi-byte read from adjacent registers.
  constexpr uint32_t kClockHz = 1000000;
  constexpr uint16_t kTimeoutMs = 1;      // Timeout per Wire operation, not per whole request.
  constexpr uint32_t kDurationMs = 10000; // Time budget: 1000 = 1 second, 10000 = 10 seconds.
  constexpr uint32_t kRequestHz = 0;      // 0 = as fast as possible; otherwise e.g. 50 or 100.
  constexpr uint8_t kMaxAttempts = 5;    // Total physical requests per due poll, including the first.
  constexpr uint32_t kMaxRequests = 0;    // 0 = no count limit; set 100 for a 100-request burst.
  constexpr bool kRepeat = false;         // false = one run; send 'r' over serial to rerun.
  constexpr uint32_t kRepeatPauseMs = 1000;
  static_assert(kDurationMs > 0, "A positive duration bounds every test");
  static_assert(kRequestHz <= 1000000, "Requested period must be at least one microsecond");

  struct Statistics
  {
    uint32_t attempts = 0, successes = 0, transmitErrors = 0, shortReads = 0;
    uint32_t polls = 0, recoveredPolls = 0, failedPolls = 0, retries = 0;
    uint64_t totalUs = 0, successUs = 0, minimumUs = UINT64_MAX, maximumUs = 0;
    uint64_t elapsedUs = 0, skippedSlots = 0;
    uint8_t lastRaw = 0, lastTransmitError = 0;
    const char *reason = "duration reached";
  };

  bool busReady = false, finished = false, summaryPending = false;
  Statistics result;

  // Ask the library to service a due poll and aggregate its physical-request
  // statistics. A recovered poll must not hide failed requests from the benchmark.
  bool readOnce(Statistics &stats)
  {
    if (kMaxRequests)
    {
      // Keep the optional physical-request cap exact, even on the last retry batch.
      const uint32_t remaining = kMaxRequests - stats.attempts;
      sensor->setMaxAttempts(static_cast<uint8_t>(min(remaining, static_cast<uint32_t>(kMaxAttempts))));
    }
    const LineSensor::TickResult poll = sensor->tick();
    if (!poll.polled) return false;
    ++stats.polls;
    stats.attempts += poll.attempts;
    stats.retries += poll.attempts - 1;
    stats.transmitErrors += poll.transmitErrors;
    stats.shortReads += poll.shortReads;
    if (poll.transmitErrors) stats.lastTransmitError = poll.lastTransmitError;
    stats.totalUs += poll.requestTotalUs;
    stats.skippedSlots += poll.skippedSlots;
    if (poll.minimumRequestUs < stats.minimumUs) stats.minimumUs = poll.minimumRequestUs;
    if (poll.maximumRequestUs > stats.maximumUs) stats.maximumUs = poll.maximumRequestUs;
    if (poll.updated)
    {
      ++stats.successes;
      stats.successUs += poll.successfulRequestUs;
      stats.lastRaw = sensor->reading().rawMask;
      if (poll.attempts > 1) ++stats.recoveredPolls;
    }
    else ++stats.failedPolls;
    return true;
  }

  // Run until the time budget or optional count cap is reached. Fixed-rate mode
  // uses the library's schedule; no independent second schedule can delay a due poll.
  Statistics runBenchmark()
  {
    Statistics stats;
    const uint64_t start = esp_timer_get_time();
    const uint64_t end = start + static_cast<uint64_t>(kDurationMs) * 1000;
    sensor->setMaxAttempts(kMaxAttempts);
    sensor->resetSchedule(); // Start each run immediately, without counting previous USB idle time.
    while (true)
    {
      const uint64_t now = esp_timer_get_time();
      if (now >= end)
        break;
      if (kMaxRequests && stats.attempts >= kMaxRequests)
      {
        stats.reason = "request limit reached";
        break;
      }
      if (!Serial)
      {
        stats.reason = "INCOMPLETE: USB disconnected";
        break;
      }
      if (!readOnce(stats))
      {
        // Wait outside request timing when tick says the next poll is not due.
        // Short waits retain cadence; low polling rates also let the RTOS idle.
        if (kRequestHz <= 1000) delay(1);
        else delayMicroseconds(50);
      }
      else if (kRequestHz == 0 && stats.polls % 256 == 0)
      {
        // Yield after each burst of polls. This affects throughput but not the
        // individual I2C-request latency statistics supplied by the library.
        delay(1);
      }
    }
    // An in-flight poll (including bounded retries) may finish past the time budget.
    stats.elapsedUs = esp_timer_get_time() - start;
    return stats;
  }

  // Print the final statistics, including failures in the overall average.
  // A successful transport read does not prove the sensor produced fresh data.
  void printSummary(const Statistics &stats)
  {
    Serial.printf("E3 END: %s\n", stats.reason);
    Serial.printf("Success: %lu / %lu (%.2f%%)\n", (unsigned long)stats.successes,
                  (unsigned long)stats.attempts, stats.attempts ? 100.0 * stats.successes / stats.attempts : 0.0);
    Serial.printf("Request time, all attempts: mean=%.2f us min=%llu us max=%llu us\n",
                  stats.attempts ? double(stats.totalUs) / stats.attempts : 0.0,
                  (unsigned long long)(stats.attempts ? stats.minimumUs : 0), (unsigned long long)stats.maximumUs);
    if (stats.successes)
      Serial.printf("Successful requests: mean=%.2f us; last raw=0x%02X\n",
                    double(stats.successUs) / stats.successes, stats.lastRaw);
    Serial.printf("Transmit errors=%lu short reads=%lu last transmit error=%u\n",
                  (unsigned long)stats.transmitErrors, (unsigned long)stats.shortReads, stats.lastTransmitError);
    Serial.printf("Polls=%lu recovered=%lu failed=%lu retry requests=%lu\n",
                  (unsigned long)stats.polls, (unsigned long)stats.recoveredPolls,
                  (unsigned long)stats.failedPolls, (unsigned long)stats.retries);
    const LineSensor::Reading reading = sensor->reading();
    if (reading.valid)
      Serial.printf("Latest raw=0x%02X age=%.3f ms\n", reading.rawMask, reading.ageUs / 1000.0);
    else Serial.println("Latest reading: none received yet");
    Serial.printf("Elapsed=%.3f ms achieved=%.2f requests/s skipped slots=%llu\n\n",
                  stats.elapsedUs / 1000.0, stats.elapsedUs ? 1e6 * stats.attempts / stats.elapsedUs : 0.0,
                  (unsigned long long)stats.skippedSlots);
  }
}

// Initialize only the line sensor's second I2C controller; no motor configuration
// or motion is involved in this experiment. USB connection is awaited in loop().
void setup()
{
  Serial.begin(115200);
  LineSensor::Config config;
  config.clockHz = kClockHz;
  config.timeoutMs = kTimeoutMs;
  config.requestHz = kRequestHz;
  config.maxAttempts = kMaxAttempts;
  // Function-local construction initializes the bus automatically after Arduino
  // startup. Its lifetime covers the whole application, including repeated runs.
  static LineSensor lineSensor(config);
  sensor = &lineSensor;
  busReady = sensor->ready();
}

// Run once when a serial listener connects, or repeat if enabled above. Retain an
// interrupted run's summary in RAM so it can be printed when USB reconnects.
// Sending 'r' reruns the same compiled settings without resetting the controller.
void loop()
{
  if (!Serial)
  {
    delay(100);
    return;
  }
  if (summaryPending)
  {
    printSummary(result);
    summaryPending = false;
    if (kRepeat)
    {
      delay(kRepeatPauseMs);
      finished = false;
    }
  }
  while (Serial.available())
  {
    if (Serial.read() == 'r')
      finished = false;
  }
  if (finished)
  {
    delay(100);
    return;
  }
  if (!busReady)
  {
    Serial.println("E3: failed to initialize I2C bus 1 on D6/D7");
    finished = true;
    return;
  }
  // Upload/reset can briefly look like a connected terminal. Let USB settle
  // before starting; a later disconnect still marks the run incomplete.
  delay(250);
  if (!Serial)
    return;
  Serial.printf("E3 START clock=%lu Hz timeout=%u ms duration=%lu ms request_hz=%lu (0=maximum) limit=%lu (0=unlimited) max_attempts=%u\n",
                (unsigned long)kClockHz, kTimeoutMs, (unsigned long)kDurationMs,
                (unsigned long)kRequestHz, (unsigned long)kMaxRequests, kMaxAttempts);
  Serial.flush(); // Drain the heading before timing begins.
  result = runBenchmark();
  finished = true;
  summaryPending = true;
}
