#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <esp_arduino_version.h>

namespace
{
  // Standalone benchmark: do not use the LineSensor class in this comparison.
  TwoWire lineBus(1);
  constexpr uint8_t kAddress = 0x12;
  constexpr uint8_t kRegister = 0x30;
  constexpr uint32_t kClockHz = 1000000;
  constexpr uint16_t kTimeoutMs = 1;
  constexpr uint32_t kDurationMs = 10000;
  constexpr uint32_t kRequestHz = 0;   // 0 = maximum rate; otherwise e.g. 50 or 100.
  constexpr uint32_t kMaxRequests = 0; // 0 = unlimited; otherwise cap physical requests.
  constexpr bool kRepeat = true;
  constexpr uint32_t kRepeatPauseMs = 1000;
  static_assert(kDurationMs > 0, "A positive duration bounds the test");
  static_assert(kRequestHz <= 1000000, "Period must be at least one microsecond");

  struct Statistics
  {
    uint32_t attempts = 0, successes = 0, transmitErrors = 0, shortReads = 0;
    uint64_t totalUs = 0, successUs = 0, minimumUs = UINT64_MAX, maximumUs = 0;
    uint64_t elapsedUs = 0, skippedSlots = 0, maximumFailedUs = 0;
    uint8_t lastRaw = 0, lastTransmitError = 0;
    const char *reason = "duration reached";
  };
  bool busReady = false, finished = false, summaryPending = false;
  Statistics result;

  // Perform one direct register-pointer write and repeated-START byte read.
  // Count each physical request once; there are no automatic retries here.
  void readOnce(Statistics &stats)
  {
    const uint64_t start = esp_timer_get_time();
    bool success = false;
    lineBus.beginTransmission(kAddress);
    lineBus.write(kRegister);
    const uint8_t error = lineBus.endTransmission(false);
    if (error)
    {
      ++stats.transmitErrors;
      stats.lastTransmitError = error;
    }
    else
    {
      const size_t count = lineBus.requestFrom(kAddress, static_cast<size_t>(1), true);
      if (count == 1 && lineBus.available() == 1)
      {
        stats.lastRaw = static_cast<uint8_t>(lineBus.read());
        success = true;
      }
      else
      {
        ++stats.shortReads;
        while (lineBus.available())
          lineBus.read(); // Discard partial feedback.
      }
    }
    const uint64_t duration = esp_timer_get_time() - start;
    ++stats.attempts;
    stats.totalUs += duration;
    if (duration < stats.minimumUs)
      stats.minimumUs = duration;
    if (duration > stats.maximumUs)
      stats.maximumUs = duration;
    if (success)
    {
      ++stats.successes;
      stats.successUs += duration;
    }
    else if (duration > stats.maximumFailedUs)
      stats.maximumFailedUs = duration; // Separate failure latency from normal successful reads.
  }

  // Run direct reads within the duration/count limits, skipping missed fixed-rate
  // slots rather than issuing a burst to catch up after a slow request.
  Statistics runBenchmark()
  {
    Statistics stats;
    const uint64_t start = esp_timer_get_time();
    const uint64_t end = start + static_cast<uint64_t>(kDurationMs) * 1000;
    const uint64_t periodUs = kRequestHz ? 1000000ULL / (kRequestHz ? kRequestHz : 1) : 0;
    uint64_t nextDue = start;
    while (esp_timer_get_time() < end)
    {
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
      if (periodUs && static_cast<uint64_t>(esp_timer_get_time()) < nextDue)
      {
        if (kRequestHz <= 1000)
          delay(1);
        else
          delayMicroseconds(50);
        continue;
      }
      readOnce(stats);
      if (periodUs)
      {
        nextDue += periodUs;
        const uint64_t now = esp_timer_get_time();
        if (now > nextDue)
        {
          const uint64_t skipped = (now - nextDue + periodUs - 1) / (periodUs ? periodUs : 1);
          stats.skippedSlots += skipped;
          nextDue += skipped * periodUs;
        }
      }
      else if (stats.attempts % 256 == 0)
        delay(1); // Yield between bursts, outside individual request timing.
    }
    stats.elapsedUs = esp_timer_get_time() - start;
    return stats;
  }

  // Print aggregate results outside the timed I2C request path.
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
    Serial.printf("Failed requests: count=%lu max=%llu us\n",
                  (unsigned long)(stats.attempts - stats.successes), (unsigned long long)stats.maximumFailedUs);
    Serial.printf("Elapsed=%.3f ms achieved=%.2f requests/s skipped slots=%llu\n\n",
                  stats.elapsedUs / 1000.0, stats.elapsedUs ? 1e6 * stats.attempts / stats.elapsedUs : 0.0,
                  (unsigned long long)stats.skippedSlots);
  }
}

// Initialize the second I2C bus directly, without constructing LineSensor.
void setup()
{
  Serial.begin(115200);
  busReady = lineBus.begin(D6, D7, kClockHz);
  lineBus.setTimeOut(kTimeoutMs);
}

// Run when USB is connected. Retain interrupted summaries; 'r' reruns the test.
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
    if (Serial.read() == 'r')
      finished = false;
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

  delay(250); // Let USB settle after upload/reset before starting the run.

  if (!Serial)
    return;
  // Include the actual compiled framework versions in captures for comparison.
  Serial.printf("E3 framework Arduino=%s ESP-IDF=%s\n", ESP_ARDUINO_VERSION_STR, ESP.getSdkVersion());
  Serial.printf("E3 START standalone clock=%lu Hz timeout=%u ms duration=%lu ms request_hz=%lu (0=maximum) limit=%lu (0=unlimited)\n",
                (unsigned long)kClockHz, kTimeoutMs, (unsigned long)kDurationMs,
                (unsigned long)kRequestHz, (unsigned long)kMaxRequests);
  Serial.flush();
  result = runBenchmark();
  finished = true;
  summaryPending = true;
}
