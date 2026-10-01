#include <Arduino.h>
#include <LineSensor.h>
#include <esp_timer.h>
#include <esp_arduino_version.h>

namespace
{
  // Construct the bus-owning object in setup(), after Arduino initialization.
  LineSensor *sensor = nullptr;
  constexpr uint32_t kClockHz = 1000000; // Fixed inside the library.
  constexpr uint16_t kTimeoutMs = 1; // Fixed inside the library.
  constexpr uint8_t kMaxAttempts = 5; // Initial request plus up to four retries.
  constexpr uint32_t kDurationMs = 10000;
  constexpr uint32_t kRequestHz = 1000; // 0 = maximum rate; otherwise e.g. 50 or 100.
  constexpr uint32_t kMaxRequests = 0;  // 0 = unlimited; otherwise cap physical requests.
  constexpr bool kRepeat = true;
  constexpr uint32_t kRepeatPauseMs = 1000;
  static_assert(kDurationMs > 0, "A positive duration bounds the test");
  static_assert(kRequestHz <= 1000, "Library maximum rate is 1000 Hz");

  struct Statistics
  {
    uint32_t attempts = 0, successes = 0, transmitErrors = 0, shortReads = 0;
    uint64_t totalUs = 0, successUs = 0, minimumUs = UINT64_MAX, maximumUs = 0;
    uint64_t elapsedUs = 0, maximumFailedUs = 0;
    uint8_t lastRaw = 0, lastTransmitError = 0;
    uint32_t retries = 0;
    uint64_t previousStartUs = 0, minimumGapUs = UINT64_MAX;
    const char *reason = "duration reached";
  };
  bool busReady = false, finished = false, summaryPending = false;
  Statistics result;

  // Ask the library for one due request. Idle calls perform no I2C traffic and
  // do not count as attempts. All counters below refer to physical requests.
  bool readOnce(Statistics &stats)
  {
    const LineSensor::Status status = sensor->tick();
    if (status == LineSensor::Status::Idle) return false;
    const LineSensor::Request &request = sensor->lastRequest();
    const uint64_t duration = request.durationUs;
    if (stats.previousStartUs)
    {
      const uint64_t gap = request.startedAtUs - stats.previousStartUs;
      if (gap < stats.minimumGapUs) stats.minimumGapUs = gap;
    }
    stats.previousStartUs = request.startedAtUs;
    ++stats.attempts;
    if (request.attempt > 1) ++stats.retries;
    stats.totalUs += duration;
    if (duration < stats.minimumUs) stats.minimumUs = duration;
    if (duration > stats.maximumUs) stats.maximumUs = duration;
    if (status == LineSensor::Status::Success)
    {
      ++stats.successes;
      stats.successUs += duration;
      stats.lastRaw = sensor->reading().rawMask;
    }
    else
    {
      // As in E4, distinguish a register-selection error from a missing byte.
      // Repeated-START writes may defer actual transmission until requestFrom().
      if (request.transmitError)
      {
        ++stats.transmitErrors;
        stats.lastTransmitError = request.transmitError;
      }
      else ++stats.shortReads;
      if (duration > stats.maximumFailedUs) stats.maximumFailedUs = duration;
    }
    return true;
  }

  // Repeatedly tick the library for the same duration/count budget as E4.
  // All request/retry timing belongs to LineSensor; no second schedule is used.
  Statistics runBenchmark()
  {
    Statistics stats;
    const uint64_t start = esp_timer_get_time();
    const uint64_t end = start + static_cast<uint64_t>(kDurationMs) * 1000;
    while (esp_timer_get_time() < end)
    {
      if (kMaxRequests && stats.attempts >= kMaxRequests)
      {
        stats.reason = "request limit reached";
        break;
      }
      // A short wait when Idle avoids the old delay(1) every-other-slot issue.
      // This wait is outside individual request latency measurements.
      if (!readOnce(stats)) delayMicroseconds(5);
    }
    stats.elapsedUs = esp_timer_get_time() - start;
    return stats;
  }

  // Print aggregate results outside the timed I2C request path.
  void printSummary(const Statistics &stats)
  {
    Serial.printf("E4 END: %s\n", stats.reason);
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
    Serial.printf("Retry requests=%lu minimum start gap=%llu us\n",
                  (unsigned long)stats.retries,
                  (unsigned long long)(stats.minimumGapUs == UINT64_MAX ? 0 : stats.minimumGapUs));
    const LineSensor::Reading reading = sensor->reading();
    if (reading.valid)
      Serial.printf("Latest raw=0x%02X age=%.3f ms\n", reading.rawMask, reading.ageUs / 1000.0);
    else Serial.println("Latest reading: none received yet");
    Serial.printf("Elapsed=%.3f ms achieved=%.2f requests/s\n\n",
                  stats.elapsedUs / 1000.0, stats.elapsedUs ? 1e6 * stats.attempts / stats.elapsedUs : 0.0);
  }
}

// Initialize the library with explicit board pin aliases on I2C controller 1.
void setup()
{
  Serial.begin(115200);
  // The constructor initializes the chosen pins automatically. No global
  // hardware initialization or separate begin() call is needed.
  static LineSensor lineSensor(D6, D7, kRequestHz, kMaxAttempts);
  sensor = &lineSensor;
  busReady = sensor->ready();
}

// Run when USB is connected. Retain interrupted summaries; 'r' reruns the test.
void loop()
{
  // Print the summary if it is pending.
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

  // Check for a 'r' command to restart the benchmark.
  while (Serial.available())
    if (Serial.read() == 'r')
      finished = false;

  // If the benchmark has finished, wait briefly and return to avoid busy looping.
  if (finished)
  {
    delay(100);
    return;
  }

  // Check if the I2C bus is ready.
  if (!busReady)
  {
    Serial.println("E4: failed to initialize I2C bus 1 on D6/D7");
    finished = true;
    return;
  }

  delay(250); // Let USB settle after upload/reset before starting the run.

  // Include the actual compiled framework versions in captures for comparison.
  Serial.printf("E4 framework Arduino=%s ESP-IDF=%s\n", ESP_ARDUINO_VERSION_STR, ESP.getSdkVersion());
  Serial.printf("E4 START library clock=%lu Hz timeout=%u ms duration=%lu ms request_hz=%lu (0=1000 Hz maximum) limit=%lu (0=unlimited)\n",
                (unsigned long)kClockHz, kTimeoutMs, (unsigned long)kDurationMs,
                (unsigned long)kRequestHz, (unsigned long)kMaxRequests);
  Serial.flush();
  result = runBenchmark();
  finished = true;
  summaryPending = true;
}
