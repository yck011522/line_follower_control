#include <Arduino.h>
#include <Wire.h>
#include <LineSensor.h>
#include <MotorDriver.h>
#include <esp_timer.h>
#include <cmath>
#include <cstdio>

namespace
{
  // Confirmed wiring: X1 is leftmost; negative M2/M4 commands drive forward.
  constexpr bool kSensorX1IsLeftmost = true;
  constexpr int16_t kLeftMotorForwardSign = -1;
  constexpr int16_t kRightMotorForwardSign = -1;
  constexpr uint32_t kLineSensorRequestFrequencyHz = 200; // Independent of steering rate.
  constexpr uint32_t kControlFrequencyHz = 100;
  constexpr uint64_t kControlPeriodUs = 1000000 / kControlFrequencyHz;
  constexpr float kFilterHalfLifeSeconds = 0.2f;
  constexpr uint64_t kMaximumLineMissingUs = 1000000; // Hold the last detected center for one second.
  constexpr uint64_t kMaximumReadingAgeUs = 100000; // Approved stale-data stop threshold.

  // Runtime settings are editable through Serial; steering has no integral term.
  float proportionalGain = 1.0f;
  float derivativeGain = 0.0f;
  int16_t defaultSpeedMmPerSecond = 100;
  float statusFrequencyHz = 5.0f;

  // Hardware objects share no I2C controller: Wire is controller 0, sensor owns 1.
  MotorDriver motorDriver(Wire);
  LineSensor *lineSensor = nullptr;
  bool motorConfigurationSucceeded = false;
  bool filterInitialized = false;
  // Line age differs from communication age: a successful 0xFF read has no line.
  bool hasDetectedLine = false;
  bool lineWasMissing = false;
  bool motorsReleasedForSensorLoss = false;
  uint64_t lastLineDetectedAtUs = 0;
  float estimatedLineCenter = 0.0f;
  float filteredLineCenter = 0.0f;
  float steeringCorrection = 0.0f;
  int16_t leftWheelCommandMmPerSecond = 0;
  int16_t rightWheelCommandMmPerSecond = 0;
  const char *runState = "starting";
  uint64_t previousControlStartUs = 0;
  uint64_t nextControlStartUs = 0;
  uint64_t previousStatusUs = 0;

  // Accumulate timing between status reports. Work time includes sensor I2C and motor writes.
  uint32_t completedControlCycles = 0;
  uint32_t missedControlSlots = 0;
  uint64_t totalControlWorkUs = 0;
  uint64_t maximumControlWorkUs = 0;
  uint64_t serialOutputWorkUs = 0;
  uint64_t sensorWorkSinceControlUs = 0; // Account for sensor polling between control cycles.
  String serialCommand;

  // Average positions of active-low detections, equally spaced from left -1 to right +1.
  // Return false for no detected line; no centroid exists for an empty mask.
  bool estimateLineCenter(uint8_t rawMask, float &lineCenter)
  {
    float positionSum = 0.0f;
    uint8_t detectedSensorCount = 0;
    for (uint8_t sensorIndex = 0; sensorIndex < 8; ++sensorIndex)
    {
      const uint8_t sensorBit = 7 - sensorIndex; // X1 is bit 7, X8 is bit 0.
      if ((rawMask & (1u << sensorBit)) == 0)
      {
        const float sensorPosition = -1.0f + 2.0f * sensorIndex / 7.0f;
        positionSum += kSensorX1IsLeftmost ? sensorPosition : -sensorPosition;
        ++detectedSensorCount;
      }
    }
    if (detectedSensorCount == 0)
      return false;
    lineCenter = positionSum / detectedSensorCount;
    return true;
  }

  // One-pole exponential smoothing: after 0.2 seconds, half of a step error remains.
  // Seed from the first observation to avoid an artificial startup steering transient.
  void filterLineCenter(float elapsedSeconds)
  {
    if (!filterInitialized)
    {
      filteredLineCenter = estimatedLineCenter;
      filterInitialized = true;
      return;
    }
    const float previousWeight = expf(-0.69314718056f * elapsedSeconds / kFilterHalfLifeSeconds);
    filteredLineCenter = previousWeight * filteredLineCenter +
                         (1.0f - previousWeight) * estimatedLineCenter;
  }

  // Attempt both shutdown writes. Sensor-loss stops retain the requested speed
  // for automatic recovery; manual stops and motor errors still latch.
  void stopAndReleaseMotors(const char *reason, bool retainRequestedSpeed = false)
  {
    if (!retainRequestedSpeed)
      defaultSpeedMmPerSecond = 0;
    leftWheelCommandMmPerSecond = 0;
    rightWheelCommandMmPerSecond = 0;
    const bool zeroSpeedAcknowledged = motorDriver.setWheelSpeedsMmPerSecond(0, 0);
    const bool releaseAcknowledged = motorDriver.releaseMotorOutputs();
    runState = reason;
    filterInitialized = false;
    if (!zeroSpeedAcknowledged || !releaseAcknowledged)
    {
      motorConfigurationSucceeded = false;
      runState = "shutdown_write_failed";
    }
  }

  // Steer once per control cycle from the independently polled sensor. Positive means right.
  // Saturation implements the requested stop-inner-wheel / maintain-outer-wheel behavior.
  void updateWheelCommands(float elapsedSeconds)
  {
    if (defaultSpeedMmPerSecond == 0 || !motorConfigurationSucceeded)
      return;
    const LineSensor::Reading reading = lineSensor->reading();
    if (!reading.valid || reading.ageUs > kMaximumReadingAgeUs)
    {
      // Preserve the selected speed while the sensor recovers. Release once,
      // keep polling, and require a detected line before commanding motion again.
      lineWasMissing = true;
      hasDetectedLine = false;
      if (!motorsReleasedForSensorLoss)
      {
        stopAndReleaseMotors("line_read_unavailable", true);
        motorsReleasedForSensorLoss = true;
      }
      return;
    }
    const uint64_t nowUs = esp_timer_get_time();
    float detectedCenter;
    const bool lineDetected = estimateLineCenter(reading.rawMask, detectedCenter);
    if (lineDetected)
    {
      estimatedLineCenter = detectedCenter;
      // Use the successful read timestamp, not this control cycle: reused
      // readings cannot advance the last-detection time after communication fails.
      lastLineDetectedAtUs = nowUs - reading.ageUs;
      hasDetectedLine = true;
      if (lineWasMissing)
        filterInitialized = false; // Re-seed the filter and suppress D on reacquisition.
      lineWasMissing = false;
      motorsReleasedForSensorLoss = false;
    }
    else
    {
      lineWasMissing = true;
      // Keep estimating/filtering from the last detected center during the gap.
      // With no previous detection, there is no center to hold: wait released.
      if (!hasDetectedLine || nowUs - lastLineDetectedAtUs > kMaximumLineMissingUs)
      {
        if (!motorsReleasedForSensorLoss)
        {
          stopAndReleaseMotors("waiting_for_line", true);
          motorsReleasedForSensorLoss = true;
        }
        return;
      }
    }
    const bool hadPreviousCenter = filterInitialized;
    const float previousCenter = filteredLineCenter;
    filterLineCenter(elapsedSeconds);
    const float centerChangePerSecond = hadPreviousCenter ? (filteredLineCenter - previousCenter) / elapsedSeconds : 0.0f;
    steeringCorrection = constrain(proportionalGain * filteredLineCenter +
                                       derivativeGain * centerChangePerSecond,
                                   -1.0f, 1.0f);
    leftWheelCommandMmPerSecond = lroundf(defaultSpeedMmPerSecond *
                                          (1.0f + fminf(steeringCorrection, 0.0f)));
    rightWheelCommandMmPerSecond = lroundf(defaultSpeedMmPerSecond *
                                           (1.0f - fmaxf(steeringCorrection, 0.0f)));
    if (!motorDriver.setWheelSpeedsMmPerSecond(
            kLeftMotorForwardSign * leftWheelCommandMmPerSecond,
            kRightMotorForwardSign * rightWheelCommandMmPerSecond))
      stopAndReleaseMotors("motor_write_failed");
    else
      runState = lineWasMissing ? "holding_last_line" : "following";
  }

  // Report interval timing rather than requested rates. Work percentage is control-loop
  // occupancy, not total CPU utilization: I2C waits count, other tasks do not.
  void printStatus()
  {
    const uint64_t statusStartedAtUs = esp_timer_get_time();
    const uint64_t intervalUs = statusStartedAtUs - previousStatusUs;
    const float actualControlHz = completedControlCycles * 1000000.0f / intervalUs;
    const float averageWorkUs = completedControlCycles ? static_cast<float>(totalControlWorkUs) / completedControlCycles : 0.0f;
    const float controlWorkPercent = 100.0f * averageWorkUs / kControlPeriodUs;
    const LineSensor::Reading reading = lineSensor->reading();
    Serial.printf("E5 state=%s raw=0x%02X center=%.3f filtered=%.3f correction=%.3f "
                  "speed=%d left=%d right=%d P=%.3f D=%.3f loop_hz=%.2f "
                  "work_pct=%.2f work_mean_us=%.1f work_peak_us=%llu "
                  "missed_slots=%lu previous_print_us=%llu\n",
                  runState, reading.rawMask, estimatedLineCenter, filteredLineCenter,
                  steeringCorrection, defaultSpeedMmPerSecond, leftWheelCommandMmPerSecond,
                  rightWheelCommandMmPerSecond, proportionalGain, derivativeGain, actualControlHz,
                  controlWorkPercent, averageWorkUs, maximumControlWorkUs,
                  static_cast<unsigned long>(missedControlSlots), serialOutputWorkUs);
    previousStatusUs = statusStartedAtUs;
    completedControlCycles = 0;
    totalControlWorkUs = 0;
    maximumControlWorkUs = 0;
    missedControlSlots = 0;
    serialOutputWorkUs = esp_timer_get_time() - statusStartedAtUs;
  }

  // Parse human-readable commands without waiting for a newline or USB connection.
  // These range checks define valid commands, rather than adding recovery behavior.
  void processSerialCommands()
  {
    while (Serial.available())
    {
      const char character = Serial.read();
      if (character == '\r')
        continue;
      if (character != '\n')
      {
        serialCommand += character;
        continue;
      }
      float requestedProportionalGain, requestedDerivativeGain, requestedStatusHz;
      int requestedSpeed;
      if (sscanf(serialCommand.c_str(), "gains %f %f", &requestedProportionalGain,
                 &requestedDerivativeGain) == 2)
      {
        proportionalGain = requestedProportionalGain;
        derivativeGain = requestedDerivativeGain;
      }
      else if (sscanf(serialCommand.c_str(), "speed %d", &requestedSpeed) == 1 &&
               requestedSpeed >= 0 && requestedSpeed <= 1000)
      {
        if (requestedSpeed == 0)
          stopAndReleaseMotors("serial_stop");
        else if (motorConfigurationSucceeded)
        {
          defaultSpeedMmPerSecond = requestedSpeed;
        }
      }
      else if (sscanf(serialCommand.c_str(), "status_hz %f", &requestedStatusHz) == 1 &&
               requestedStatusHz > 0.0f)
        statusFrequencyHz = requestedStatusHz;
      else if (serialCommand == "status")
        printStatus();
      else
        Serial.println("Commands: gains <P> <D>, speed <0..1000>, status_hz <Hz>, status");
      serialCommand = "";
    }
  }
}

// Configure both buses and the saved motor baseline; do not wait for USB Serial.
// User requested automatic 100 mm/s startup for operation disconnected from USB.
void setup()
{
  Serial.begin(115200);
  delay(200);
  static LineSensor sensor(D6, D7, kLineSensorRequestFrequencyHz, 5);
  lineSensor = &sensor;
  const bool motorBusStarted = Wire.begin(D4, D5, 400000);
  motorConfigurationSucceeded = motorBusStarted && motorDriver.initialize();
  delay(200);
  if (!motorConfigurationSucceeded || !lineSensor->ready())
    stopAndReleaseMotors("initialization_failed");
  else
  {
    Serial.println("Setup complete.");
  }
  previousStatusUs = esp_timer_get_time();
  nextControlStartUs = previousStatusUs;
}

// Schedule at 100 Hz without catch-up bursts. Status is outside timed control work,
// but its delay still affects actual frequency and the missed-slot count.
void loop()
{
  processSerialCommands();
  // Tick frequently; the sensor library schedules actual reads at 200 Hz.
  const uint64_t sensorTickStartedAtUs = esp_timer_get_time();
  lineSensor->tick();
  sensorWorkSinceControlUs += esp_timer_get_time() - sensorTickStartedAtUs;
  const uint64_t nowUs = esp_timer_get_time();
  if (nowUs >= nextControlStartUs)
  {
    const uint64_t lateSlots = (nowUs - nextControlStartUs) / kControlPeriodUs;
    missedControlSlots += lateSlots;
    nextControlStartUs += (lateSlots + 1) * kControlPeriodUs;
    const float elapsedSeconds = previousControlStartUs ? (nowUs - previousControlStartUs) / 1000000.0f : kControlPeriodUs / 1000000.0f;
    previousControlStartUs = nowUs;
    updateWheelCommands(elapsedSeconds);
    const uint64_t workUs = esp_timer_get_time() - nowUs + sensorWorkSinceControlUs;
    sensorWorkSinceControlUs = 0;
    totalControlWorkUs += workUs;
    maximumControlWorkUs = workUs > maximumControlWorkUs ? workUs : maximumControlWorkUs;
    ++completedControlCycles;
  }
  if (esp_timer_get_time() - previousStatusUs >= 1000000.0f / statusFrequencyHz)
    printStatus();
  delayMicroseconds(50); // Yield between deadlines without the old millisecond pacing issue.
}
