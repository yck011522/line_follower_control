#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>
#include <MotorDriver.h>
#include <MotorSettings.h>
#include <cstdarg>
#include <cstring>

namespace
{
  MotorDriver driver(Wire);
  // Each target starts from zero: baseline 1 s, settling 1 s, measurement 2 s,
  // then rest at zero 1 s, so each target takes 5 s.
  constexpr int16_t kDefaultMin = -100;
  constexpr int16_t kDefaultMax = 100;
  constexpr int16_t kDefaultInc = 10;
  constexpr int16_t kSpeedLimit = 1000; // Driver speed range is +-1000; larger values are silently ignored.
  constexpr uint32_t kMaxSteps = 200;
  constexpr uint32_t kTickMs = 20;
  constexpr uint32_t kBaselineMs = 1000;
  constexpr uint32_t kSettleMs = 1000;
  constexpr uint32_t kMeasureMs = 2000;
  constexpr uint32_t kRestMs = 1000;
  constexpr uint32_t kTrialMs = kBaselineMs + kSettleMs + kMeasureMs + kRestMs;
  // Sweep progression and configuration readiness are kept together rather
  // than spread across unrelated globals. No motion begins until startSweep().
  struct SweepState
  {
    bool configured = false;
    bool running = false;
    int16_t minimum = kDefaultMin;
    int16_t maximum = kDefaultMax;
    int16_t increment = kDefaultInc;
    int16_t target = kDefaultMin;
    uint8_t step = 0;
    uint32_t deadlineMs = 0;
    uint32_t startedAtMs = 0;
    uint32_t trialStartedAtMs = 0;
    uint32_t nextSampleAtMs = 0;
  };

  // Counters for the current sweep. Dropped log output is counted separately
  // from failed motor communication and skipped sampling deadlines.
  struct Statistics
  {
    uint32_t rows = 0;
    uint32_t dropped = 0;
    uint32_t missedTicks = 0;
    uint32_t errors = 0;
  };

  // A fixed command buffer avoids dynamic allocation. An overflowed command
  // is discarded until its newline; the immediate stop character bypasses it.
  struct SerialInput
  {
    char buffer[32] = {};
    size_t length = 0;
    bool overflow = false;
  };

  // The phase tells sampling which CSV label and speed command to use.
  struct TrialPhase
  {
    const char *name;
    bool motorsActive;
  };

  SweepState sweep;
  Statistics statistics;
  SerialInput serialInput;

  // Queue one formatted record only when USB has enough space. Count drops
  // instead of blocking the motor loop while the PC is not consuming output.
  bool writeLog(const char *format, ...)
  {
    // Never block motion control waiting for the host to read USB output.
    char buffer[256];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (n <= 0)
      return false;
    if (n >= static_cast<int>(sizeof(buffer)))
    {
      ++statistics.dropped;
      return false;
    }
    if (Serial && Serial.availableForWrite() >= n)
    {
      Serial.write(reinterpret_cast<const uint8_t *>(buffer), n);
      return true;
    }
    ++statistics.dropped;
    return false;
  }

  // Report the adopted configuration; PID is saved/assumed, never read via I2C.
  bool printSettings()
  {
    return writeLog("# SETTINGS type=%u deadzone=%u lines=%u ratio=%u diameter_mm=%.1f pid=%g,%g,%g pid_source=stored_unverified\n",
                  MotorSettings::motorType, MotorSettings::pwmDeadZone, MotorSettings::encoderPulsesPerMotorRevolution,
                  MotorSettings::gearRatio, MotorSettings::wheelDiameterMm,
                  MotorSettings::storedP, MotorSettings::storedI, MotorSettings::storedD);
  }

  // End the run and attempt both zero speed and output release on every exit.
  // Failed acknowledgments invalidate configuration readiness for future motion.
  void finishSweep(const char *reason)
  {
    sweep.running = false;
    // Attempt both writes even if the first fails. ACK is not physical stop proof.
    const bool stopped = driver.setWheelSpeedsMmPerSecond(0, 0);
    const bool released = driver.releaseMotorOutputs();
    if (!stopped || !released)
    {
      sweep.configured = false;
      ++statistics.errors;
    }
    writeLog("# END reason=%s rows=%lu dropped=%lu missed_ticks=%lu errors=%lu stop_ack=%u release_ack=%u\n",
           reason, (unsigned long)statistics.rows, (unsigned long)statistics.dropped,
           (unsigned long)statistics.missedTicks, (unsigned long)statistics.errors, stopped, released);
  }

  // Reapply the five I2C-accessible settings on boot or explicit config.
  // Successful acknowledgments are required before allowing a sweep to start.
  void configureMotorDriver()
  {
    // Retain saved PID: no documented I2C access.
    sweep.configured = driver.initialize();
    writeLog("# CONFIG ack_all=%u error=%u pid_source=stored_unverified\n", sweep.configured, driver.lastCommunicationError());
    printSettings();
  }

  // Report readiness and reset information without changing the experiment.
  void printStatus()
  {
    // uptime_ms restarting from a small value means a real reset; reset_reason 1=power-on, 3=software, 4=panic, 15=brownout.
    writeLog("# STATUS configured=%u running=%u schema=2 clock=400000 pid_source=stored_unverified uptime_ms=%lu reset_reason=%d\n",
           sweep.configured, sweep.running, (unsigned long)millis(), static_cast<int>(esp_reset_reason()));
  }

  // Validate the start command and initialize a bounded sweep. The serial
  // headers must all queue successfully before the motion timer is started.
  void startSweep(const char *command)
  {
    if (!sweep.configured)
    {
      writeLog("# REFUSED configuration_failed\n");
      return;
    }
    // "start" alone uses the defaults; otherwise all three of min, max and inc are required.
    int minimum = kDefaultMin, maximum = kDefaultMax, increment = kDefaultInc;
    if (command[5] != '\0' && sscanf(command + 5, "%d %d %d", &minimum, &maximum, &increment) != 3)
    {
      writeLog("# REFUSED usage: start [min max inc]\n");
      return;
    }
    if (increment <= 0 || minimum > maximum || minimum < -kSpeedLimit || maximum > kSpeedLimit ||
        static_cast<uint32_t>((maximum - minimum) / increment + 1) > kMaxSteps)
    {
      writeLog("# REFUSED bad_range limit=%d max_steps=%lu\n", kSpeedLimit, (unsigned long)kMaxSteps);
      return;
    }
    sweep.minimum = minimum;
    sweep.maximum = maximum;
    sweep.increment = increment;
    const uint32_t steps = (maximum - minimum) / increment + 1;
    sweep.deadlineMs = steps * kTrialMs + 5000;
    statistics.rows = statistics.dropped = statistics.missedTicks = statistics.errors = 0;
    sweep.target = sweep.minimum;
    sweep.step = 0;
    // These three records together exceed the native USB TX buffer. Pace them
    // before starting the motion timer, and refuse motion if any cannot be queued.
    if (!printSettings())
      return;
    delay(10);
    if (!writeLog("# START schema=2 mode=speed clock_hz=400000 min=%d max=%d inc=%d steps=%lu baseline_ms=%lu settle_ms=%lu measure_ms=%lu rest_ms=%lu\n",
                sweep.minimum, sweep.maximum, sweep.increment, (unsigned long)steps, (unsigned long)kBaselineMs,
                (unsigned long)kSettleMs, (unsigned long)kMeasureMs, (unsigned long)kRestMs))
      return;
    delay(10);
    if (!writeLog("step,command,phase,step_ms,t2_us,m2_count,t4_us,m4_count,m2_recent,m4_recent,write_us,read_us,applied\n"))
      return;
    sweep.startedAtMs = sweep.trialStartedAtMs = sweep.nextSampleAtMs = millis();
    sweep.running = true;
  }

  // Dispatch serial commands. Stop/status remain available during a sweep;
  // other commands are refused while motion is running, as before.
  void handleCommand(const char *command)
  {
    if (!strcmp(command, "stop"))
    {
      finishSweep("user_stop");
      return;
    }
    if (!strcmp(command, "status"))
    {
      printStatus();
      return;
    }
    if (sweep.running)
    {
      writeLog("# BUSY use stop or !\n");
      return;
    }
    if (!strcmp(command, "config"))
    {
      configureMotorDriver();
      return;
    }
    if (!strcmp(command, "help"))
    {
      writeLog("# E2: start [min max inc] | stop | ! | status | config | help. Default -100 100 10, M2/M4 together.\n");
      return;
    }
    if (strncmp(command, "start", 5) || (command[5] != '\0' && command[5] != ' '))
    {
      writeLog("# REFUSED unknown_command\n");
      return;
    }
    startSweep(command);
  }

  // Identify the four trial phases without a nested conditional expression.
  // Only settling and measurement apply the target; baseline/rest command zero.
  TrialPhase trialPhase(uint32_t elapsedMs)
  {
    if (elapsedMs < kBaselineMs) return {"baseline", false};
    if (elapsedMs < kBaselineMs + kSettleMs) return {"settle", true};
    if (elapsedMs < kBaselineMs + kSettleMs + kMeasureMs) return {"measure", true};
    return {"rest", false};
  }

  // Command M2/M4, read their encoder feedback, and log one schema-2 CSV row.
  // Any failed transaction immediately ends the sweep through finishSweep().
  void sampleMotors()
  {
    const uint32_t trialElapsedMs = millis() - sweep.trialStartedAtMs;
    const TrialPhase phase = trialPhase(trialElapsedMs);
    const int16_t applied = phase.motorsActive ? sweep.target : 0;
    const uint32_t sampleStartedAtUs = micros();
    if (!driver.setWheelSpeedsMmPerSecond(applied, applied))
    {
      ++statistics.errors;
      finishSweep("write_error");
      return;
    }
    const uint32_t writeDurationUs = micros() - sampleStartedAtUs;
    int16_t motor2Recent, motor4Recent;
    int32_t motor2Count, motor4Count;
    if (!driver.readEncoderCountLast10Ms(MotorDriver::Wheel::Left, motor2Recent) || !driver.readEncoderCountLast10Ms(MotorDriver::Wheel::Right, motor4Recent) || !driver.readEncoderPosition(MotorDriver::Wheel::Left, motor2Count))
    {
      ++statistics.errors;
      finishSweep("read_error");
      return;
    }
    // Timestamp each cumulative count separately; reads do not occur together.
    const uint32_t motor2ReadAtUs = micros();
    if (!driver.readEncoderPosition(MotorDriver::Wheel::Right, motor4Count))
    {
      ++statistics.errors;
      finishSweep("read_error");
      return;
    }
    const uint32_t motor4ReadAtUs = micros();
    ++statistics.rows;
    // Raw counts and separate read timestamps allow signed velocity calculations,
    // including counter/timer wraparound, without assuming encoder scaling here.
    writeLog("%u,%d,%s,%lu,%lu,%ld,%lu,%ld,%d,%d,%lu,%lu,%d\n", sweep.step, sweep.target, phase.name,
           (unsigned long)trialElapsedMs, (unsigned long)motor2ReadAtUs, (long)motor2Count,
           (unsigned long)motor4ReadAtUs, (long)motor4Count, motor2Recent, motor4Recent, (unsigned long)writeDurationUs,
           (unsigned long)(motor4ReadAtUs - sampleStartedAtUs - writeDurationUs), applied);
  }

  // Consume at most 32 bytes per loop, preserving time for motor sampling.
  // '!' stops immediately and discards the remaining partial command line.
  void processSerialInput()
  {
    for (uint8_t i = 0; i < 32 && Serial.available(); ++i)
    {
      const char c = Serial.read();
      if (c == '!')
      {
        finishSweep("user_stop");
        serialInput.length = 0;
        serialInput.overflow = true;
      }
      else if (c == '\n' || c == '\r')
      {
        if (serialInput.length && !serialInput.overflow)
        {
          serialInput.buffer[serialInput.length] = '\0';
          handleCommand(serialInput.buffer);
        }
        serialInput.length = 0;
        serialInput.overflow = false;
      }
      else if (!serialInput.overflow)
      {
        if (serialInput.length + 1 < sizeof(serialInput.buffer))
          serialInput.buffer[serialInput.length++] = c;
        else
        {
          serialInput.overflow = true;
          writeLog("# REFUSED input_too_long\n");
        }
      }
    }
  }

  // Enforce the overall deadline, advance trials, and service one due sample.
  // Late sample slots are counted and skipped rather than replayed in a burst.
  void updateSweep()
  {
    if (!sweep.running)
      return;

    if (millis() - sweep.startedAtMs >= sweep.deadlineMs)
    {
      finishSweep("overall_timeout");
      return;
    }

    if (millis() - sweep.trialStartedAtMs >= kTrialMs)
    {
      if (sweep.target + sweep.increment > sweep.maximum)
      {
        finishSweep("complete");
        return;
      }
      // Each new target restarts its own baseline/settle/measure/rest trial.
      sweep.target += sweep.increment;
      ++sweep.step;
      sweep.trialStartedAtMs = sweep.nextSampleAtMs = millis();
    }

    if (static_cast<int32_t>(millis() - sweep.nextSampleAtMs) >= 0)
    {
      const uint32_t latenessMs = millis() - sweep.nextSampleAtMs;
      statistics.missedTicks += latenessMs / kTickMs;
      sweep.nextSampleAtMs += (latenessMs / kTickMs + 1) * kTickMs;
      sampleMotors();
    }
  }
} // namespace

// Initialize USB and the motor I2C bus, then configure without starting motion.
void setup()
{
  Serial.begin(115200); // PC <-> ESP32 only; no motor-driver UART.
  Serial.setTxTimeoutMs(0);
  Wire.begin(D4, D5, 400000);
  Wire.setTimeOut(2);
  delay(500);
  configureMotorDriver();
}

// Process commands first so an immediate stop is handled before sampling.
// Then advance the sweep without blocking for entire trial phases.
void loop()
{
  processSerialInput();
  updateSweep();
  delay(1);
}
