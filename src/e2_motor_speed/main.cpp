#include <Arduino.h>
#include <Wire.h>
#include <MotorDriver.h>
#include <cstdarg>
#include <cstring>

namespace
{
  MotorDriver driver(Wire);
  // Control tick period, per-step settle time, and total time per sweep step.
  constexpr uint32_t kTickMs = 20, kSettleMs = 1000, kStepMs = 2000;
  // Sweep range/increment; defaults are set per mode when a sweep starts.
  int16_t sweepMin = -240, sweepMax = 240, sweepInc = 20;
  bool pwmMode = false;    // true: open-loop PWM (reg 0x07); false: closed-loop speed (reg 0x06)
  int32_t deadZone = 1650; // PWM dead zone written at startup, 0..3600 (manufacturer default is 1600).
  uint16_t encoderLines = 2000, gearRatio = 23;
  float wheelDiameter = 65.0f; // mm
  bool manual = false;         // true while a manual pwm/speed command is holding the motors
  bool reporting = true;       // 5 Hz "R,..." status lines while no sweep is running
  uint32_t nextReport = 0;
  constexpr uint32_t kReportMs = 200;
  uint32_t deadline = 60000; // Overall sweep time limit in ms; recomputed from the step count.
  bool configured = false, running = false, ready = false;
  uint32_t sweepStart = 0, stepStart = 0, nextTick = 0, rows = 0, dropped = 0, missed = 0;
  uint32_t errors = 0;
  int16_t commandSpeed = -240; // Current command; a speed or PWM value depending on mode.
  uint8_t step = 0;
  char input[32];                  // Serial command line buffer.
  const char *lastReason = "none"; // Reason the last sweep ended, shown by status.
  size_t used = 0;
  bool overflow = false;

  void output(const char *format, ...)
  {
    // printf-style write that drops (and counts) lines instead of blocking on a full USB buffer.
    char buffer[256];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (n <= 0)
      return;
    size_t count = min(static_cast<size_t>(n), sizeof(buffer) - 1);
    if (Serial && Serial.availableForWrite() >= static_cast<int>(count))
      Serial.write(reinterpret_cast<const uint8_t *>(buffer), count);
    else
      ++dropped;
  }

  void finish(const char *reason)
  {
    // Ends the sweep: zero speed, then release outputs (zero PWM), then report.
    running = false;
    manual = false;
    lastReason = reason;
    const bool stopped = driver.setSpeeds(0, 0);
    const uint8_t stopError = driver.error();
    const bool released = driver.release();
    if (!stopped || !released)
    {
      configured = false;
      ++errors;
    }
    output("# END reason=%s rows=%lu dropped=%lu missed_ticks=%lu errors=%lu stop_ack=%u release_ack=%u stop_err=%u release_err=%u\n",
           reason, (unsigned long)rows, (unsigned long)dropped, (unsigned long)missed,
           (unsigned long)errors, stopped, released, stopError, driver.error());
  }

  void configure()
  {
    // Writes motor type, encoder lines, gear ratio, wheel diameter and dead zone; 100 ms between writes.
    configured = false;
    bool ok = driver.release();
    if (ok)
    {
      ok = driver.setType(1);
      delay(100);
    }
    if (ok)
    {
      ok = driver.setParameter(0x03, encoderLines, false);
      delay(100);
    }
    if (ok)
    {
      ok = driver.setParameter(0x04, gearRatio, false);
      delay(100);
    }
    if (ok)
    {
      ok = driver.setDiameter(wheelDiameter);
      delay(100);
    }
    if (ok && deadZone >= 0)
    {
      // Register 0x02 is the driver's PWM dead zone; it persists in the driver's flash.
      ok = driver.setParameter(0x02, static_cast<uint16_t>(deadZone), false);
      delay(100);
    }
    const uint8_t error = driver.error();
    const bool released = driver.release();
    configured = ok && released;
    output("# CONFIG ack_all=%u error=%u type=1 lines=%u ratio=%u diameter_mm=%.1f dead_zone=%ld uint16=BE float=LE\n",
           configured, error, encoderLines, gearRatio, wheelDiameter, (long)deadZone);
  }

  // Writes one uint16 driver parameter and reports whether the driver acknowledged it.
  void writeParameter(const char *name, uint8_t reg, long value, long maximum, int32_t *stored)
  {
    if (value < 0 || value > maximum)
    {
      output("# REFUSED %s must be 0..%ld\n", name, maximum);
      return;
    }
    const bool ok = driver.setParameter(reg, static_cast<uint16_t>(value), false);
    if (ok)
      *stored = value;
    output("# %s=%ld ack=%u error=%u\n", name, value, ok, driver.error());
  }

  // One 5 Hz status line: R,ms,M2 recent counts/10 ms,M2 total counts,M4 recent,M4 total.
  void report()
  {
    uint16_t r2, r4;
    uint32_t c2, c4;
    if (driver.readWord(0x11, r2) && driver.readWord(0x13, r4) &&
        driver.readCumulative(0x22, c2) && driver.readCumulative(0x26, c4))
      output("R,%lu,%d,%ld,%d,%ld\n", (unsigned long)millis(), static_cast<int16_t>(r2),
             (long)static_cast<int32_t>(c2), static_cast<int16_t>(r4), (long)static_cast<int32_t>(c4));
    else
    {
      ++errors;
      output("# READ_ERROR err=%u\n", driver.error());
    }
  }

  bool apply()
  {
    // Sends the current command to both motors in the active mode.
    return pwmMode ? driver.setPwm(commandSpeed, commandSpeed) : driver.setSpeeds(commandSpeed, commandSpeed);
  }

  bool startStep()
  {
    // Begins a new sweep step: apply the command and restart the step timer.
    if (!apply())
    {
      ++errors;
      finish("command_error");
      return false;
    }
    stepStart = millis();
    nextTick = stepStart;
    output("# STEP index=%u command=%d settle_ms=1000 measure_ms=1000\n", step, commandSpeed);
    return true;
  }

  void execute(const char *cmd)
  {
    // Handles one serial command line: stop, deadzone N, start/pwm [min max inc], config, status, help.
    if (!strcmp(cmd, "stop"))
    {
      finish("user_stop");
      return;
    }
    if (running)
    {
      output("# BUSY: stop or ! cancels\n");
      return;
    }
    if (!strncmp(cmd, "deadzone ", 9))
    {
      int32_t v = deadZone;
      writeParameter("deadzone", 0x02, atol(cmd + 9), 3600, &v);
      deadZone = v;
      return;
    }
    if (!strncmp(cmd, "lines ", 6))
    {
      int32_t v = encoderLines;
      writeParameter("lines", 0x03, atol(cmd + 6), 65535, &v);
      encoderLines = v;
      return;
    }
    if (!strncmp(cmd, "ratio ", 6))
    {
      int32_t v = gearRatio;
      writeParameter("ratio", 0x04, atol(cmd + 6), 65535, &v);
      gearRatio = v;
      return;
    }
    if (!strncmp(cmd, "diameter ", 9))
    {
      const float mm = atof(cmd + 9);
      if (!(mm > 0.0f && mm <= 1000.0f))
      {
        output("# REFUSED diameter must be >0 and <=1000 mm\n");
        return;
      }
      const bool ok = driver.setDiameter(mm);
      if (ok)
        wheelDiameter = mm;
      output("# diameter=%.1f ack=%u error=%u\n", mm, ok, driver.error());
      return;
    }
    if (!strncmp(cmd, "report ", 7))
    {
      reporting = atoi(cmd + 7) != 0;
      output("# report=%u\n", reporting);
      return;
    }
    // "pwm N" / "speed N": hold both motors at one value (open-loop PWM or closed-loop speed).
    if (!strncmp(cmd, "pwm ", 4) || !strncmp(cmd, "speed ", 6))
    {
      const bool isPwm = cmd[0] == 'p';
      int v, extra;
      if (sscanf(cmd, "%*s %d %d", &v, &extra) == 1)
      {
        const int limit = isPwm ? 3600 : 1000;
        if (v < -limit || v > limit)
        {
          output("# REFUSED %s must be -%d..%d\n", isPwm ? "pwm" : "speed", limit, limit);
          return;
        }
        pwmMode = isPwm;
        commandSpeed = v;
        manual = true;
        const bool ok = apply();
        if (!ok)
          ++errors;
        output("# %s=%d ack=%u error=%u\n", isPwm ? "pwm" : "speed", v, ok, driver.error());
        return;
      }
    }
    if (!strncmp(cmd, "start", 5) || !strncmp(cmd, "pwm", 3))
    {
      if (!configured)
      {
        output("# REFUSED configuration_failed; use config after powering driver\n");
        return;
      }
      pwmMode = cmd[0] == 'p';
      // Mode defaults, overridden below if the command supplies min max inc.
      if (pwmMode)
      {
        sweepMin = -1000;
        sweepMax = 1000;
        sweepInc = 50;
      }
      else
      {
        sweepMin = -240;
        sweepMax = 240;
        sweepInc = 20;
      }
      int a, b, c;
      const int given = sscanf(cmd, "%*s %d %d %d", &a, &b, &c);
      if (given == 2)
      {
        output("# REFUSED sweep needs min max inc\n");
        return;
      }
      if (given == 3)
      {
        if (c <= 0 || a > b || a < -3600 || b > 3600 || (b - a) / c > 200)
        {
          output("# REFUSED bad range\n");
          return;
        }
        sweepMin = a;
        sweepMax = b;
        sweepInc = c;
      }
      const uint32_t steps = (sweepMax - sweepMin) / sweepInc + 1;
      deadline = steps * kStepMs + 5000; // 5 s margin beyond the nominal sweep time
      rows = dropped = missed = errors = 0;
      commandSpeed = sweepMin;
      step = 0;
      output("# START mode=%s clock_hz=400000 command_units=unverified settle_ms=1000 measure_ms=1000 min=%d max=%d inc=%d steps=%lu\n",
             pwmMode ? "pwm" : "speed", sweepMin, sweepMax, sweepInc, (unsigned long)steps);
      output("step,command,phase,step_ms,t2_us,m2_count,t4_us,m4_count,m2_recent,m4_recent,write_us,read_us\n");
      sweepStart = millis();
      running = true;
      startStep();
    }
    else if (!strcmp(cmd, "config"))
      configure();
    else if (!strcmp(cmd, "status"))
      output("# STATUS configured=%u running=%u clock=%lu errors=%lu last_end=%s rows=%lu dropped=%lu manual=%u deadzone=%ld lines=%u ratio=%u diameter_mm=%.1f\n", configured, running, (unsigned long)Wire.getClock(), (unsigned long)errors, lastReason, (unsigned long)rows, (unsigned long)dropped, manual, (long)deadZone, encoderLines, gearRatio, wheelDiameter);
    else if (!strcmp(cmd, "help"))
    {
      output("# Manual: pwm N (-3600..3600); speed N (-1000..1000 mm/s); stop or ! releases; report 0|1 toggles 5 Hz R lines\n");
      output("# Config: deadzone N (0..3600); lines N; ratio N; diameter MM; config rewrites all; status\n");
      output("# Sweeps: start|pwm min max inc (default -240 240 20 / -1000 1000 50)\n");
      output("# Boot configures at 400kHz; sweeps require lifted wheels. Final output is released.\n");
    }
    else if (*cmd)
      output("# Unknown command: %s\n", cmd);
  }

  void tick()
  {
    // One 50 Hz sample: refresh the command, read speed and cumulative counts, print a CSV row.
    const uint32_t begin = micros();
    if (!apply())
    {
      ++errors;
      finish("write_error");
      return;
    }
    const uint32_t writeUs = micros() - begin;
    uint16_t r2, r4; // Recent-count registers 0x11 (M2) and 0x13 (M4).
    uint32_t c2, c4; // Cumulative encoder counts, registers 0x22 and 0x26.
    if (!driver.readWord(0x11, r2) || !driver.readWord(0x13, r4) || !driver.readCumulative(0x22, c2))
    {
      ++errors;
      finish("read_error");
      return;
    }
    const uint32_t t2 = micros();
    if (!driver.readCumulative(0x26, c4))
    {
      ++errors;
      finish("read_error");
      return;
    }
    const uint32_t t4 = micros();
    const uint32_t stepMs = millis() - stepStart;
    ++rows;
    output("%u,%d,%s,%lu,%lu,%ld,%lu,%ld,%d,%d,%lu,%lu\n", step, commandSpeed,
           stepMs < kSettleMs ? "settle" : "measure", (unsigned long)stepMs,
           (unsigned long)t2, (long)static_cast<int32_t>(c2), (unsigned long)t4,
           (long)static_cast<int32_t>(c4), static_cast<int16_t>(r2), static_cast<int16_t>(r4),
           (unsigned long)writeUs, (unsigned long)(t4 - begin - writeUs));
  }
} // namespace

void setup()
{
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  Wire.begin(D4, D5, 400000);
  Wire.setTimeOut(5);
  delay(500);
  configure();
}

void loop()
{
  if (Serial && !ready)
  {
    execute("help");
    execute("status");
    ready = true;
  }
  for (uint8_t i = 0; i < 32 && Serial.available(); ++i)
  {
    // Line-based command input; '!' cancels immediately without needing Enter.
    char c = Serial.read();
    if (c == '!')
    {
      finish("user_stop");
      used = 0;
      overflow = true;
    }
    else if (c == '\n' || c == '\r')
    {
      if (used && !overflow)
      {
        input[used] = '\0';
        execute(input);
      }
      used = 0;
      overflow = false;
    }
    else if (!overflow)
    {
      if (used + 1 < sizeof(input))
        input[used++] = c;
      else
      {
        overflow = true;
        output("# Input too long; discarded\n");
      }
    }
  }
  if (running)
  {
    if (millis() - sweepStart >= deadline)
      finish("overall_timeout");
    else
    {
      if (millis() - stepStart >= kStepMs)
      {
        // Step finished: stop after the last one, otherwise advance to the next command.
        if (commandSpeed >= sweepMax)
          finish("complete");
        else
        {
          commandSpeed += sweepInc;
          ++step;
          startStep();
        }
      }
      if (running && static_cast<int32_t>(millis() - nextTick) >= 0)
      {
        const uint32_t late = millis() - nextTick;
        missed += late / kTickMs; // Ticks skipped because the loop ran late.
        nextTick += (late / kTickMs + 1) * kTickMs;
        tick();
      }
    }
  }
  if (!running && reporting && static_cast<int32_t>(millis() - nextReport) >= 0)
  {
    nextReport = millis() + kReportMs;
    report();
  }
  delay(1);
}
