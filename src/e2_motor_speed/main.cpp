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
  constexpr int16_t kDefaultMin = -100, kDefaultMax = 100, kDefaultInc = 10;
  constexpr int16_t kSpeedLimit = 1000; // Driver speed range is +-1000; larger values are silently ignored.
  constexpr uint32_t kMaxSteps = 200;
  constexpr uint32_t kTickMs = 20, kBaselineMs = 1000, kSettleMs = 1000;
  constexpr uint32_t kMeasureMs = 2000, kRestMs = 1000;
  constexpr uint32_t kTrialMs = kBaselineMs + kSettleMs + kMeasureMs + kRestMs;
  int16_t sweepMin = kDefaultMin, sweepMax = kDefaultMax, sweepInc = kDefaultInc;
  uint32_t deadlineMs = 0;
  bool configured = false, running = false;
  uint32_t sweepStart = 0, stepStart = 0, nextTick = 0;
  uint32_t rows = 0, dropped = 0, missed = 0, errors = 0;
  int16_t target = kDefaultMin;
  uint8_t step = 0;
  char input[32];
  size_t used = 0;
  bool overflow = false;

  bool output(const char *format, ...)
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
      ++dropped;
      return false;
    }
    if (Serial && Serial.availableForWrite() >= n)
    {
      Serial.write(reinterpret_cast<const uint8_t *>(buffer), n);
      return true;
    }
    ++dropped;
    return false;
  }

  bool settings()
  {
    return output("# SETTINGS type=%u deadzone=%u lines=%u ratio=%u diameter_mm=%.1f pid=%g,%g,%g pid_source=stored_unverified\n",
                  MotorSettings::type, MotorSettings::deadZone, MotorSettings::pulseLine,
                  MotorSettings::pulsePhase, MotorSettings::diameterMm,
                  MotorSettings::storedP, MotorSettings::storedI, MotorSettings::storedD);
  }

  void finish(const char *reason)
  {
    running = false;
    // Attempt both writes even if the first fails. ACK is not physical stop proof.
    const bool stopped = driver.setSpeeds(0, 0);
    const bool released = driver.release();
    if (!stopped || !released)
    {
      configured = false;
      ++errors;
    }
    output("# END reason=%s rows=%lu dropped=%lu missed_ticks=%lu errors=%lu stop_ack=%u release_ack=%u\n",
           reason, (unsigned long)rows, (unsigned long)dropped,
           (unsigned long)missed, (unsigned long)errors, stopped, released);
  }

  void configure()
  {
    // Retain saved PID: no documented I2C access.
    configured = driver.applySettings();
    output("# CONFIG ack_all=%u error=%u pid_source=stored_unverified\n", configured, driver.error());
    settings();
  }

  void status()
  {
    // uptime_ms restarting from a small value means a real reset; reset_reason 1=power-on, 3=software, 4=panic, 15=brownout.
    output("# STATUS configured=%u running=%u schema=2 clock=400000 pid_source=stored_unverified uptime_ms=%lu reset_reason=%d\n",
           configured, running, (unsigned long)millis(), static_cast<int>(esp_reset_reason()));
  }

  void execute(const char *cmd)
  {
    if (!strcmp(cmd, "stop"))
    {
      finish("user_stop");
      return;
    }
    if (!strcmp(cmd, "status"))
    {
      status();
      return;
    }
    if (running)
    {
      output("# BUSY use stop or !\n");
      return;
    }
    if (!strcmp(cmd, "config"))
    {
      configure();
      return;
    }
    if (!strcmp(cmd, "help"))
    {
      output("# E2: start [min max inc] | stop | ! | status | config | help. Default -100 100 10, M2/M4 together.\n");
      return;
    }
    if (strncmp(cmd, "start", 5) || (cmd[5] != '\0' && cmd[5] != ' '))
    {
      output("# REFUSED unknown_command\n");
      return;
    }
    if (!configured)
    {
      output("# REFUSED configuration_failed\n");
      return;
    }
    // "start" alone uses the defaults; otherwise all three of min, max and inc are required.
    int lo = kDefaultMin, hi = kDefaultMax, inc = kDefaultInc;
    if (cmd[5] != '\0' && sscanf(cmd + 5, "%d %d %d", &lo, &hi, &inc) != 3)
    {
      output("# REFUSED usage: start [min max inc]\n");
      return;
    }
    if (inc <= 0 || lo > hi || lo < -kSpeedLimit || hi > kSpeedLimit ||
        static_cast<uint32_t>((hi - lo) / inc + 1) > kMaxSteps)
    {
      output("# REFUSED bad_range limit=%d max_steps=%lu\n", kSpeedLimit, (unsigned long)kMaxSteps);
      return;
    }
    sweepMin = lo;
    sweepMax = hi;
    sweepInc = inc;
    const uint32_t steps = (hi - lo) / inc + 1;
    deadlineMs = steps * kTrialMs + 5000;
    rows = dropped = missed = errors = 0;
    target = sweepMin;
    step = 0;
    // These three records together exceed the native USB TX buffer. Pace them
    // before starting the motion timer, and refuse motion if any cannot be queued.
    if (!settings())
      return;
    delay(10);
    if (!output("# START schema=2 mode=speed clock_hz=400000 min=%d max=%d inc=%d steps=%lu baseline_ms=%lu settle_ms=%lu measure_ms=%lu rest_ms=%lu\n",
                sweepMin, sweepMax, sweepInc, (unsigned long)steps, (unsigned long)kBaselineMs,
                (unsigned long)kSettleMs, (unsigned long)kMeasureMs, (unsigned long)kRestMs))
      return;
    delay(10);
    if (!output("step,command,phase,step_ms,t2_us,m2_count,t4_us,m4_count,m2_recent,m4_recent,write_us,read_us,applied\n"))
      return;
    sweepStart = stepStart = nextTick = millis();
    running = true;
  }

  void tick()
  {
    const uint32_t elapsed = millis() - stepStart;
    const bool active = elapsed >= kBaselineMs && elapsed < kBaselineMs + kSettleMs + kMeasureMs;
    const char *phase = elapsed < kBaselineMs ? "baseline" : elapsed < kBaselineMs + kSettleMs ? "settle"
                                                         : active                              ? "measure"
                                                                                               : "rest";
    const int16_t applied = active ? target : 0;
    const uint32_t begin = micros();
    if (!driver.setSpeeds(applied, applied))
    {
      ++errors;
      finish("write_error");
      return;
    }
    const uint32_t writeUs = micros() - begin;
    int16_t recent2, recent4;
    int32_t c2, c4;
    if (!driver.readRecent(2, recent2) || !driver.readRecent(4, recent4) || !driver.readTotal(2, c2))
    {
      ++errors;
      finish("read_error");
      return;
    }
    const uint32_t t2 = micros();
    if (!driver.readTotal(4, c4))
    {
      ++errors;
      finish("read_error");
      return;
    }
    const uint32_t t4 = micros();
    ++rows;
    // Raw counts and separate read timestamps allow signed velocity calculations,
    // including counter/timer wraparound, without assuming encoder scaling here.
    output("%u,%d,%s,%lu,%lu,%ld,%lu,%ld,%d,%d,%lu,%lu,%d\n", step, target, phase,
           (unsigned long)elapsed, (unsigned long)t2, (long)c2,
           (unsigned long)t4, (long)c4, recent2, recent4, (unsigned long)writeUs,
           (unsigned long)(t4 - begin - writeUs), applied);
  }
} // namespace

void setup()
{
  Serial.begin(115200); // PC <-> ESP32 only; no motor-driver UART.
  Serial.setTxTimeoutMs(0);
  Wire.begin(D4, D5, 400000);
  Wire.setTimeOut(2);
  delay(500);
  configure();
}

void loop()
{
  for (uint8_t i = 0; i < 32 && Serial.available(); ++i)
  {
    const char c = Serial.read();
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
        output("# REFUSED input_too_long\n");
      }
    }
  }
  if (running)
  {
    if (millis() - sweepStart >= deadlineMs)
      finish("overall_timeout");
    else
    {
      if (millis() - stepStart >= kTrialMs)
      {
        if (target + sweepInc > sweepMax)
          finish("complete");
        else
        {
          target += sweepInc;
          ++step;
          stepStart = nextTick = millis();
        }
      }
      if (running && static_cast<int32_t>(millis() - nextTick) >= 0)
      {
        const uint32_t late = millis() - nextTick;
        missed += late / kTickMs;
        nextTick += (late / kTickMs + 1) * kTickMs;
        tick();
      }
    }
  }
  delay(1);
}
