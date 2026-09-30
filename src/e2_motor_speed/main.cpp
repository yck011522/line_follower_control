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
  // then rest at zero 1 s. Total: 21 targets x 5 seconds = 105 seconds.
  constexpr int16_t kMin = -100, kMax = 100, kIncrement = 10;
  constexpr uint32_t kTickMs = 20, kBaselineMs = 1000, kSettleMs = 1000;
  constexpr uint32_t kMeasureMs = 2000, kRestMs = 1000;
  constexpr uint32_t kTrialMs = kBaselineMs + kSettleMs + kMeasureMs + kRestMs;
  constexpr uint32_t kSteps = (kMax - kMin) / kIncrement + 1;
  constexpr uint32_t kDeadlineMs = kSteps * kTrialMs + 5000;
  bool configured = false, running = false;
  uint32_t sweepStart = 0, stepStart = 0, nextTick = 0;
  uint32_t rows = 0, dropped = 0, missed = 0, errors = 0;
  int16_t target = kMin;
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
    return output("# SETTINGS type=%u deadzone=%u lines=%u ratio=%u diameter_mm=%.1f pid=3,0.375,0.5 pid_source=stored_unverified\n",
                  MotorSettings::type, MotorSettings::deadZone, MotorSettings::pulseLine,
                  MotorSettings::pulsePhase, MotorSettings::diameterMm);
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
    // Rewrite five settings on boot. Retain saved PID: no documented I2C access.
    configured = false;
    bool ok = driver.release();
    if (ok)
    {
      ok = driver.setType(MotorSettings::type);
      delay(100);
    }
    if (ok)
    {
      ok = driver.setParameter(0x02, MotorSettings::deadZone, false);
      delay(100);
    }
    if (ok)
    {
      ok = driver.setParameter(0x03, MotorSettings::pulseLine, false);
      delay(100);
    }
    if (ok)
    {
      ok = driver.setParameter(0x04, MotorSettings::pulsePhase, false);
      delay(100);
    }
    if (ok)
    {
      ok = driver.setDiameter(MotorSettings::diameterMm);
      delay(100);
    }
    const uint8_t error = driver.error();
    const bool released = driver.release();
    configured = ok && released;
    output("# CONFIG ack_all=%u error=%u pid_source=stored_unverified\n", configured, error);
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
      output("# E2: start | stop | ! | status | config | help. Fixed -100..100 step 10, M2/M4 together.\n");
      return;
    }
    if (strcmp(cmd, "start"))
    {
      output("# REFUSED unknown_command\n");
      return;
    }
    if (!configured)
    {
      output("# REFUSED configuration_failed\n");
      return;
    }
    rows = dropped = missed = errors = 0;
    target = kMin;
    step = 0;
    // These three records together exceed the native USB TX buffer. Pace them
    // before starting the motion timer, and refuse motion if any cannot be queued.
    if (!settings())
      return;
    delay(10);
    if (!output("# START schema=2 mode=speed clock_hz=400000 min=-100 max=100 inc=10 steps=21 baseline_ms=1000 settle_ms=1000 measure_ms=2000 rest_ms=1000\n"))
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
    uint16_t r2, r4;
    uint32_t c2, c4;
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
    ++rows;
    // Raw counts and separate read timestamps allow signed velocity calculations,
    // including counter/timer wraparound, without assuming encoder scaling here.
    output("%u,%d,%s,%lu,%lu,%ld,%lu,%ld,%d,%d,%lu,%lu,%d\n", step, target, phase,
           (unsigned long)elapsed, (unsigned long)t2, (long)static_cast<int32_t>(c2),
           (unsigned long)t4, (long)static_cast<int32_t>(c4), static_cast<int16_t>(r2),
           static_cast<int16_t>(r4), (unsigned long)writeUs,
           (unsigned long)(t4 - begin - writeUs), applied);
  }
} // namespace

void setup()
{
  Serial.begin(115200); // PC <-> ESP32 only; no motor-driver UART.
  Serial.setTxTimeoutMs(0);
  Wire.begin(D4, D5, 400000);
  Wire.setTimeOut(5);
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
    if (millis() - sweepStart >= kDeadlineMs)
      finish("overall_timeout");
    else
    {
      if (millis() - stepStart >= kTrialMs)
      {
        if (target >= kMax)
          finish("complete");
        else
        {
          target += kIncrement;
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
