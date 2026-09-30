#include <Arduino.h>
#include <Wire.h>
#include <MotorDriver.h>
#include <cstring>
#include <cstdarg>

namespace {
MotorDriver driver(Wire);
constexpr uint32_t kSampleMs = 20;
constexpr uint32_t kPulseMs = 5000;
constexpr int16_t kSpeed = 100;  // Driver units; physical mm/s scale not validated.
uint32_t clockHz = 400000;
uint32_t nextWrite = 0, runReads = 0, runWrites = 0, readMax = 0, writeMax = 0;
uint64_t readSum = 0, writeSum = 0;
bool configured = false, armed = false, active = false, watch = false;
bool observeStop = false, previousValid = false, stable = false;
uint8_t motor = 0;
uint32_t started = 0, stopped = 0, stableSince = 0, nextSample = 0, nextPrint = 0;
uint32_t previous2 = 0, previous4 = 0, samples = 0, errors = 0, dropped = 0;
bool readyPrinted = false;
char command[64];
size_t commandLength = 0;
bool overflow = false;

// Drop output instead of letting a slow/disconnected USB host delay stop handling.
void report(const char* format, ...) {
  char line[256];
  va_list args;
  va_start(args, format);
  const int size = vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  if (size <= 0) return;
  const size_t length = min(static_cast<size_t>(size), sizeof(line) - 1);
  if (Serial && Serial.availableForWrite() >= static_cast<int>(length)) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
  } else { ++dropped; }
}

void help() {
  Serial.println("E1: SDA=D4/GPIO5 SCL=D5/GPIO6; address=0x26; boot clock=400kHz");
  Serial.println("Boot configures motors then releases outputs; motion requires arm + command.");
  Serial.println("Commands (newline): probe, read, watch, quiet, stop, release");
  Serial.println("  restart / split : select repeated-START / STOP-separated reads");
  Serial.println("  config-be / config-le : explicit uint16 configuration byte order");
  Serial.println("  arm : wheels lifted, configuration and encoder direction checked");
  Serial.println("  m2 / m4 / both : +100 driver units, 5 seconds, then zero speed");
  Serial.println("  100 / 400 : select I2C clock in kHz while stopped; disarms");
  Serial.println("  status, help; ! immediately stops without needing a newline");
  Serial.println("Config: type=1, encoder=500, gear=23 APPROXIMATE, diameter=65mm.");
  Serial.println("uint16 config: BE initial bench assumption; diameter float=LE; reads=repeated START.");
}

void stopMotor(const char* reason, bool observe) {
  active = false;
  armed = false;
  const uint32_t begin = micros();
  const bool ok = driver.setSpeeds(0, 0);
  observeStop = observe && ok;
  stopped = millis();
  stable = false;
  previousValid = false;
  report("STOP reason=%s ack=%u write_us=%lu err=%u; ACK is not proof of stopped motion\n",
         reason, ok, static_cast<unsigned long>(micros() - begin), driver.error());
  if (!ok) {
    configured = false;
    const bool released = driver.release();
    report("STOP FAILED; release_attempt_ack=%u. Remove motor power if moving.\n", released);
  }
  report("TIMING clock=%lu reads=%lu mean_us=%lu max_us=%lu writes=%lu mean_us=%lu max_us=%lu\n",
         static_cast<unsigned long>(clockHz), static_cast<unsigned long>(runReads),
         static_cast<unsigned long>(runReads ? readSum / runReads : 0), static_cast<unsigned long>(readMax),
         static_cast<unsigned long>(runWrites), static_cast<unsigned long>(runWrites ? writeSum / runWrites : 0),
         static_cast<unsigned long>(writeMax));
}

void serviceDeadline() {
  if (active && (!Serial || static_cast<uint32_t>(millis() - started) >= kPulseMs)) {
    stopMotor(Serial ? "pulse deadline" : "USB disconnected", true);
  }
}

void sample(bool forcePrint) {
  const bool duringRun = active;
  const uint32_t begin = micros();
  uint16_t recent2 = 0, recent4 = 0;
  uint32_t count2 = 0, count4 = 0;
  bool ok = driver.readWord(0x11, recent2);
  serviceDeadline();
  if (ok) ok = driver.readWord(0x13, recent4);
  serviceDeadline();
  if (ok) ok = driver.readCumulative(0x22, count2);
  serviceDeadline();
  if (ok) ok = driver.readCumulative(0x26, count4);
  const uint8_t error = driver.error();
  serviceDeadline();
  const uint32_t readDuration = micros() - begin;
  if (duringRun) { ++runReads; readSum += readDuration; readMax = max(readMax, readDuration); }
  ++samples;
  if (!ok) {
    ++errors;
    previousValid = false;
    stable = false;
    if (active) stopMotor("feedback error", true);
    if (forcePrint || millis() >= nextPrint) {
      report("READ FAIL err=%u sample=%lu (128=short read,129=incoherent count)\n", error,
             static_cast<unsigned long>(samples));
      nextPrint = millis() + 200;
    }
    return;
  }
  const int32_t delta2 = static_cast<int32_t>(count2 - previous2);
  const int32_t delta4 = static_cast<int32_t>(count4 - previous4);
  if (observeStop) {
    const bool stationary = previousValid && delta2 == 0 && delta4 == 0 &&
                            recent2 == 0 && recent4 == 0;
    if (!stationary) stable = false;
    else if (!stable) { stable = true; stableSince = millis(); }
    if (stable && static_cast<uint32_t>(millis() - stableSince) >= 200) {
      report("STOP OBSERVED: counts unchanged and recent=0 for 200ms; elapsed_ms=%lu\n",
             static_cast<unsigned long>(millis() - stopped));
      observeStop = false;
    }
  }
  if (forcePrint || static_cast<int32_t>(millis() - nextPrint) >= 0) {
    report("t=%lu M2 recent=%d total=%ld M4 recent=%d total=%ld read_us=%lu\n",
           static_cast<unsigned long>(millis()), static_cast<int16_t>(recent2),
           static_cast<long>(static_cast<int32_t>(count2)), static_cast<int16_t>(recent4),
           static_cast<long>(static_cast<int32_t>(count4)), static_cast<unsigned long>(readDuration));
    nextPrint = millis() + 200;
  }
  previous2 = count2; previous4 = count4; previousValid = true;
}

void execute(const char* line) {
  if (!strcmp(line, "stop")) { stopMotor("user", true); return; }
  if (active) { report("BUSY: only stop or ! accepted during pulse\n"); return; }
  if (!strcmp(line, "help")) { help(); }
  else if (!strcmp(line, "probe")) {
    const uint32_t begin = micros();
    const uint8_t error = driver.probe();
    report("PROBE address=0x26 err=%u (0=ACK) time_us=%lu\n", error,
           static_cast<unsigned long>(micros() - begin));
  } else if (!strcmp(line, "read")) { sample(true); }
  else if (!strcmp(line, "100") || !strcmp(line, "400")) {
    if (observeStop) { report("REFUSED: wait for stop observation\n"); return; }
    armed = false;
    if (!driver.setSpeeds(0, 0)) { configured = false; report("Clock change refused: stop write failed\n"); return; }
    const uint32_t requested = !strcmp(line, "100") ? 100000 : 400000;
    if (!Wire.setClock(requested)) { configured = false; report("Clock change failed\n"); return; }
    clockHz = requested;
    report("CLOCK requested=%lu actual=%lu\n", static_cast<unsigned long>(clockHz),
           static_cast<unsigned long>(Wire.getClock()));
    execute("probe");
    sample(true);
  }
  else if (!strcmp(line, "watch")) { watch = true; report("WATCH: reads at 50Hz, display at 5Hz; rotate one wheel at a time\n"); }
  else if (!strcmp(line, "quiet")) { watch = false; }
  else if (!strcmp(line, "status")) {
    report("clock=%lu configured=%u armed=%u active=%u samples=%lu errors=%lu dropped_output=%lu\n",
           static_cast<unsigned long>(clockHz), configured, armed, active, static_cast<unsigned long>(samples),
           static_cast<unsigned long>(errors), static_cast<unsigned long>(dropped));
  } else if (!strcmp(line, "release")) {
    armed = false; observeStop = false;
    const bool ok = driver.release();
    if (!ok) configured = false;
    report("RELEASE zero PWM ack=%u err=%u; verify wheel is free manually\n", ok, driver.error());
  } else if (!strcmp(line, "restart") || !strcmp(line, "split")) {
    driver.repeatedStart(!strcmp(line, "restart"));
    armed = false; previousValid = false;
    report("Read transaction mode=%s\n", line);
  } else if (!strcmp(line, "config-be") || !strcmp(line, "config-le")) {
    configured = false; armed = false; observeStop = false;
    const bool le = !strcmp(line, "config-le");
    bool ok = driver.release();
    if (ok) { ok = driver.setType(1); delay(100); }
    if (ok) { ok = driver.setParameter(0x03, 500, le); delay(100); }
    if (ok) { ok = driver.setParameter(0x04, 23, le); delay(100); }
    if (ok) { ok = driver.setDiameter(65.0f); delay(100); }
    const uint8_t error = driver.error();
    const bool released = driver.release();
    configured = ok && released;
    report("CONFIG ack_all=%u err=%u type=1 lines=500 gear=23 diameter=65 uint16=%s float=LE\n",
           configured, error, le ? "LE" : "BE");
    report("ACK confirms transport only; parameter application/scaling NOT verified.\n");
  } else if (!strcmp(line, "arm")) {
    if (!configured || observeStop) { report("REFUSED: configure first and finish stop observation\n"); return; }
    const uint32_t failures = errors;
    sample(true);
    armed = errors == failures;
    report("ARMED=%u for one 5-second run. Keep motor-power disconnect accessible.\n", armed);
  } else if (!strcmp(line, "m2") || !strcmp(line, "m4") || !strcmp(line, "both")) {
    if (!armed || observeStop) { report("REFUSED: use arm after checking wheels/encoder direction\n"); return; }
    armed = false;
    motor = !strcmp(line, "m2") ? 2 : (!strcmp(line, "m4") ? 4 : 6);
    runReads = runWrites = readMax = writeMax = 0;
    readSum = writeSum = 0;
    started = millis();  // Include command transaction time in pulse deadline.
    const uint32_t writeStart = micros();
    const bool ok = driver.setSpeeds(motor != 4 ? kSpeed : 0, motor != 2 ? kSpeed : 0);
    writeSum = writeMax = micros() - writeStart; runWrites = 1;
    if (!ok) { stopMotor("command error", true); return; }
    active = true;
    nextWrite = started + kSampleMs;
    report("START selection=%s command=+%d driver_units duration=%lums clock=%lu; M1/M3=0\n",
           line, kSpeed, static_cast<unsigned long>(kPulseMs), static_cast<unsigned long>(clockHz));
  } else if (*line) { report("Unknown command: %s; use help\n", line); }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin(D4, D5, 400000);
  Wire.setTimeOut(5);
  delay(500);  // Allow the separately powered driver to become ready.
  execute("config-be");
}

void loop() {
  serviceDeadline();
  if (active && static_cast<int32_t>(millis() - nextWrite) >= 0) {
    nextWrite = millis() + kSampleMs;
    const uint32_t begin = micros();
    const bool ok = driver.setSpeeds(motor != 4 ? kSpeed : 0, motor != 2 ? kSpeed : 0);
    const uint32_t duration = micros() - begin;
    ++runWrites; writeSum += duration; writeMax = max(writeMax, duration);
    if (!ok) stopMotor("command refresh error", true);
    serviceDeadline();
  }
  if (Serial && !readyPrinted) {
    help();
    execute("probe");
    sample(true);
    readyPrinted = true;
  }
  // Bound input work; overlong input is discarded through the next newline.
  for (uint8_t i = 0; i < 32 && Serial.available(); ++i) {
    const char c = Serial.read();
    if (c == '!') { stopMotor("emergency serial", true); commandLength = 0; overflow = true; }
    else if (c == '\r' || c == '\n') {
      if (!overflow && commandLength) { command[commandLength] = '\0'; execute(command); }
      commandLength = 0; overflow = false;
    } else if (!overflow) {
      if (commandLength + 1 < sizeof(command)) command[commandLength++] = c;
      else { overflow = true; report("Input too long; discarded\n"); }
    }
    serviceDeadline();
  }
  if ((watch || active || observeStop) && static_cast<int32_t>(millis() - nextSample) >= 0) {
    nextSample = millis() + kSampleMs;
    sample(false);
  }
  if (observeStop && static_cast<uint32_t>(millis() - stopped) >= 2000) {
    observeStop = false;
    report("STOP NOT VERIFIED within 2s: inspect feedback; remove motor power if still moving\n");
  }
  delay(1);
}
