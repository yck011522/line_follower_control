# Project working conventions

- Do not add host-side unit tests, mock hardware headers, or simulated-device
  test harnesses unless the user explicitly requests them. Use firmware builds
  and direct hardware experiments for verification.

- All new C++ files must have human-readable comments before every function and
  at critical points within functions, explaining purpose, timing, and non-obvious behavior.

- Read `docs/motor-settings.md` before changing motor configuration or experiments.
  The adopted baseline is type 1, dead zone 1650, pulse line 500, pulse phase
  (gear ratio) 23, wheel diameter 65 mm, PID P=3/I=0.375/D=0.5.
- Reapply type, dead zone, pulse line, ratio and diameter over I2C at each ESP32
  boot; require successful acknowledgments before motion. PID is the explicit
  exception: rely on the driver's saved P=3/I=0.375/D=0.5. E2 is I2C-only and must
  not send motor-driver serial commands. I2C cannot read/write PID in the supplied
  protocol; report it as stored/assumed, never verified. Do not invent registers.
- Every Python tool opening a motor-driver or ESP32 test serial port must set
  `rts=False` BEFORE opening it. Construct `serial.Serial(port=None, ...)`, set
  `dtr=True` and `rts=False`, assign the port, then call `open()`. Flow-control
  flags alone do not set line levels. Do not pulse RTS during tests or cleanup.
  Documented firmware-upload/reset tools are a separate workflow.
- Hardware experiments must start motion only on an explicit command, bound
  runtime, attempt zero speed and output release on errors/cancellation, and
  retain raw logs and incomplete-run status. Preserve historical measurement files.
- Driver-reported speed and encoder-derived physical speed are different
  measurements. State the scale and calibration assumptions in plots/reports.
