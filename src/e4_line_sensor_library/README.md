# E4 firmware - line sensor library reliability

Uses [LineSensor](../../lib/LineSensor/README.md) with E6's bus configuration,
CR/LF command parser and fixed-rate schedule. Defaults are 200 Hz, 400 kHz,
a requested 1 ms I2C timeout and a 10-second duration. No test starts on boot.

Send each command followed by CR, LF or CRLF. Empty lines are ignored.

| Command | Responsibility |
| --- | --- |
| `D 28800` | Set test duration in seconds (eight hours here). |
| `R 200` | Set sensor request frequency in Hz. |
| `C 400000` | Set sensor I2C clock in Hz. |
| `T 1` | Set sensor I2C timeout in milliseconds. |
| `S` | Reset statistics and start the test. |
| `X` | Stop the test and report results. |

Settings can only change while stopped. Supply positive reading rates and valid
bus settings; timeout must fit a 16-bit millisecond value. Input validation
is unchanged from E6.

The firmware times each complete `lineSensor.read()` call, counts successful
responses and reports success percentage, mean/minimum/maximum request time,
successful-request mean, failure count and maximum failure time. It also reports
elapsed time, achieved request rate, skipped slots and the last successful byte.
All measurements and scheduling live in E4, outside the library. Each call is one
attempt with no retries; missed polling slots do not trigger catch-up bursts.

E4 preserves E6's dummy motor workload: controller 0 on D4/D5 at 400 kHz,
1 ms timeout, baseline configuration on boot and zero-PWM writes at 100 Hz during
the test. It commands no motion. Sensor controller 1 uses D6/D7.

Build: `platformio run -e e4_line_sensor_library`.
Upload and capture results using the existing project serial-port conventions.
Historical measurements are unchanged. This firmware must be tested on hardware
to compare its results with the eight-hour E6 run.
