# E4 firmware - line sensor library

Uses [LineSensor](../../lib/LineSensor/README.md), constructed with D6/D7.
E3 remains the standalone reference. E4 uses Arduino 3.3.12 / ESP-IDF 5.5.5
through the same pinned PlatformIO platform as E3.

Current settings: 1000 Hz, five total attempts, 10 seconds per run, automatic
repeat with a 1-second pause. The library fixes the bus clock at 1 MHz and the
requested timeout at 1 ms. Send `r` to request a rerun. No Python runner is added.

The benchmark counts physical requests, including retry failures. It reports
request latency, success count, retry count, latest mask/age and minimum request
start gap. Idle ticks do not count as requests. Timing is managed only by the
library; the benchmark waits 5 us between Idle ticks.

Build/upload: `platformio run -e e4_line_sensor_library -t upload --upload-port COM4`.
Results: [test/E4_line_sensor_library](../../test/E4_line_sensor_library/README.md).
