# E3 firmware — line sensor

The configurable benchmark uses the [LineSensor library](../../lib/LineSensor/README.md) to read the eight-channel status byte at address `0x12`, register `0x30`, on the second I²C bus (D6/D7). Edit the constants at the top of `main.cpp` to set bus clock, timeout, polling frequency, duration, request cap, and maximum attempts.

Current settings: 1 MHz, 1 ms timeout, 10 seconds, maximum polling rate, unlimited requests, five total attempts per due poll. `r` reruns the same settings. Output includes physical-request success/latency, recovered and exhausted polls, retries, and the last valid mask's age. Summary output stays outside the timed request path.

Host automation and results: [test/E3_line_sensor](../../test/E3_line_sensor/README.md). Build/environment and measurement plan: [testing](../../docs/testing.md).
