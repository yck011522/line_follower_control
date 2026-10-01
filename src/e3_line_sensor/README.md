# E3 firmware ? line sensor

E3 currently uses a standalone `TwoWire(1)` benchmark to compare direct I2C reads with the LineSensor class. The earlier standalone file was not committed before the class refactor, so this restores the direct-read approach rather than an exact historical file. The library remains in `lib/LineSensor` but is not used by this firmware.

Settings: SDA D6/GPIO43, SCL D7/GPIO44, address `0x12`, register `0x30`, repeated START, 1 MHz clock, 1 ms requested timeout, 10 seconds, maximum polling rate, unlimited requests, no automatic retries. Edit the constants in `main.cpp`; send `r` over serial to rerun.

The summary reports physical-request success, latency, failures and throughput. Arduino core diagnostics remain silenced. Failed driver calls may exceed the configured timeout.

E3 now pins pioarduino platform `55.03.312-1`, packaging Espressif Arduino
`3.3.12` / ESP-IDF `5.5.5`. The platform integration is community maintained;
the Arduino framework is Espressif's official release. E1/E2 retain their
existing platform. The startup heading prints the compiled framework versions,
and the summary separates maximum failed-request latency from normal latency.

Build/upload with `platformio run -e e3_line_sensor -t upload --upload-port COM4`.

Results: [test/E3_line_sensor](../../test/E3_line_sensor/README.md).
