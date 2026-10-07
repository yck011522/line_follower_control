# LineSensor

A small header-only library using the direct read procedure tested in E6.
The caller owns the bus, configures its pins, clock and timeout, and schedules reads.

```cpp
TwoWire sensorBus(1);
LineSensor lineSensor(sensorBus);

// In setup():
sensorBus.begin(D6, D7, 400000);
sensorBus.setTimeOut(1);

// When a reading is due:
uint8_t rawReading = 0;
bool success = lineSensor.read(rawReading);
```

`read()` selects register `0x30` at address `0x12`, uses repeated START,
and requests one byte with STOP. It returns true only when exactly one byte
is received and available. Every failure returns false and leaves the output
unchanged; incomplete receive data is discarded as in E6.

Each call performs one blocking request. The library has no retries, polling
schedule, statistics or serial output. The original active-low byte is returned
unchanged: bit 7 is X1, bit 0 is X8, and zero denotes black. Call from one task.

The E6 eight-hour run at 200 Hz, 400 kHz and a requested 1 ms timeout completed
5,760,000 requests successfully. E4 separately measures the new library calls;
that E6 result is not a hardware validation of this refactor.
