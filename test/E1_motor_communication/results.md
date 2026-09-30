# E1 results - motor driver communication

Raw logs in `results/` are Git-ignored; this file is the committed summary. Detail: [initial results](INITIAL_RESULTS.md) and [five-second runs](LONG_RUN_RESULTS.md).

## Confirmed

- **I²C link works** at address `0x26` on D4 (SDA) / D5 (SCL), at both 100 kHz and 400 kHz.
- **Reads must use a repeated START.** A STOP between the register write and the read returned all zeros even with the wheels turning.
- **Encoders respond independently** when each wheel is turned by hand; cumulative counts are signed and cross zero.
- **Both motors move** under the speed command at +100 for five seconds, at both clocks, with no reported transport errors.
- **Startup configuration** (type 1, 500 encoder lines, ratio 23, 65 mm) is acknowledged on every write.

## Timing (five-second runs, both motors, 50 Hz)

| Measurement | 100 kHz | 400 kHz |
| --- | ---: | ---: |
| Mean encoder read (8 word reads) | 4396 µs | 1411 µs |
| Mean speed write | 970 µs | 273 µs |
| Maximum speed write | 986 µs | 279 µs |
| Reported feedback errors | 0 | 0 |

400 kHz is about 3.5x faster than 100 kHz, not 4x, because of fixed overhead.

## Not passed or not tested

- **Strict stop criterion failed** at both clocks: small encoder oscillations continued after zero speed (zero-speed PID hunting). Zero-PWM release does settle the outputs.
- Only a constant target (+20 for 250 ms pulses, +100 for 5 s) was written; **rapid target changes were not tested**, and the planned 100 Hz loop was not run.
- Physical speed calibration, motor direction mapping and configuration readback were not done.
