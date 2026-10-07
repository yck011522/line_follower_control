

E6 START duration=10 s reading_rate=100 Hz clock=400000 Hz timeout=1 ms
E6 END: duration reached
Success: 1000 / 1000 (100.00%)
Request time, all attempts: mean=248.29 us min=246 us max=363 us
Successful requests: mean=248.29 us; last raw=0x3F
Transmit errors=0 short reads=0 last transmit error=0
Failed requests: count=0 max=0 us
Elapsed=10000.001 ms achieved=100.00 requests/s skipped slots=0

E6 START duration=10 s reading_rate=500 Hz clock=400000 Hz timeout=1 ms
E6 END: duration reached
Success: 5000 / 5000 (100.00%)
Request time, all attempts: mean=247.98 us min=245 us max=338 us
Successful requests: mean=247.98 us; last raw=0x3F
Transmit errors=0 short reads=0 last transmit error=0
Failed requests: count=0 max=0 us
Elapsed=10000.001 ms achieved=500.00 requests/s skipped slots=0

E6 START duration=10 s reading_rate=1000 Hz clock=400000 Hz timeout=1 ms
E6 END: duration reached
Success: 10000 / 10000 (100.00%)
Request time, all attempts: mean=248.07 us min=245 us max=335 us
Successful requests: mean=248.07 us; last raw=0xFF
Transmit errors=0 short reads=0 last transmit error=0
Failed requests: count=0 max=0 us
Elapsed=10000.000 ms achieved=1000.00 requests/s skipped slots=0

## After adding Dummy Motor driver Read, e6 firmware

E6 START duration=600 s reading_rate=500 Hz clock=400000 Hz timeout=1 ms
E6 END: duration reached
Success: 300000 / 300000 (100.00%)
Request time, all attempts: mean=247.88 us min=246 us max=328 us
Successful requests: mean=247.88 us; last raw=0xBF
Transmit errors=0 short reads=0 last transmit error=0
Failed requests: count=0 max=0 us
Elapsed=600000.000 ms achieved=500.00 requests/s skipped slots=0

E6 START duration=28800 s reading_rate=200 Hz clock=400000 Hz timeout=1 ms
E6 END: duration reached
Success: 5760000 / 5760000 (100.00%)
Request time, all attempts: mean=247.94 us min=245 us max=317 us
Successful requests: mean=247.94 us; last raw=0xBF
Transmit errors=0 short reads=0 last transmit error=0
Failed requests: count=0 max=0 us
Elapsed=28800000.000 ms achieved=200.00 requests/s skipped slots=0

## Using new LineSensorlibrary, tested with e4 firmware
Test duration set to 3600 seconds
---- Sent utf8 encoded message: "S\r\n" ----
E4 START duration=3600 s reading_rate=200 Hz clock=400000 Hz timeout=1 ms
E4 END: duration reached
Success: 720000 / 720000 (100.00%)
Request time, all attempts: mean=248.43 us min=246 us max=330 us
Successful requests: mean=248.43 us; last raw=0xBF
Failed requests: count=0 max=0 us
Elapsed=3600000.001 ms achieved=200.00 requests/s skipped slots=0

## Same condition as E4 but deliberately resetting the sensor board 2 times.
E4 START duration=30 s reading_rate=200 Hz clock=400000 Hz timeout=1 ms
E4 END: duration reached
Success: 5931 / 6000 (98.85%)
Request time, all attempts: mean=247.02 us min=108 us max=995 us
Successful requests: mean=248.62 us; last raw=0xBF
Failed requests: count=69 max=152 us
Elapsed=30000.001 ms achieved=200.00 requests/s skipped slots=0

## Same condition as E4 but unplugging the sensor i2c cable in between.
---- Sent utf8 encoded message: "S\r\n" ----
E4 START duration=30 s reading_rate=200 Hz clock=400000 Hz timeout=1 ms
E4 END: duration reached
Success: 4827 / 6000 (80.45%)
Request time, all attempts: mean=221.92 us min=108 us max=347 us
Successful requests: mean=248.77 us; last raw=0xDF
Failed requests: count=1173 max=170 us
Elapsed=30000.001 ms achieved=200.00 requests/s skipped slots=0