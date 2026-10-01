# E3 — Line-sensor experiment

Firmware: [src/e3_line_sensor](../../src/e3_line_sensor/README.md).

Tested I2C at 100k, 400k, and 1M Hz. All works.
In some cases, the I2C bus may experience errors or short reads, as shown in the test results below.
We have silenced the I2C diagnostics in the build flags.


The class-refactor benchmark used the reusable [LineSensor class](../../lib/LineSensor/README.md). The results below are preserved measurements from before this refactor. Its benchmark settings were unchanged (1 MHz, 1 ms timeout, 10 seconds, maximum rate); retry support adds five total attempts per due poll. Physical-request failures remain included in the success count denominator even when a retry recovers a poll. That summary also reports recovered/failed polls and last-reading age.

Library verification on 2026-10-01: uploaded E3 to COM4 and recorded [the raw serial log](results/20261001T075834Z_library/serial.log). The run reported 280 successful requests out of 290, two failed polls, and eight retry requests. Successful requests averaged 175.18 us; the longest failed request took 999999 us despite the configured 1 ms Wire timeout. The last valid mask stayed at `0x3F`, with its age increasing to about 10 seconds during failed polls. The capture also retains an initial USB-disconnected incomplete run; the listener closed before the final elapsed-time summary line of the subsequent run was captured.

This confirms stale-reading retention on hardware, but also shows that synchronous retries can block for seconds. Integration must account for the observed driver latency rather than treating the configured timeout as a strict bound.

Current diagnosis: E3 has been switched back to a standalone direct-I2C benchmark for comparison. See [diagnosis captures](results/20261001T080811Z_diagnosis/README.md). Both implementations failed while the reader was stuck; the operator subsequently power-cycled the reader and restored standalone reads.

Results

E3 START clock=400000 Hz timeout=1 ms duration=1000 ms request_hz=0 (0=maximum) limit=0 (0=unlimited)
E3 END: duration reached
Success: 4480 / 4480 (100.00%)
Request time, all attempts: mean=218.51 us min=215 us max=332 us
Successful requests: mean=218.51 us; last raw=0x3F
Transmit errors=0 short reads=0 last transmit error=0
Elapsed=1000.206 ms achieved=4479.08 requests/s skipped slots=0

E3 START clock=1000000 Hz timeout=2 ms duration=1000 ms request_hz=0 (0=maximum) limit=0 (0=unlimited)
E3 END: duration reached
Success: 5571 / 5571 (100.00%)
Request time, all attempts: mean=173.02 us min=171 us max=272 us
Successful requests: mean=173.02 us; last raw=0x3F
Transmit errors=0 short reads=0 last transmit error=0
Elapsed=1000.138 ms achieved=5570.23 requests/s skipped slots=0

E3 START clock=1000000 Hz timeout=1 ms duration=1000 ms request_hz=100 (0=maximum) limit=0 (0=unlimited)
E3 END: duration reached
Success: 100 / 100 (100.00%)
Request time, all attempts: mean=174.00 us min=171 us max=277 us
Successful requests: mean=174.00 us; last raw=0x3F
Transmit errors=0 short reads=0 last transmit error=0
Elapsed=1000.002 ms achieved=100.00 requests/s skipped slots=0


E3 START clock=1000000 Hz timeout=1 ms duration=10000 ms request_hz=0 (0=maximum) limit=0 (0=unlimited)
[ 39507][E][Wire.cpp:499] requestFrom(): i2cWriteReadNonStop returned Error 263
[ 40562][E][Wire.cpp:499] requestFrom(): i2cWriteReadNonStop returned Error 263
[ 41676][E][Wire.cpp:499] requestFrom(): i2cWriteReadNonStop returned Error 263
[ 42861][E][Wire.cpp:499] requestFrom(): i2cWriteReadNonStop returned Error 263
[ 43938][E][Wire.cpp:499] requestFrom(): i2cWriteReadNonStop returned Error 263
[ 45035][E][Wire.cpp:499] requestFrom(): i2cWriteReadNonStop returned Error 263
[ 46057][E][Wire.cpp:499] requestFrom(): i2cWriteReadNonStop returned Error 263
E3 END: duration reached
Success: 16423 / 16430 (99.96%)
Request time, all attempts: mean=602.11 us min=171 us max=1007714 us
Successful requests: mean=173.06 us; last raw=0x3F
Transmit errors=0 short reads=7 last transmit error=0
Elapsed=10000.057 ms achieved=1642.99 requests/s skipped slots=0

After Silencing I2C Diagnostics:

E3 START clock=1000000 Hz timeout=1 ms duration=10000 ms request_hz=0 (0=maximum) limit=0 (0=unlimited)
E3 END: duration reached
Success: 27844 / 27849 (99.98%)
Request time, all attempts: mean=352.40 us min=170 us max=1000313 us
Successful requests: mean=172.90 us; last raw=0x3F
Transmit errors=0 short reads=5 last transmit error=0
Elapsed=10000.156 ms achieved=2784.86 requests/s skipped slots=0


## One-second failure latency diagnosis (2026-10-01)

The operator power-cycled the line-reader board. The standalone firmware then reported 33391/33395 successful requests (99.99%), 173.07 us mean successful latency, and a maximum request duration of 1000575 us. This is an operator-supplied summary, not a newly captured raw log. E3 remains standalone; the class has not been reintroduced.

The installed Arduino framework uses ESP-IDF 4.4.7. Local `Wire.cpp` passes its configured timeout to `i2cWriteReadNonStop`, which forwards it to the IDF combined write/read operation. With the installed 1000 Hz FreeRTOS tick rate, 1 ms converts to one tick; this is not a zero-tick conversion problem.

In the matching [ESP-IDF 4.4.7 driver source](https://github.com/espressif/esp-idf/blob/v4.4.7/components/driver/i2c.c#L1376-L1421), `I2C_CMD_ALIVE_INTERVAL_TICK` is 1000 ms. The transaction-event wait is increased to at least that interval, including when the requested timeout has already elapsed. A normal completion event wakes the wait immediately; absence of an event can therefore block for approximately one second before returning `ESP_ERR_TIMEOUT` (263). This source path matches the observed delay; it was not instrumented inside the compiled driver during this diagnosis.

This explains why `Wire.setTimeOut(1)` does not guarantee a 1 ms return on every failure, and why increasing it to 5 ms did not shorten the observed failure. It does not establish why the sensor hangs or why a completion event is absent. Avoid treating all short reads as equivalent faults. A software retry loop or an age check after the call cannot interrupt this blocking driver wait. Changing the behavior requires evaluating a different driver/framework or a controlled driver patch; no framework or driver patch was applied here.


## Arduino 3.3.12 verification

E3 now pins pioarduino 55.03.312-1 / Arduino 3.3.12 / ESP-IDF 5.5.5.
Three hardware runs recorded maximum failed-read times of 125, 1475 and
1017 us. The third run failed continuously, confirming millisecond rather
than one-second returns in that condition. Success rates deteriorated from
99.94% to 15.96% to 0%; sensor/bus reliability remains unresolved.
See [the report and raw log](results/20261001T084145Z_arduino3312/README.md).

## Log when the sensor board is frozen
---- Sent utf8 encoded message: "r\r" ----
E3 framework Arduino=3.3.12 ESP-IDF=v5.5.5
E3 START standalone clock=1000000 Hz timeout=1 ms duration=10000 ms request_hz=0 (0=maximum) limit=0 (0=unlimited)
E3 END: duration reached
Success: 0 / 9962 (0.00%)
Request time, all attempts: mean=998.07 us min=983 us max=1017 us
Transmit errors=0 short reads=9962 last transmit error=0
Failed requests: count=9962 max=1017 us
Elapsed=10000.010 ms achieved=996.20 requests/s skipped slots=0

## Log after resetting the sensor board
---- Sent utf8 encoded message: "r\r" ----
E3 framework Arduino=3.3.12 ESP-IDF=v5.5.5
E3 START standalone clock=1000000 Hz timeout=1 ms duration=10000 ms request_hz=0 (0=maximum) limit=0 (0=unlimited)
E3 END: duration reached
Success: 34537 / 34552 (99.96%)
Request time, all attempts: mean=284.03 us min=83 us max=736 us
Successful requests: mean=284.12 us; last raw=0x3F
Transmit errors=0 short reads=15 last transmit error=0
Failed requests: count=15 max=91 us
Elapsed=10000.001 ms achieved=3455.20 requests/s skipped slots=0

## Frozen
Typically after 2 x 10 seconds run, the board will be frozen.