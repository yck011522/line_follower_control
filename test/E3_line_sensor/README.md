# E3 — Line-sensor experiment

Firmware: [src/e3_line_sensor](../../src/e3_line_sensor/README.md).

Tested I2C at 100k, 400k, and 1M Hz. All works.
In some cases, the I2C bus may experience errors or short reads, as shown in the test results below.
We have silenced the I2C diagnostics in the build flags.


The firmware now uses the reusable [LineSensor class](../../lib/LineSensor/README.md). The results below are preserved measurements from before this refactor. Benchmark settings are unchanged (1 MHz, 1 ms timeout, 10 seconds, maximum rate); retry support adds five total attempts per due poll. Physical-request failures remain included in the success count denominator even when a retry recovers a poll. The new summary also reports recovered/failed polls and last-reading age.

Library verification on 2026-10-01: uploaded E3 to COM4 and recorded [the raw serial log](results/20261001T075834Z_library/serial.log). The run reported 280 successful requests out of 290, two failed polls, and eight retry requests. Successful requests averaged 175.18 us; the longest failed request took 999999 us despite the configured 1 ms Wire timeout. The last valid mask stayed at `0x3F`, with its age increasing to about 10 seconds during failed polls. The capture also retains an initial USB-disconnected incomplete run; the listener closed before the final elapsed-time summary line of the subsequent run was captured.

This confirms stale-reading retention on hardware, but also shows that synchronous retries can block for seconds. Integration must account for the observed driver latency rather than treating the configured timeout as a strict bound.

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
