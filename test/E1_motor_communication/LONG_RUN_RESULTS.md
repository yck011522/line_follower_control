# Five-second motor runs — 2026-09-30

Both M2 and M4 were commanded together at +100 driver units for five seconds, first at 100 kHz, then at 400 kHz. The firmware refreshed speed commands and sampled feedback at 50 Hz, displaying feedback at 5 Hz. M1/M3 remained zero. User had confirmed the wheels were lifted. Physical mm/s calibration remains unverified.

The user confirmed that both motors rotated visibly during the five-second runs.

Startup automatically configured type 1, encoder lines 500, integer ratio 23, and wheel diameter 65 mm, then released the outputs. After upload, `status` reported `configured=1` without a manual configuration command. This confirms successful configuration writes, not parameter readback.

| Measurement | 100 kHz | 400 kHz |
| --- | ---: | ---: |
| Complete encoder reads during run | 250 | 250 |
| Mean encoder read time | 4396 µs | 1411 µs |
| Maximum encoder read time | 4400 µs | 1416 µs |
| Speed writes during run | 250 | 250 |
| Mean speed write time | 970 µs | 273 µs |
| Maximum speed write time | 986 µs | 279 µs |
| Zero-speed write time | 972 µs | 275 µs |
| Reported feedback errors | 0 | 0 |
| Sustained encoder movement | Both motors | Both motors |
| Strict stationary criterion after zero speed | Failed | Failed |

The read workload includes both recent-count words and high–low–high snapshots for each cumulative count (eight word reads total). Write timing covers one four-channel speed transaction. Means/maxima come from device timing, not USB arrival times. These are single short runs, not a long-duration or integrated-control qualification.

## Stop behavior

After zero speed, motion dropped sharply but small encoder oscillations persisted. At 100 kHz both motors showed small oscillations; at 400 kHz M2 continued oscillating while M4 settled. Thus neither run met the existing criterion of unchanged cumulative counts and zero recent counts for 200 ms within two seconds. This is consistent with hunting in the driver's zero-speed control loop, but the cause has not been proven or tuned.

Zero-PWM release was sent after each observation. Stationary readings were confirmed before changing bus clock; after the final release both recent counts were zero. Do not claim that the strict active-stop test passed, or hide this by relaxing its tolerance. The default motor PID/dead zone may need later adjustment for the GT50.

## Evidence

Local raw captures: `results/20260930T042527Z_long_runs/100k.log` and `400k.log` (generated files ignored by Git). Build/upload succeeded on COM4 using `e1_motor_communication`. The driver acknowledged at both requested clock rates; `Wire.getClock()` reported 100000 and 400000 respectively. No additional firmware upload was needed to switch clocks. The final state was outputs released at 400 kHz; an ESP32 reset returns the test clock to 100 kHz and reapplies configuration.

The earlier 250 ms tests used command 20 without periodic refresh. These runs changed duration, command magnitude, and refresh behavior together; they do not isolate which change made sustained motion easier to observe.

Uploaded firmware SHA256: `0A04202BDFD5FC2936FFDB8B4F15D0F9BE37B54275AD8B7A2DC89B67732D4A3D`.
