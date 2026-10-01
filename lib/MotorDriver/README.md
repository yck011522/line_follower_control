# MotorDriver

Minimal I2C interface for M2 (left) and M4 (right). The application owns the bus and polling schedule; M1/M3 commands are zero. Vehicle-forward signs remain to be mapped.

Construct `MotorDriver driver(Wire)`, start `Wire.begin(D4, D5, 400000)`, then require successful `driver.initialize()` before motion. Initialization reapplies the five baseline settings at every boot, releases outputs before and after configuration, and retains the existing approximately 500 ms of save delays. It may be retried while stopped.

[MotorSettings.h](include/MotorSettings.h) names the baseline: type 1, dead zone 1650, encoder pulses 500, ratio 23, diameter 65 mm. PID remains stored/assumed at 3/0.375/0.5; I2C cannot verify or configure it.

| Function | Purpose |
| --- | --- |
| `initialize()` | Apply boot configuration and report transaction success. |
| `setWheelSpeedsMmPerSecond(left, right)` | Signed closed-loop targets; caller supplies -1000 to +1000 mm/s. |
| `readWheelSpeedsMmPerSecond(left, right)` | Encoder-derived estimated wheel speeds in mm/s. |
| `releaseMotorOutputs()` | Send zero PWM, leaving speed PID mode. |
| `readEncoderPosition(wheel, counts)` | Signed accumulated encoder position for measurements or odometry. |
| `readEncoderCountLast10Ms(wheel, counts)` | Raw signed 10-ms counts, retained for E2. |
| `lastCommunicationError()` | Wire code, or 0x80 for short read / 0x81 for inconsistent position words. |

Zero speed keeps PID active; output release disables holding. An acknowledgment does not prove a wheel has stopped.

Speed feedback uses `counts_in_last_10_ms * pi * 65 / 44998 / 0.010`. This uses nominal wheel diameter and E2's hand-measured 44,998 encoder counts per wheel revolution, independently of the configured integer ratio. It is an encoder-derived estimate, distinct from the driver's serial speed report, and is not independently verified physical distance. Resolution is about 0.454 mm/s per count; no filtering is applied.

Left/right reads are sequential. Both speed outputs remain unchanged if either read fails. E2 still logs raw counts and accumulated positions and uses actual timestamps for longer-window analysis; its CSV schema is unchanged. Register access and byte encoding are private. E1's old diagnostics are archived.
