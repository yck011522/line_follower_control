# E5 line following

Build with `platformio run -e e5_line_follow`. No Python runner or host tests.

Confirmed orientation: X1 is leftmost and both motors use negative electrical commands for vehicle-forward motion. Firmware starts automatically at 100 mm/s after successful boot configuration, without waiting for USB. It runs indefinitely, as requested, until a stop command or fault. Upload is left to the operator.

E5 polls the line sensor independently at 200 Hz and runs steering at 100 Hz.
Set `kLineSensorRequestFrequencyHz` in main.cpp to adjust polling; the sensor
constructor takes SDA/SCL pins and request frequency. Retries obey that frequency.
The motor library is unchanged. Active-low sensor detections are averaged over equally spaced positions -1 (left) to +1 (right). Exponential smoothing has a 0.2-second half-life. PD uses filtered center and its derivative per second; defaults P=1, D=0. Steering correction is limited to -1..+1. Only the inner wheel slows; neither wheel reverses. No integral control or speed-feedback loop is added; the motor board handles its saved speed PID.

Serial commands at 115200 baud:

- `gains 1 0`: set line-following P and D, independently of stored motor PID.
- `speed 100`: set forward speed.
- `speed 0`: command zero speed and release motor outputs.
- `status_hz 5`: set periodic printing frequency (e.g. 10 or 20).
- `status`: print immediately.

When no sensor detects a line, E5 continues filtering and steering from the last detected center for up to one second. After that it commands zero speed and releases outputs, retaining the selected speed. It keeps polling and automatically resumes when the line returns. Reacquisition resets the filter and derivative history, including after a short gap; there is no integral accumulator in this PD controller. No driver-internal PID reset is attempted.

Status shows `holding_last_line` during the one-second gap and `waiting_for_line` after release. If no line has ever been detected, it waits released immediately. Unavailable/stale I2C readings (100 ms) release the motors while preserving the selected speed and continuing to poll. Fresh line detection automatically resumes motion with filter/PD history reset; fresh reads without a line keep the motors released. Motor errors and `speed 0` still latch a stop requiring a new speed command. Motor initialization failure prevents motion.

Status reports raw mask, raw/filtered center, correction, wheel commands, gains, actual control Hz, average/peak control-work microseconds, missed slots, and previous status-print duration. Wheel values are commands, not measured speeds. `work_pct` is average control-work time divided by the 10 ms target cycle; it includes blocking sensor/motor I2C work and excludes serial parsing/printing and other ESP32 tasks. It is not total CPU utilization. Missed slots and actual Hz include logging effects; peak work includes accumulated sensor polling between control cycles and gives additional timing-margin evidence. Capture serial output externally if retaining hardware-run logs.
