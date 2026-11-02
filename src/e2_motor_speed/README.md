# E2 - Parametric low-speed I2C sweep

> Retired: the firmware and its library have been removed. Historical results
> and analysis tools remain; the build and capture instructions are historical.

The former firmware used an I2C motor driver. No motor UART
is used. Boot rewrites five I2C settings; PID 3/0.375/0.5 is assumed in driver flash.

`start [min max inc]` sweeps M2/M4 together (default -100 through +100 in steps of 10; limits are
+-1000 and 200 steps), with zero baseline and rest at every target. Measurement uses cumulative
encoder counts and actual timestamps. No automatic motion at boot, no runtime tuning options.

See the historical [E2 operator guide](../../test/E2_motor_speed/README.md).

The firmware calls `initialize()` after starting I2C and permits motion only
after successful configuration. Commands use `setWheelSpeedsMmPerSecond()`;
shutdown uses zero speed followed by `releaseMotorOutputs()`. Explicit raw
encoder reads preserve the existing CSV schema and analysis.
