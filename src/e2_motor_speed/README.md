# E2 - Fixed low-speed I2C sweep

[main.cpp](main.cpp) uses the shared MotorDriver register interface and
[MotorSettings.h](../../lib/MotorDriver/include/MotorSettings.h). No motor UART
is used. Boot rewrites five I2C settings; PID 3/0.375/0.5 is assumed in driver flash.

`start` sweeps M2/M4 together from -100 through +100 in steps of 10, with zero
baseline and rest at every target. Measurement uses cumulative encoder counts
and actual timestamps. No automatic motion at boot, no runtime tuning options.

See the [E2 operator guide](../../test/E2_motor_speed/README.md) and
[project motor settings](../../docs/motor-settings.md).
