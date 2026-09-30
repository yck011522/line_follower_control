# E2 — Motor Speed Command Test

Independent application in [main.cpp](main.cpp), using the shared `MotorDriver` library. PlatformIO environment: `e2_motor_speed`. Motor communication uses 400 kHz on D4/D5 with repeated START.

Startup configures and releases the motors. Serial `start` performs one simultaneous M2/M4 sweep from -240 through +240 in increments of 20. Each step has one second settling plus one second measurement; the full run is approximately 50 seconds. `stop` or `!` cancels. No motion starts on reset.

See the [operator and analysis guide](../../test/E2_motor_speed/README.md).
