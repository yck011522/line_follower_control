# E1 firmware — motor driver

One interactive firmware covers the 100/400 kHz communication checks, manual wheel rotation, and 5-second M2/M4 or both-motor runs. Uses the shared `lib/MotorDriver` register interface. Startup automatically configures the driver and releases outputs; motion requires serial commands. During motion, speed commands are refreshed at 50 Hz.

Build/upload environment: `e1_motor_communication`. Motor SDA/SCL: D4/D5. Serial: 115200 over USB. See the [operator guide](../../test/E1_motor_communication/README.md) for commands, current assumptions, and limitations.

Host automation and results: [test/E1_motor_communication](../../test/E1_motor_communication/README.md).
