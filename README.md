# ESP32 line-following robot controller

Controller firmware and repeatable hardware experiments for four differential-drive toy cars on a reconfigurable game board. Each car uses an ESP32-S3, an eight-channel line sensor, an RC522 NFC reader, and a motor controller that handles wheel-speed regulation internally.

The first development phase measures peripheral performance independently before integrating line following and wireless control. PlatformIO is the intended firmware build/upload tool; Python will automate uploads, test triggering, serial capture, and analysis on Windows.

## System intent

- Follow a black line on a white board initially; support the opposite polarity later.
- Command left/right wheel speeds through the motor driver. The robot firmware implements steering control, while the motor driver handles its existing motor PID loops.
- Accept forward/stop and left/straight/right turn indicators from a central Windows PC through a USB-connected ESP32 radio bridge. ESP-NOW is the presumed wireless transport, pending confirmation.
- Use NFC tag UIDs to identify zones, branch entry/exit, and merge entry/exit. Latch the turn choice at the branch decision point until the branch completes.
- Report robot identity, acknowledged world state, NFC observations, motion state, wheel speeds, line readings, battery voltage, and uptime.

Initial targets are a 50 Hz internal control loop, 50 Hz world-state broadcasts, 20 Hz robot telemetry, and 100 mm/s driving speed. Experiments will determine whether 100 Hz or higher local control is practical. These are targets, not measured capabilities.

## Repository layout

```text
docs/                       System requirements, interfaces, and progress
lib/                        Reusable firmware drivers and control/protocol modules
src/
  e1_motor_communication/    Motor-driver communication and basic motion
  e2_motor_speed/            Separate motor speed-command sweep
  e3_line_sensor/            Future line-sensor experiment
  e4_nfc_reader/             Standalone RC522 experiment
  e5_wireless/               Robot/bridge communication experiment
  robot/                    Future integrated robot application
  bridge/                   Future USB radio bridge application
test/
  E0_motor_tuning/           Direct USB PID tuning and speed-response plots
  E1_motor_communication/    Communication test guide and results
  E2_motor_speed/            Speed-sweep analysis and results
  E3_line_sensor/            Future Python runner and E3 results
  E4_nfc_reader/             Future Python runner and E4 results
  E5_wireless/               Future Python runner and E5 results
```

Start with the [documentation index](docs/README.md), [pin assignments](docs/hardware.md), and [development progress](docs/progress.md). The [test workflow](docs/testing.md) defines the intended automation and result layout.

## Current status

The interactive E1 motor tester builds and uploads to the Seeed Studio XIAO ESP32S3. One firmware automatically configures the driver at startup and supports 100/400 kHz I²C checks, manual encoder observation, and five-second M2/M4 or both-motor runs through serial commands. See the [operator guide](test/E1_motor_communication/README.md). Automated Python runners and integrated robot firmware remain future work; hardware validation is in progress.

Experiments are now numbered by test: **E0 Motor PID Tuning → E1 Motor Driver Communication Test → E2 Motor Speed Command Test → E3 line sensor → E4 NFC reader → E5 wireless → integration**. Existing folders and results were moved to match; historical measurement values remain unchanged. Each PlatformIO environment selects one application and reuses modules from `lib/`.

The default environment is `e2_motor_speed`; `e1_motor_communication` remains independently buildable. Both default to 400 kHz motor I²C. E2 sweeps both motors from -240 to +240 in steps of 20, with one second settling plus one second measuring at each command. See the [E2 guide](test/E2_motor_speed/README.md).

Start with [E0 direct USB PID tuning](test/E0_motor_tuning/README.md), then the [E1 motor test plan](test/E1_motor_communication/PLAN.md) and [motor interface reference](reference/RC_Car_Motor_Driver_Interface.md). All project pin numbers use board D labels: motor SDA D4/SCL D5 (GPIO5/GPIO6), line-sensor SDA D6/SCL D7 (GPIO43/GPIO44). Remaining register/scaling details need verification before motion testing. The line-sensor model/protocol can follow later. Remaining questions are tracked in [open questions](docs/open-questions.md).
