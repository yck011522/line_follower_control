# ESP32 line-following robot controller

[E7 manual-push data collection](src/e7_data_collection/README.md) is a retired
experiment; its documentation and historical recordings are preserved.

Controller firmware and repeatable hardware experiments for four differential-drive toy cars on a reconfigurable game board. E14 uses an ESP32-S3, an eight-channel line sensor, an FM17550 NFC reader, and direct PWM/DIR motor control.

The first development phase measures peripheral performance independently before integrating line following and wireless control. PlatformIO is the intended firmware build/upload tool; Python will automate uploads, test triggering, serial capture, and analysis on Windows.

## System intent

- Follow a black line on a white board initially; support the opposite polarity later.
- Command left/right wheel speeds using the firmware's motor speed controller.
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
  e3_line_sensor/            Standalone line-sensor benchmark
  e4_line_sensor_library/    Line sensor library experiment
  e5_line_follow/            PD line-following experiment
  robot/                    Future integrated robot application
  bridge/                   Future USB radio bridge application
test/
  E0_motor_tuning/           Direct USB PID tuning and speed-response plots
  E1_motor_communication/    Communication test guide and results
  E2_motor_speed/            Speed-sweep analysis and results
  E3_line_sensor/            Future Python runner and E3 results
  E4_line_sensor_library/    Line sensor library results
  E5_nfc_reader/             Future Python runner and E5 results
  E6_wireless/               Future Python runner and E6 results
```

Start with the [documentation index](docs/README.md), [pin assignments](docs/hardware.md), and [development progress](docs/progress.md). The [test workflow](docs/testing.md) defines the intended automation and result layout.

## Current status

E1, E2, E4, E6 and E7 data collection used the retired I2C motor-driver library. Their dependent firmware and build environments have been removed; historical results and recording tools remain. E14 is the current PWM/DIR baseline.

Experiments are numbered by test: **E0 Motor PID Tuning -> E1 Motor Driver Communication -> E2 Motor Speed Command -> E3 standalone line sensor -> E4 line sensor library -> E5 NFC reader -> E6 wireless -> integration**. Historical measurement values remain unchanged. Each PlatformIO environment selects one application and reuses modules from `lib/`.

The default environment is `e12_motor_line_nfc`. See the historical
[E2 guide](test/E2_motor_speed/README.md) and [E0 findings](test/E0_motor_tuning/README.md).
All Python test connections must set **RTS off before opening the port**.

The E8 [FM17550 I2C UID poller](src/e8_nfc_fm17550_i2c/README.md) continuously
tests ISO14443A tags on D4/D5 and reports variable-length UIDs and
detection-to-UID timing.

Start with [E0 direct USB PID tuning](test/E0_motor_tuning/README.md), then the [E1 motor test plan](test/E1_motor_communication/PLAN.md) and [motor interface reference](reference/RC_Car_Motor_Driver_Interface.md). All project pin numbers use board D labels: motor SDA D4/SCL D5 (GPIO5/GPIO6), line-sensor SDA D6/SCL D7 (GPIO43/GPIO44). Remaining register/scaling details need verification before motion testing. The line-sensor model/protocol can follow later. Remaining questions are tracked in [open questions](docs/open-questions.md).

[E5 line following](src/e5_line_follow/README.md) combines the sensor and motor
libraries with test-local center estimation, filtering, and PD steering. It starts
at 100 mm/s automatically and runs until stopped or a fault occurs. Status defaults
to 5 Hz and includes measured loop frequency and control-work timing.
