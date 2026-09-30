# Adopted motor configuration and tuning evidence

This is the project baseline selected after E0 on 2026-09-30. All future motor
applications must reapply the five I2C-accessible settings at every ESP32 boot.
PID is the explicit exception: use the driver's saved 3/0.375/0.5, as selected
by the user. E2 uses I2C only and does not send motor-driver serial commands.

| Setting | Adopted value | Serial command / flash label |
| --- | --- | --- |
| Motor type | 1 | `$mtype:1#` / `Motor_type` |
| PWM dead zone | 1650 | `$deadzone:1650#` / `Dead_Zone` |
| Pulse line | 500 | `$mline:500#` / `Pulse_Line` |
| Pulse phase (gear ratio) | 23 | `$mphase:23#` / `Pulse_Phase` |
| Wheel diameter | 65 mm | `$wdiameter:65#` / `wheel_diameter` |
| Speed PID | P=3, I=0.375, D=0.5 | `$MPID:3,0.375,0.5#` / `P`, `I`, `D` |

Pulse_Line is the encoder's **500 ppr**, not 2000. The driver applies the x4
quadrature factor itself: E2's sweep with 2000 ran the wheel about four times
faster than commanded, and a hand-turned wheel revolution gave 44998 counts
(500 ppr x 4 x 22.5 gearbox). Ratio 23 is the
selected driver setting; the motor datasheet's fractional 22.569 ratio remains
relevant to independent odometry calibration. Diameter 65 mm is nominal.

M2 is the left drive motor, M4 the right; M1/M3 are unused. The protocol exposes
one board-level PID triplet. Positive/negative electrical command signs have not
yet been mapped to vehicle forward/reverse for both mounted wheels.

## What E0 established

The chosen values are a tested working baseline, not a claim of optimal tuning.
E0 exercised M2 through direct driver USB serial. Six additional sweeps compared
reduced integral gains after the user's original three runs.

- Removing D (3/3/0) worsened low-speed variation relative to 3/3/0.5.
- Reducing P to 1.5 with I=3/D=0.5 also worsened the target-20 result: final-second
  standard deviation rose from 3.84 to 5.73, and the peak rose from 26.86 to 31.85.
- Reducing I while keeping P=3/D=0.5 improved sustained target-20 tracking.
- In matched five-second tests, I=0.375 at target 20 averaged 19.66, with standard
  deviation 2.68 and whole-step peak 24.75. I=0.75 averaged 18.81, with standard
  deviation 2.73 and peak 31.41. The lower-I candidate also held target 80 more
  smoothly, though it was not better on every metric at target 40.
- **Target 10 is unresolved:** both candidates showed near-zero readings and
  bursts toward 30. Zero-speed hunting also remained in some runs. Do not assume
  this baseline guarantees smooth inner-wheel operation in differential steering.

See [detailed results and run IDs](../test/E0_motor_tuning/TUNING_RESULTS.md).
These are driver speed units and serial arrival times; neither physical speed
calibration nor loaded two-wheel cornering was established. M4, reverse commands,
normal-to-low-speed transitions, and representative loads need further evidence.

## Boot configuration and interfaces

The vendor documents persistent PID storage. The project therefore relies on
**P=3, I=0.375, D=0.5 already saved in the driver**, and does not touch the motor
serial protocol in E2. Reapply type, dead zone, pulse line, ratio and diameter on
every ESP32 boot using I2C at address 0x26, 400 kHz, SDA D4 / SCL D5.

Require successful I2C acknowledgments before motion. These configuration
registers are documented as write-only: acknowledgments establish successful
transactions, not value readback. PID has no documented I2C read/write register;
firmware and metadata explicitly label it `stored_unverified`. Replacing or
factory-resetting the driver invalidates that assumption. E0 remains a historical
commissioning tool; no UART wiring or boot-time serial configuration is added.

E2 measures speed from cumulative encoder differences over actual ESP32 timestamps.
Raw counts/s is the primary measurement. Its nominal mm/s estimate uses
`counts/s * pi * 65 / 44998` (hand-measured counts per wheel revolution),
independent of the driver's line and ratio settings. This is not independently verified physical distance.
It is not the serial `$MSPD` value.

Zero speed keeps PID active; zero PWM releases the outputs. Tests should record
zero-speed behavior where relevant and release outputs on completion or failure.

## Serial-port rule for every Python test tool

Use 115200 baud, 8N1, no flow control, **DTR on / RTS off**. Set the line states
before opening the port:

```python
port = serial.Serial(port=None, baudrate=115200, timeout=0.05,
                     write_timeout=1, rtscts=False, dsrdtr=False, xonxoff=False)
port.dtr = True
port.rts = False
port.port = port_name
port.open()
```

`rtscts=False` only disables flow control; it does not turn RTS off. The earlier
E0 implementation left RTS at its default on state. A failed run answered its
initial flash query but stopped responding after the PID-triggered reset. Using
the serial monitor's DTR-on/RTS-off settings allowed repeated reconnects and PID
resets. This supports a control-line/reset explanation, not proven flash corruption.

Do not intentionally toggle RTS on cleanup. Bound communication waits, space
setup/shutdown commands, and allow output to drain before close. The OS/USB
driver still controls physical line transitions when a handle opens or closes.
PlatformIO upload/reset behavior is separate from automatic test communication.
