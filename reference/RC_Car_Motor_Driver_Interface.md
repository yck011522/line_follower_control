# RC Car Motor Driver and Motor Interface

## Purpose

This document defines the interface between the main ESP32 controller and the motor subsystem.

The main controller should treat the four-channel motor driver board as a **motion-control coprocessor**. The driver board is responsible for the low-level DC motor interface, encoder acquisition, and closed-loop speed control. The main controller should normally work in terms of **left/right motion setpoints and motor feedback**, rather than raw PWM or encoder electrical details.

For this vehicle:

- **M2 = left drive motor**
- **M4 = right drive motor**
- **M1 and M3 are unused**
- Runtime communication with the main ESP32 is planned over **I2C**
- Motor driver I2C slave address: **0x26**

The board documentation also provides a UART protocol. UART is useful for commissioning and diagnostics, but is not required for normal runtime control if the I2C functions are sufficient.

## Motor Driver Board

The board is a four-channel encoder DC motor driver with an onboard microcontroller.

Relevant specifications:

| Parameter | Value |
|---|---:|
| Recommended supply input | 5-12 V |
| 5 V auxiliary output | 0.7 A |
| 3.3 V auxiliary output | 500 mA |
| Continuous current per motor channel | 4 A |
| Stated maximum per motor channel | 5.5 A |
| Motor interfaces | 4 encoder-motor channels |
| Host interfaces | UART and I2C |
| I2C address | 0x26 |

The onboard MCU performs the low-level motor and encoder processing. This is important architecturally: the main ESP32 does not need to run the wheel-speed PID loop itself.

## Drive Motor

The drive motor is a **Wheeltec GT50P22.5_12V** geared DC motor.

Current known parameters from the supplied motor documentation:

| Parameter | Value |
|---|---:|
| Rated voltage | 12 V |
| Gear ratio | nominally 22.5:1 |
| Detailed datasheet ratio | 22.569:1 |
| Rated current | 0.7 A |
| Stall current | 4 A |
| No-load output speed | approximately 450 rpm |
| Rated output speed | approximately 390 rpm |
| Rated torque | 1.19 kgf.cm |
| Stall torque | 5.5 kgf.cm |
| Rated power | 4.5 W |
| Encoder | GMR magnetic quadrature encoder |
| Encoder resolution | 500 ppr |
| Encoder supply | 3.3-5 V |

The 4 A motor stall-current figure is equal to the board's stated continuous current per channel, so the pairing is plausible. The battery and wiring should nevertheless be sized for approximately **8 A combined stall current** for the two drive motors, with additional margin.

## Encoder Compatibility

The GT50 motor uses a **GMR magnetic encoder**, rather than the lower-resolution Hall encoder used on some of the board's predefined motors.

The driver documentation allows the following parameters to be configured independently:

- encoded/non-encoded motor type
- encoder line count
- gearbox ratio
- wheel diameter
- speed-control PID

This suggests that a nonstandard encoded motor can be used, provided that its encoder presents a compatible A/B quadrature signal.

The GT50 encoder documentation indicates:

- A/B quadrature output
- 500 ppr
- 3.3-5 V supply
- pull-up output

The exact electrical compatibility and A/B polarity should still be verified experimentally when the full encoder pinout is available.

A useful reference number is:

`500 ppr x 22.569 ≈ 11,284.5 encoder pulses per gearbox output revolution`

This is only a scaling reference. The driver's exact interpretation of "ppr" and whether it counts one, two, or four quadrature edges per cycle should be confirmed experimentally.

## Motor Driver Configuration

The motor driver firmware has predefined motor-type IDs:

| ID | Firmware label |
|---:|---|
| 1 | 520 motor |
| 2 | 310 motor |
| 3 | TT motor with encoder |
| 4 | TT motor without encoder |

The GT50 is not one of these predefined motors.

For encoded motors, the documentation indicates that types 1, 2, or 3 can be used, while the motor type also appears to determine encoder phase/polarity conventions. Therefore, the GT50 should **not simply be assumed to be a "520" motor**. The correct type should be selected by a short direction/encoder-sign test.

The custom motor parameters then need to be configured separately:

- encoder line count: nominally **500 ppr**
- gearbox ratio: **22.569:1 actual / 22.5:1 nominal**
- wheel diameter: **65 mm**, supplied by the user for initial commissioning
- speed PID: initially use the board default unless testing shows that retuning is needed
- motor dead zone: initially use default, then tune only if necessary

### Important Gear-Ratio Issue

The I2C gearbox-ratio register is documented as a **uint16_t**. This means it cannot directly represent the GT50's fractional ratio of 22.569.

This needs to be tested or clarified before relying on the driver's internally calculated physical speed. Simply entering 22 or 23 creates roughly a 2% scaling error.

Possible approaches are:

1. Verify whether the firmware actually accepts a scaled/fractional representation despite the documentation.
2. Use the nearest integer ratio and calibrate the effective wheel-diameter parameter.
3. Treat the driver's speed value as a calibrated internal unit and perform precise odometry in the main controller from raw encoder counts.
4. Ask Wheeltec how their own GT50 motor is intended to be configured with this driver.

Do not alter the encoder line-count parameter merely to compensate for the gearbox ratio until the firmware's internal calculation is understood.

## I2C Runtime Interface

The documented I2C slave address is:

`0x26`

The useful registers are:

| Register | R/W | Type | Purpose |
|---|---|---|---|
| `0x01` | W | uint8 | motor type |
| `0x02` | W | uint16 | PWM dead zone |
| `0x03` | W | uint16 | encoder line count |
| `0x04` | W | uint16 | gearbox ratio |
| `0x05` | W | float | wheel diameter |
| `0x06` | W | 4 x int16 | closed-loop speed command for M1-M4 |
| `0x07` | W | 4 x int16 | direct PWM command for M1-M4 |
| `0x08` | R | uint16 | battery voltage |
| `0x10` | R | int16 | M1 encoder pulse count over 10 ms |
| `0x11` | R | int16 | M2 encoder pulse count over 10 ms |
| `0x12` | R | int16 | M3 encoder pulse count over 10 ms |
| `0x13` | R | int16 | M4 encoder pulse count over 10 ms |
| `0x20-0x21` | R | 2 x 16-bit | M1 cumulative encoder count |
| `0x22-0x23` | R | 2 x 16-bit | M2 cumulative encoder count |
| `0x24-0x25` | R | 2 x 16-bit | M3 cumulative encoder count |
| `0x26-0x27` | R | 2 x 16-bit | M4 cumulative encoder count |

For this vehicle, the normal closed-loop speed command is written to register `0x06` as four signed 16-bit values:

`[M1, M2, M3, M4] = [0, LEFT, 0, RIGHT]`

The documentation specifies **big-endian** byte order for each speed/PWM int16 value.

Example:

- left command = +200
- right command = +200

Payload:

`[0, +200, 0, +200]`

The exact sign of left and right commands should be calibrated after installation, because the two motors may be physically mirrored.

### Stop Behaviour

The serial documentation states that a speed setpoint of zero leaves the PID active, so the wheel resists being turned.

Therefore there are conceptually two different stop modes:

- **speed = 0:** closed-loop hold / active stop
- **PWM = 0:** motor output released from the speed PID

The I2C speed and PWM registers appear to access the same underlying control modes, so this distinction should be tested and preserved in the vehicle control logic.

## Recommended Runtime Data Exchange

For normal operation, the main ESP32 only needs a small motor-driver interface.

### Commands to the Motor Driver

At the vehicle control-loop rate:

- write left/right wheel speed to `0x06`
- M1 and M3 remain zero

Occasionally:

- use `0x07` only for special cases such as explicit release/coast or low-level testing

### Feedback from the Motor Driver

Recommended minimum feedback:

- M2 recent encoder pulses: `0x11`
- M4 recent encoder pulses: `0x13`
- battery voltage: `0x08` at a much lower rate

Optional:

- M2 cumulative encoder count: `0x22-0x23`
- M4 cumulative encoder count: `0x26-0x27`

For odometry, cumulative encoder counts are generally preferable because they avoid losing distance if one short-period sample is missed.

## I2C Versus UART

The two interfaces overlap substantially, but they are not completely equivalent.

| Function | UART | I2C |
|---|:---:|:---:|
| Set motor type | Yes | Yes |
| Set dead zone | Yes | Yes |
| Set encoder line count | Yes | Yes |
| Set gearbox ratio | Yes | Yes |
| Set wheel diameter | Yes | Yes |
| Closed-loop speed command | Yes | Yes |
| Direct PWM command | Yes | Yes |
| Read battery voltage | Yes | Yes |
| Read short-period encoder pulses | Yes | Yes |
| Read cumulative encoder counts | Yes | Yes |
| Configure PID gains | **Yes** | **Not documented** |
| Factory reset | **Yes** | **Not documented** |
| Read stored flash configuration | **Yes** | **Not documented** |
| Driver-calculated wheel-speed telemetry | **Yes** | **Not documented** |
| Automatic continuous telemetry streaming | **Yes** | **No; host polls registers** |

The most important loss when choosing I2C is therefore **not normal motor control**. It is mainly commissioning, diagnostics, PID configuration, and convenient streaming telemetry.

## Recommended Communication Strategy

For this project, **I2C is suitable as the normal runtime interface**.

A useful division is:

### Commissioning

Use UART temporarily if necessary to:

- inspect flash configuration
- tune or set the speed PID
- restore factory defaults
- verify parameters and firmware behaviour

The UART documentation explicitly marks the motor type, dead zone, encoder line count, gearbox ratio, wheel diameter, and PID parameters as power-cycle persistent.

The I2C documentation does **not** explicitly state whether configuration-register writes are stored in flash. This should be tested.

If UART-configured parameters remain stored in flash, the final main controller does not need to know the detailed motor model at all.

### Runtime

Use only I2C for:

- wheel-speed commands
- encoder feedback
- battery monitoring

This keeps the motor subsystem hidden behind a very small interface:

`setWheelSpeeds(left, right)`

`readWheelEncoders()`

`readBatteryVoltage()`

## Communication Performance and Latency

The documentation does not provide:

- I2C bus clock frequency
- command-processing latency
- UART-versus-I2C benchmarks
- motor PID update frequency
- measured round-trip time

Therefore there is **no documented evidence that choosing I2C changes the motor's closed-loop control performance**.

The board contains its own MCU and performs the motor-speed control locally. Consequently, once a speed setpoint has reached the board, the host communication protocol should not be part of the inner motor PID loop.

For the intended vehicle control rate of roughly tens of hertz, either interface should have ample bandwidth.

As a rough transport comparison:

- UART is specified at **115200 baud**
- a typical ASCII speed command takes on the order of 1-2 ms on the wire
- an I2C four-motor speed write contains only about 8 data bytes plus addressing/register overhead
- at 100 kHz I2C this is below approximately 1 ms of raw bus time
- at 400 kHz it is only a few tenths of a millisecond

These I2C numbers are estimates only because the vendor does not specify the supported I2C clock rate.

The likely practical difference is therefore:

- **UART:** more diagnostics and easy continuous telemetry
- **I2C:** smaller binary transactions and convenient register access
- **motor-control quality:** primarily determined by the driver's local encoder/PID implementation, not by UART versus I2C

For a 30-50 Hz vehicle-level controller, transport latency is unlikely to be the limiting factor unless the board firmware itself introduces unexpected delays.

## Items Still to Verify

The following should remain open until the motor/encoder documentation or bench tests provide the answer:

- full GT50 encoder pinout
- A/B electrical levels and polarity with this driver board
- whether 500 ppr is interpreted by the board exactly as expected
- how the firmware handles the fractional 22.569 gearbox ratio
- effective rolling diameter calibration (nominal wheel diameter is 65 mm)
- correct predefined motor-type ID for encoder direction
- whether I2C configuration writes persist across power cycles
- supported I2C clock rate: 100 kHz, 400 kHz, or otherwise
- signed behaviour of the 32-bit cumulative encoder count
- actual internal PID/update frequency
- whether default PID gains are appropriate for the GT50
- exact behaviour of zero-speed versus zero-PWM over I2C
- I2C SDA/SCL voltage and pull-up level

## Servo-Control Note

The uploaded motor-driver documentation describes **closed-loop DC motor speed control**, but it does not document a separate hobby-servo output or hobby-servo command set.

If "servo control" refers to the closed-loop wheel motors, it is already covered by the speed PID above.

If the board is also expected to control a separate RC/hobby servo, the corresponding servo documentation is still required and should be added to this file later.
