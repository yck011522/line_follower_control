# E12 Dual BLDC Motor, Line Sensor & NFC Reader Firmware

Integrated firmware combining closed-loop BLDC motor control, synchronous 8-channel line sensing, and asynchronous NFC tag detection over a shared I2C bus at 100 Hz.

## Hardware Connections (Seeed Studio XIAO ESP32-S3)

| Component | Signal | Board Pin | Chip Mapping | Description |
|---|---|---|---|---|
| Left Motor | White (PWM) | D0 | GPIO 1 | 20 kHz LEDC PWM output (0..255 duty) |
| Left Motor | Orange (CW/CCW) | D2 | GPIO 3 | Direction control (Inverted: HIGH = Forward, LOW = Reverse) |
| Left Motor | Yellow (CAP) | D6 | GPIO 43 | FG feedback pulses via level shifter (5V -> 3.3V) |
| Right Motor | White (PWM) | D1 | GPIO 2 | 20 kHz LEDC PWM output (0..255 duty) |
| Right Motor | Orange (CW/CCW) | D3 | GPIO 4 | Direction control (Normal: LOW = Forward, HIGH = Reverse) |
| Right Motor | Yellow (CAP) | D7 | GPIO 44 | FG feedback pulses via level shifter (5V -> 3.3V) |
| Shared I2C | SDA | D4 | GPIO 5 | 400 kHz I2C Data (Line Sensor 0x12, NFC FM17550 0x28) |
| Shared I2C | SCL | D5 | GPIO 6 | 400 kHz I2C Clock (Line Sensor 0x12, NFC FM17550 0x28) |

## Control Loop & Architecture

- **Main Control Loop**: 100 Hz (10 ms period).
  1. Synchronous I2C read of the 8-channel line sensor (register 0x30 @ 0x12) via repeated-START (~250 µs).
  2. Motor speed calculation from CAP encoder interrupts via Exponential Moving Average (EMA, $\alpha = 0.24$).
  3. Closed-loop Feedforward + PI motor speed regulator with anti-windup protection.
  4. Trigger asynchronous NFC read step when idle.
- **Asynchronous NFC Service**: Serviced cooperatively in `loop()` between control ticks without blocking the bus or motor regulation.
- **Telemetry Stream**: Emitted every **0.1 seconds** (10 Hz):
  `P=102,L=51,R=50,LINE=11111111,NFC=NONE,AGE=--.-s,RATE=100.0Hz`
  - `P` : Commanded / active average PWM duty (-255..+255).
  - `L` : Left motor speed in mm/sec (rounded integer, no decimal).
  - `R` : Right motor speed in mm/sec (rounded integer, no decimal).
  - `LINE` : 8-bit line sensor binary mask (bit 7 to bit 0, active-low: 0 = black line, 1 = white background).
  - `NFC` : NFC tag UID if scanned within the past 1.0 second (e.g. `11:E5:99:23`), or `NONE` if older / no tag.
  - `AGE` : Elapsed age since last tag detection in seconds with 1 decimal (e.g. `0.3s`, `1.5s`, or `--.-s`).
  - `RATE` : Measured main control loop frequency in Hz with 1 decimal (e.g. `100.0Hz`).

## Serial Command Grammar

Baud rate: `115200`, newline terminated (`\n` or `\r\n`).

- `S<speed>` : Sets closed-loop line following base speed in mm/sec (`-120.0` to `+120.0`, clamped out of range, e.g. `S15`, `S30`, `S0`). Line following engages automatically when a valid line is present and halts safely if the line is lost (`11111111` or `00000000`).
  - **Steering Control**: Proportional-derivative differential steering based on 8-channel weighted centroid error ($K_p = 2.4$, $K_d = 0.05$).
  - **Inner Wheel Reversing**: To negotiate sharp corners aggressively without forward drift, the inner wheel transitions from forward speed to active reverse as error increases.
  - **Reverse Speed Limit**: The inner wheel's reverse speed is bounded by the negative of the outer wheel's speed ($v_\text{inner} \ge -v_\text{outer}$). At maximum error, the robot performs an in-place zero-radius pivot turn with zero forward advancement until the line is re-centered.
- `P<val>` : Sets open-loop PWM for both motors simultaneously (`-255` to `+255`).
- `L<val>` : Sets open-loop PWM for Left motor only (`-255` to `+255`).
- `R<val>` : Sets open-loop PWM for Right motor only (`-255` to `+255`).
- `STOP` : Immediately zeroes motor PWM, stops motion, and resets speed controller integrals.
- `STATUS` : Prints diagnostic snapshot of motor speeds, line follower state, pulse counts, line sensor mask, loop rate, and NFC status.
- `HELP` : Prints command grammar.

## Hardware Verification Results (COM17)

Tested live on hardware with wheels propped up:
- Line sensor responding at address `0x12` (`LINE=11111111` in free air).
- NFC reader initialized at address `0x28` (`NFC=RUNNING`, gain 43 dB).
- Motor speeds accurately regulated at 100 Hz:
  - `S50` $\rightarrow$ `L=50 mm/s`, `R=50 mm/s` (PWM ~102)
  - `S80` $\rightarrow$ `L=80 mm/s`, `R=80 mm/s` (PWM ~160)
  - `S-50` $\rightarrow$ `L=-50 mm/s`, `R=-50 mm/s` (PWM ~-103)
  - `STOP` $\rightarrow$ `L=0 mm/s`, `R=0 mm/s` (PWM 0)
- Telemetry format matched 100% of frames over a 10-second continuous run without dropped cycles or serial blocking.
