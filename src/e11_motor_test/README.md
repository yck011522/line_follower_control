# E11 Dual BLDC Motor Speed & PWM Test Firmware

Firmware for testing the new JGY2430 brushless DC motors with open-loop PWM control and closed-loop speed measurement via CAP encoder interrupts.

## Hardware Connections (Seeed Studio XIAO ESP32-S3)

| Component | Signal | Board Pin | Chip Mapping | Description |
|---|---|---|---|---|
| Left Motor | White (PWM) | D0 | GPIO 1 | 20 kHz LEDC PWM output (0..255 duty) |
| Left Motor | Orange (CW/CCW) | D2 | GPIO 3 | Direction control |
| Left Motor | Yellow (CAP) | D6 | GPIO 43 | FG feedback pulses via level shifter (5V -> 3.3V) |
| Right Motor | White (PWM) | D1 | GPIO 2 | 20 kHz LEDC PWM output (0..255 duty) |
| Right Motor | Orange (CW/CCW) | D3 | GPIO 4 | Direction control |
| Right Motor | Yellow (CAP) | D7 | GPIO 44 | FG feedback pulses via level shifter (5V -> 3.3V) |

## Control Loop & Speed Calculation

- **Control Loop Rate**: 100 Hz (10 ms period).
- **Instantaneous Speed**: Calculated each 10 ms cycle as `delta_pulses / dt` in steps per second.
- **Direction Sign**: Signed with the active motor command (positive for forward PWM, negative for reverse PWM).
- **Filter**: Exponential Moving Average (EMA) with $\alpha = 0.24$ at 100 Hz:
  $$\text{EMA}_t = 0.24 \times \text{instSpeed} + 0.76 \times \text{EMA}_{t-1}$$
- **Output Cadence**: Emits telemetry every 0.2 seconds (5 Hz):
  $$\text{P}=150,\text{L}=50.5,\text{R}=50.5$$

## Serial Command Grammar

Baud rate: `115200`, newline terminated (`\n` or `\r\n`).

- `P<val>` : Sets PWM for both motors simultaneously (`-255` to `+255`, e.g. `P150`, `P-150`, `P0`).
- `L<val>` : Sets PWM for Left motor only (`-255` to `+255`).
- `R<val>` : Sets PWM for Right motor only (`-255` to `+255`).
- `STOP` / `S` : Sets PWM to 0 immediately for both motors.
- `STATUS` : Prints current commanded PWM, raw pulse totals, and speeds.
- `HELP` : Prints command summary.

## Hardware Behavior & Speed Calibration (COM17 Verified)

- **Active-Low PWM**: The JGY2430 onboard controller interprets duty 0 (0 V) as maximum speed and 255 (3.3 V) as motor stop. The firmware transparently handles this inversion (`ledcDuty = 255 - dutyMagnitude`).
- **Speed Calibration Sweep (Empirical @ 12V supply)**:

| Commanded PWM | Left Motor (steps/s) | Right Motor (steps/s) | L/R Balance |
|:---:|:---:|:---:|:---:|
| -200 | -656.8 | -650.4 | 99.0% |
| -150 | -484.9 | -482.4 | 99.5% |
| -100 | -322.5 | -310.4 | 96.2% |
| -50  | -133.0 | -135.9 | 97.9% |
| 0    | 0.0    | 0.0    | 100.0% |
| 50   | 139.2  | 136.3  | 97.9% |
| 100  | 315.9  | 310.1  | 98.2% |
| 150  | 490.5  | 481.7  | 98.2% |
| 200  | 654.9  | 648.7  | 99.1% |

