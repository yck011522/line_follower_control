# E11 Dual BLDC Motor Speed & PWM Test Firmware

Firmware for testing the new JGY2430 brushless DC motors with open-loop PWM control and closed-loop speed measurement via CAP encoder interrupts.

## Hardware Connections (Seeed Studio XIAO ESP32-S3)

| Component | Signal | Board Pin | Chip Mapping | Description |
|---|---|---|---|---|
| Left Motor | White (PWM) | D0 | GPIO 1 | 20 kHz LEDC PWM output (0..255 duty) |
| Left Motor | Orange (CW/CCW) | D2 | GPIO 3 | Direction control (Inverted: HIGH = Forward, LOW = Reverse) |
| Left Motor | Yellow (CAP) | D6 | GPIO 43 | FG feedback pulses via level shifter (5V -> 3.3V) |
| Right Motor | White (PWM) | D1 | GPIO 2 | 20 kHz LEDC PWM output (0..255 duty) |
| Right Motor | Orange (CW/CCW) | D3 | GPIO 4 | Direction control (Normal: LOW = Forward, HIGH = Reverse) |
| Right Motor | Yellow (CAP) | D7 | GPIO 44 | FG feedback pulses via level shifter (5V -> 3.3V) |

## Physical Geometry & Unit Conversion

Per [docs/hardware.md](C:/Users/scmuser/Documents/line_follower_control/docs/hardware.md):
- **Motor**: JGY2430 12V Brushless Worm Gear Motor, 150:1 gearbox ratio, 9 pulses per rotor rev ($150 \times 9 = 1350$ steps per wheel rev).
- **Drive Wheel**: Rubber Tyre, 66.5 mm diameter.
- **Wheel Circumference**: $\pi \times 66.5\text{ mm} \approx 208.916\text{ mm}$.
- **Resolution**:
  $$\text{Steps per mm} = \frac{1350}{\pi \times 66.5} \approx 6.461947\text{ steps/mm}$$
  $$\text{Distance per step} = \frac{\pi \times 66.5}{1350} \approx 0.1547525\text{ mm/step}$$

## Control Architecture & PI + Feedforward Speed Controller

- **Control Loop Rate**: 100 Hz (10 ms period).
- **Instantaneous Speed**: Calculated each 10 ms cycle as $\text{delta\_pulses} / \Delta t$ in steps per second.
- **Filter**: Exponential Moving Average (EMA) with $\alpha = 0.24$ at 100 Hz:
  $$\text{EMA}_t = 0.24 \times \text{instSpeed} + 0.76 \times \text{EMA}_{t-1}$$
- **Feedforward + PI Controller**:
  $$u = \text{FF} + \text{P} + \text{I}$$
  - $\text{FF} = K_{ff} \times v_{\text{target}}$ with $K_{ff} = 1.98\text{ PWM / (mm/s)}$ (derived from hardware sweep).
  - $\text{P} = K_p \times (v_{\text{target}} - v_{\text{actual}})$ with $K_p = 0.60\text{ PWM / (mm/s)}$.
  - $\text{I} = \text{I} + K_i \times (v_{\text{target}} - v_{\text{actual}}) \times \Delta t$ with $K_i = 1.50\text{ PWM / (mm/s}\cdot\text{s)}$.
- **Anti-Windup Protection**:
  - **Conditional Integration**: Integration freezes when the output is saturated at $\pm 255$ and the error pushes further into saturation. This ensures the integral term never blows up if battery level drops or target speeds exceed physical limits.
  - **Hard Integral Clamping**: $|I| \le 50.0\text{ PWM}$.
  - **Zero Target Reset**: Setting target speed to 0 immediately resets the integral term and stops motion.
- **Output Cadence**: Emits telemetry every 0.2 seconds (5 Hz):
  $$\text{P}=150,\text{L}=50.5,\text{R}=50.5$$

## Serial Command Grammar

Baud rate: `115200`, newline terminated (`\n` or `\r\n`).

- `S<speed>` : Sets closed-loop target speed in mm/sec (`-120.0` to `+120.0`, clamped out of range, e.g. `S100`, `S-60`, `S0`).
- `P<val>` : Sets open-loop PWM for both motors simultaneously (`-255` to `+255`, e.g. `P150`, `P-150`, `P0`).
- `L<val>` : Sets open-loop PWM for Left motor only (`-255` to `+255`).
- `R<val>` : Sets open-loop PWM for Right motor only (`-255` to `+255`).
- `STOP` / `S` : Sets speed and PWM to 0 immediately for both motors.
- `STATUS` : Prints current mode, target speed, PWM, raw pulse counts, and speeds in steps/s and mm/s.
- `HELP` : Prints command summary.

## Hardware Behavior & Speed Calibration (COM17 Verified)

- **Active-Low PWM**: The JGY2430 onboard controller interprets duty 0 (0 V) as maximum speed and 255 (3.3 V) as motor stop. The firmware transparently handles this inversion (`ledcDuty = 255 - dutyMagnitude`).
- **Direction Alignment**: Left motor direction inverted (`INVERT_LEFT_DIR = true`, D2 HIGH = forward) so that positive commands drive both wheels forward simultaneously.
- **Closed-Loop Speed Controller Verification (Hardware Benchmark)**:

| Target (mm/s) | Target (steps/s) | Applied PWM (avg) | Measured Left (mm/s) | Measured Right (mm/s) | Error |
|:---:|:---:|:---:|:---:|:---:|:---:|
| -150.0 (clamped -120) | -775.4 | -237.5 | -119.7 | -120.2 | <0.3% |
| -120.0 | -775.4 | -237.8 | -119.9 | -119.5 | <0.4% |
| -60.0  | -387.7 | -120.2 | -60.1  | -60.5  | <0.8% |
| 0.0    | 0.0    | 0.0    | 0.0    | 0.0    | 0.0%  |
| 30.0   | 193.9  | 64.5   | 29.8   | 29.6   | <1.3% |
| 50.0   | 323.1  | 101.4  | 50.0   | 49.8   | <0.3% |
| 60.0   | 387.7  | 120.7  | 59.6   | 59.9   | <0.6% |
| 80.0   | 517.0  | 158.9  | 80.6   | 80.6   | <0.8% |
| 100.0  | 646.2  | 198.2  | 100.5  | 99.7   | <0.5% |
| 120.0  | 775.4  | 237.0  | 119.7  | 120.1  | <0.3% |
| 150.0 (clamped 120)  | 775.4 | 237.2 | 119.8 | 120.0 | <0.2% |

