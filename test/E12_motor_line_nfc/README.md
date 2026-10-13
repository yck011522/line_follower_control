# E12 Dual BLDC Motor, Line Sensor & NFC Reader Test Suite

Comprehensive experimental test scripts, raw logs, analytical plots, and performance documentation for the **E12** integrated firmware running on the Seeed Studio XIAO ESP32-S3.

Firmware source: [src/e12_motor_line_nfc](../../src/e12_motor_line_nfc/)

---

## Objectives

1. **Integrated 100 Hz Control Loop**: Validate stable 100.0 Hz execution of the main control cycle (synchronous 8-channel line sensing, encoder EMA velocity estimation, and dual-motor closed-loop PI + Feedforward speed regulation).
2. **Telemetry Stream Validation**: Verify 10 Hz (every 0.1 s) non-blocking serial telemetry output:
   ```
   P=<pwm>,L=<mm/s>,R=<mm/s>,LINE=<8-bits>,NFC=<uid|NONE>,AGE=<sec>,RATE=<Hz>
   ```
3. **Closed-Loop Speed Regulation**: Verify accurate velocity control under command `S<speed>` from $-120$ to $+120\text{ mm/s}$ using calibrated feedforward ($K_\text{ff} = 1.98$) plus PI regulation with anti-windup protection.
4. **Line-Following Differential Steering**:
   - Proportional-derivative steering using weighted centroid error across the 8-channel line sensor.
   - **Inner Wheel Reversing**: Allow the inner wheel to actively transition into reverse during sharp corners to generate tight yaw pivoting without overshooting the line.
   - **Reverse Speed Constraint**: Strictly bound inner wheel reverse speed to the outer wheel's speed ($v_\text{inner} \ge -v_\text{outer}$) to prevent vehicle backward motion while enabling in-place zero-radius turns.
5. **Fail-Safe Stop**: Guarantee immediate motor shutdown within 10 ms upon reaching the end of the line (`LINE=11111111`) or off-surface detection (`LINE=00000000`).
6. **Concurrent Asynchronous NFC**: Verify non-blocking NFC tag scanning over the shared 400 kHz I2C bus without stalling line sensing or motor timing.

---

## Directory Contents

| File / Folder | Description |
|---|---|
| [README.md](README.md) | This document: test architecture, usage instructions, and safety conventions. |
| [result.md](result.md) | Full analytical experimental report with performance tables, dynamic observations, and embedded plots. |
| [run_line_following.py](run_line_following.py) | Interactive test runner for commanded table line-following runs with live telemetry logging and safe auto-stop. |
| [test_e12.py](test_e12.py) | Automated sanity verification script checking boot banners, telemetry formatting, and speed step responses. |
| [plot_results.py](plot_results.py) | Offline log parser that generates high-resolution analytical plots from raw test logs. |
| [results/](results/) | Raw test run logs (`.log`) capturing full serial output across speeds from S15 to S100. |
| [images/](images/) | Generated figures embedded in [result.md](result.md). |

---

## Test Tools & Usage

### 1. Interactive Table Line-Following Runner (`run_line_following.py`)

Used for safe testing on table tracks (including tethered operation):
- Connects to the ESP32-S3 serial port with `dtr=True`, `rts=False`.
- Queries baseline line sensor and motor state via `STATUS`.
- Sends speed command (e.g., `S15`, `S25`, `S40`, `S70`, `S100`).
- Streams real-time telemetry at 10 Hz with decoded speeds and line bits.
- Automatically halts motors if `LINE=11111111` persists for $\ge 0.5\text{ s}$ or when `--timeout` expires.
- Saves raw timestamped logs into `results/`.

```powershell
# Run line-following at 25 mm/s on COM17 with 10-second safety timeout
python test/E12_motor_line_nfc/run_line_following.py --port COM17 --speed 25 --timeout 10

# Run line-following at high speed (100 mm/s)
python test/E12_motor_line_nfc/run_line_following.py --port COM17 --speed 100 --timeout 8
```

### 2. Plot Generation (`plot_results.py`)

Parses the raw `.log` files in `results/` and regenerates all figures in `images/`:

```powershell
python test/E12_motor_line_nfc/plot_results.py
```

Generated plots:
- `images/e12_speed_comparison.png`: Multi-speed comparison of wheel velocities and line error tracking.
- `images/e12_detailed_dynamics.png`: Synchronized telemetry traces (speeds, error, PWM) for S25 and S100.
- `images/e12_inner_wheel_reverse_constraint.png`: Scatter verification plot proving $v_\text{inner} \ge -v_\text{outer}$.

---

## Safety Conventions

- **Tethered Cable Clearance**: Ensure USB cables have sufficient slack before starting motion.
- **Fail-Safe Line Loss**: The firmware immediately clears motor output when the track ends or the sensor leaves the black line (`LINE=11111111`).
- **Emergency Halt**: Sending `STOP`, `S0`, or pressing `Ctrl+C` in the runner script immediately stops both motors and clears integrator windup.
