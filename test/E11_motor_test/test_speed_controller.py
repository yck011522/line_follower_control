"""E11 Dual BLDC Motor Closed-Loop Speed Controller Test.

Tests the PI + Feedforward speed controller across forward, reverse,
and clamped velocity targets (mm/sec), verifying steady-state tracking,
left/right balance, and anti-windup clamping under COM17 hardware execution.
"""

import argparse
import csv
import datetime
import pathlib
import re
import sys
import time
import serial

ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULTS_DIR = ROOT / "test" / "E11_motor_test" / "results"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)

TELEMETRY_RE = re.compile(r"^P=(-?\d+),L=(-?\d+\.\d+),R=(-?\d+\.\d+)$")

# Conversion constant: 1350 pulses / (pi * 66.5 mm) = ~6.461947 steps/mm
STEPS_PER_MM = 1350.0 / (3.141592653589793 * 66.5)


def run_speed_test(port="COM17", baud=115200, targets=None, step_run_time=3.0, settle_time=0.8):
    if targets is None:
        # Range of target velocities in mm/sec
        targets = [30.0, 50.0, 80.0, 100.0, -50.0, -80.0, 150.0, 0.0]

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_log_path = RESULTS_DIR / f"speed_ctrl_{timestamp}_{port}_raw.log"
    csv_path = RESULTS_DIR / f"speed_ctrl_{timestamp}_{port}_summary.csv"

    print(f"Opening {port} at {baud} baud (dtr=True, rts=False)...")
    ser = serial.Serial(port=None, baudrate=baud, timeout=0.1)
    ser.dtr = True
    ser.rts = False
    ser.port = port

    raw_lines = []
    test_results = []
    completed = False

    try:
        ser.open()
        time.sleep(0.4)
        ser.reset_input_buffer()

        # Stop motors to ensure baseline
        ser.write(b"STOP\n")
        ser.flush()
        time.sleep(0.5)

        # Query status
        ser.write(b"STATUS\n")
        ser.flush()
        time.sleep(0.2)
        while ser.in_waiting:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                print(f"[INIT] {line}")
                raw_lines.append(f"[INIT] {line}")

        print(f"\nBeginning closed-loop speed test ({len(targets)} targets: {targets} mm/s)")
        print(f"{'Target (mm/s)':>14} | {'Target (st/s)':>14} | {'PWM (avg)':>10} | {'Left (mm/s)':>12} | {'Right (mm/s)':>12} | {'Err L (%)':>10} | {'Err R (%)':>10}")
        print("-" * 92)

        for target in targets:
            expected_clamped = max(-120.0, min(120.0, target))
            expected_steps_s = expected_clamped * STEPS_PER_MM

            # Send speed command S<val>
            cmd_str = f"S{target}\n"
            ser.write(cmd_str.encode("ascii"))
            ser.flush()

            # Settle window (allow speed controller PI loop to track and settle)
            settle_deadline = time.monotonic() + settle_time
            while time.monotonic() < settle_deadline:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    raw_lines.append(f"[SETTLE S={target}] {line}")

            # Measurement window
            meas_samples = []
            meas_deadline = time.monotonic() + step_run_time
            while time.monotonic() < meas_deadline:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    raw_lines.append(f"[MEAS S={target}] {line}")
                    m = TELEMETRY_RE.match(line)
                    if m:
                        pwm_out = int(m.group(1))
                        spd_l = float(m.group(2))
                        spd_r = float(m.group(3))
                        meas_samples.append((pwm_out, spd_l, spd_r))

            # Query STATUS for diagnostic dump
            ser.write(b"STATUS\n")
            ser.flush()
            time.sleep(0.1)
            while ser.in_waiting:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    raw_lines.append(f"[STATUS S={target}] {line}")

            # Intermediate stop between test points (if not testing 0)
            if target != 0.0:
                ser.write(b"STOP\n")
                ser.flush()
                rest_deadline = time.monotonic() + 0.5
                while time.monotonic() < rest_deadline:
                    line = ser.readline().decode("ascii", errors="replace").strip()
                    if line:
                        raw_lines.append(f"[REST S={target}] {line}")

            if meas_samples:
                # Use second half of measurement window (fully settled steady state)
                steady_samples = meas_samples[len(meas_samples) // 2:] if len(meas_samples) >= 4 else meas_samples
                avg_pwm = sum(s[0] for s in steady_samples) / len(steady_samples)
                avg_l_steps = sum(s[1] for s in steady_samples) / len(steady_samples)
                avg_r_steps = sum(s[2] for s in steady_samples) / len(steady_samples)

                avg_l_mm = avg_l_steps / STEPS_PER_MM
                avg_r_mm = avg_r_steps / STEPS_PER_MM

                err_l_pct = ((avg_l_mm - expected_clamped) / expected_clamped * 100.0) if abs(expected_clamped) > 1.0 else 0.0
                err_r_pct = ((avg_r_mm - expected_clamped) / expected_clamped * 100.0) if abs(expected_clamped) > 1.0 else 0.0

                test_results.append({
                    "target_mm_s": target,
                    "clamped_target_mm_s": round(expected_clamped, 1),
                    "expected_steps_s": round(expected_steps_s, 1),
                    "avg_pwm": round(avg_pwm, 1),
                    "left_steps_s": round(avg_l_steps, 1),
                    "right_steps_s": round(avg_r_steps, 1),
                    "left_mm_s": round(avg_l_mm, 1),
                    "right_mm_s": round(avg_r_mm, 1),
                    "err_l_pct": round(err_l_pct, 1),
                    "err_r_pct": round(err_r_pct, 1),
                    "samples": len(steady_samples),
                })

                print(f"{target:>14.1f} | {expected_steps_s:>14.1f} | {avg_pwm:>10.1f} | {avg_l_mm:>12.1f} | {avg_r_mm:>12.1f} | {err_l_pct:>9.1f}% | {err_r_pct:>9.1f}%")
            else:
                print(f"{target:>14.1f} | {expected_steps_s:>14.1f} | {'NO DATA':>10} | {'NO DATA':>12} | {'NO DATA':>12} | {'--':>10} | {'--':>10}")

        completed = True

    except KeyboardInterrupt:
        print("\n\n[ABORT] User interrupted test. Stopping motors immediately...")
    except Exception as e:
        print(f"\n\n[ERROR] Exception during test: {e}. Stopping motors...")
    finally:
        try:
            if ser.is_open:
                ser.write(b"STOP\n")
                ser.flush()
                time.sleep(0.1)
                ser.close()
        except Exception:
            pass

    # Save raw log file
    with open(raw_log_path, "w", encoding="utf-8") as f:
        f.write(f"# E11 Closed-Loop Speed Controller Test: {timestamp}\n")
        f.write(f"# Port: {port}, Targets: {targets}\n")
        f.write(f"# Status: {'COMPLETED' if completed else 'INCOMPLETE'}\n\n")
        for line in raw_lines:
            f.write(f"{line}\n")
    print(f"\nRaw log saved to: {raw_log_path}")

    # Save summary CSV
    if test_results:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            fieldnames = [
                "target_mm_s", "clamped_target_mm_s", "expected_steps_s",
                "avg_pwm", "left_steps_s", "right_steps_s",
                "left_mm_s", "right_mm_s", "err_l_pct", "err_r_pct", "samples"
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(test_results)
        print(f"Summary CSV saved to: {csv_path}")

    return test_results


def main():
    parser = argparse.ArgumentParser(description="Test E11 BLDC motor speed controller")
    parser.add_argument("--port", default="COM17", help="Serial port (default: COM17)")
    parser.add_argument("--targets", nargs="+", type=float, default=[30.0, 50.0, 80.0, 100.0, -50.0, -80.0, 150.0, 0.0],
                        help="Target velocities in mm/sec")
    parser.add_argument("--step-time", type=float, default=2.5, help="Measurement time per target in seconds")
    parser.add_argument("--settle-time", type=float, default=0.8, help="Settle time in seconds")
    args = parser.parse_args()

    run_speed_test(port=args.port, targets=args.targets, step_run_time=args.step_time, settle_time=args.settle_time)


if __name__ == "__main__":
    main()
