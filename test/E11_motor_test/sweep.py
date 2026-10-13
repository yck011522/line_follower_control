"""E11 Dual BLDC Motor PWM-to-Speed Sweep Script.

Sequentially steps PWM across a defined range (e.g. -200..+200 step 50),
records settled speed telemetry (P, Left, Right), saves raw logs and CSV summary,
and guarantees motor zero-stop on errors/interrupts.
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


def run_sweep(port="COM17", baud=115200, targets=None, step_run_time=2.0, settle_time=0.5):
    if targets is None:
        # Default sweep: -200 to +200 in steps of 50
        targets = [-200, -150, -100, -50, 0, 50, 100, 150, 200]

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_log_path = RESULTS_DIR / f"sweep_{timestamp}_{port}_raw.log"
    csv_path = RESULTS_DIR / f"sweep_{timestamp}_{port}_summary.csv"

    print(f"Opening {port} at {baud} baud (dtr=True, rts=False)...")
    ser = serial.Serial(port=None, baudrate=baud, timeout=0.1)
    ser.dtr = True
    ser.rts = False
    ser.port = port

    raw_lines = []
    sweep_results = []
    completed = False

    try:
        ser.open()
        time.sleep(0.3)
        ser.reset_input_buffer()

        # Stop motors to ensure clean baseline
        ser.write(b"STOP\n")
        ser.flush()
        time.sleep(0.5)

        print(f"\nBeginning sweep with {len(targets)} targets: {targets}")
        print(f"{'Target PWM':>12} | {'Left (step/s)':>15} | {'Right (step/s)':>15} | {'Samples':>8}")
        print("-" * 58)

        for target in targets:
            # 1. Baseline / settle to target
            cmd_str = f"P{target}\n"
            ser.write(cmd_str.encode("ascii"))
            ser.flush()

            # Settle window (allow motor to reach steady state)
            settle_deadline = time.monotonic() + settle_time
            while time.monotonic() < settle_deadline:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    raw_lines.append(f"[SETTLE P={target}] {line}")

            # Measurement window
            meas_samples = []
            meas_deadline = time.monotonic() + step_run_time
            while time.monotonic() < meas_deadline:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    raw_lines.append(f"[MEAS P={target}] {line}")
                    m = TELEMETRY_RE.match(line)
                    if m and int(m.group(1)) == target:
                        meas_samples.append((float(m.group(2)), float(m.group(3))))

            # Intermediate stop between test points
            ser.write(b"STOP\n")
            ser.flush()
            rest_deadline = time.monotonic() + settle_time
            while time.monotonic() < rest_deadline:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    raw_lines.append(f"[REST P={target}] {line}")

            if meas_samples:
                # Discard first sample to avoid edge filter transition
                active_data = meas_samples[1:] if len(meas_samples) > 2 else meas_samples
                avg_l = sum(s[0] for s in active_data) / len(active_data)
                avg_r = sum(s[1] for s in active_data) / len(active_data)
                sweep_results.append({
                    "pwm": target,
                    "left_speed": round(avg_l, 1),
                    "right_speed": round(avg_r, 1),
                    "samples": len(active_data),
                })
                print(f"{target:>12} | {avg_l:>15.1f} | {avg_r:>15.1f} | {len(active_data):>8}")
            else:
                sweep_results.append({
                    "pwm": target,
                    "left_speed": 0.0,
                    "right_speed": 0.0,
                    "samples": 0,
                })
                print(f"{target:>12} | {'NO DATA':>15} | {'NO DATA':>15} | {0:>8}")

        completed = True

    except KeyboardInterrupt:
        print("\n\n[ABORT] User interrupted sweep. Stopping motors immediately...")
    except Exception as e:
        print(f"\n\n[ERROR] Exception during sweep: {e}. Stopping motors...")
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
        f.write(f"# E11 Motor Sweep: {timestamp}\n")
        f.write(f"# Port: {port}, Targets: {targets}\n")
        f.write(f"# Status: {'COMPLETED' if completed else 'INCOMPLETE'}\n\n")
        for line in raw_lines:
            f.write(f"{line}\n")
    print(f"\nRaw log saved to: {raw_log_path}")

    # Save summary CSV
    if sweep_results:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["pwm", "left_speed", "right_speed", "samples"])
            writer.writeheader()
            writer.writerows(sweep_results)
        print(f"Summary CSV saved to: {csv_path}")

    return sweep_results


def main():
    parser = argparse.ArgumentParser(description="Run E11 BLDC motor speed sweep")
    parser.add_argument("--port", default="COM17", help="Serial port (default: COM17)")
    parser.add_argument("--min", type=int, default=-200, help="Minimum PWM (default: -200)")
    parser.add_argument("--max", type=int, default=200, help="Maximum PWM (default: 200)")
    parser.add_argument("--step", type=int, default=50, help="PWM step size (default: 50)")
    parser.add_argument("--step-time", type=float, default=2.0, help="Measurement time per target in seconds")
    parser.add_argument("--settle-time", type=float, default=0.5, help="Settle time in seconds")
    args = parser.parse_args()

    targets = list(range(args.min, args.max + 1, args.step))
    if targets[-1] != args.max:
        targets.append(args.max)

    run_sweep(port=args.port, targets=targets, step_run_time=args.step_time, settle_time=args.settle_time)


if __name__ == "__main__":
    main()
