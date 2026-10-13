"""E11 dual BLDC motor test script.

Sends PWM commands (P-255..P255, STOP) over USB serial, monitors
telemetry output (P=...,L=...,R=...), and safely halts motors on completion or failure.
Complies with project serial guidelines (rts=False before open, bounded motion, zero-stop cleanup).
"""

import argparse
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


def main():
    parser = argparse.ArgumentParser(description="Test E11 BLDC motors over serial")
    parser.add_argument("--port", default="COM17", help="Serial port (default: COM17)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--pwm", type=int, default=100, help="PWM to test (-255..255, default: 100)")
    parser.add_argument("--run-time", type=float, default=2.5, help="Duration to run at target PWM in seconds")
    parser.add_argument("--settle-time", type=float, default=1.0, help="Baseline/settle time before and after in seconds")
    args = parser.parse_args()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file_path = RESULTS_DIR / f"run_{timestamp}_{args.port}_pwm{args.pwm}.log"

    print(f"Connecting to {args.port} (dtr=True, rts=False)...")
    ser = serial.Serial(port=None, baudrate=args.baud, timeout=0.1)
    ser.dtr = True
    ser.rts = False
    ser.port = args.port

    raw_lines = []
    telemetry_samples = []
    completed = False

    try:
        ser.open()
        time.sleep(0.3)
        ser.reset_input_buffer()

        print(f"Collecting baseline (P=0) for {args.settle_time:.1f}s...")
        ser.write(b"STOP\n")
        ser.flush()

        baseline_end = time.monotonic() + args.settle_time
        while time.monotonic() < baseline_end:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                raw_lines.append(f"[BASELINE] {line}")
                print(f"  {line}")
                m = TELEMETRY_RE.match(line)
                if m:
                    telemetry_samples.append((time.monotonic(), int(m.group(1)), float(m.group(2)), float(m.group(3))))

        # Apply commanded PWM
        cmd_str = f"P{args.pwm}\n"
        print(f"\nSending command: {cmd_str.strip()} for {args.run_time:.1f}s...")
        ser.write(cmd_str.encode("ascii"))
        ser.flush()

        run_end = time.monotonic() + args.run_time
        while time.monotonic() < run_end:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                raw_lines.append(f"[RUN] {line}")
                print(f"  {line}")
                m = TELEMETRY_RE.match(line)
                if m:
                    telemetry_samples.append((time.monotonic(), int(m.group(1)), float(m.group(2)), float(m.group(3))))

        # Ramp down / Stop
        print(f"\nStopping motors and settling for {args.settle_time:.1f}s...")
        ser.write(b"STOP\n")
        ser.flush()

        settle_end = time.monotonic() + args.settle_time
        while time.monotonic() < settle_end:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                raw_lines.append(f"[STOP] {line}")
                print(f"  {line}")
                m = TELEMETRY_RE.match(line)
                if m:
                    telemetry_samples.append((time.monotonic(), int(m.group(1)), float(m.group(2)), float(m.group(3))))

        completed = True

    except KeyboardInterrupt:
        print("\nInterrupted by user. Ensuring motors are stopped...")
        try:
            ser.write(b"STOP\n")
            ser.flush()
        except Exception:
            pass
    except Exception as e:
        print(f"\nError occurred: {e}")
        try:
            ser.write(b"STOP\n")
            ser.flush()
        except Exception:
            pass
    finally:
        try:
            if ser.is_open:
                ser.write(b"STOP\n")
                ser.flush()
                ser.close()
        except Exception:
            pass

    # Save log file
    with open(log_file_path, "w", encoding="utf-8") as f:
        f.write(f"# E11 Motor Test Run: {timestamp}\n")
        f.write(f"# Port: {args.port}, Target PWM: {args.pwm}, Duration: {args.run_time}s\n")
        f.write(f"# Status: {'COMPLETED' if completed else 'INCOMPLETE'}\n\n")
        for line in raw_lines:
            f.write(f"{line}\n")

    print(f"\nLog saved to: {log_file_path}")

    # Summary analysis
    run_samples = [s for s in telemetry_samples if s[1] == args.pwm]
    if run_samples:
        avg_left = sum(s[2] for s in run_samples) / len(run_samples)
        avg_right = sum(s[3] for s in run_samples) / len(run_samples)
        max_left = max(s[2] for s in run_samples)
        max_right = max(s[3] for s in run_samples)
        print(f"\n=== Summary for P={args.pwm} ===")
        print(f"  Samples: {len(run_samples)}")
        print(f"  Left Motor  : Avg = {avg_left:.1f} steps/s, Max = {max_left:.1f} steps/s")
        print(f"  Right Motor : Avg = {avg_right:.1f} steps/s, Max = {max_right:.1f} steps/s")
    else:
        print("\nWarning: No telemetry samples captured matching target PWM.")


if __name__ == "__main__":
    main()
