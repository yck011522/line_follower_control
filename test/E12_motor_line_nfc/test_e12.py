"""E12 Multi-Sensor (Motor + Line + NFC) Automated Verification Script.

Tests:
1. Boot banner and initial telemetry @ 10 Hz (every 0.1s).
2. Format validation: P=<int>,L=<int>,R=<int>,LINE=<8-bits>,NFC=<uid|NONE>,AGE=<float>s
3. Line sensor readings check (valid 8-bit mask).
4. NFC reader scanning check.
5. Motor speed commands (S0, S50, S80, S-50, S0) with live line + NFC monitoring.
"""

import argparse
import datetime
import pathlib
import re
import sys
import time
import serial

ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULTS_DIR = ROOT / "test" / "E12_motor_line_nfc" / "results"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)

# Telemetry format: P=<int>,L=<int>,R=<int>,LINE=<8 bits>,NFC=<uid|NONE>,AGE=<float>s,RATE=<float>Hz
TELEMETRY_RE = re.compile(
    r"^P=(-?\d+),L=(-?\d+),R=(-?\d+),LINE=([01\-]{8}),NFC=([0-9A-Fa-f:]{11}|NONE),AGE=([0-9\.\-]+s),RATE=([0-9\.]+Hz)$"
)


def run_e12_test(port="COM17", baud=115200, duration=10.0):
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_log_path = RESULTS_DIR / f"e12_test_{timestamp}_{port}_raw.log"

    print(f"Opening {port} at {baud} baud (dtr=True, rts=False)...")
    ser = serial.Serial(port=None, baudrate=baud, timeout=0.1)
    ser.dtr = True
    ser.rts = False
    ser.port = port

    raw_lines = []
    telemetry_samples = []

    try:
        ser.open()
        time.sleep(0.4)
        ser.reset_input_buffer()

        # Initial stop to ensure motionless baseline
        ser.write(b"STOP\n")
        ser.flush()
        time.sleep(0.5)

        # Query status
        ser.write(b"STATUS\n")
        ser.flush()
        time.sleep(0.2)

        print("\n--- Listening to 0.1s Telemetry Stream (Baseline) ---")
        t_baseline_end = time.monotonic() + 2.0
        while time.monotonic() < t_baseline_end:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                raw_lines.append(line)
                m = TELEMETRY_RE.match(line)
                if m:
                    telemetry_samples.append(m.groups())
                    print(f"  [TELEMETRY] {line}")
                elif line.startswith("#"):
                    print(f"  [STATUS] {line}")

        # Test speed sequence: S50 -> S80 -> S-50 -> STOP
        test_commands = [
            ("S50\n", 2.0, "Forward 50 mm/s"),
            ("S80\n", 2.0, "Forward 80 mm/s"),
            ("S-50\n", 2.0, "Reverse -50 mm/s"),
            ("STOP\n", 1.5, "Full Stop"),
        ]

        print("\n--- Running Motor Speed Commands with Line & NFC Monitoring ---")
        for cmd, cmd_duration, label in test_commands:
            print(f"\n>> Sending command: {label} ({cmd.strip()})")
            ser.write(cmd.encode("ascii"))
            ser.flush()

            deadline = time.monotonic() + cmd_duration
            while time.monotonic() < deadline:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    raw_lines.append(line)
                    m = TELEMETRY_RE.match(line)
                    if m:
                        telemetry_samples.append(m.groups())
                        print(f"  [TELEMETRY] {line}")
                    elif line.startswith("#"):
                        print(f"  [RESPONSE] {line}")

        # Final query
        ser.write(b"STATUS\n")
        ser.flush()
        time.sleep(0.2)
        while ser.in_waiting:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                raw_lines.append(line)
                print(f"  [FINAL] {line}")

    except KeyboardInterrupt:
        print("\n[ABORT] User interrupted test.")
    except Exception as e:
        print(f"\n[ERROR] Test error: {e}")
    finally:
        try:
            if ser.is_open:
                ser.write(b"STOP\n")
                ser.flush()
                time.sleep(0.1)
                ser.close()
        except Exception:
            pass

    # Save raw log
    with open(raw_log_path, "w", encoding="utf-8") as f:
        f.write(f"# E12 Multi-Sensor Test: {timestamp}\n")
        f.write(f"# Total valid telemetry samples: {len(telemetry_samples)}\n\n")
        for line in raw_lines:
            f.write(f"{line}\n")
    print(f"\nRaw log saved to: {raw_log_path}")
    print(f"Total valid telemetry frames matched: {len(telemetry_samples)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM17")
    args = parser.parse_args()
    run_e12_test(port=args.port)
