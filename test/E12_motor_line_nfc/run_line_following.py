"""E12 Table Line-Following Run and Observation Script.

Interactive script to command safe line-following test runs on the table,
log real-time telemetry (10 Hz), and analyze performance.

Usage:
    python test/E12_motor_line_nfc/run_line_following.py --port COM17 --speed 15 --timeout 8
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


def run_line_test(port="COM17", baud=115200, speed=15.0, timeout=10.0):
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_log_path = RESULTS_DIR / f"line_follow_S{int(speed)}_{timestamp}_{port}_raw.log"

    print("=" * 65)
    print(f"  E12 Line Following Test Run — Target Speed: S{speed} mm/s")
    print(f"  Port: {port} | Baud: {baud} | Safe Timeout: {timeout:.1f} s")
    print("=" * 65)

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

        # Send initial STOP to ensure calm starting state
        ser.write(b"STOP\n")
        ser.flush()
        time.sleep(0.3)

        # Baseline reading
        print("\n[STEP 1] Checking line sensor state before motion...")
        ser.write(b"STATUS\n")
        ser.flush()
        t_base = time.monotonic() + 0.8
        initial_mask = None
        while time.monotonic() < t_base:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                raw_lines.append(line)
                m = TELEMETRY_RE.match(line)
                if m:
                    initial_mask = m.group(4)
                    print(f"  Initial Telemetry: {line}")
                elif line.startswith("#"):
                    print(f"  {line}")

        if initial_mask in ("11111111", "00000000"):
            print(f"\n[NOTE] Line sensor mask is currently {initial_mask} (no line active).")
            print("Place the car directly over the black line before or right as motion starts.")

        print(f"\n[STEP 2] Commanding line-following: S{speed}")
        cmd_str = f"S{speed}\n"
        ser.write(cmd_str.encode("ascii"))
        ser.flush()

        deadline = time.monotonic() + timeout
        consecutive_stops = 0
        motion_started = False

        while time.monotonic() < deadline:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                raw_lines.append(line)
                m = TELEMETRY_RE.match(line)
                if m:
                    telemetry_samples.append((time.monotonic(),) + m.groups())
                    p_val = int(m.group(1))
                    l_spd = int(m.group(2))
                    r_spd = int(m.group(3))
                    line_bits = m.group(4)
                    nfc_uid = m.group(5)
                    rate_str = m.group(7)

                    # Check if line was detected and motion occurred
                    if line_bits not in ("11111111", "00000000"):
                        motion_started = True

                    # Detect if robot halted due to reaching end of line or losing line
                    if motion_started and line_bits in ("11111111", "00000000") and l_spd == 0 and r_spd == 0:
                        consecutive_stops += 1
                    else:
                        consecutive_stops = 0

                    print(f"  [TELEMETRY] P={p_val:+4d} | L={l_spd:+3d} mm/s | R={r_spd:+3d} mm/s | LINE={line_bits} | RATE={rate_str}")

                    # If car has stopped at end of line for 5 consecutive frames (0.5s), finish gracefully
                    if consecutive_stops >= 5:
                        print("\n[INFO] Car stopped at end of line / out-of-bounds as expected.")
                        break
                elif line.startswith("#"):
                    print(f"  [DEVICE] {line}")

        print("\n[STEP 3] Test duration elapsed or stop condition met. Stopping motors.")
        ser.write(b"STOP\n")
        ser.flush()
        time.sleep(0.3)

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
        print("\n[ABORT] User interrupted line-following test.")
    except Exception as e:
        print(f"\n[ERROR] Test run encountered error: {e}")
    finally:
        try:
            if ser.is_open:
                ser.write(b"STOP\n")
                ser.flush()
                time.sleep(0.1)
                ser.close()
        except Exception:
            pass

    # Save log
    with open(raw_log_path, "w", encoding="utf-8") as f:
        f.write(f"# E12 Line Following Run: S{speed} mm/s at {timestamp}\n")
        f.write(f"# Total valid telemetry frames: {len(telemetry_samples)}\n\n")
        for line in raw_lines:
            f.write(f"{line}\n")

    print(f"\nRaw test log saved to: {raw_log_path}")
    print(f"Collected {len(telemetry_samples)} telemetry samples.")
    print("=" * 65)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run E12 Line Following Table Test")
    parser.add_argument("--port", default="COM17", help="Serial port (default: COM17)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--speed", type=float, default=15.0, help="Target forward speed in mm/s (default: 15.0)")
    parser.add_argument("--timeout", type=float, default=10.0, help="Max run duration in seconds (default: 10.0)")
    args = parser.parse_args()

    run_line_test(port=args.port, baud=args.baud, speed=args.speed, timeout=args.timeout)
