"""Capture serial status lines from firmware E10 on real hardware and validate timing/formatting."""
import argparse
import datetime
import pathlib
import re
import time
import serial

ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULTS_FOLDER = ROOT / "src/e10_line_nfc_control/results"
RESULTS_FOLDER.mkdir(exist_ok=True)

STATUS_PATTERN = re.compile(
    r"^POS:\s+([01]{8}|[-]{8})\s+\|\s+UID:\s+([0-9A-Fa-f:]{11}|\s{11})\s+\|\s+AGE:\s+(\s*\d+\.\d)s\s+\|\s+RATE:\s+(\s*\d+\.\d)\s+Hz(?:\s+\|\s+ERR:(.*))?$"
)

def run_capture(port_name="COM12", duration_s=10.0):
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = RESULTS_FOLDER / f"{timestamp_str}_{port_name}.log"

    connection = serial.Serial(port=None, baudrate=115200, timeout=0.1)
    connection.dtr = True
    connection.rts = False
    connection.port = port_name

    raw_bytes = bytearray()
    complete = False

    print(f"Opening {port_name} (dtr=True, rts=False) for {duration_s}s capture...")
    with log_path.open("xb") as log_file:
        log_file.write(f"CAPTURE_STATUS: incomplete; port={port_name}, duration={duration_s}s\n".encode())
        try:
            connection.open()
            # Drain any old pending bytes
            time.sleep(0.1)
            connection.reset_input_buffer()

            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                chunk = connection.read(4096)
                if chunk:
                    raw_bytes.extend(chunk)
                    log_file.write(chunk)
                    log_file.flush()
            complete = True
        finally:
            if connection.is_open:
                connection.close()
            log_file.write(f"\nCAPTURE_STATUS: {'completed' if complete else 'incomplete'}\n".encode())

    # Decode and analyze captured lines
    decoded_text = raw_bytes.decode("ascii", errors="replace")
    lines = [line.strip() for line in decoded_text.splitlines() if line.strip()]

    status_lines = [l for l in lines if l.startswith("POS: ")]
    debug_lines = [l for l in lines if not l.startswith("POS: ")]

    print(f"Capture finished. Total status lines: {len(status_lines)}, Debug/other lines: {len(debug_lines)}")
    print(f"Log written to: {log_path}")

    if debug_lines:
        print("\nStartup/Debug lines:")
        for dl in debug_lines[:10]:
            print("  ", dl)

    # Validate status lines
    if not status_lines:
        raise RuntimeError("No status lines captured from E10 firmware!")

    print("\nSample captured status lines:")
    for sl in status_lines[:5]:
        print("  ", sl)
    print("   ...")
    for sl in status_lines[-3:]:
        print("  ", sl)

    # Check line lengths and formatting
    base_lengths = set()
    rates = []
    ages = []
    uids = set()
    positions = set()
    errors_detected = 0

    for line in status_lines:
        match = STATUS_PATTERN.match(line)
        if not match:
            raise ValueError(f"Status line did not match expected E10 regex: {line}")
        pos, uid, age, rate, err = match.groups()
        positions.add(pos)
        if uid.strip():
            uids.add(uid)
        ages.append(float(age))
        rates.append(float(rate))
        if err:
            errors_detected += 1

        # Calculate base length without appended errors
        base_part = line.split(" | ERR:")[0]
        base_lengths.add(len(base_part))

    print(f"\nValidation Summary:")
    print(f"  Base status line lengths: {base_lengths}")
    print(f"  Unique Line positions seen: {sorted(positions)}")
    print(f"  Unique NFC UIDs detected: {sorted(uids)}")
    print(f"  Age range: {min(ages):.1f}s .. {max(ages):.1f}s")
    print(f"  Actual control loop rates: min={min(rates):.1f} Hz, avg={sum(rates)/len(rates):.1f} Hz, max={max(rates):.1f} Hz")
    print(f"  Lines with reading errors: {errors_detected} / {len(status_lines)}")
    print(f"  Achieved status report frequency: {len(status_lines) / duration_s:.1f} lines/sec (target: 10 Hz)")

    # Assertions
    assert len(base_lengths) == 1, f"Expected uniform base message length, got {base_lengths}"
    avg_rate = sum(rates) / len(rates)
    assert 95.0 <= avg_rate <= 105.0, f"Expected control loop rate near 100 Hz, got {avg_rate}"
    achieved_report_rate = len(status_lines) / duration_s
    assert 9.0 <= achieved_report_rate <= 11.0, f"Expected status reporting rate near 10 Hz, got {achieved_report_rate}"

    print("\n[SUCCESS] E10 real-hardware verification passed!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture E10 firmware output on COM port.")
    parser.add_argument("--port", default="COM12", help="Serial COM port (default: COM12)")
    parser.add_argument("--duration", type=float, default=10.0, help="Duration in seconds (default: 10.0)")
    args = parser.parse_args()
    run_capture(port_name=args.port, duration_s=args.duration)
