"""Retain a bounded real-hardware E9B monitor capture and verify RF-off cleanup."""
import datetime
import pathlib
import time
import serial

root = pathlib.Path(__file__).resolve().parents[2]
folder = root / "src/e9b_nfc_serial_monitor/results"
folder.mkdir(exist_ok=True)
path = folder / (datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".log")
connection = serial.Serial(port=None, baudrate=115200, timeout=0.1)
connection.dtr = True
connection.rts = False
connection.port = "COM9"
received = bytearray()
cleanup = bytearray()
complete = False
with path.open("xb") as log:
    log.write(b"CAPTURE_STATUS: incomplete; COM9, 12 seconds, tag-present test\n")
    try:
        connection.open()
        connection.write(b"S\n")
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            chunk = connection.read(4096)
            received.extend(chunk)
            log.write(chunk)
            log.flush()
        complete = True
    finally:
        if connection.is_open:
            connection.write(b"X\n")
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                chunk = connection.read(4096)
                cleanup.extend(chunk)
                log.write(chunk)
            connection.close()
        stopped = b"E9B stopped; RF off=yes" in cleanup
        log.write(f"\nCAPTURE_STATUS: {'completed' if complete and stopped else 'incomplete'}\n".encode())
lines = received.decode(errors="replace").splitlines()
uids = [line for line in lines if line.startswith("UID ")]
reports = [line for line in lines if line.startswith("REPORT ")]
print(f"UID lines: {len(uids)}; values: {sorted(set(uids))}")
for line in reports:
    print(line)
print(f"RF off confirmed: {stopped}; raw log: {path}")
if not complete or not stopped or len(reports) < 2 or set(uids) != {"UID 11:E5:99:23"}:
    raise SystemExit("Hardware capture did not meet expected tag-present checks")
