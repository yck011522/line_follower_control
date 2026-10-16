"""Bounded COM9 experiment: query flood without reads, close, reopen, RF cleanup."""
import datetime
import pathlib
import re
import time
import serial

root = pathlib.Path(__file__).resolve().parents[2]
path = root / "src/e9b_nfc_serial_monitor/results" / (datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + "_backpressure.log")
connection = serial.Serial(port=None, baudrate=115200, timeout=0.1, write_timeout=2)
connection.dtr = True
connection.rts = False
connection.port = "COM9"
received = bytearray()
complete = False
with path.open("xb") as log:
    log.write(b"CAPTURE_STATUS: incomplete; serial backpressure experiment\n")
    try:
        connection.open()
        # Query replies exercise output even when there is no tag underneath.
        connection.write(b"G\n" * 3000)
        log.write(b"HOST: 3000 gain queries; no host reads for 3 seconds\n")
        time.sleep(3)
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            chunk = connection.read(4096)
            received.extend(chunk)
            log.write(chunk)
            log.flush()
        connection.close()
        log.write(b"\nHOST: port closed for 6 seconds; no reset or RTS pulse\n")
        log.flush()
        time.sleep(6)
        connection.open()
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            chunk = connection.read(4096)
            received.extend(chunk)
            log.write(chunk)
            log.flush()
        reports = [line for line in received.decode(errors="replace").splitlines() if line.startswith("REPORT ")]
        assert len(reports) >= 2, "Missing reports after host resumed"
        assert any(int(re.search(r"serial_dropped=(\d+)", line)[1]) > 0 for line in reports), "No dropped output observed"
        assert all(int(re.search(r"polls=(\d+)", line)[1]) > 0 for line in reports), "Polling did not progress"
        complete = True
    finally:
        if not connection.is_open:
            connection.open()
        connection.write(b"X\n")
        cleanup = bytearray()
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            cleanup.extend(connection.read(4096))
        log.write(cleanup)
        stopped = b"E9B stopped; RF off=yes" in cleanup
        log.write(f"\nCAPTURE_STATUS: {'completed' if complete and stopped else 'incomplete'}\n".encode())
        # Leave the requested monitor running after the bounded experiment.
        connection.write(b"S\n")
        time.sleep(0.2)
        connection.close()
for line in reports:
    print(line)
print(f"RF off verified: {stopped}; monitoring resumed; raw log: {path}")
if not complete or not stopped:
    raise SystemExit("Incomplete hardware experiment; raw log retained")
