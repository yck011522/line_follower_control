"""Capture one bounded E8 NFC test without pulsing the ESP32 reset lines."""
import datetime
import hashlib
import pathlib
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM9"
command = sys.argv[2].upper() if len(sys.argv) > 2 else "S"
if command not in {"S", "R"}:
    raise SystemExit("Usage: capture.py [COM9] [S|R]")
stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
path = pathlib.Path(__file__).parent / f"run_{stamp}_{port}.log"
source = pathlib.Path(__file__).resolve().parents[2] / "src/e8_nfc_fm17550_i2c/main.cpp"
source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
connection = serial.Serial(port=None, baudrate=115200, timeout=0.2)
connection.dtr = True
connection.rts = False
connection.port = port
with path.open("xb") as log:
    log.write(b"STATUS: incomplete until capture ends\n")
    log.write(f"PORT: {port}; command: {command}; source SHA256: {source_hash}\n".encode())
    log.flush()
    received = bytearray()
    try:
        connection.open()
        time.sleep(0.5)
        connection.write((command + "\n").encode())
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            chunk = connection.read(4096)
            received.extend(chunk)
            log.write(chunk)
            log.flush()
            print(chunk.decode("utf-8", errors="replace"), end="", flush=True)
        status = "completed" if b"E8 END:" in received else "incomplete (no E8 END received)"
        log.write(f"\nSTATUS: {status}; see E8 END for hardware result\n".encode())
    finally:
        if connection.is_open and b"E8 END:" not in received:
            try:
                connection.write(b"X\n")
                # Retain late cleanup messages after cancellation without pulsing RTS.
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    log.write(connection.read(4096))
                    log.flush()
            except serial.SerialException as error:
                log.write(f"\nCancellation failed: {error}\n".encode())
        connection.close()
print(f"Raw log: {path}")
