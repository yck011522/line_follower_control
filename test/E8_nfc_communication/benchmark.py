"""Run one real-device asynchronous UID benchmark and retain its complete serial output."""
import argparse
import datetime
import hashlib
import json
import pathlib
import time

import serial

parser = argparse.ArgumentParser()
parser.add_argument("--port", default="COM9")
parser.add_argument("--duration-ms", type=int, default=5000)
parser.add_argument("--clock", type=int, default=400000)
parser.add_argument("--adaptive", type=int, choices=(0, 1), default=1)
parser.add_argument("--poll-us", type=int, default=710)
parser.add_argument("--timeout-us", type=int, default=1000)
parser.add_argument("--burst", type=int, choices=(0, 1), default=1)
parser.add_argument("--cache", type=int, choices=(0, 1), default=1)
parser.add_argument("--fast-tx", type=int, choices=(0, 1), default=1)
parser.add_argument("--heartbeat-us", type=int, default=10000)
parser.add_argument("--gain", type=int, choices=(4, 5, 6, 7), default=6)
parser.add_argument("--combined", type=int, choices=(0, 1), default=1)
parser.add_argument("--interval-us", type=int, default=0)
parser.add_argument("--cancel-after-ms", type=int, default=0)
parser.add_argument("--label", default="tag_present")
args = parser.parse_args()
if not 100 <= args.duration_ms <= 120000:
    parser.error("duration must be 100..120000 ms")
root = pathlib.Path(__file__).resolve().parents[2]
stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
folder = root / "src/e8_nfc_fm17550_i2c/results"
raw_folder = folder / "raw"
raw_folder.mkdir(parents=True, exist_ok=True)
path = raw_folder / f"run_{stamp}.log"
source_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in (root / "src/e8_nfc_fm17550_i2c").glob("*.cpp")}
source_hashes.update({"NfcAsync.h": hashlib.sha256((root / "src/e8_nfc_fm17550_i2c/NfcAsync.h").read_bytes()).hexdigest()})
firmware_hash = hashlib.sha256((root / ".pio/build/e8_nfc_fm17550_i2c/firmware.bin").read_bytes()).hexdigest()
snapshots = folder / "sources"
snapshots.mkdir(exist_ok=True)
for name, digest in source_hashes.items():
    snapshot = snapshots / f"{digest}_{name}.txt"
    if not snapshot.exists():
        snapshot.write_bytes((root / "src/e8_nfc_fm17550_i2c" / name).read_bytes())
command = (f"B {args.duration_ms} {args.clock} {args.adaptive} {args.poll_us} "
           f"{args.timeout_us} {args.burst} {args.cache} {args.heartbeat_us} {args.fast_tx} {args.gain} {args.combined} {args.interval_us}\n")
connection = serial.Serial(port=None, baudrate=115200, timeout=0.2)
connection.dtr = True
connection.rts = False
connection.port = args.port
received = bytearray()
complete = False
with path.open("xb") as log:
    log.write(b"CAPTURE_STATUS: incomplete until E8 END\n")
    log.write(("METADATA " + json.dumps({"label": args.label, "port": args.port,
              "command": command.strip(), "cancel_after_ms": args.cancel_after_ms,
              "source_hashes": source_hashes, "firmware_sha256": firmware_hash}) + "\n").encode())
    log.flush()
    try:
        connection.open()
        time.sleep(0.3)
        connection.write(command.encode())
        sent_at = time.monotonic()
        cancelled = False
        deadline = time.monotonic() + args.duration_ms / 1000 + 30
        while time.monotonic() < deadline:
            chunk = connection.read(4096)
            received.extend(chunk)
            log.write(chunk)
            log.flush()
            if args.cancel_after_ms and not cancelled and time.monotonic() - sent_at >= args.cancel_after_ms / 1000:
                connection.write(b"X\n")
                log.write(b"\nHOST: explicit cancellation sent\n")
                log.flush()
                cancelled = True
            if b"E8 END:" in received:
                complete = True
                break
        log.write(f"\nCAPTURE_STATUS: {'completed' if complete else 'incomplete'}\n".encode())
    finally:
        if connection.is_open and not complete:
            try:
                connection.write(b"X\n")
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    log.write(connection.read(4096))
                    log.flush()
            except serial.SerialException as error:
                log.write(f"\nCancellation failed: {error}\n".encode())
        connection.close()
for line in received.decode(errors="replace").splitlines():
    if line.startswith(("INIT", "SUMMARY", "OPTIONS", "E8 START", "E8 END")):
        print(line)
print(f"Raw log: {path}")
if not complete:
    raise SystemExit("Incomplete capture; raw log retained")
