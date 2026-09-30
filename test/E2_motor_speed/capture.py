"""Capture the fixed E2 I2C sweep through the ESP32 USB port, then analyze/plot it.

This never connects to the motor driver's UART and has no motor tuning options.
"""

import argparse
import json
import re
import time
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent


def open_esp32(name):
    import serial

    # Set lines while CLOSED: rtscts=False alone does not turn RTS off.
    port = serial.Serial(
        port=None,
        baudrate=115200,
        timeout=0.05,
        write_timeout=1,
        rtscts=False,
        dsrdtr=False,
        xonxoff=False,
    )
    port.dtr, port.rts = True, False
    port.port = name
    try:
        port.open()
    except BaseException:
        port.close()
        raise
    return port


class Lines:
    """Keep incomplete lines across reads and handshake/capture boundaries."""

    def __init__(self, port):
        self.port, self.buffer = port, b""

    def read(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.buffer += self.port.read(max(1, min(self.port.in_waiting, 4096)))
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                yield line.decode("ascii", "replace").strip()
            if len(self.buffer) > 8192:
                raise RuntimeError("Serial line exceeds 8192 bytes")


def send(port, command):
    data = (command + "\n").encode("ascii")
    if port.write(data) != len(data):
        raise RuntimeError("Incomplete ESP32 command write")


def handshake(port, reader, log):
    send(port, "!")
    # Cancel an older run and drain its END before looking for idle status.
    for line in reader.read(0.5):
        log.write(line + "\n")
    retried_config = False
    for attempt in range(12):
        send(port, "status")
        for line in reader.read(0.5):
            log.write(line + "\n")
            match = re.match(r"# STATUS configured=(\d) running=(\d) schema=2\b", line)
            if not match:
                continue
            if match.groups() == ("1", "0"):
                return
            if match[2] == "1":
                send(port, "!")
            elif not retried_config:
                send(
                    port, "config"
                )  # Retry the same fixed configuration, never tune it.
                retried_config = True
    raise RuntimeError(
        "No idle/configured E2 schema=2 status; check firmware, driver power and I2C"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", default="COM4", help="ESP32 USB port, not direct motor-driver USB"
    )
    parser.add_argument("--out", type=Path)
    parser.add_argument(
        "--range",
        nargs=3,
        type=int,
        metavar=("MIN", "MAX", "INC"),
        help="sweep range; the firmware default is -100 100 10",
    )
    args = parser.parse_args()
    folder = args.out or HERE / "results" / datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S_%fZ"
    )
    folder.mkdir(parents=True, exist_ok=False)
    meta = dict(
        started_utc=datetime.now(timezone.utc).isoformat(),
        port=args.port,
        environment="e2_motor_speed",
        schema=2,
        status="incomplete",
        dtr=True,
        rts=False,
        baud=115200,
        pid_assumed=[3, 0.375, 0.5],
        pid_source="driver_flash_not_verified_over_i2c",
        cleanup_errors=[],
    )
    port = reader = None
    started = ended = False
    with (folder / "serial.log").open("w", encoding="utf-8", buffering=1) as log:
        try:
            port = open_esp32(args.port)
            reader = Lines(port)
            handshake(port, reader, log)
            send(
                port, "start" + (" %d %d %d" % tuple(args.range) if args.range else "")
            )
            deadline = time.monotonic() + 120
            last_rx = time.monotonic()
            while not ended and time.monotonic() < deadline:
                for line in reader.read(0.2):
                    log.write(line + "\n")
                    last_rx = time.monotonic()
                    if line.startswith("# START "):
                        started = True
                    if line.startswith("# REFUSED"):
                        raise RuntimeError(line)
                    if started and line.startswith("# END "):
                        meta["end_record"] = line
                        ended = True
                        break
                if time.monotonic() - last_rx > 3:
                    raise RuntimeError("ESP32 stopped sending data for three seconds")
            if not ended:
                raise RuntimeError("Sweep completion timeout")
        except (Exception, KeyboardInterrupt) as error:
            meta["error"] = f"{type(error).__name__}: {error}"
            print(meta["error"])
        finally:
            if port:
                try:
                    send(port, "!")
                    # Read the cancellation response before closing. Keep it separate
                    # from the sweep's END record so it cannot disguise a failed run.
                    for line in reader.read(0.5):
                        log.write("# CLEANUP " + line + "\n")
                except (Exception, KeyboardInterrupt) as error:
                    meta["cleanup_errors"].append(str(error))
                finally:
                    try:
                        port.close()
                    except Exception as error:
                        meta["cleanup_errors"].append(str(error))
    if ended and "error" not in meta:
        try:
            from analyze import analyze
            from plot import plot_run

            if analyze(folder / "serial.log") and not meta["cleanup_errors"]:
                meta["status"] = "complete"
            plot_run(folder)
        except Exception as error:
            meta["analysis_error"] = f"{type(error).__name__}: {error}"
            meta["status"] = "incomplete"
            print(meta["analysis_error"])
    (folder / "metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"{meta['status']}: {folder}")
    return 0 if meta["status"] == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
