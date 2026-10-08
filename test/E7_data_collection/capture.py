"""Discover E7 car 1 through the master radio and record manually pushed runs."""

import argparse
import csv
import json
import math
import os
import queue
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import serial


# Field order is the E7 version-1 radio protocol, documented with the firmware.
TELEMETRY_COLUMNS = [
    "message", "version", "car_id", "boot_id", "sample_sequence", "cycle_us",
    "line_read_offset_us", "line_raw_mask", "line_error",
    "left_count", "left_read_offset_us", "left_mm_s", "left_error",
    "right_count", "right_read_offset_us", "right_mm_s", "right_error",
    "motor_configured", "release_error", "acquisition_us", "skipped_slots",
    "tx_busy_skips", "tx_submission_errors", "tx_delivery_failures",
]


def open_master(port_name):
    """Set the required control-line levels before opening the master USB port."""
    port = serial.Serial(port=None, baudrate=115200, timeout=0.05,
                         write_timeout=1, rtscts=False, dsrdtr=False, xonxoff=False)
    port.dtr = True
    port.rts = False
    port.port = port_name
    try:
        port.open()
    except BaseException:
        port.close()
        raise
    return port


def read_enter_presses(enter_events):
    """Wait for console input separately so serial reception never waits on Enter."""
    while True:
        try:
            input()
        except EOFError:
            enter_events.put(None)
            return
        enter_events.put(time.monotonic_ns())


def parse_telemetry(raw_line):
    """Validate one complete E7 record; raw bytes are retained even on rejection."""
    fields = raw_line.decode("ascii").strip().split(",")
    if len(fields) != len(TELEMETRY_COLUMNS) or fields[:2] != ["E7", "1"]:
        raise ValueError("Not an E7 version-1 record")
    values = dict(zip(TELEMETRY_COLUMNS, fields))

    # Speeds can be nan on first/failed reads. Other numeric fields are integers.
    for name in TELEMETRY_COLUMNS[1:]:
        if name == "boot_id":
            if len(values[name]) != 8:
                raise ValueError("Invalid boot ID")
            int(values[name], 16)
        elif name in ("left_mm_s", "right_mm_s"):
            values[name] = float(values[name])
            if math.isinf(values[name]):
                raise ValueError("Infinite speed")
        else:
            values[name] = int(values[name])

    # Explicit status values distinguish missing data from an all-white mask or
    # a stationary encoder. Leave failed numeric placeholders blank in the CSV.
    if values["line_error"]:
        values["line_raw_mask"] = ""
    elif not 0 <= values["line_raw_mask"] <= 255:
        raise ValueError("Invalid line mask")
    for wheel in ("left", "right"):
        if values[f"{wheel}_error"]:
            values[f"{wheel}_count"] = ""
        if values[f"{wheel}_error"] or math.isnan(values[f"{wheel}_mm_s"]):
            values[f"{wheel}_mm_s"] = ""
    return values


def save_metadata(path, metadata):
    """Replace the metadata atomically, keeping incomplete status during capture."""
    temporary_path = path.with_suffix(".tmp")
    with temporary_path.open("w", encoding="utf-8") as output:
        json.dump(metadata, output, indent=2)
        output.write("\n")
        output.flush()
        os.fsync(output.fileno())
    temporary_path.replace(path)


def main():
    """Drain the master continuously, mark Enter locally, and finalize on Ctrl-C."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="USB port of the master radio")
    parser.add_argument("--map", required=True, dest="map_id", help="Map name/version")
    parser.add_argument("--route", default="", help="Intended path, including intersection choices")
    parser.add_argument("--notes", default="")
    parser.add_argument("--car-id", type=int, default=1)
    parser.add_argument("--max-seconds", type=float, default=0,
                        help="Optional recording limit; 0 records until Ctrl-C")
    parser.add_argument("--out", type=Path, default=Path(__file__).parent / "results")
    arguments = parser.parse_args()
    if arguments.max_seconds < 0:
        parser.error("--max-seconds must be nonnegative")

    # A new directory per invocation preserves all previous and interrupted runs.
    run_name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    run_directory = arguments.out / run_name
    run_directory.mkdir(parents=True, exist_ok=False)
    metadata_path = run_directory / "metadata.json"
    metadata = {
        "status": "incomplete", "created_utc": datetime.now(timezone.utc).isoformat(),
        "map_id": arguments.map_id, "route": arguments.route, "notes": arguments.notes,
        "car_id": arguments.car_id, "port": arguments.port, "protocol": "E7,1",
        "sample_rate_hz": 100, "counts_per_wheel_revolution": 44998,
        "wheel_diameter_mm": 65, "speed_sign": "driver electrical sign, not inverted",
        "speed_source": "cumulative count differences / actual per-wheel elapsed time",
        "motor_settings": {"type": 1, "dead_zone": 1650, "pulse_line": 500, "ratio": 23},
        "pid": {"p": 3, "i": 0.375, "d": 0.5, "status": "stored_unverified"},
        "max_seconds": arguments.max_seconds, "rows": 0, "boot_ids": [],
        "missing_sample_sequences": 0, "duplicate_or_out_of_order": 0,
        "malformed_lines": 0, "discovery_packets": 0,
    }
    save_metadata(metadata_path, metadata)

    port = None
    recording_started_ns = None
    last_sample_key = None
    last_received_ns = 0
    last_healthy_ns = 0
    next_discovery_ns = 0
    next_flush_ns = 0
    rate_window_started_ns = time.monotonic_ns()
    rate_window_packets = 0
    pending_bytes = bytearray()
    enter_events = queue.SimpleQueue()
    exit_code = 0

    # Raw bytes include pre-Enter discovery and partial final lines. CSV contains
    # only recording rows. Both files exist immediately, even for failed runs.
    with (run_directory / "raw_serial.bin").open("xb") as raw_output, \
            (run_directory / "samples.csv").open("x", newline="", encoding="utf-8") as csv_output:
        writer = csv.DictWriter(csv_output, fieldnames=[
            "host_receive_utc", "host_elapsed_s", "sequence_gap", *TELEMETRY_COLUMNS])
        writer.writeheader()
        try:
            port = open_master(arguments.port)
            print(f"Saving to {run_directory}")
            print("Discovering car. Press Enter when READY and you want to start pushing; Ctrl-C ends recording.")
            threading.Thread(target=read_enter_presses, args=(enter_events,), daemon=True).start()

            while True:
                now_ns = time.monotonic_ns()
                # Repeat discovery until telemetry arrives; try again after a
                # dropout or car reboot. These packets never request movement.
                if now_ns - last_received_ns > 2_000_000_000 and now_ns >= next_discovery_ns:
                    if port.write(b"W,0\n") != 4:
                        raise RuntimeError("Incomplete discovery write")
                    metadata["discovery_packets"] += 1
                    next_discovery_ns = now_ns + 500_000_000

                # Enter is a local timestamp marker, not a radio start command.
                # Require recently healthy data before calling a recording ready.
                while not enter_events.empty():
                    pressed_at_ns = enter_events.get()
                    if pressed_at_ns is None:
                        raise RuntimeError("Console input closed")
                    if recording_started_ns is None:
                        if not last_healthy_ns or now_ns - last_healthy_ns > 1_000_000_000:
                            print("Not ready: need recent sensor/encoder data and acknowledged motor release. Press Enter again when READY.")
                        else:
                            recording_started_ns = pressed_at_ns
                            last_sample_key = None
                            metadata["recording_started_utc"] = datetime.now(timezone.utc).isoformat()
                            metadata["recording_start_monotonic_ns"] = pressed_at_ns
                            save_metadata(metadata_path, metadata)
                            print("RECORDING — start pushing. Ctrl-C finishes and saves.")

                # Read bounded chunks; pySerial timeout permits prompt Ctrl-C and
                # periodic flushing even when the radio stops delivering bytes.
                received = port.read(max(1, min(port.in_waiting, 4096)))
                received_at_ns = time.monotonic_ns()
                received_utc = datetime.now(timezone.utc).isoformat()
                raw_output.write(received)
                pending_bytes.extend(received)
                while b"\n" in pending_bytes:
                    raw_line, _, remainder = pending_bytes.partition(b"\n")
                    pending_bytes = bytearray(remainder)
                    try:
                        values = parse_telemetry(raw_line)
                    except (ValueError, UnicodeError):
                        metadata["malformed_lines"] += 1
                        continue
                    if values["car_id"] != arguments.car_id:
                        continue

                    last_received_ns = received_at_ns
                    rate_window_packets += 1
                    if values["motor_configured"] == 1 and not any(values[name] for name in
                            ("line_error", "left_error", "right_error", "release_error")):
                        last_healthy_ns = received_at_ns
                    if recording_started_ns is None or received_at_ns < recording_started_ns:
                        continue

                    # Sequence gaps count missing acquisition records, not a
                    # claim of radio-only loss. Firmware separately counts missed slots.
                    sequence_gap = 0
                    sample_key = (values["boot_id"], values["sample_sequence"])
                    if last_sample_key and last_sample_key[0] == sample_key[0]:
                        step = (sample_key[1] - last_sample_key[1]) & 0xFFFFFFFF
                        if 0 < step < 0x80000000:
                            sequence_gap = step - 1
                            last_sample_key = sample_key
                        else:
                            metadata["duplicate_or_out_of_order"] += 1
                    else:
                        last_sample_key = sample_key
                    metadata["missing_sample_sequences"] += sequence_gap
                    if sample_key[0] not in metadata["boot_ids"]:
                        metadata["boot_ids"].append(sample_key[0])

                    writer.writerow({"host_receive_utc": received_utc,
                                     "host_elapsed_s": f"{(received_at_ns - recording_started_ns) / 1e9:.6f}",
                                     "sequence_gap": sequence_gap, **values})
                    metadata["rows"] += 1

                # Retain malformed/partial input in raw_serial.bin; reject a
                # runaway unterminated stream rather than growing memory forever.
                if len(pending_bytes) > 8192:
                    raise RuntimeError("Master serial line exceeds 8192 bytes")
                if received_at_ns >= next_flush_ns:
                    for output in (raw_output, csv_output):
                        output.flush()
                        os.fsync(output.fileno())
                    save_metadata(metadata_path, metadata)
                    next_flush_ns = received_at_ns + 1_000_000_000

                    # Host arrival rate is diagnostic only; analysis uses ESP32
                    # timestamps because USB can deliver several samples in a burst.
                    elapsed_s = (received_at_ns - rate_window_started_ns) / 1e9
                    ready = last_healthy_ns and received_at_ns - last_healthy_ns < 1_000_000_000
                    state = "RECORDING" if recording_started_ns is not None else ("READY" if ready else "WAITING")
                    print(f"{state}: {rate_window_packets / max(elapsed_s, 0.001):.1f} packets/s, "
                          f"rows={metadata['rows']}, missing={metadata['missing_sample_sequences']}")
                    rate_window_started_ns = received_at_ns
                    rate_window_packets = 0

                if recording_started_ns is not None and arguments.max_seconds and \
                        received_at_ns - recording_started_ns >= arguments.max_seconds * 1e9:
                    metadata["status"] = "complete"
                    metadata["stop_reason"] = "duration_limit"
                    break
        except KeyboardInterrupt:
            metadata["status"] = "complete" if recording_started_ns is not None else "cancelled_before_start"
            metadata["stop_reason"] = "ctrl_c"
        except Exception as error:
            metadata["stop_reason"] = str(error)
            exit_code = 1
            print(f"Recording error: {error}")
        finally:
            # Closing the master needs no robot stop command: E7 never powers the
            # motors. Do not pulse RTS during cleanup or discard buffered raw data.
            if port is not None:
                port.close()
            for output in (raw_output, csv_output):
                output.flush()
                os.fsync(output.fileno())
            metadata["ended_utc"] = datetime.now(timezone.utc).isoformat()
            save_metadata(metadata_path, metadata)
    print(f"Saved {metadata['rows']} rows ({metadata['status']}) to {run_directory}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
