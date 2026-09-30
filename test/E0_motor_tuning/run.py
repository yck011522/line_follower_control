"""Test one motor PID candidate through the motor driver's USB serial port.

No ESP32 or PlatformIO is needed. Run with, for example:
    python run.py --port COM5 --pid 3 3 0.5

The experiment checks the stored motor settings, writes the requested PID,
waits for the board to restart, then tests each speed from zero. Speed feedback
is recorded before, during, and after each step, and plotted at the end.

Reading guide:
    Recorder handles serial commands and saves incoming measurements.
    plot() turns a saved run into a graph and a short statistics report.
    main() runs the experiment in order and handles stopping and cleanup.
"""

import argparse
import csv
import json
import math
import re
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path

# Keep results next to this script, regardless of the terminal's working folder.
HERE = Path(__file__).resolve().parent

# Patterns for signed numbers and a complete four-channel speed message,
# for example: $MSPD:0.00,22.20,0.00,0.00#
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"
SPEED = re.compile(r"\$MSPD:(" + NUMBER + r"(?:," + NUMBER + r"){3})#", re.I)

# These are the user's established board settings. We check them, not rewrite
# them. Despite its name, the board's Pulse_Phase field is the gear ratio.
EXPECTED = {"Motor_type": 1, "Dead_Zone": 1650, "Pulse_Line": 2000,
            "Pulse_Phase": 23, "wheel_diameter": 65}


def open_driver(name):
    """Open with the same control-line settings as the working serial monitor."""
    import serial

    # port=None constructs a CLOSED port. Setting RTS after opening would first
    # expose the board to pySerial's default RTS=True, potentially affecting boot.
    # Flow-control switches (rtscts/dsrdtr) do not specify these line levels.
    port = serial.Serial(port=None, baudrate=115200, timeout=0.02, write_timeout=1,
                         rtscts=False, dsrdtr=False, xonxoff=False)
    port.dtr = True   # Monitor: DTR checked.
    port.rts = False  # Monitor: RTS unchecked. Keep it off during PID resets too.
    port.port = name
    try:
        port.open()
    except BaseException:
        port.close()
        raise
    return port


class Recorder:
    """Send commands and label each feedback sample with its trial and phase."""

    def __init__(self, port, folder):
        # A monotonic clock measures elapsed time without being affected by
        # adjustments to the PC's date/time while the experiment is running.
        self.port = port
        self.start = time.monotonic()
        self.buffer = ""  # Unfinished speed packets, carried into the next read.
        self.text = ""  # Recent reply text, also used to parse read_flash output.
        self.rows = []
        self.events = []
        # main() updates these labels as it moves through the experiment.
        # Trial zero means setup, before the numbered speed tests begin.
        self.trial = 0
        self.phase = "setup"
        self.target = 0
        self.last_sample = None
        # Keep both the original traffic and a convenient table of measurements.
        # Line-buffer the traffic log so completed log lines are saved promptly.
        self.log = (folder / "serial.log").open("w", encoding="utf-8", buffering=1)
        self.file = (folder / "samples.csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=[
            "time_s", "trial", "phase", "target", "m1", "m2", "m3", "m4"])
        self.writer.writeheader()

    def send(self, command):
        """Send one ASCII command and return its host-side start time in seconds."""
        elapsed = time.monotonic() - self.start
        self.log.write(f"{elapsed:.6f} TX {command}\n")
        data = command.encode("ascii")
        if self.port.write(data) != len(data):
            raise RuntimeError("Incomplete serial write")
        # This records a successful host write, not an acknowledgment from the
        # driver: speed commands do not return an acknowledgment in this protocol.
        self.events.append({"time_s": elapsed, "command": command,
                            "trial": self.trial, "phase": self.phase})
        return elapsed

    def speed(self, value, motor):
        """Command M1, M2, M3, or M4; send zero to the other three channels."""
        self.target = value
        values = [0, 0, 0, 0]
        # Motor labels start at 1; Python list positions start at 0.
        values[motor - 1] = value
        return self.send("$spd:" + ",".join(map(str, values)) + "#")

    def feed(self, chunk):
        """Log received bytes and extract any complete speed packets from them."""
        # This is when the PC receives the data, not when the driver measured it.
        # Packets delivered in the same read therefore share a timestamp.
        now = time.monotonic()
        elapsed = now - self.start
        decoded = chunk.decode("ascii", errors="replace")
        self.log.write(f"{elapsed:.6f} RX {decoded!r}\n")
        # Keep enough recent text for flash replies without growing indefinitely.
        self.text = (self.text + decoded)[-32768:]
        self.buffer += decoded
        # Serial reads need not align with messages: one read might contain half
        # a packet or several packets. '#' ends a packet; a newline is optional.
        while "#" in self.buffer:
            packet, self.buffer = self.buffer.split("#", 1)
            match = SPEED.search(packet + "#")
            if match:
                values = list(map(float, match[1].split(",")))
                if not all(math.isfinite(v) for v in values):
                    continue
                row = dict(time_s=elapsed, trial=self.trial,
                           phase=self.phase, target=self.target,
                           **dict(zip(["m1", "m2", "m3", "m4"], values)))
                self.rows.append(row)
                self.writer.writerow(row)
                # Preserve partial measurements if a later trial is interrupted.
                self.file.flush()
                self.last_sample = now
        self.buffer = self.buffer[-4096:]  # Bound memory if incoming text has no '#'.

    def collect(self, seconds, require_speed=False):
        """Read and save serial data for a fixed duration, optionally checking feedback."""
        end = time.monotonic() + seconds
        initial = time.monotonic()
        while time.monotonic() < end:
            # Read available bytes promptly. If none are waiting, read one byte
            # with the port's short timeout so we can keep checking the deadline.
            chunk = self.port.read(max(1, min(self.port.in_waiting, 4096)))
            if chunk:
                self.feed(chunk)
            if require_speed and time.monotonic() - (self.last_sample or initial) > 1:
                # Raising here lets main() attempt to stop the motors. Setup and
                # reset waits leave this check off because upload is not active.
                raise RuntimeError("No valid MSPD telemetry for one second")

    def flash(self):
        """Request stored settings, retrying until all configuration and PID fields arrive."""
        # After a reset the driver may not answer immediately. A complete reply
        # is our readiness check; a fixed delay alone would not establish that.
        received = False
        for _ in range(8):
            self.text = ""
            self.send("$read_flash#")
            self.collect(0.75)
            received = received or bool(self.text)
            values = {}
            # Parse labels instead of relying on line order or tab spacing.
            for key in [*EXPECTED, "P", "I", "D"]:
                match = re.search(r"\b" + key + r"\s*:\s*(" + NUMBER + ")", self.text, re.I)
                if match:
                    values[key] = float(match[1])
            if len(values) == len(EXPECTED) + 3:
                return values
        detail = "received text, but required fields were missing" if received else "no received bytes"
        raise RuntimeError(f"Timed out reading complete flash configuration ({detail})")

    def drain_output(self, timeout=1):
        """Wait for queued TX bytes without an unbounded serial.flush() call."""
        deadline = time.monotonic() + timeout
        while self.port.out_waiting:
            if time.monotonic() >= deadline:
                raise RuntimeError("Timed out draining serial output before close")
            self.collect(0.02)

    def idle(self):
        """Stop, release, and disable upload; return any errors after trying all three."""
        errors = []
        for command in ["$spd:0,0,0,0#", "$pwm:0,0,0,0#", "$upload:0,0,0#"]:
            try:
                self.send(command)
                # Give the board time to process each command and keep reading.
                # Previously all three were queued in the same millisecond.
                self.drain_output()
                self.collect(0.15)
            except (Exception, KeyboardInterrupt) as error:
                errors.append(f"{command}: {type(error).__name__}: {error}")
        return errors

    def close(self):
        """Close the output files; main() separately closes the serial port."""
        self.file.close()
        self.log.close()


def check_connections(name, count, folder):
    """Repeatedly open/read/close; send only read_flash, with no PID or motor writes."""
    report = {"port": name, "dtr": True, "rts": False, "status": "incomplete",
              "requested_connections": count, "connections": []}
    try:
        for index in range(1, count + 1):
            connection = {"index": index}
            report["connections"].append(connection)
            path = folder / f"connection_{index}"
            path.mkdir()
            port = open_driver(name)
            rec = None
            try:
                rec = Recorder(port, path)
                rec.collect(0.5)
                connection["flash"] = rec.flash()
                rec.drain_output()
                rec.collect(0.15)
                print(f"Connection {index}/{count}: flash read OK (DTR on, RTS off)", flush=True)
            finally:
                # Do not manually toggle DTR/RTS on exit. The OS/USB driver still
                # controls what happens to the physical lines when the handle closes.
                try:
                    port.close()
                finally:
                    if rec:
                        rec.close()
            time.sleep(0.25)
        report["status"] = "complete"
    except (Exception, KeyboardInterrupt) as error:
        report["error"] = f"{type(error).__name__}: {error}"
        print(report["error"], flush=True)
    (folder / "connection_check.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"Connection check {report['status']}: {folder}")
    return 0 if report["status"] == "complete" else 1


def plot(folder):
    """Read saved files and create speed.png and summary.md without using hardware."""
    import matplotlib
    # Render straight to a PNG, without requiring an interactive plot window.
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    meta = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
    with (folder / "samples.csv").open(encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    trials = meta["trials"]
    if not trials:
        return
    # One panel per target speed makes each separate zero-to-speed test readable.
    fig, axes = plt.subplots(len(trials), 1, figsize=(11, 3.3 * len(trials)), squeeze=False)
    summaries = []
    key = f"m{meta['motor']}"
    for ax, trial in zip(axes[:, 0], trials):
        data = [r for r in rows if int(r["trial"]) == trial["trial"]]
        # Put the nonzero command at t=0 in each panel. Negative times show the
        # baseline, and positive times show acceleration and the return to zero.
        origin = trial["step_time_s"]
        x = [float(r["time_s"]) - origin for r in data]
        y = [float(r[key]) for r in data]
        ax.plot(x, y, ".-", markersize=3, linewidth=1, label=f"M{meta['motor']} feedback")
        stop = trial.get("stop_time_s")
        if x:
            if stop is None:
                # An interrupted step may have no recorded normal stop time.
                # Do not invent a return-to-zero time for its command trace.
                command_x, command_y = [min(x), 0, max(0, max(x))], [0, trial["speed"], trial["speed"]]
            else:
                stop_x = stop - origin
                command_x = [min(x), 0, stop_x, max(max(x), stop_x)]
                command_y = [0, trial["speed"], 0, 0]
            ax.step(command_x, command_y, where="post", linestyle="--", label="Command")
        ax.axvline(0, color="gray", linewidth=0.8)
        ax.set(xlabel="Seconds from speed command (host arrival timestamps)",
               ylabel="Driver speed units", title=f"Trial {trial['trial']}: 0 to {trial['speed']} to 0")
        ax.grid(alpha=0.3)
        ax.legend()
        # Summarize only the final second of the planned nonzero-speed interval.
        # These values describe tracking error and variation, but do not prove
        # that the motor settled or identify the cause of any oscillation.
        tail = [float(r[key]) for r in data if r["phase"] == "step"
                and float(r["time_s"]) - origin >= meta["duration"] - 1]
        if tail:
            summaries.append(f"| {trial['speed']} | {len(tail)} | {statistics.mean(tail):.2f} | "
                             f"{statistics.mean(tail) - trial['speed']:.2f} | "
                             f"{statistics.pstdev(tail):.2f} | {max(tail)-min(tail):.2f} |")
    fig.suptitle(f"E0: M{meta['motor']} | PID {meta['pid']} | {meta['status']}")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(folder / "speed.png", dpi=160)
    plt.close(fig)
    (folder / "summary.md").write_text(
        f"# E0 motor tuning\n\nStatus: {meta['status']}\n\nPID: {meta['pid']}\n\n"
        "Final one second of each step (not proof of settling):\n\n"
        "| Target | Samples | Mean | Mean error | Std dev | Peak-to-peak |\n"
        "| --- | --- | --- | --- | --- | --- |\n" + "\n".join(summaries) +
        "\n\nTimes are host receive times; USB buffering and the upload rate limit temporal resolution. "
        "Speed units are the driver's reported scale, not independently calibrated mm/s. "
        "Inspect the raw trace for oscillation and ramp-up; no automatic PID recommendation is made.\n",
        encoding="utf-8")


def main():
    """Parse options, run one PID candidate, stop the outputs, and save results."""
    # 1. Read test settings from the command line. These defaults reproduce the
    # requested M2 test: 20/40/60/80, with three seconds at each target speed.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--pid", type=float, nargs=3, metavar=("P", "I", "D"))
    parser.add_argument("--motor", type=int, choices=range(1, 5), default=2)
    parser.add_argument("--speeds", type=int, nargs="+", default=[20, 40, 60, 80])
    parser.add_argument("--duration", type=float, default=3)
    parser.add_argument("--baseline", type=float, default=1)
    parser.add_argument("--rest", type=float, default=2)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--plot", type=Path, help="Replot an existing run; does not open serial")
    parser.add_argument("--check-port", type=int, metavar="N",
                        help="Open/read flash/close N times; no motor or PID commands")
    args = parser.parse_args()
    # Replotting is an offline operation: it needs neither PID arguments nor a
    # serial connection, so handle it before validating live-test options.
    if args.plot:
        if args.check_port is not None:
            parser.error("choose --plot or --check-port, not both")
        plot(args.plot)
        return 0
    if args.check_port is not None:
        if args.check_port < 1 or args.pid is not None:
            parser.error("--check-port requires N >= 1 and must not be combined with --pid")
        folder = args.out or HERE / "results" / datetime.now(timezone.utc).strftime("check_%Y%m%dT%H%M%S_%fZ")
        folder.mkdir(parents=True, exist_ok=False)
        return check_connections(args.port, args.check_port, folder)
    # Reject invalid settings before opening the board or sending any commands.
    if args.pid is None or any(not math.isfinite(v) or v < 0 for v in args.pid):
        parser.error("--pid P I D is required, with finite nonnegative gains")
    if any(not math.isfinite(v) or v < 1 for v in (args.duration, args.baseline, args.rest)):
        parser.error("duration, baseline and rest must each be finite and at least 1 second")
    if any(v == 0 or abs(v) > 1000 for v in args.speeds):
        parser.error("test speeds must be nonzero integers in [-1000, 1000]")
    # 2. Create a new results folder. Refuse to overwrite an existing run.
    folder = args.out or HERE / "results" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    folder.mkdir(parents=True, exist_ok=False)
    # Start as incomplete; only a fully finished sweep becomes complete.
    meta = {"started_utc": datetime.now(timezone.utc).isoformat(),
            "port": args.port, "baud": 115200, "motor": args.motor, "pid": args.pid,
            "speeds": args.speeds, "duration": args.duration, "baseline": args.baseline,
            "rest": args.rest, "expected_configuration": EXPECTED,
            "serial_settings": {"dtr": True, "rts": False, "flow_control": "none"},
            "status": "incomplete", "trials": [], "cleanup_errors": []}
    port = None
    rec = None
    try:
        # 3. Open the driver's serial port (default 8N1, with flow control off).
        # Short read and bounded write timeouts keep communication waits bounded.
        port = open_driver(args.port)
        rec = Recorder(port, folder)
        # Establish an idle starting state: zero speed, release PID output with
        # zero PWM, then disable any telemetry left enabled by an earlier session.
        startup_errors = rec.idle()
        if startup_errors:
            raise RuntimeError(f"Could not establish idle state: {startup_errors}")
        rec.collect(1)
        # Verify scaling/configuration before changing PID or starting a sweep.
        before = rec.flash()
        meta["flash_before"] = before
        mismatches = {k: before[k] for k, expected in EXPECTED.items() if before[k] != expected}
        if mismatches:
            raise RuntimeError(f"Stored motor configuration differs from expected: {mismatches}")
        # 4. Writing PID saves the gains in flash and resets the board. Retain the
        # previous settings in metadata, and verify the new settings after reset.
        rec.send("$MPID:" + ",".join(f"{v:g}" for v in args.pid) + "#")
        rec.collect(2)  # Reset grace period, followed by an actual readback handshake.
        after = rec.flash()
        meta["flash_after"] = after
        # The observed flash output prints three decimal places, so allow about
        # half of the last printed digit when comparing requested and read gains.
        if any(after[k] != v for k, v in EXPECTED.items()) or any(
                not math.isclose(after[k], v, abs_tol=0.00051, rel_tol=0)
                for k, v in zip(["P", "I", "D"], args.pid)):
            raise RuntimeError("Post-reset configuration/PID readback does not match")
        # 5. Enable speed upload while the command is still zero. Require feedback
        # before moving so each step can be compared with a recorded baseline.
        rec.speed(0, args.motor)
        rec.collect(0.15)
        rec.send("$upload:0,0,1#")
        rec.collect(1, require_speed=True)
        if rec.last_sample is None:
            raise RuntimeError("No speed feedback; refusing to start sweep")
        # 6. Repeat baseline -> speed step -> zero-speed rest for each target.
        # Keep logging during every phase, including any oscillation around zero.
        for index, speed in enumerate(args.speeds, 1):
            print(f"Trial {index}: M{args.motor}, 0 -> {speed} -> 0", flush=True)
            # Baseline: observe the motor with its zero-speed PID still active.
            rec.trial, rec.phase = index, "baseline"
            rec.speed(0, args.motor)
            rec.collect(args.baseline, require_speed=True)
            # Step: timestamp the command so the plot can align this trial to t=0.
            rec.phase = "step"
            trial = {"trial": index, "speed": speed, "step_time_s": rec.speed(speed, args.motor)}
            meta["trials"].append(trial)
            rec.collect(args.duration, require_speed=True)
            # Rest: command zero before the next target. This is a timed wait,
            # not a measurement-based guarantee that the wheel has stopped.
            rec.phase = "rest"
            trial["stop_time_s"] = rec.speed(0, args.motor)
            rec.collect(args.rest, require_speed=True)
        meta["status"] = "complete"
    except (Exception, KeyboardInterrupt) as error:
        # Record communication failures and Ctrl+C while retaining partial data.
        meta["error"] = f"{type(error).__name__}: {error}"
        print(meta["error"], flush=True)
    finally:
        # 7. Always attempt shutdown, whether the sweep finished or raised an error.
        # A disconnected board cannot receive these commands; this is best effort.
        if rec:
            # Zero speed requests a stop; zero PWM releases the active PID hold;
            # upload off ends the stream. Try each even if a previous write fails.
            rec.phase, rec.target = "cleanup", 0
            meta["cleanup_errors"].extend(rec.idle())
            meta["events"] = rec.events
            rec.close()
        if port:
            # Leave DTR/RTS at their working levels; don't introduce a reset pulse.
            try:
                port.close()
            except (Exception, KeyboardInterrupt) as error:
                meta["cleanup_errors"].append(f"close: {type(error).__name__}: {error}")
        if meta["cleanup_errors"]:
            meta["status"] = "incomplete"
        (folder / "metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    # 8. Plot after releasing the serial port. Failed runs with a started trial
    # also get a plot, labeled incomplete, so their partial data can be inspected.
    if meta["trials"]:
        plot(folder)
    print(f"{meta['status']}: {folder}")
    return 0 if meta["status"] == "complete" else 1


if __name__ == "__main__":
    # Run only when launched as a script, not when imported by the software tests.
    # Exit code 0 means complete; 1 means the experiment did not complete cleanly.
    raise SystemExit(main())
