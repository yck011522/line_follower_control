"""Exercise real capture/analysis code without opening hardware or moving motors."""
import csv
import io
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import analyze
import capture
import plot


def synthetic_log():
    """A known +/- mapping crossing both the count and microsecond timer wrap."""
    lines = [
        "# END reason=user_stop rows=0",  # Pre-handshake END must be ignored.
        "# SETTINGS type=1 deadzone=1650 lines=2000 ratio=23 diameter_mm=65.0 pid=3,0.375,0.5 pid_source=stored_unverified",
        "# START schema=2 min=-100 max=100 inc=10 steps=21 baseline_ms=1000 settle_ms=1000 measure_ms=2000 rest_ms=1000",
        "step,command,phase,step_ms,t2_us,m2_count,t4_us,m4_count,m2_recent,m4_recent,write_us,read_us,applied",
    ]
    counter, timestamp = 2**31 + 10, 2**32 - 100000
    rows = 0
    for step, command in enumerate(range(-100, 101, 10)):
        for ms in range(0, 5000, 20):
            phase = "baseline" if ms < 1000 else "settle" if ms < 2000 else "measure" if ms < 4000 else "rest"
            applied = command if phase in ("settle", "measure") else 0
            counter += applied * 2  # 100 counts/s per command unit at 20 ms.
            timestamp += 20000
            count = (counter + 2**31) % 2**32 - 2**31
            lines.append(f"{step},{command},{phase},{ms},{timestamp % 2**32},{count},{(timestamp+300) % 2**32},{count},{applied},{applied},200,1000,{applied}")
            rows += 1
    lines += [f"# END reason=complete rows={rows} dropped=0 missed_ticks=0 errors=0 stop_ack=1 release_ack=1",
              "# CLEANUP # END reason=user_stop rows=5250"]
    return "\n".join(lines) + "\n"


class Clock:
    def __init__(self):
        self.now = 0

    def monotonic(self):
        self.now += 0.001
        return self.now


class Port:
    def __init__(self, failure=False):
        self.dtr = self.rts = True
        self.pending = b""
        self.commands = []
        self.closed = False
        self.failure = failure
        self.started = False

    def open(self):
        assert self.dtr is True and self.rts is False

    @property
    def in_waiting(self):
        return len(self.pending)

    def write(self, data):
        self.commands.append(data)
        if data == b"!\n":
            self.pending = b"# END reason=user_stop\n"
        elif data == b"status\n":
            self.pending += b"# STATUS configured=1 running=0 schema=2 clock=400000\n"
        elif data == b"start\n":
            self.started = True
            self.pending += synthetic_log().encode()
        return len(data)

    def read(self, count):
        if self.failure and self.started:
            self.failure = False
            raise KeyboardInterrupt()
        # Split arbitrary lines to exercise the persistent reader buffer.
        count = min(count, 701)
        result, self.pending = self.pending[:count], self.pending[count:]
        return result

    def close(self):
        self.closed = True


class E2Tests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.log = self.folder / "serial.log"
        self.log.write_text(synthetic_log())

    def test_mapping_reverse_rollover_and_plot(self):
        self.assertTrue(analyze.analyze(self.log))
        with (self.folder / "summary.csv").open() as file:
            rows = list(csv.DictReader(file))
        self.assertEqual(len(rows), 21)
        for row in rows:
            expected = int(row["command"]) * 100
            self.assertAlmostEqual(float(row["m2_counts_s"]), expected)
            self.assertAlmostEqual(float(row["m4_estimated_mm_s"]), expected * math.pi * 65 / 46000)
        plot.plot_run(self.folder)
        self.assertGreater((self.folder / "summary.png").stat().st_size, 1000)

    def test_counter_wrap_in_both_directions(self):
        self.assertEqual(analyze.delta(2147483647, -2147483648), 1)
        self.assertEqual(analyze.delta(-2147483648, 2147483647), -1)

    def test_truncated_sweep_is_rejected_but_samples_preserved(self):
        self.log.write_text("\n".join(synthetic_log().splitlines()[:300]))
        with self.assertRaisesRegex(ValueError, "Incomplete"):
            analyze.analyze(self.log)
        self.assertTrue((self.folder / "samples.csv").exists())

    def test_missing_end_is_not_complete(self):
        self.log.write_text(synthetic_log().replace("# END reason=complete", "# LOST_END reason=complete"))
        self.assertFalse(analyze.analyze(self.log))

    def test_missing_sample_row_count_is_degraded(self):
        text = synthetic_log()
        lines = text.splitlines()
        del lines[120]
        self.log.write_text("\n".join(lines))
        self.assertFalse(analyze.analyze(self.log))

    def test_wrong_phase_or_applied_command_rejected(self):
        self.log.write_text(synthetic_log().replace(",baseline,0,", ",measure,0,", 1))
        with self.assertRaisesRegex(ValueError, "phase/timing"):
            analyze.analyze(self.log)

    def test_old_schema_is_rejected(self):
        self.log.write_text(synthetic_log().replace("schema=2", "schema=1"))
        with self.assertRaisesRegex(ValueError, "schema=2"):
            analyze.analyze(self.log)

    def test_capture_uses_rts_off_and_finishes(self):
        self.capture_test(False)

    def test_capture_ctrl_c_still_cancels_and_closes(self):
        self.capture_test(True)

    def capture_test(self, failure):
        folder = self.folder / "capture"
        port, clock = Port(failure), Clock()
        with patch("serial.Serial", return_value=port) as factory, \
                patch.object(capture.time, "monotonic", clock.monotonic), \
                patch("sys.argv", ["capture.py", "--port", "FAKE", "--out", str(folder)]):
            result = capture.main()
        self.assertIsNone(factory.call_args.kwargs["port"])
        self.assertEqual(port.commands[-1], b"!\n")
        self.assertTrue(port.closed)
        self.assertEqual(result, 1 if failure else 0)
        meta = json.loads((folder / "metadata.json").read_text())
        self.assertEqual(meta["status"], "incomplete" if failure else "complete")


if __name__ == "__main__":
    unittest.main()
