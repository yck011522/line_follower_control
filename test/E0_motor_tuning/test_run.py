"""Protocol and failure-path tests using a simulated driver; no hardware access."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import run


class Clock:
    value = 0

    def monotonic(self):
        return self.value


class Driver:
    def __init__(self, clock, *, mismatch=False, dropout=False, interrupt=False):
        self.clock = clock
        self.mismatch = mismatch
        self.dropout = dropout
        self.interrupt = interrupt
        self.commands = []
        self.pending = b""
        self.upload = False
        self.values = "0,0,0,0"
        self.pid = "1,6,1.5"
        self.closed = False
        self.dtr = True
        self.rts = True
        self.open_settings = []

    def open(self):
        self.open_settings.append((self.dtr, self.rts))
        if not self.dtr or self.rts:
            raise RuntimeError("Wrong control-line settings at open")
        self.closed = False

    @property
    def out_waiting(self):
        return 0

    @property
    def in_waiting(self):
        return len(self.pending)

    def write(self, data):
        command = data.decode()
        self.commands.append(command)
        if command == "$read_flash#":
            p, i, d = self.pid.split(",")
            lines = 500 if self.mismatch else 2000
            self.pending += (f"read_flash:OK!\nMotor_type:1\nDead_Zone:1650\n"
                             f"Pulse_Line:{lines}\nPulse_Phase:23\nwheel_diameter:65\n"
                             f"P:{p} I:{i} D:{d}\n").encode()
        elif command.startswith("$MPID:"):
            self.pid = command[6:-1]
            self.pending += b"Reset MCU\nMotor_Version:1.7.3\n"
        elif command.startswith("$spd:"):
            self.values = command[5:-1]
        elif command.startswith("$upload:"):
            self.upload = command == "$upload:0,0,1#"
        return len(data)

    def read(self, count):
        self.clock.value += 0.02
        moving = self.values != "0,0,0,0"
        if moving and self.interrupt:
            self.interrupt = False
            raise KeyboardInterrupt()
        if not self.pending and self.upload and not (self.dropout and moving):
            self.pending = f"$MSPD:{self.values}#".encode()
        chunk, self.pending = self.pending[:count], self.pending[count:]
        return chunk

    def close(self):
        self.closed = True


class Tests(unittest.TestCase):
    def experiment(self, **options):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        folder = Path(temporary.name) / "run"
        clock = Clock()
        driver = Driver(clock, **options)
        argv = ["run.py", "--pid", "3", "3", "0.5", "--out", str(folder)]
        with patch("sys.argv", argv), patch("serial.Serial", return_value=driver), \
                patch.object(run.time, "monotonic", clock.monotonic):
            result = run.main()
        meta = json.loads((folder / "metadata.json").read_text())
        self.assertTrue(driver.closed)
        self.assertEqual(driver.open_settings, [(True, False)])
        self.assertEqual(driver.commands[-3:],
                         ["$spd:0,0,0,0#", "$pwm:0,0,0,0#", "$upload:0,0,0#"])
        return result, meta, driver, folder

    def test_complete_sweep_and_plot(self):
        result, meta, driver, folder = self.experiment()
        self.assertEqual(result, 0)
        self.assertEqual(meta["status"], "complete")
        self.assertEqual(len(meta["trials"]), 4)
        for trial in meta["trials"]:
            self.assertGreaterEqual(trial["stop_time_s"] - trial["step_time_s"], 3)
        self.assertTrue((folder / "speed.png").exists())
        self.assertTrue((folder / "summary.md").exists())
        self.assertLess(driver.commands.index("$upload:0,0,1#"),
                        driver.commands.index("$spd:0,20,0,0#"))
        events = meta["events"]
        for sequence in [events[:3], events[-3:]]:
            for first, second in zip(sequence, sequence[1:]):
                self.assertGreaterEqual(second["time_s"] - first["time_s"], 0.15)

    def test_port_is_constructed_closed(self):
        clock = Clock()
        driver = Driver(clock)
        with patch("serial.Serial", return_value=driver) as factory:
            self.assertIs(run.open_driver("COM5"), driver)
        self.assertIsNone(factory.call_args.kwargs["port"])
        self.assertEqual(driver.port, "COM5")
        self.assertEqual(driver.open_settings, [(True, False)])

    def test_read_only_reconnects_never_write_pid_or_motor_commands(self):
        with tempfile.TemporaryDirectory() as temp:
            clock = Clock()
            driver = Driver(clock)
            with patch("serial.Serial", return_value=driver), \
                    patch.object(run.time, "monotonic", clock.monotonic), \
                    patch.object(run.time, "sleep"):
                result = run.check_connections("COM5", 3, Path(temp))
            self.assertEqual(result, 0)
            self.assertTrue(driver.closed)
            self.assertEqual(driver.open_settings, [(True, False)] * 3)
            self.assertEqual(driver.commands, ["$read_flash#"] * 3)

    def test_shutdown_continues_after_failed_stop_write(self):
        with tempfile.TemporaryDirectory() as temp:
            clock = Clock()
            driver = Driver(clock)
            with patch.object(run.time, "monotonic", clock.monotonic):
                rec = run.Recorder(driver, Path(temp))
                original_write = driver.write

                def fail_stop(data):
                    if data == b"$spd:0,0,0,0#":
                        raise OSError("Disconnected during stop")
                    return original_write(data)

                try:
                    with patch.object(driver, "write", side_effect=fail_stop):
                        errors = rec.idle()
                    self.assertEqual(len(errors), 1)
                    self.assertEqual(driver.commands, ["$pwm:0,0,0,0#", "$upload:0,0,0#"])
                finally:
                    rec.close()

    def test_output_drain_has_a_deadline(self):
        with tempfile.TemporaryDirectory() as temp:
            clock = Clock()
            driver = Driver(clock)
            from unittest.mock import PropertyMock
            with patch.object(run.time, "monotonic", clock.monotonic), \
                    patch.object(Driver, "out_waiting", new_callable=PropertyMock, return_value=1):
                rec = run.Recorder(driver, Path(temp))
                try:
                    with self.assertRaisesRegex(RuntimeError, "draining serial output"):
                        rec.drain_output(timeout=0.1)
                    self.assertLess(clock.value, 0.2)
                finally:
                    rec.close()

    def test_configuration_mismatch_prevents_pid_and_motion(self):
        result, meta, driver, _ = self.experiment(mismatch=True)
        self.assertEqual(result, 1)
        self.assertFalse(any(c.startswith("$MPID:") for c in driver.commands))
        self.assertEqual(meta["trials"], [])

    def test_telemetry_dropout_stops_and_preserves_partial_plot(self):
        result, meta, _, folder = self.experiment(dropout=True)
        self.assertEqual(result, 1)
        self.assertIn("No valid MSPD", meta["error"])
        self.assertTrue((folder / "speed.png").exists())

    def test_ctrl_c_stops(self):
        result, meta, _, _ = self.experiment(interrupt=True)
        self.assertEqual(result, 1)
        self.assertIn("KeyboardInterrupt", meta["error"])

    def test_fragmented_concatenated_and_invalid_packets(self):
        with tempfile.TemporaryDirectory() as temp:
            rec = run.Recorder(None, Path(temp))
            try:
                for chunk in [b"noise\n$MS", b"PD:0,-2.5,0,4#", b"$MSPD:bad#",
                              b"$MSPD:1,2,3,4#$MSPD:5,6,7,8#"]:
                    rec.feed(chunk)
                self.assertEqual(len(rec.rows), 3)
                self.assertEqual(rec.rows[0]["m2"], -2.5)
                self.assertEqual(rec.rows[-1]["m4"], 8)
            finally:
                rec.close()


if __name__ == "__main__":
    unittest.main()
