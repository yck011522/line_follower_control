# E1 — Line-sensor experiment

Status: planned. Firmware: [src/e1_line_sensor](../../src/e1_line_sensor/README.md).

The future `run.py` will upload, wait for readiness, trigger a bounded acquisition, and capture all eight sensor channels and operation timing. Verify channel order, polarity, repeat/fresh-data behavior, and errors before comparing 50 Hz, 100 Hz, and higher requested rates.

Write each run under `results/<UTC-run-id>/` using the shared [test workflow](../../docs/testing.md). No firmware, runner, or measurements exist yet.
