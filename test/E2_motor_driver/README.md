# E2 — Motor-driver experiment

Status: planned. Firmware: [src/e2_motor_driver](../../src/e2_motor_driver/README.md).

The future `run.py` will exercise explicit bounded speed commands and read supported speed/battery telemetry. Measure writes and reads separately, verify units and stop behavior, and distinguish commanded speed from measured speed. Confirm direction with wheels clear before driving tests.

Write each run under `results/<UTC-run-id>/` using the shared [test workflow](../../docs/testing.md). No firmware, runner, or measurements exist yet.
