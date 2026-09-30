# E2 — Motor-driver experiment

Status: plan prepared; no hardware test run. **This is now the first experiment.** Firmware: [src/e2_motor_driver](../../src/e2_motor_driver/README.md).

Read the [motor commissioning and timing plan](PLAN.md) for the M2/M4 command, encoder, stop, and 100/400 kHz comparison sequence. [Development environment](../../docs/development-environment.md) records PlatformIO discovery and the observed COM4 port.

The future `run.py` will exercise explicit bounded speed commands and read supported speed/battery telemetry. Measure writes and reads separately, verify units and stop behavior, and distinguish commanded speed from measured speed. Confirm direction with wheels clear before driving tests.

Write each run under `results/<UTC-run-id>/` using the shared [test workflow](../../docs/testing.md). No firmware, runner, or measurements exist yet.
