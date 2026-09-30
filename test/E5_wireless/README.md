# E5 — Wireless/bridge experiment

Status: planned. Firmware: [src/e5_wireless](../../src/e5_wireless/README.md).

The future `run.py` will identify bridge and robot ports explicitly, upload the relevant roles, and measure matched request/response round trips. Start with one robot, then exercise all four at 50 Hz world-state broadcasts and 20 Hz telemetry per robot. Track loss, duplicates, ordering, acknowledged sequence numbers, and latency distributions.

Separate radio-only timing from the complete host→USB→radio→robot→radio→USB→host path. Write each run under `results/<UTC-run-id>/` using the shared [test workflow](../../docs/testing.md). No firmware, runner, or measurements exist yet.
