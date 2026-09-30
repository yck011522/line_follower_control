# E3 — NFC-reader experiment

Status: planned. Firmware: [src/e3_nfc_reader](../../src/e3_nfc_reader/README.md).

The future `run.py` will capture UID-only reads, durations, tag absence, and reader errors. Include continuous presence, removal/re-entry, and different UIDs. Moving-pass tests follow bench timing; repeated detection must be distinguishable from a new navigation event.

Write each run under `results/<UTC-run-id>/` using the shared [test workflow](../../docs/testing.md). No firmware, runner, or measurements exist yet.
