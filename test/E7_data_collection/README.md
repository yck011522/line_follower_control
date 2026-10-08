# E7 recording tool

See the [E7 firmware and recording guide](../../src/e7_data_collection/README.md)
for Wi-Fi/OTA settings, the telemetry schema, master discovery and hardware checks.

```powershell
../../.venv/Scripts/python.exe capture.py --port COM7 --map map_a_v1 --route "left at intersection"
```

Run this command from this directory. Select the **master radio's** USB port.
Wait for `READY`, press Enter to begin the push, and Ctrl-C to finalize CSV,
raw serial bytes and metadata in a new timestamped `results` directory.
