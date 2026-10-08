# E7 manual-push data collection

Car **1** uses one 100 Hz schedule: release motor outputs, read the E6 raw line
sensor byte, read cumulative M2/M4 encoders, calculate mm/s, and send telemetry.
There are no powered-motion commands. Acquisition continues before discovery;
the first valid `W,<signed int32 sequence>` packet identifies the master by its
sender MAC. The master can stop sending after telemetry arrives. `W,-1` is also
discovery; it does not reset E7. The learned master stays selected until reboot.

## Build and upload

Edit `[e7_network]` in the root `platformio.ini` for the AP SSID/password,
static address, gateway, OTA target and OTA password. Defaults match the lab
settings in the local `esp32_broadcast_research` checkout. Reserve the static
address for this car, and change it if another car already uses `.200`.
The AP must use **channel 6**, matching that repository's master firmware.

```powershell
pio run -e e7_data_collection
pio run -e e7_data_collection -t upload --upload-port COM4
# After the first USB flash, use the AP for subsequent uploads:
pio run -e e7_data_collection_ota -t upload
```

If `pio` is not on PATH, use
`& "C:/Users/leungp/.platformio/penv/Scripts/platformio.exe"` instead.
USB ports are examples; identify the car/master ports before uploading/logging.

Wi-Fi uses STA mode, static IP and `WiFi.setSleep(false)` from the reference.
Initial association waits at most 20 seconds; a later connection also starts
OTA. OTA handling stays outside the 100 Hz gate. Acquisition pauses during an
upload and outputs are released again. If the AP is unavailable or moves to a
different channel, ESP-NOW delivery is not guaranteed; check AP/channel first.

## Hardware and measurements

- Line sensor: controller 1, SDA D6, SCL D7, 400 kHz, 1 ms timeout. E6's direct
  repeated-START read selects `0x30` at `0x12` and reads one byte. No centroid,
  filtering, or intersection classification is applied. Active-low bit 7 is X1,
  bit 0 is X8; retain all eight channels for offline analysis.
- Motor driver: Wire, SDA D4, SCL D5, 400 kHz, 1 ms timeout. Reapply type 1,
  dead zone 1650, pulse line 500, ratio 23, diameter 65 mm at every boot via
  `MotorDriver::initialize()`. PID 3/0.375/0.5 is **stored_unverified**.
- M2 is left, M4 is right. Counts are the driver's signed cumulative positions,
  read with the existing high/low/high consistency check. E7 does not zero them.
- Speed is `delta_counts * pi * 65 / 44998 / elapsed_seconds`, using the earlier
  wheel calibration. It is encoder-derived mm/s, not the driver's serial speed
  feedback. Both counts and speed retain the driver's electrical sign; mounted
  forward travel is expected to be negative. Confirm with a short forward push.
- Each wheel uses its own read-start timestamp. Its first valid read has speed
  `nan`. After failed reads, the next valid speed averages across the intervening
  gap. The rollover calculation assumes less than half the 32-bit count range
  of displacement between successful reads. A motor-driver reset during a run
  is not distinguishable from a count jump; end/restart the recording if that occurs.
- Zero PWM releases outputs each cycle; no zero-speed PID holding is used.
  Configuration/release errors are reported, and the logger requires recent
  healthy measurements plus release ACK before accepting Enter. An I2C ACK is
  transaction evidence, not independent proof of the physical output state.

## Version-1 telemetry

The payload is ASCII CSV, without a trailing newline or NUL. The existing
master adds a newline and forwards it to USB without knowing its schema.
Packets are limited to 250 bytes; oversized packets are discarded and counted.
The exact column order follows. Offsets and durations are microseconds.

```text
message,version,car_id,boot_id,sample_sequence,cycle_us,
line_read_offset_us,line_raw_mask,line_error,
left_count,left_read_offset_us,left_mm_s,left_error,
right_count,right_read_offset_us,right_mm_s,right_error,
motor_configured,release_error,acquisition_us,skipped_slots,
tx_busy_skips,tx_submission_errors,tx_delivery_failures
```

| Field | Meaning |
| --- | --- |
| `message,version,car_id` | `E7,1,1` |
| `boot_id` | Random 8-digit hex identifier, new at ESP32 boot |
| `sample_sequence` | uint32 acquisition attempt counter, including unsent samples |
| `cycle_us` | 64-bit ESP32 cycle-start uptime; not host wall time |
| `*_read_offset_us` | Read-start time relative to `cycle_us`; sequential, not atomic |
| `line_raw_mask` | Raw decimal byte, or -1 on failure |
| `*_error` | 0 success, Wire error otherwise; 128 short read, 129 incoherent encoder, 255 bus unavailable |
| `left_count,right_count` | Signed raw positions; ignore placeholders when their error is nonzero |
| `left_mm_s,right_mm_s` | Encoder-derived speed to 3 decimal places; `nan` when unavailable |
| `motor_configured` | Boot baseline configuration/release all acknowledged; not value readback |
| `acquisition_us` | Time from cycle start through release and all sensor/encoder reads |
| `skipped_slots` | Cumulative missed 10 ms schedule slots; no catch-up bursts |
| `tx_busy_skips` | Samples not sent because the previous ESP-NOW completion is pending |
| `tx_submission_errors` | Rejected send calls or oversize/format failures |
| `tx_delivery_failures` | Failed send callbacks; MAC-layer delivery, not host application ACK |

Counters in a packet describe prior transmissions, not that packet's eventual
delivery. Sample sequence gaps expose missing samples end to end; they cannot
alone distinguish radio loss from master USB queue loss. The sender allows only
one pending transmission and does not retransmit old samples. No I2C or printing
runs inside radio callbacks. The firmware has only one acquisition/send schedule.

## Record a push

Use the existing `master_radio_broadcast` firmware for discovery without needing
the car's MAC in its peer list. `master_radio` also works if its configured slave
list includes this car's MAC. Both receive/forward E7 telemetry unchanged.
They forward host packets; neither generates discovery autonomously.

```powershell
.venv/Scripts/python.exe -m pip install -r test/E7_data_collection/requirements.txt
.venv/Scripts/python.exe test/E7_data_collection/capture.py --port COM7 --map map_a_v1 --route "straight, left at crossing"
```

The logger sends `W,0` every 500 ms until car 1 telemetry arrives. It resumes
discovery after two seconds without telemetry, including after a car reboot.
It drains data continuously while waiting for Enter. When `READY` appears,
press **Enter**, push the car, then press **Ctrl-C** to finalize the run.
Optional `--max-seconds 300` bounds recording time after Enter.

Each invocation creates a unique directory under `test/E7_data_collection/results`:

- `samples.csv`: recording samples, host arrival time, elapsed time from Enter,
  sequence gaps and the telemetry fields above. Invalid measurements are blank.
- `raw_serial.bin`: exact master bytes from port opening, including pre-Enter
  traffic, malformed lines and any incomplete final line.
- `metadata.json`: map/route/notes, calibration, settings, boot IDs, row/gap counts
  and completion status. It starts `incomplete` and is refreshed each second.

CSV and raw data are flushed to disk every second, then on normal exit/error.
Ctrl-C after Enter is a normal completed run; cancellation before Enter is
labelled separately. Exceptions leave `incomplete` status. Power loss can lose
the final unflushed second; earlier data and incomplete metadata remain.
No existing measurement files are overwritten. Serial opens at 115200 with
DTR on and RTS off **before opening**, and RTS is never intentionally pulsed.

Enter marks a host receive boundary, not a perfectly synchronized device sample
boundary; buffered/in-flight samples near Enter can precede the physical push.
Use ESP32 acquisition timestamps for analysis, not USB arrival spacing. Firmware
continues passive telemetry after the logger exits. Use map/route labels to
separate intersections and hold out whole runs/maps for offline evaluation.

## Verification

Both `e7_data_collection` and `e7_data_collection_ota` compiled successfully on
2026-10-07 with the pinned Arduino 3.3.12 toolchain (53,220 bytes RAM,
938,366 bytes program flash). Python syntax compilation and CLI help passed.
No firmware upload or physical data-collection run was performed in this change.

Build both E7 environments before uploading. No host mock/device harness is used.
After upload, confirm roughly 100 received packets/s, zero read errors, acceptable
sequence gaps and `acquisition_us < 10000` under normal operation. Make a short
forward push to check count/speed signs and an intersection pass to confirm the
raw mask is preserved. Stop discovery after telemetry starts and verify streaming
continues. Then verify Ctrl-C finalization and OTA on the actual AP/master setup.
Hardware timing, wireless delivery and OTA require that physical verification.
