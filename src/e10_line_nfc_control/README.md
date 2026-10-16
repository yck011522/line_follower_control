# E10: Shared I2C Line Sensor & NFC Reader at 100 Hz

Firmware **E10** combines the Line Sensor reliability measurements from **E6** and the FM17550 asynchronous NFC tag monitor from **E9B** onto a **single shared I²C bus** on pins **D4 (SDA)** and **D5 (SCL)** running at **400 kHz**.

## Architecture & Timing

- **Shared I²C Bus**: Wire peripheral on Seeed Studio XIAO ESP32S3 using pins `D4` (GPIO5) and `D5` (GPIO6) at 400 kHz with a 1 ms bus timeout.
  - Line Sensor: address `0x12`, register `0x30`, read via repeated-START (~250 µs).
  - NFC Reader (FM17550): address `0x28`, handled cooperatively by `NfcAsync` with pointer caching disabled for bus sharing safety.
- **Main Control Loop**:
  - Anchored fixed-rate timing at **100 Hz** (10 ms / 10,000 µs period).
  - On each 100 Hz tick:
    1. Line Sensor is read immediately and synchronously (~250 µs).
    2. A fresh NFC tag read is triggered on `NfcAsync` if the reader is idle.
  - Between 100 Hz ticks:
    - The asynchronous NFC state machine is cooperatively serviced (`nfcReader.service()`), advancing RF polling without blocking the bus.
- **Status Reporting**:
  - Broadcast every **0.1 second (10 Hz)** to the Serial port at 115200 baud.
  - Protected by a **non-drained serial guard** (`Serial.availableForWrite()` and `Serial.setTxTimeoutMs(0)`) so a disconnected or slow USB host never blocks or stalls the 100 Hz control loop.
  - Fixed-length padded output (62 characters base message):
    - `POS`: Latest Line sensor position as an 8-bit binary mask (`bit 7` = X1 to `bit 0` = X8, active-low where `0` = line detected, or `--------` before first valid read).
    - `UID`: Latest detected NFC UID (11 characters, e.g. `11:E5:99:23`, or 11 spaces if no tag detected within the past 10 seconds).
    - `AGE`: Age of the most recent NFC detection in seconds with 1 decimal place (e.g. ` 0.0s` up to `10.0s`).
    - `RATE`: Measured actual main control loop execution rate in Hz (e.g. `100.0 Hz`).
  - **Error handling**: Any reading errors (I²C transmit NACK, short reads, NFC bus/protocol errors) are appended to the end of the 0.1 s status line (` | ERR: ...`) rather than interrupting the stream with new lines.

## Status Line Format Examples

Tag present within last 10 seconds:
```text
POS: 11011000 | UID: 11:E5:99:23 | AGE:  0.0s | RATE: 100.0 Hz
```

No tag detected in past 10 seconds (padded with 11 spaces for UID and 10.0s age):
```text
POS: 11011000 | UID:             | AGE: 10.0s | RATE: 100.0 Hz
```

Reading error occurred in reporting window (appended to end of line):
```text
POS: 11011000 | UID: 11:E5:99:23 | AGE:  0.0s | RATE: 100.0 Hz | ERR: LINE_TX=2(1)
```

## Build and Upload

Compile and upload using PlatformIO:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e e10_line_nfc_control -t upload --upload-port COM12
```

## Verification

Run the Python verification tool:

```powershell
python test/E10_line_nfc_control/capture.py --port COM12 --duration 10
```

Hardware capture results on COM12:
- Achieved status report frequency: 10.0 lines/sec (100 lines in 10.0 seconds).
- Control loop rate: measured steady at 100.0 Hz.
- Base status line length: strictly constant at 62 characters across all lines.
- Sensor reads: Line sensor mask `11011000` and NFC UID `11:E5:99:23` both acquired with zero errors.
