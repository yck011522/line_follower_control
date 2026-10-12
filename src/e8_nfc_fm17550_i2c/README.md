# E8 asynchronous NFC UID reader

The firmware now uses `NfcAsync.h` / `NfcAsync.cpp` for cooperative UID reads.
Only NFC address **0x28** is accessed. D4 / D5 are GPIO5 / GPIO6 on the XIAO
ESP32-S3. The line sensor at 0x12 and the motor driver are not addressed.

Final tag-present results on COM9, 2026-10-12: approximately **504 fresh UID
reads/s**, median **1.984 ms**, p99 **1.993 ms**, and longest measured service
call **271 us**. Three ten-second runs returned **15,116 / 15,116** matching
UIDs with no published protocol/bus errors or UID mismatches. One rejected empty
RF frame recovered through a bounded retry. A separate 100 Hz request run
returned **1,000 / 1,000** matching UIDs. See the
[benchmark report](results/README.md) for raw logs,
comparisons, timing definitions, and limits. [FINDINGS.md](FINDINGS.md) records
the startup investigation and the changes that made UID reading faster.

## Reader API and scheduling

1. Initialize `Wire` on D4/D5, then call `reader.begin(NfcAsync::Options{})`
   once while the application is idle. Configuration/reset and the cold-tag
   power-up guard are blocking initialization work.
2. Call `reader.startRead()` to schedule a read; this function performs no I2C.
3. Call `reader.service(esp_timer_get_time())` frequently from the fast Arduino
   loop while `reader.busy()` is true. Each call advances at most one I2C
   operation or returns immediately while waiting for RF. There are no RF wait
   loops or `delay()` calls in `service()`.
4. Consume `reader.takeResult(uid)`. Use the four UID bytes **only** when the
   result is `NfcAsync::Result::Uid`. `NoTag`, `ProtocolError`, and `BusError`
   never publish an old UID as a fresh detection. A bus error requires ending
   the session and reinitializing while idle.
5. `reader.end()` cancels pending work, attempts Idle/RF-off, and checks the
   antenna-enable bits. It returns false when bus failure prevents verified
   cleanup. End/reset are session operations, not normal per-read operations.

Schedule control work every 10 ms independently of NFC servicing. Call service
between those control updates; calling it only once per 10 ms will still avoid
RF blocking, but will spread one UID read over many control periods. Do not put
`delay(10)` after every state-machine step if approximately 2 ms UID delivery
is wanted. The same main-loop thread can use the shared bus for other devices
between NFC operations; this class must be the sole owner of NFC address 0x28.
The shared bus clock applies to every device on that bus.

`Wire` transfers are synchronous: this is asynchronous **RF waiting**, not a
zero-blocking I2C implementation. Combined register reads include a pointer
write and repeated START/read in one driver operation. The longest observed
call was 0.271 ms; the configured I2C fault timeout is 2 ms, not an absolute
real-time guarantee for application scheduling. The benchmark's control
heartbeat is a synthetic scheduling marker, not a real motor/PID workload.

The following lifecycle illustrates application use, after successful setup:

```cpp
// Schedule a new fresh UID read when the application's polling interval is due.
if (!reader.busy() && readDue) reader.startRead();
// Advance NFC between independently scheduled 100 Hz control updates.
if (reader.busy()) reader.service(esp_timer_get_time());
// Consume completed events; never reuse the UID after a NoTag/error event.
uint8_t uid[4];
const auto result = reader.takeResult(uid);
if (result == NfcAsync::Result::Uid) { /* handle these freshly received bytes */ }
```

Consume an outstanding result before starting the next read: `startRead()`
returns false while a result is still waiting to be consumed. The benchmark
main.cpp provides a complete working scheduling example.

## Adopted settings and fast path

- Requested I2C clock: **400 kHz**, with combined register reads, cached register
  pointer where valid, and burst FIFO transfers.
- Type A RF: **106 kbit/s**, 100% ASK, RxThreshold 0x55, RxSel 0x80, automatic
  stronger-I/Q channel selection preserving reset demodulator timing bits.
- Receiver gain: **43 dB**, encoded as gain 6. At the current fixed position,
  48 dB produced empty frames and 33 dB produced many rejected frames; 38 and
  43 dB both worked. This is a bench result, not a range/antenna calibration.
- RF-response timer: **1 ms**; host fallback is RF timeout plus 2 ms. Normal
  poll interval and first status-poll delay: **710 us**. Even when deadlines
  have elapsed, received-data flags are checked before reporting a timeout.
- First discovery uses seven-bit **WUPA (0x52)**, then anticollision **93 20**.
  Subsequent reads try anticollision directly while the tag is believed READY.
  This hint skips a handshake; it never substitutes cached UID bytes for RF data.
- Every UID is freshly received, has the expected five-byte UID/BCC frame
  length, complete final byte, no reported chip error flags, and a valid BCC.
  SELECT/SAK/CRC_A are omitted because this test only needs the UID.
- Persistent Transceive is reused. An empty FIFO and unchanged framing are
  reused only after the previous response was fully consumed. Timeout/error
  paths abort and prepare the chip again. An empty receive frame is rejected
  and retried at most once per request, with `empty_frames` retained in reports.

The fast READY probe falls back to discovery after a timeout. This code path
will be assessed physically during the next tag-absent and arrival/removal
round; the present results do not validate those scenarios. The reader currently
supports a single four-byte-UID Type A tag. Seven-byte UIDs and collision
resolution are not implemented. There are no tag-memory writes/authentication
commands. Reader VersionReg remains 0x88; exact chip revision is unconfirmed.

## Commands, build, and direct hardware experiments

USB serial is 115200 baud. **Commands end with a newline.** No test starts at boot.

- `S`: five-second maximum-throughput benchmark with current settings.
- `X`: cancel an active run and verify RF-off cleanup.
- `H`: show command syntax.
- `B duration_ms clock_hz adaptive poll_us rf_timeout_us burst cache heartbeat_us fast_tx gain combined interval_us`:
  run an explicit bounded profile. Duration is limited to 100..120000 ms.

Default command:

```text
B 5000 400000 1 710 1000 1 1 10000 1 6 1 0
```

The last argument is the interval between request starts: 0 means saturation;
10000 means a 100 Hz request schedule. Serial samples are buffered in RAM and
printed only after the timed experiment and RF cleanup have ended. The buffer
holds 8192 per-request samples; `samples_dropped` reports overflow, while summary
counters/histograms still cover the whole run. Ending at the duration boundary
may cancel one outstanding partial read, which is not counted as a completed
request. Cancellation retains completed samples and reports `CANCELLED`.

From the repository root:

```powershell
$env:PYTHONIOENCODING='utf-8'
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e e8_nfc_fm17550_i2c -t upload --upload-port COM9
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" test/E8_nfc_communication/benchmark.py --duration-ms 10000
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" test/E8_nfc_communication/benchmark.py --duration-ms 10000 --interval-us 10000
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" test/E8_nfc_communication/analyze_async.py --output src/e8_nfc_fm17550_i2c/results/summary.json
```

The Python tool sets DTR on and RTS off before opening the serial port, never
pulses RTS, keeps raw logs and incomplete-capture status, and attempts `X` on
capture error/cancellation. Flashing/reset tools are a separate workflow.
Final captures include firmware/source SHA256 and source snapshots. New captures
go into `results/raw`; selected verification/comparison logs are directly available
there. Earlier raw measurements are compressed in `results/archive/iterations.zip`,
with byte hashes and locations recorded in `results/manifest.json`. Redundant
loose copies were removed after verification. The original UID/ATQA/SAK test is
recorded in [UID_TEST_HISTORY.md](UID_TEST_HISTORY.md), with its source saved as
`results/history/baseline_uid_main.cpp.txt`.
