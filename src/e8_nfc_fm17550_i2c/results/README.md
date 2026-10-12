# E8 asynchronous UID benchmark: tag present

Test date: 2026-10-12. XIAO ESP32-S3 on COM9; NFC device 0x28 on D4/D5.
The NXP MIFARE Classic 1K tag remained beneath the reader, reference identifier
`11:E5:99:23`. No transactions addressed the line sensor at 0x12 or a motor
driver. Verification used firmware builds/uploads and direct hardware captures;
no host unit tests, mock headers, or simulated reader were used.

## Outcome

The final firmware delivers approximately **504 fresh UID reads/s**, with a
steady-state median of **1.984 ms** and p99 **1.993 ms**. Three ten-second runs
returned **15,116/15,116** reference-matching UIDs. No published protocol errors,
bus errors, no-tag results, mismatches, or sample-buffer overflows occurred in
those runs. One internally rejected empty receive frame was retried successfully.

The longest measured `service()` call was **271 us**. Each call either performs
one short synchronous I2C operation or returns immediately while the chip waits
for RF. RF waits never spin inside the service function. This is cooperative
asynchronous RF operation; Arduino Wire itself remains synchronous.

The final **100 Hz** request test returned **1,000/1,000** matching UIDs, with
no errors/retries, median/p99 **1.982 ms**, and longest service call **268 us**.
The synthetic 100 Hz scheduling marker missed no deadlines. Its maximum
lateness was 1 us in that deliberately aligned request schedule; the saturated
runs had up to 266 us lateness. This is not a test of a real motor/PID workload.

## Final profile

```text
B 10000 400000 1 710 1000 1 1 10000 1 6 1 0
```

| Parameter | Adopted value |
| --- | --- |
| Requested I2C clock | 400 kHz |
| Receiver gain | 43 dB (gain field 6; RFCfg 0x68 on this reader) |
| Receiver channel | Automatic stronger I/Q; demodulator timing bits preserved |
| RxThreshold / RxSel | 0x55 / 0x80 |
| RF-response timer | 1 ms |
| Host fallback | RF timeout plus 2 ms, inspecting receive flags before timeout |
| First IRQ poll / subsequent poll interval | 710 us |
| FIFO transfers | Burst writes/reads |
| Register reads | Combined pointer write/repeated START/read; latch reuse when valid |
| Read strategy | WUPA + anticollision initially; direct anticollision while READY |
| Reused chip state | Transceive command, framing, empty FIFO after full response consumption |
| UID validation | Complete five-byte frame, no chip errors, byte alignment, XOR BCC |
| Empty RF frame handling | Reject and retry the same exchange at most once |
| Fault timeout | Wire timeout configured to 2 ms per transfer |

SELECT/SAK are omitted because UID-only identification is the goal. The UID
is always read afresh over RF; the remembered READY state only chooses the next
command. The driver does not report a cached UID after a timeout or error.
The current implementation supports one four-byte-UID Type A tag, without
seven-byte cascade handling, collision resolution, authentication, or memory writes.

## Optimization evidence

All timing/rate values below are measured in separate bounded runs with the tag
present. The early baseline is an asynchronous, UID-only implementation, not
the previous one-shot UID+SAK program; protocols and settings change across rows.

| Profile | Valid UID/s | Median UID latency | Rejected outcomes | Longest service call |
| --- | ---: | ---: | ---: | ---: |
| 100 kHz, discovery each request, byte-at-a-time FIFO | 52.0 | 13.474 ms | 163 / 319 requests | 366 us |
| 400 kHz, READY reuse, burst FIFO, initial sequencing | 343.0 | 2.700 ms | 100 / 1815 requests | 204 us |
| Persistent Transceive and reduced preparation | 446.2 | about 2.2 ms | 40 / 2271 requests | 210 us |
| 43 dB, delayed poll, separate pointer/read calls | 456.2 | about 2.2 ms | 0 / 2281 requests | 207 us |
| Combined reads, 710 us polling, final firmware | 503.8-503.9 | 1.984 ms | 0 / 15116 requests | 271 us |

Important observed improvements:

1. **Repeated discovery was inefficient.** WUPA before every UID attempt
   yielded many timeouts with the continuously present tag. Direct anticollision
   for a tag already READY avoided that exchange. A failed READY probe falls
   back to discovery, but its absence/reappearance behavior is not yet physically
   validated.
2. **Maximum gain was worse at this fixed tag position.** With other settings
   held constant, 48 dB produced empty receive frames; 43 and 38 dB each passed
   2281/2281 requests in five seconds. At 33 dB there were 901 rejected protocol
   outcomes among 2248 requests. This supports selecting 43 dB here, but is
   not a measurement of antenna range or a general calibration for other distances.
3. **FIFO burst transfers and register-latch reuse removed repeated overhead.**
   The supplied FM17550 datasheet sections 4.4.6-4.4.7 specify repeated data
   at the same register address, supporting FIFO bursts. Its Figure 4-9 permits
   omitting pointer selection when the previous access used the same register.
4. **Persistent Transceive removed redundant commands.** Datasheet section
   14.3.7 states that Transceive continues until another command is issued;
   StartSend triggers each transmission. The optimized path reuses it only
   after a complete response has been consumed. Error/timeout paths abort.
5. **Combined reads traded a longer individual call for lower overall latency.**
   The final steady path needs three transmit/preparation writes and five
   status/data reads, normally eight driver operations per UID. It preserves
   error, FIFO-length, partial-byte, and BCC checks.

Timing sweep results reached a local plateau: polls at 600-690 us were often
too early and required another poll, giving approximately 390-414 UID/s.
710 us gave 503.8 UID/s; 720, 730, and 740 us gave approximately 500, 499,
and 496 UID/s. A 750 us poll had similar reliability but longer latency.
Continuous immediate polling was slower (approximately 483 UID/s) and occupied
more service wall time. RF timeouts of 0.5, 1, and 2 ms all read the present
tag; shorter timeout did not materially improve present-tag throughput.
The 1 ms setting remains a candidate pending the absence round.

These results identify the best configuration in the tested matrix, not a proof
that all possible software/hardware designs are optimal. Faster bus speeds
were not used: the supplied protocol's fast-mode ceiling is 400 kHz. Changing
the connection, IRQ wiring, driver architecture, or validation requirements
would be a different experiment. No improvement is claimed for those options.

## Final verification logs

| Capture | UIDs / completed requests | UID/s | Median / p99 | Maximum request latency | Empty frames recovered |
| --- | ---: | ---: | ---: | ---: | ---: |
| [Saturation 1](raw/run_20261012_205650_227870.log) | 5038 / 5038 | 503.8 | 1.984 / 1.993 ms | 4.592 ms | 1 |
| [Saturation 2](raw/run_20261012_205701_115787.log) | 5039 / 5039 | 503.9 | 1.984 / 1.993 ms | 4.601 ms | 0 |
| [Saturation 3](raw/run_20261012_205712_003475.log) | 5039 / 5039 | 503.9 | 1.984 / 1.993 ms | 4.605 ms | 0 |
| [100 Hz request schedule](raw/run_20261012_205822_478645.log) | 1000 / 1000 | 100.0 | 1.982 / 1.982 ms | 4.610 ms | 0 |

Each maximum request latency above was the first discovery+UID request. The
roughly 2 ms steady-state latency applies to an already READY tag, not initial
discovery, power-on initialization, or replacement by another tag. Initialization
reset/configuration and its 10 ms RF power-up guard are outside the timing window.

The [cancellation check](raw/run_20261012_205939_775731.log) retained 96
completed matching UID reads, reported `CANCELLED`, and verified `RF_OFF`.
All final runs verified RF-off at session end. No motion was commanded.

At 100 Hz requests, summed wall time inside service calls was approximately
15.2% of the ten-second window. It was approximately 76.5% during saturation.
This includes synchronous driver waits and is not a CPU-utilization measurement.

## Measurement and provenance

- UID latency: ESP32 `esp_timer_get_time()` from scheduling a fresh read until
  the foreground loop consumes its completed result. It includes bus work,
  RF exchange(s), state transitions, and any internal retry; there is no serial
  printing inside the timed path.
- Service latency: wall time for one `service()` invocation, including any
  synchronous I2C operation. RF-wait/no-op service calls are also measured.
- Percentiles: nearest-rank empirical quantiles of retained successful UID
  samples. Service histogram percentiles use four-microsecond bins and are
  upper bounds; reported maximum service times are directly measured.
- Rates use all completed UID events divided by the elapsed benchmark window.
  One outstanding read may be cancelled at the duration boundary and is not
  included in completed-request counters.
- Raw captures contain every retained per-request sample, final-frame status
  bytes, retry counts, full service histogram, settings, and completion/cleanup
  status. Final captures include source and firmware SHA256; matching source
  snapshots are under [async/sources](sources/).
- [summary.json](summary.json) contains analysis of all captured profiles,
  including rejected candidates and the partial cancellation run. Regenerate it
  with `analyze_async.py`; the original raw measurement files remain unchanged.
- The prior UID+ATQA+SAK source is preserved in
  [baseline_uid_main.cpp.txt](history/baseline_uid_main.cpp.txt). Its historical logs and
  the new optimization captures are retained independently.

## Next physical test

Keep this firmware/profile as the tag-present baseline. When the user removes
the tag, assess NoTag latency, main-loop service latency, false detections,
repeated absence, and arrival/removal recovery. Then compare shorter RF timers
and request intervals. No tag-absent performance or recovery result is claimed
from this round. Reader VersionReg is still 0x88; exact chip identity/revision
remains unconfirmed despite successful tag identification.

References: supplied `reference/FM17550_datasheet.pdf` for host/register behavior;
[NXP MIFARE Classic EV1 1K datasheet](https://www.nxp.com/docs/en/data-sheet/MF1S50YYX_V1.pdf)
for tag activation/anticollision. Requested I2C clock and nominal RF settings
were not independently measured with an oscilloscope or logic analyzer.

## Result organization

Ten useful verification/comparison logs are available in `raw/`. Forty earlier
raw captures are preserved in [archive/iterations.zip](archive/iterations.zip).
The [manifest](manifest.json) records every original path, current location,
byte size, SHA256, and the archive SHA256. All archived bytes were verified
before redundant loose originals were removed. The summary analyzer reads both
curated files and archived logs, so all 36 asynchronous profiles remain included.

For the investigation narrative and optimization explanation, see
[../FINDINGS.md](../FINDINGS.md). Tools remain under
`test/E8_nfc_communication/`; new captures use this results folder.
