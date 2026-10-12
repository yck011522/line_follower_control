# E8 NFC findings

Bench date: 2026-10-12. XIAO ESP32-S3, COM9, NFC reader at I2C address
**0x28**, SDA **D4** / SCL **D5** (GPIO5 / GPIO6). Only the NFC reader was
addressed; the line sensor at 0x12 and the motor driver were left untouched.
The tag was an NXP MIFARE Classic 1K with phone-reported identifier
**11:E5:99:23**, ATQA **0x0004**, and SAK **0x08**.

## Communication and startup

I2C acknowledgments and register reads worked initially, but this did not mean
the radio engine was ready. Software reset remained at CommandReg 0x1F;
explicit wake/Idle left 0x10, and the ASK/timer registers did not retain writes.
An RF attempt left the outgoing byte in FIFO without a transmit-complete event.

A full module power cycle cleared that startup state. CommandReg subsequently
read 0x20, configuration writes retained their values, and RF transmission
completed. The physical reason for the initial bad state was not established.
Power cycling must remove potential back-power through SDA/SCL as well as VCC.

VersionReg at 0x37 consistently reads **0x88**, while the supplied FM17550
datasheet specifies 0xA1. Successful communication and tag reads do not resolve
that chip identity/revision discrepancy.

## First successful identification

The original working exchange was:

```text
REQA:          26                 -> 04 00
Anticollision: 93 20              -> 11 E5 99 23 4E
SELECT:        93 70 ... 03 0B    -> 08 B6 DD
```

The UID matched the phone. BCC 0x4E and the SAK response CRC passed. ATQA is
assembled from the low-byte-first response 04 00. This established identification
and selection, not authentication or access to protected tag memory.

Receiver settings were changed together to achieve the first success; that
experiment did not isolate which individual setting was necessary. Later
continuous-read experiments isolated gain: at this fixed position, 48 dB caused
empty receive frames, 33 dB caused many rejected frames, and 38/43 dB worked
reliably. **43 dB** became the asynchronous reader's baseline. This is not an
antenna-range calibration.

## How the UID read became faster

1. **Initialize once per session.** Reset/configuration and the RF power-up
   guard moved out of individual UID reads. The antenna stays enabled during
   the session and is disabled with readback at session end.
2. **Read only what is needed.** Anticollision returns the UID. SELECT/SAK
   and their CRC processing are omitted from the UID-only fast path.
3. **Reuse the tag's READY state.** First discovery uses WUPA, then
   anticollision. Subsequent requests try anticollision directly. The READY
   hint skips discovery; every reported UID still comes from fresh RF bytes.
4. **Use 400 kHz I2C.** The original communication test used 100 kHz.
5. **Use FIFO bursts and combined register reads.** Command/FIFO bytes move
   together. Pointer write and register read use a repeated START, and the
   register latch is reused when valid.
6. **Reuse Transceive, framing, and a verified-consumed FIFO.** Redundant
   Idle, command restart, framing, and FIFO-flush writes are skipped on the
   steady path. It normally needs **eight I2C driver operations per UID**.
7. **Poll at the useful time.** A **710 us** initial poll/interval gave the
   best measured balance. Earlier polls often missed frame completion and
   needed another poll; later polls increased delivery latency.
8. **Remove RF waiting loops and timed-path serial printing.** `service()`
   performs at most one short I2C operation or returns immediately while RF
   is pending. Samples are printed after the experiment has stopped.

Frame length, chip error flags, byte alignment, and BCC checks remain enabled.
An empty receive frame is rejected and retried at most once, with retry counts
retained. No software-cached UID is published as a fresh result.

## Best measured tag-present results

| Measurement | Result |
| --- | ---: |
| Saturation rate | 503.8-503.9 fresh UID reads/s |
| Steady median / p99 UID latency | 1.984 / 1.993 ms |
| First discovery plus UID | approximately 4.6 ms |
| Longest service call | 271 us |
| Three ten-second verification runs | 15,116 / 15,116 matching UIDs |
| Separate 100 Hz request run | 1,000 / 1,000 matching UIDs |
| Published protocol/bus errors or UID mismatches in final runs | 0 |
| Missed synthetic control heartbeat deadlines | 0 |

One empty frame recovered internally across the three saturation runs. The
100 Hz run had no retries. Cancellation was also tested and verified RF-off.
The roughly 2 ms result applies to an already READY tag, not initial setup,
new-tag discovery, or replacement by another tag.

## Async contract and remaining work

This is cooperative asynchronous **RF waiting**. Arduino Wire is still
synchronous during an individual I2C operation: the longest observed call was
0.271 ms, and the configured bus fault timeout is 2 ms. Initialization and
session cleanup can block and belong outside active control work.

For a future 10 ms control interval, schedule control independently and call
`startRead()`, `service()`, and `takeResult()` from the fast loop between control
updates. Calling service only once every 10 ms will spread a UID read over
multiple control periods. The benchmark heartbeat was a scheduling marker,
not a real motor/PID workload.

The timing/gain/bus strategy sweep reached a measured plateau; these are the
best settings tested, not proof of a universal optimum. The current reader
handles one four-byte-UID Type A tag. Seven-byte cascade handling, collision
resolution, authentication, and tag-memory operations are not implemented.

**Tag-absent and arrival/removal tests remain pending.** The READY-probe fallback
and shorter RF timeout candidates must be evaluated physically in that round.
No absence/recovery performance is claimed from the present-tag results.

## Evidence

- [Result index and detailed benchmark report](results/README.md)
- [Curated raw captures](results/raw/)
- [All asynchronous profile summaries](results/summary.json)
- [Historical raw captures archive](results/archive/iterations.zip)
- [Checksum/location manifest](results/manifest.json)

The archive retains earlier/rejected raw measurements byte-for-byte. Redundant
loose copies were removed after archive/hash verification. Final firmware source
snapshots and the original successful UID/ATQA/SAK program are also retained.
