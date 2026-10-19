# E9B serial UID monitor

Build/upload the `e9b_nfc_serial_monitor` PlatformIO environment to COM9, then open its serial monitor at 115200 baud (DTR=1, RTS=0).

```powershell
pio run -e e9b_nfc_serial_monitor -t upload --upload-port COM9
pio device monitor -e e9b_nfc_serial_monitor -p COM9
```

Monitoring starts automatically. Every successful fresh read prints `UID 11:E5:99:23`, including repeated reads of the same tag. An unsuccessful or absent-tag read produces no UID line. A `REPORT` line appears every five seconds, even without a tag. Send `X` to stop and disable RF; send `S` to initialize and resume.

Set receiver gain with `G 23` followed by Enter (serial monitor line ending: newline, carriage return, or both). Send `G` followed by Enter to query the configured gain. Allowed values are **18, 23, 33, 38, 43, 48 dB**, discrete settings rather than a continuous range. Default is **23 dB**, based on the replacement reader's successful hardware test. Gain is volatile and returns to the default on reboot. Invalid values leave the setting unchanged.

The supplied datasheet section 3.2.3.7, table 3-81 (printed page 36), defines RFCfgReg 0x26 bits 6:4: codes 0/2=18, 1/3=23, 4=33, 5=38, 6=43, 7=48 dB. E9B uses codes 0 and 1 for the duplicate lower settings. The shared driver's input validation now allows all eight documented codes. Prior E8 capture snapshots retain the exact earlier driver source.

A gain change while running cancels the pending read, disables RF, reinitializes with register readback verification, then resumes and resets the reporting window. This explicit configuration command may block briefly. While stopped, it only stores the requested setting until `S`. Reports include `gain_db`. Higher gain is not always better: the earlier fixed-position tests found empty frames at 48 dB, while 38/43 dB worked well.

Gain-command hardware verification: [command log](results/20261012_222837_gain.log) confirmed register-verified application of all six settings and rejection of `G 99` and `G bad`. That capture remains marked incomplete because its expected UID check failed: no UID was received during this session. A [follow-up report capture](results/20261012_222907_gain_report.log) confirmed continued polling at 43 dB: 2,043 no-tag results, zero errors, no UID lines, and successful RF-off cleanup. Monitoring was resumed at 43 dB. The physical tag position was not confirmed during this check.

- `polls/s`: completed reads divided by the actual reporting-window duration. Includes successful, absent-tag and failed reads. Serial output affects achievable throughput.
- `avg_poll_ms`: mean elapsed time from scheduling a read until its result, including asynchronous RF waits and internal discovery/retries, measured before printing the UID.
- `max_poll_ms`: longest completed read in that window, not the longest individual `service()` call.
- `polls`, `tags`, `no_tag`, `errors`: window counts. A read crossing a window boundary belongs to the window in which it completes.

The firmware reuses the E8 async driver, with 400 kHz I2C, 710 microsecond response polling, 1 ms RF timeout and an E9B default of 23 dB receiver gain. Only address 0x28 is accessed; the line sensor is untouched. SDA/SCL are board labels D4/D5 (GPIO5/GPIO6). Reads target four-byte ISO14443A UIDs; this does not read MIFARE memory or authenticate a tag.

All E9B output uses a fixed 256-byte formatting buffer and checks both the USB serial connection and `availableForWrite()` for the entire line before writing. USB transmit timeout is zero, including transmit-lock waits. Disconnected/full-buffer output is discarded, without retries or an application backlog. A connection change during a write can still produce a partial line; this is counted rather than retried. UID lines, reports and command replies can therefore be lost when the host is not draining. NFC polling continues, and the three-second startup wait for a monitor has been removed. The shared driver's direct initialization print is disabled for E9B.

Reports include cumulative counts since boot: `serial_dropped` (skipped output attempts), `serial_short` (partial/failed writes), and `serial_max_us` (longest guarded output call while serial appeared connected, including formatting and capacity checks). These are distinct from RF poll latency. USB calls still take CPU time; zero timeout prevents waiting for buffer space, rather than promising zero execution time. Command processing is limited to 32 incoming bytes per loop iteration.

[Backpressure hardware capture](results/20261012_224423_backpressure.log): 3,000 gain queries with no host reads for three seconds, then the monitor port closed for six seconds and reopened. Output dropped 256 lines, with zero short writes and a longest guarded output call of 355 microseconds. Polling continued at about 408 polls/s with zero NFC errors (no tag detected during this run). RF-off cleanup was verified, then monitoring resumed. This tests monitor closure and buffer pressure with USB still attached, not physical cable removal.

Initialization and explicit start/stop/gain operations can block. Ongoing NFC service remains cooperative. Use the quiet E8 driver integration for timing-sensitive application work.

Hardware verification on COM9: [12-second raw capture](results/20261012_222039.log) contained 5,826 UID lines, all `11:E5:99:23`. The full steady reporting window measured 486.80 polls/s, 1.998 ms average poll time and 2.238 ms longest poll time, with zero errors. The first report included time before the host capture opened and is not a full connected-monitor throughput measurement. Stop/RF-off was verified after capture; continuous monitoring was then resumed. Physical no-tag testing remains pending.
