# E1 motor commissioning and I²C timing plan

Status: **initial interactive firmware implemented and uploaded; bench validation in progress**. The [operator guide](README.md) describes the current subset. This document retains the broader commissioning/automation roadmap; E1 remains the first hardware experiment.

Source: [motor/driver interface reference](../../reference/RC_Car_Motor_Driver_Interface.md). This is sufficient to design the experiment, but some wire details and physical-speed scaling still need confirmation before closed-loop motion.

## Objectives

1. Command M2 (left) and M4 (right) through the driver's speed loop.
2. Confirm motion independently through recent encoder counts and cumulative encoder changes.
3. Command zero speed and measure settling to a stationary encoder count.
4. Repeat the same workload at 100 kHz and 400 kHz I²C; compare reliability and latency.

M1 and M3 remain zero in every four-channel command. Start with wheels lifted and the chassis secured. The first experiment establishes command/feedback correctness and timing, not accurate mm/s control or loaded driving performance.

## Before implementing the motion phase

| Item | Current evidence / required action |
| --- | --- |
| Controller | Seeed Studio XIAO ESP32S3; Arduino proposed for the initial PlatformIO environment |
| USB | PlatformIO detected one device on COM4 on 2026-09-30; re-enumerate before each run |
| I²C wiring | User clarified all pin numbers are board D labels: **SDA D4 / SCL D5** (GPIO5/GPIO6), matching the board's default I²C pins. Use `D4`/`D5` aliases explicitly |
| Driver address | Use 7-bit address `0x26`; this is separate from cumulative register `0x26` |
| Motor configuration | Type 1 is a candidate, not a verified GT50 preset. Confirm encoder wiring and direction before sustained closed-loop control |
| Command units | The reference does not state the physical unit/range of `0x06`; obtain the vendor command definition before selecting a motion setpoint. Its `+200` example is not a suitable unverified initial speed |
| Register encoding | Speed/PWM int16 values are big-endian. Read byte order, cumulative word order, configuration byte order/float encoding, and read transaction sequence still need confirmation |
| Supply | Reference lists recommended driver input 5–12 V, while the project mentions a 12.6 V pack. Verify the board's permitted maximum; use a supply within the documented range for initial commissioning |

A wheels-up test still needs a way to remove motor power if an incorrect encoder sign makes the local PID accelerate or if I²C fails. The host cannot guarantee a stop over a failed bus. Confirm level conversion, common ground, and encoder connections before powering motion.

## Motor type and scaling

Start with **type 1 (520)** only as a commissioning candidate. Types 1–3 may differ in encoder phase conventions and/or defaults; resemblance of the motor housing does not validate the selection. Do not cycle through types automatically while running. Stop/release the motors before changing configuration, and reapply documented custom parameters after selecting a type in case the preset changes them.

The GT50 reference specifies 500 ppr and 22.569:1 gearing. Verify the pulse/edge convention by manually rotating the output wheel a known number of turns with output released. About 11,284.5 cycles/output revolution is a reference calculation; a quadrature-edge count may differ by a factor of two or four. Do not assume the driver’s `encoder line count` uses the same convention as the motor datasheet.

The user supplied a 65 mm wheel diameter. The uint16 gearbox setting cannot express 22.569 directly under the supplied description. Initial short bench pulses use 23 as an explicitly provisional approximation while the speed formula is still unverified, following the user's preference to test locally. Physical speed remains uncalibrated. Retain raw counts as the primary evidence. Keep default PID/dead-zone settings initially; do not assume they are tuned for this motor.

Configuration writes may be persistent. Perform them only during explicit commissioning, record exact values, and never rewrite configuration at the sampling rate. Configuration readback and persistence are not established by the I²C reference; report values as requested, not verified.

## Registers and transaction checks

| Operation | Register(s) | Intended check |
| --- | --- | --- |
| Closed-loop setpoint | `0x06` | `[0, left, 0, right]`, four signed big-endian int16 values |
| Active stop | `0x06` | Eight zero payload bytes for all four channels |
| Release candidate | `0x07` | All-zero PWM; actual release/brake semantics must be verified |
| M2 recent encoder count | `0x11` | Signed int16, documented as pulses over 10 ms |
| M4 recent encoder count | `0x13` | Signed int16, documented as pulses over 10 ms |
| M2 cumulative | `0x22`, `0x23` | Two 16-bit words; assembly and snapshot semantics pending |
| M4 cumulative | `0x26`, `0x27` | Two 16-bit words; assembly and snapshot semantics pending |
| Battery | `0x08` | uint16 raw reading; voltage scaling pending |

Confirm whether reads use a register-pointer write followed by repeated START or STOP, whether multi-register bursts are supported, and whether recent-count reads clear or latch data. Preserve raw bytes alongside interpreted values until decoding is verified. Do not extrapolate speed-command byte order to all reads without evidence.

For cumulative counts, establish signedness, high/low-word order, wrap handling, and whether the two words are read atomically. Prefer a documented latched/burst read. If reads are independent live words, use a bounded coherence method such as high–low–high with retry when the high word changes, after identifying the high word; mark an unsuccessful snapshot invalid. Never treat a torn count as wheel motion.

## Staged procedure

### A. Idle communication at 100 kHz

After the eventual explicit firmware upload, wait for a ready handshake; movement must require a separate start request. Use only the confirmed pins and probe address `0x26`. Establish transaction correctness using battery/raw encoder reads, then verify all-zero output commands before permitting nonzero commands. An ACK alone does not validate command meaning.

Collect two seconds of stationary encoder baseline at 50 Hz. With the driver confirmed released, manually rotate each wheel separately and verify that only its corresponding encoder changes. Record count sign and counts per output revolution. This also tests cumulative decoding independently of the speed PID.

### B. First motion and stop, one motor at a time

With command units established, choose one explicit low commissioning setpoint `S` and record it in run metadata. Begin with a **maximum 250 ms pulse**, first M2 only and then M4 only, with an explicit zero-speed command afterward. Observe physical direction and encoder feedback. Keep these commissioning pulses separate from the repeatable benchmark; increase pulse duration only after response is controlled. If sign, stability, or motion is unexpected, stop and investigate rather than increasing the setpoint automatically.

If encoder/controller polarity remains ambiguous, a separate, deliberately selected short low-PWM direction check may help; it is not part of the default closed-loop test. Do not test motor types 2/3 blindly to compensate for wiring or decoding errors.

After the short pulses pass, run this cycle three times for each motor selection:

| Phase | Duration | Command |
| --- | --- | --- |
| Baseline | 1 s | `[0, 0, 0, 0]` at `0x06` |
| Motion | 2 s | M2 only: `[0, S_left, 0, 0]`; M4 only: `[0, 0, 0, S_right]`; then both: `[0, S_left, 0, S_right]` |
| Stop observation | Up to 2 s | `[0, 0, 0, 0]` at `0x06` |

`S_left` and `S_right` represent the calibrated forward signs and the same chosen magnitude; mirrored installations may require opposite signs. Do not introduce a reverse-travel test. Start at a 50 Hz command/read cycle; read both recent counters and both cumulative counters each cycle, battery at 2 Hz. Keep each transaction's timestamp because sequential reads are not simultaneous.

The stop check requires both recent counters to return to their stationary baseline and cumulative counts to remain within baseline tolerance for **200 ms continuously**, within a provisional **2 s deadline** after stop. Derive count tolerance from the idle baseline; unexplained encoder noise is a commissioning failure, not a reason to silently enlarge tolerance. Record stopping time and residual counts, not just a successful I²C write.

Zero-speed hold and zero-PWM release are separate follow-up observations. The I²C hold/release distinction is inferred from UART documentation and needs bench verification; do not label either as proven in advance.

### C. Matched clock/rate comparison

Only after B passes, compare this matrix without changing the motor type, setpoint, configuration, harness, or logging mode:

| I²C clock | Host command/read rate | Purpose |
| --- | --- | --- |
| 100 kHz | 50 Hz | Baseline |
| 100 kHz | 100 Hz | Test 10 ms host period at normal bus speed |
| 400 kHz | 50 Hz | Isolate bus-clock effect |
| 400 kHz | 100 Hz | Test 10 ms host period at fast bus speed |

At 400 kHz, start with idle reads/zero commands before repeating the same motion/stop cycles. No clocks above 400 kHz in this experiment. Change clock only while stopped. If fast-mode transactions fail, abort that run, attempt a bounded stop, and return to 100 kHz for diagnosis; do not silently downgrade and label the result a 400 kHz success.

Run the three motion selections (M2, M4, both) with three repetitions per matrix cell. Once validated, collect a 30 s both-motor timing run per cell, with a local duration limit, then a verified stop. This supplies enough observations for useful latency distributions. Report sample count with all percentiles.

100/400 kHz is the requested **bus clock**; 50/100 Hz is the **host loop rate**. Neither proves the motor board's PID frequency or fresh feedback rate. A nominal 10 ms encoder window may repeat or be missed depending on device phase/read semantics. Cumulative differences over actual elapsed time provide the motion cross-check; do not blindly sum recent counts at 50 Hz as odometry.

## Automation and bounded execution (future implementation)

- Python runner in this folder; project Python dependencies installed in repository `.venv/`. Reuse the existing PlatformIO Core executable for uploads instead of installing a second copy unless needed.
- Separate commissioning from benchmark mode. Validate pins, command units, decoding, and setpoint before enabling the benchmark.
- Explicit run ID and parameters; ready → start acknowledgment → phase/sample records → completion. No automatic start on reset or USB connection.
- Firmware enforces per-phase and total run deadlines, bounded I²C calls/retries, and stop attempts on completion, cancellation, feedback failure, or heartbeat timeout. Proposed host heartbeat timeout: 500 ms, to verify during commissioning.
- Sample on device monotonic time. Buffer serial output so host logging delays do not schedule motor control; log dropped records and missed deadlines.
- Firmware reset or a failed I²C bus may leave the separate driver holding its previous command. Confirm a driver watchdog/enable mechanism if available; a local ESP32 deadline alone is not a guarantee after an ESP32 crash. Keep motor-power removal available during commissioning.
- Do not reset cumulative counts unless the vendor explicitly documents a reset command. Use baseline subtraction with verified wrap semantics.

## Data and success criteria

Each run uses `results/<UTC-run-id>/` with `metadata.json`, `serial.log`, `samples.csv`, `summary.md`, and optional timing/encoder plots. Suggested CSV fields:

```text
run_id,phase,sample_index,device_time_us,clock_hz,target_loop_hz,
m2_command,m4_command,m2_recent,m4_recent,m2_cumulative,m4_cumulative,
m2_delta,m4_delta,battery_raw,write_us,m2_recent_read_us,m4_recent_read_us,
m2_cumulative_read_us,m4_cumulative_read_us,battery_read_us,cycle_us,
period_us,deadline_missed,i2c_status,feedback_valid
```

Store raw read bytes/status for each transaction in structured diagnostic records in `serial.log`; use blank decoded fields for invalid reads, not zero. Metadata includes byte/word-order decisions, snapshot algorithm, per-call timeout, motor configuration, setpoint units, pins, supply, firmware revision, PlatformIO version, and phase limits.

Pass criteria are deliberately split:

- **Functional:** selected motor moves in the observed forward direction; recent counts and cumulative deltas agree on motion; unselected motor remains at its stationary baseline; zero speed settles within the stated stop criterion. Encoder evidence demonstrates shaft motion, not necessarily ground travel.
- **Transport:** zero NACKs, short reads, timeouts, or invalid snapshots in the acceptance run at the selected clock. Retain failures and retry counts in the report.
- **Timing:** report min/median/p95/p99/max command, read, complete-cycle time, actual period, and deadline misses. Claim a demonstrated 50/100 Hz motor-interface workload only if it meets that period without missed deadlines in the measured run. Reserve time for other peripherals before claiming an integrated robot-loop rate.
- **Closed-loop accuracy:** deferred until command units and counts-to-speed scaling are validated. Raw encoder movement alone is not proof that the requested physical speed is being regulated accurately.

End-to-end command-to-first-motion and stop-to-stationary observations include motor dynamics and the feedback window, so report them separately from I²C transaction latency. Do not promise a fourfold overall improvement from a fourfold bus clock increase.
