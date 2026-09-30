# Peripheral testing and automation

## Layout and build plan

Each experiment has firmware in `src/eN_name/` and a Python runner plus generated results in `test/EN_name/`. Shared drivers belong in `lib/`. Proposed PlatformIO environment names match firmware directory names; `robot` and `bridge` are reserved for integration. E4 may need distinct robot and bridge environments.

Once board/framework details are known, configure each environment to compile only its intended entry point using source filters. Hardware drivers must not be copied between experiments and integration. Keep device addresses, bus objects, and pin configuration explicit at application boundaries.

The `test/` directory is intended for host-driven hardware experiments. If PlatformIO unit testing is added later, configure discovery separately so these Python experiments are not mistaken for embedded unit tests.

## Intended Python workflow

1. Accept an explicit serial port, environment, duration/sample count, and experiment parameters. Support skipping upload for repeat runs.
2. Invoke PlatformIO to build/upload the selected firmware to that port, recording its outcome.
3. Reset/reconnect using the confirmed board's USB behavior. Close upload handles before opening the serial listener; use bounded retries for re-enumeration.
4. Wait for a firmware-ready handshake with a timeout. Reset alone must not start data collection or motor motion.
5. Send an explicit start request with a run ID and parameters. Firmware confirms the request before sampling.
6. Capture structured records until a completion response, timeout, disconnect, or user interruption. Record failures as incomplete runs.
7. Write raw captures, CSV measurements, a Markdown summary, and optional Matplotlib plots to the experiment's results directory.
8. Stop activity and release the port on completion/interruption. E2 firmware also needs a bounded local motor run so host disconnection cannot leave a test running indefinitely.

The exact serial command/record schema remains to be designed with E1. It should distinguish ready, start acknowledgment, samples, errors, and completion, and include sample indices plus device timestamps. Do not depend on arbitrary startup sleep durations alone.

Proposed future runner name: `run.py` in each experiment folder. No runner or command schema exists yet.

## Results convention

```text
test/E1_line_sensor/results/<UTC-run-id>/
  metadata.json      Run parameters and hardware/software identity
  serial.log        Original serial capture
  samples.csv       Multi-row measurements
  summary.md        Outcome, statistics, and interpretation
  timing.png        Optional plots
```

Use a unique directory per run; never overwrite previous evidence. Record UTC start time, firmware Git revision and dirty status, board/device models, PlatformIO environment, serial settings, bus clocks, parameters, requested sample count, and completion status. Generated results are ignored by Git by default; intentionally add selected reference runs when needed and link them from the progress log.

## Measurements

- Use device-side monotonic timestamps around peripheral calls to measure operation duration. Capture actual sample intervals, failures, timeouts, and deadline misses.
- Report sample counts, achieved rate, min/median/p95/p99/max durations, and missed deadlines against both 20 ms and 10 ms budgets where applicable. Small runs should identify percentile limitations.
- Distinguish transaction duration, device conversion/update rate, and end-to-end freshness. Repeated register reads do not prove fresh sensor samples.
- Record host serial arrival times separately. Buffered serial delivery is not a direct measurement of peripheral latency; measure and account for logging overhead.
- For radio/USB round trips, time request and matching response on the same originating clock. A send callback alone is not an application round trip. Do not infer one-way latency by halving round-trip time without a justified model.
- Include both successful and unsuccessful operations. NFC tag absence, read errors, and timeouts must be distinguishable.

## Experiment sequence

| Experiment | Initial scope | Evidence needed before integration |
| --- | --- | --- |
| E1 line sensor | Read all eight channels, record values and timing; compare requested rates | Correct channel order/polarity, fresh-data behavior, bus errors, latency distribution, measured sustainable rate |
| E2 motor driver | Command bounded wheel speeds, read speed and battery telemetry where supported | Protocol/units verified; write/read durations, physical stop behavior, feedback availability |
| E3 NFC reader | Read UID only, with present/absent and repeated-tag cases | UID correctness, repeat detection behavior, present/absent latency, timeout impact; later test moving passes |
| E4 wireless | USB bridge ↔ robot request/response, then four-robot traffic | Round-trip latency, loss, duplicates, ordering, sequence acknowledgments at 50 Hz downlink and 20 Hz per robot uplink |

After individual tests, measure the complete control cycle under concurrent sensing, motor traffic, wireless traffic, and telemetry. Individual operation timings alone do not establish the integrated loop budget. Bench-validate motor commands with wheels clear before driving tests; confirm wheel direction and stop behavior first.
