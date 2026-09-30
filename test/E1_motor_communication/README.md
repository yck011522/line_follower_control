# E1 — Interactive motor tester

One firmware upload supports all initial checks. Firmware: [main.cpp](../../src/e1_motor_communication/main.cpp); reusable interface: [MotorDriver](../../lib/MotorDriver/include/MotorDriver.h). No automated Python runner or serial-data parser is implemented. Initial captures use a plain serial connection.

The first communication/manual/pulse checks are in [initial results](INITIAL_RESULTS.md). The current five-second both-motor runs at 100/400 kHz are in [longer-run results](LONG_RUN_RESULTS.md). Both clocks worked, but the strict zero-speed stop criterion failed due to small encoder oscillations; zero-PWM release settled the outputs.

## Upload and monitor

Run these separately from PowerShell in the repository. Close any serial monitor before uploading:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e e1_motor_communication -t upload --upload-port COM4
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor -e e1_motor_communication --port COM4 --echo
```

Alternatively use the PlatformIO serial monitor in VS Code. Send commands with Enter. Use Ctrl+C to exit the monitor. COM4 was detected during initial commissioning; recheck after reconnecting. Monitor DTR/RTS are disabled in the project configuration to avoid unintended reset requests.

## Initial sequence

Keep the powered robot's wheels lifted. Startup automatically releases the outputs, configures type 1 / 500 encoder lines / ratio 23 / 65 mm diameter at 400 kHz, then releases the outputs again. It does not start motion. If the driver was not powered at startup, power it and send `config-be`; `status` must show `configured=1` before arming.

1. `probe` — expect `err=0` at address `0x26`, default 400 kHz, SDA D4/SCL D5.
2. `release`, then `watch` — zero PWM; observe M2/M4 recent and cumulative counts while turning each wheel by hand separately. Reads run at a requested 50 Hz; display is 5 Hz. `quiet` stops the display/read loop.
3. `config-be` — first experimental configuration: type 1, 500 encoder lines, integer ratio 23, diameter 65 mm. Writes are separated by 100 ms. ACK does not prove the intended values were applied.
4. `arm`, then `m2` — one +100 driver-unit run for 5 seconds, then zero speed. Observe motion and count direction. Wait for the stop observation before proceeding.
5. `arm`, then `m4` — same check for M4. `arm`, then `both` runs both motors together. M1/M3 remain zero. Positive wheel directions may differ mechanically.
6. `100` or `400` selects the I?C clock in kHz while stopped, probes the driver, reads feedback, and disarms. Repeat `arm` / `both` at each clock. Speed commands are refreshed at 50 Hz during a run.
7. `stop` sends zero speed; `!` does so without waiting for Enter. `release` sends zero PWM after testing. `status` reports sample/error/output-drop counters.

Each pulse consumes its arm. Communication errors abort a pulse. USB disconnection also triggers a stop attempt. A failed bus or crashed controller can prevent stopping; keep motor power accessible.

The stop observer requires both recent counts to be zero and cumulative counts unchanged for 200 ms within 2 s. A zero-speed ACK alone is not reported as a physical stop. Encoder activity must first be validated; an unresponsive encoder can also appear stationary.

## Experimental choices

- Speed/PWM commands and feedback words are big-endian. Cumulative counts use high–low–high reads, with at most two attempts for coherence.
- `config-be` uses big-endian uint16 configuration fields as the initial bench assumption. `config-le` changes only those fields for local diagnosis; never change configuration while moving.
- Diameter is a little-endian 32-bit float: 65 mm. Gear setting 23 approximates 22.569:1; physical speed is uncalibrated. Encoder counting conventions also need measurement.
- Command 100 is labelled **driver units**. mm/s is the working hypothesis, not a measured fact.
- Default reads use repeated START, verified on the connected driver. STOP-separated reads returned all zeros. `split` remains a diagnostic option; `restart` restores the working mode without re-uploading.
- Wire timeout is 5 ms per call. The 5 s run deadline is checked between feedback operations; a blocking call/snapshot may add an overrun. This is not a hard real-time stop guarantee.
- Each run prints read/write count, mean, and maximum transaction time. Automated runners remain later work.

Use `help` for the menu. Raw captures go under `results/`; findings are tracked in [progress](../../docs/progress.md). The broader [test plan](PLAN.md) remains the roadmap.
