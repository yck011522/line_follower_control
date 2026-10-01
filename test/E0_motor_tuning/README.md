# E0 - Motor PID tuning over direct USB

Committed findings and plots: [results.md](results.md).

This precedes E1 communication and E2 speed sweeps. Connect the motor driver
directly to the PC (currently **COM5**) with the ESP32 disconnected. No firmware
build/upload is needed. Protocol source: [Motor Driver Serial Commands.pdf](../../reference/Motor%20Driver%20Serial%20Commands.pdf).

## Run one PID candidate

Close the serial terminal so Python can open COM5. Lift the drive wheels clear
and keep motor power accessible. The script starts motion when invoked.

The port uses **DTR on, RTS off**, matching the working serial monitor. These
levels are set before opening the port and retained through PID-triggered resets.
Disabling RTS/CTS flow control alone does not turn RTS off.

From the repository root:

```powershell
.\.venv\Scripts\python.exe -m pip install -r test/E0_motor_tuning/requirements.txt
.\.venv\Scripts\python.exe test/E0_motor_tuning/run.py --port COM5 --pid 3 3 0.5
```

The example reproduces your last candidate; it is not a recommended optimum.
Choose another candidate with `--pid P I D`. Defaults are **M2 only**, speeds
**20, 40, 60, 80**, **1 second baseline**, **3 seconds driving**, and **2 seconds
at zero** after each step. All other channels receive zero setpoints.

```powershell
.\.venv\Scripts\python.exe test/E0_motor_tuning/run.py --pid 1 6 0 --motor 2 --speeds 20 40 60 80 --duration 3 --baseline 1 --rest 2
```

Use `--motor 4` for a separate right-motor run. The protocol exposes one PID
triplet for the board, not separate per-channel gains. PID changes persist in
flash; the script leaves the candidate installed, recording the previous values
in `metadata.json` so you can deliberately restore them later.

## Sequence

1. Open 115200 baud, 8N1, no flow control, DTR on, RTS off. Send zero speed,
   zero PWM, and disable upload, allowing 150 ms between these commands.
2. Read flash and require motor type 1, dead zone **1650**, pulse line **2000**,
   gear ratio (`Pulse_Phase`) **23**, and wheel diameter **65 mm**. Abort on mismatch;
   the runner does not rewrite those settings.
3. Send `$MPID:P,I,D#`, allow two seconds for reset, then query flash with bounded
   retries. Require the expected settings and PID readback before continuing.
4. Send zero speed and `$upload:0,0,1#`; verify feedback before any nonzero command.
5. For each target, record a zero-speed baseline, command the target and record
   three seconds, then command zero and record the rest interval. The zero-speed
   interval is timed, not a claim that the wheel has physically settled; increase
   `--rest` if the plot shows it has not stopped before the next trial.
6. On completion, Ctrl+C, or error, attempt zero speed, zero PWM (release), and
   upload off independently, draining queued output with a timeout and allowing
   150 ms for each command to be processed before closing. Save partial data and
   mark failures incomplete. Do not manually toggle DTR or RTS at shutdown.

No valid speed packet for one second aborts collection. A disconnected USB cable
or crashed host can prevent stop commands reaching the board; the supplied
protocol does not document a motor watchdog. Cut motor power if needed.

### Check repeated connections without starting a motor test

```powershell
.\.venv\Scripts\python.exe test/E0_motor_tuning/run.py --port COM5 --check-port 3
```

This opens the port, reads flash, and closes it three times. It sends only
`$read_flash#`: it does not change PID, reset the board, or command motor outputs.
It also does not stop motion that was already active. Each connection has a raw
log in a new `results/check_<time>/` folder, with `connection_check.json` recording
the readbacks and outcome.

An earlier version left RTS at pySerial's default on state. In the failed run
`20260930T082809_320758Z`, the initial flash query succeeded, but after the PID
write the driver printed `Reset MCU` and never returned its normal version
banner or answered further queries. That evidence points to a reboot/control-line
problem rather than establishing flash corruption. The corrected implementation
matches the monitor's known working line settings. The OS/USB driver may still
change physical line levels at open/close; Python cannot guarantee their state
while the port is closed.

Hardware verification on 2026-09-30 passed three read-only reconnects, then two
PID-triggered resets using the already-stored P=1/I=3/D=0, followed by two more
read-only reconnects. No nonzero speed commands were sent during these checks.
Logs are in `results/check_20260930T083241_890919Z/` and
`results/reset_check_20260930T083314_718050Z/` (generated results are Git-ignored).
This supports the fix; it does not establish the board's electrical reset wiring.

The PDF distinguishes zero speed (PID remains active) from zero PWM (release).
E0 deliberately keeps zero-speed PID active between trials so zero-speed
oscillation is visible, and releases the outputs at the end.

**The historical E1 firmware configures 500 encoder lines at startup.** It can
replace the 2000-line setup used here. Revised E2 reapplies the adopted 2000-line
setting. The motor's 500 ppr specification and the driver's configured 2000 are
distinct quantities; see the [project motor baseline](../../docs/motor-settings.md).

## Results and comparison

Each run creates a unique folder under `results/` with:

- `serial.log`: timestamped TX commands and original RX chunks, escaped to preserve boundaries.
- `samples.csv`: host arrival time, trial, phase, command, and all four motor speeds.
- `metadata.json`: settings before/after PID reset, candidate, command times, outcome, and errors.
- `speed.png`: one subplot per started speed step, showing measured speed and target including baseline and stop.
- `summary.md`: final-second mean, mean error, standard deviation, and peak-to-peak variation.

Replot saved data without opening COM5:

```powershell
.\.venv\Scripts\python.exe test/E0_motor_tuning/run.py --plot test/E0_motor_tuning/results/<run-folder>
```

Compare candidates with the same wheel load and supply conditions. First inspect
overshoot, repeated oscillation, ramp-up, and return to zero. Change one gain at a
time and retain each run. Three seconds may not establish steady state; extend
`--duration` if needed. Final-second variation alone does not prove oscillation.

Speed axes use the driver's reported units: the serial section does not explicitly
define their physical scale. Do not apply E2's encoder conversion to `$MSPD`.
Times are PC receive times, not device sampling times; packets arriving together
share a timestamp. Your sample log suggests roughly 60 ms uploads, which cannot
resolve all fast internal PID behavior. The plot preserves unsmoothed samples.

Compare saved runs with a shared vertical scale for the 20-speed plots:

```powershell
.\.venv\Scripts\python.exe test/E0_motor_tuning/compare.py --out test/E0_motor_tuning/results/comparison test/E0_motor_tuning/results/<run1> test/E0_motor_tuning/results/<run2>
```

This writes `comparison.csv`, `comparison.md`, and `low_speed_comparison.png`.
RMSE includes both average tracking error and variation; rest RMS measures
residual motion feedback during the final second after commanding zero.
