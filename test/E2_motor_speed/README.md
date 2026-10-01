# E2 - I2C motor speed mapping

Committed findings and plots: [results.md](results.md).

E2 measures input-to-output speed response with fixed settings. The ESP32 talks
to the driver only over I2C (D4 SDA / D5 SCL, 400 kHz, address 0x26). PC USB
serial is used only to start/cancel the ESP32 experiment and capture its log.
No motor-driver UART wiring or serial commands are used.

## Configuration

On every ESP32 boot, firmware writes type **1**, dead zone **1650**, pulse line
**500**, pulse phase/ratio **23**, and wheel diameter **65 mm**, requiring all
I2C acknowledgments. `config` retries these same fixed values if driver power
was absent at boot. There are no interactive tuning or PWM/manual-speed options.

**PID 3/0.375/0.5 is assumed already saved in the driver.** I2C cannot read/write
PID in the supplied register map. `configured=1` means the I2C writes succeeded;
it does not verify stored PID or read back the write-only settings. Replacing or
factory-resetting the board invalidates the stored-PID assumption. See
[project motor settings and E0 findings](../../docs/motor-settings.md).

## Build, upload, run

With ESP32 reconnected to the driver over the existing I2C wiring:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e e2_motor_speed
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e e2_motor_speed -t upload --upload-port COM4
.\.venv\Scripts\python.exe -m pip install -r test/E2_motor_speed/requirements.txt
.\.venv\Scripts\python.exe test/E2_motor_speed/capture.py --port COM4
.\.venv\Scripts\python.exe test/E2_motor_speed/capture.py --port COM4 --range -50 50 10
```

COM4 is the historical ESP32 port; check the actual port after reconnecting.
Close other serial monitors. Lift the wheels clear: capture.py explicitly starts
the sweep after the idle/configured handshake. The firmware itself never starts
motion automatically at boot. Python sets DTR on / RTS off **before opening**.

## Sweep

- Both M2 and M4 receive the same signed command; M1 and M3 receive zero.
- `start [min max inc]`: targets from min to max in steps of inc. Default
  **-100 to 100 step 10** (21 points). Limits: |value| <= 1000 (the driver ignores
  larger speeds) and at most 200 points. `capture.py --range MIN MAX INC` sends it.
- Each trial takes 5 s per target (about 105 s for the default); see the phases below.
- Commands and encoder reads run at 50 Hz; actual per-motor read timestamps are logged.
- Commands: `start [min max inc]`, `stop`, `!` (immediate cancellation), `status`, `config`, `help`.
- No dead-zone/line/ratio/diameter setters, PWM sweeps or indefinite holds.

### Trial phases

Every 20 ms sample is labelled with the phase of its trial; `step_ms` is the time since the trial started.

| Phase | `step_ms` | Speed command sent | Purpose |
| --- | --- | --- | --- |
| baseline | 0-1000 | 0 | Starts each trial from the same zero-speed state; shows creeping or hunting at a zero target. |
| settle | 1000-2000 | target | Lets the speed loop reach the target. Logged but not used for the result. |
| measure | 2000-4000 | target | The window the analyzer uses for speed (cumulative count change over elapsed time). |
| rest | 4000-5000 | 0 | Returns to zero before the next target; shows how the motors stop. |

In `samples.csv`, `command` is always the trial's target and `applied` is what was actually sent
(0 during baseline and rest). "Zero" is a speed target of 0, so the driver's PID keeps holding the
wheels; outputs are not released until the run ends. One second of settling is an allowance, not proof
that the speed has stopped changing.

A timed zero interval is not proof that the wheel has physically stopped.
Zero-speed PID remains active during baseline/rest to expose hunting. Completion,
USB disconnection, cancellation, I2C errors or the 110 s local deadline attempt
zero speed then zero-PWM release. A failed bus or crashed ESP32 cannot guarantee
physical stopping. Python also attempts cancellation on timeout/interruption.
Signed motor commands have not been mapped to mounted vehicle forward direction.

## Results

capture.py saves a unique `results/<UTC-run>/` folder containing `serial.log`,
`metadata.json`, `samples.csv`, `summary.csv`, `summary.md`, and `summary.png`.
Logs use **schema=2**, including the fixed settings and `pid_source=stored_unverified`.
Failures retain raw logs and incomplete status. The analyzer rejects missing
phases/steps, broken timing, wrong settings or partial data; reported drops,
missed ticks, stop failures and row-count mismatches mark otherwise usable data degraded.

Reanalyze without opening any port:

```powershell
.\.venv\Scripts\python.exe test/E2_motor_speed/analyze.py test/E2_motor_speed/results/<run>/serial.log
.\.venv\Scripts\python.exe test/E2_motor_speed/plot.py test/E2_motor_speed/results/<run>
```

Measured speed is the signed cumulative encoder difference divided by actual
elapsed device time, separately for M2 and M4. Only the measurement phase
(trial time 2-4 s) contributes to the mapping. Raw **counts/second** is retained.

For a nominal input/output comparison, the graph also shows:

`estimated mm/s = counts/s * pi * 65 / 44998`

44998 is the hand-measured counts per wheel revolution (500 ppr x 4 x 22.5), independent of the
driver's line and ratio settings; override with `analyze.py --counts-per-rev`.
It is **not independently calibrated physical speed** or the UART `$MSPD` value.
The dashed y=x line is a nominal reference, not proof of correct scaling. Error
bars show sample-to-sample velocity standard deviation, not confidence intervals.
Calibrate one known wheel revolution before treating these values as odometry.

Earlier E2 logs, including wider sweeps and PWM investigations, remain historical
artifacts. The current analyzer expects schema=2 and does not relabel old results.
