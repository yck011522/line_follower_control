# E2 - I2C motor speed mapping

E2 measures input-to-output speed response with fixed settings. The ESP32 talks
to the driver only over I2C (D4 SDA / D5 SCL, 400 kHz, address 0x26). PC USB
serial is used only to start/cancel the ESP32 experiment and capture its log.
No motor-driver UART wiring or serial commands are used.

## Configuration

On every ESP32 boot, firmware writes type **1**, dead zone **1650**, pulse line
**2000**, pulse phase/ratio **23**, and wheel diameter **65 mm**, requiring all
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
```

COM4 is the historical ESP32 port; check the actual port after reconnecting.
Close other serial monitors. Lift the wheels clear: capture.py explicitly starts
the sweep after the idle/configured handshake. The firmware itself never starts
motion automatically at boot. Python sets DTR on / RTS off **before opening**.

## Fixed sweep

- Both M2 and M4 receive the same signed command; M1 and M3 receive zero.
- Targets: **-100, -90, ... -10, 0, 10, ... 90, 100** (21 points).
- Each trial: **1 s at zero, 1 s settling at target, 2 s measuring at target,
  1 s back at zero**. Total approximately **105 s**.
- Commands and encoder reads run at 50 Hz; actual per-motor read timestamps are logged.
- Commands: `start`, `stop`, `!` (immediate cancellation), `status`, `config`, `help`.
- No range overrides, dead-zone/line/ratio/diameter setters, PWM sweeps or indefinite holds.

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

`estimated mm/s = counts/s * pi * 65 / (2000 * 23)`

This assumes 46000 counts/output revolution and does not apply another x4 factor.
It is **not independently calibrated physical speed** or the UART `$MSPD` value.
The dashed y=x line is a nominal reference, not proof of correct scaling. Error
bars show sample-to-sample velocity standard deviation, not confidence intervals.
Calibrate one known wheel revolution before treating these values as odometry.

Earlier E2 logs, including wider sweeps and PWM investigations, remain historical
artifacts. The current analyzer expects schema=2 and does not relabel old results.

## Verification

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s test/E2_motor_speed -p "test_*.py"
```

The revised firmware must still be validated on the connected ESP32 and motor
board. Synthetic tests establish software handling, not new hardware measurements.
