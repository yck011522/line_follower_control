# E2 — Motor Speed Command Test

E2 is a separate application from [E1 motor communication](../E1_motor_communication/README.md). Both reuse `lib/MotorDriver`. Default motor-bus clock is now **400 kHz**.

## Run

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e e2_motor_speed -t upload --upload-port COM4
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor -e e2_motor_speed --port COM4 --echo --filter log2file
```

Close any other serial connection before using the monitor. `log2file` records the terminal session; for analysis retain exactly one sweep and remove any monitor prefixes or echoed commands, leaving CSV rows and `#` comment records. The local bench capture uses plain serial bytes without these additions.

With the wheels lifted, send `status` and check `configured=1`, then `start`. The sweep drives both motors with identical signed commands: **-240, -220, …, 0, …, +220, +240**. Each of 25 steps lasts two seconds: one second allowed for settling and one second measured. Total duration is about 50 seconds. It includes reverse motor rotation; signed commands are not yet mapped to vehicle forward/backward.

Commands: `help`, `status`, `config` (retry startup configuration), `start`, `stop`. `!` immediately requests cancellation without Enter. Motion never starts automatically. Startup writes type 1, 500 encoder lines, integer ratio 23, and 65 mm diameter, then releases outputs. Configuration uint16 values are BE; diameter float is LE. These remain commissioning settings with uncalibrated physical scaling.

Speed commands refresh at 50 Hz; feedback samples at 50 Hz. The firmware logs both settling and measuring rows with per-motor device timestamps, recent counts, cumulative counts, and transaction timings. Completion, cancellation, USB disconnect, or a transaction error attempts zero speed followed by zero PWM release. A 60-second overall deadline bounds a stalled sweep. This does not guarantee stopping if I²C fails or the ESP32 crashes.

## Analyze a raw log

```powershell
.\.venv\Scripts\python.exe test/E2_motor_speed/analyze.py test/E2_motor_speed/results/<run>/serial.log
```

The standard-library analyzer creates `samples.csv`, `summary.csv`, and `summary.md` beside the log. It rejects missing steps and reports incomplete/error/dropped-data runs. It does not upload firmware or command motors.

Primary measured speed is signed **encoder counts/second**, computed from cumulative differences over actual time using only rows from 1–2 seconds into each step. Recent-count statistics and first-half versus second-half speed change are also retained to assess whether settling is adequate. One second is an initial allowance, not proof of settled speed. Zero-speed PID oscillation observed in E1 is not filtered away.

The optional mm/s estimate uses wheel circumference `pi * 65` and an explicit assumed counts/output revolution: `500 * 4 * 22.569 = 45138`. The factor of four is **not yet calibrated**. Override with `--counts-per-rev` after measuring a known wheel revolution. The driver is configured with integer gear ratio 23, so requested speed scaling is also approximate. Do not treat agreement in this estimate as proof that the command is calibrated mm/s.

Determining which sign corresponds to vehicle forward is a separate observation/test; E2 labels signs as encoder/command polarity only.
