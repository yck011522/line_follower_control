# Initial bench results — 2026-09-30

The interactive tester built and uploaded successfully to XIAO ESP32S3 on COM4. The user confirmed the driver was powered and both wheels lifted. These are commissioning observations, not a speed-accuracy or long-duration reliability benchmark.

## Results

| Check | Observation |
| --- | --- |
| 100 kHz I²C | Address `0x26` acknowledged; probe approximately 152–154 µs |
| Read transaction | STOP-separated pointer write/read returned zeros even while wheels were turned. Repeated START returned real counts. Alternating modes reproduced the difference |
| Manual rotation | Both channels responded separately; recent counts and signed cumulative counts changed, including crossing zero |
| Complete encoder snapshot | Approximately 4.37–4.40 ms for two recent-count reads plus two high–low–high cumulative reads at 100 kHz |
| M2 +20-unit, 250 ms pulse | Cumulative count -96017 → -95884 (+133); M4 unchanged during the pulse/stop observation |
| M4 +20-unit, 250 ms pulse | Cumulative count -139343 → -138961 (+382); M2 unchanged during the pulse/stop observation |
| Stop commands | ACK received; write approximately 971–972 µs. Both pulses reached the 200 ms stationary observation criterion 234 ms after stop |
| Transport errors | Zero reported in the successful manual/motor sequence; final sequence counter was 1062 feedback samples |
| Final firmware check | Re-uploaded with repeated START as default; probe/read worked without a mode command. Unarmed `m2` was refused. Final zero-PWM release acknowledged |

Configuration requested: type 1, 500 encoder lines, ratio 23, diameter 65 mm; uint16 configuration fields big-endian, float little-endian. All writes acknowledged. The correct interpretation of configuration values, physical speed units, count scaling, motor-direction convention for vehicle forward, and tuning remain unvalidated. Different count changes between motors are a reason to investigate response/scaling later, not evidence of accurate equal-speed control.

Recent-count fields in the printed short-pulse lines were zero despite cumulative movement; the 5 Hz display can miss motion inside the driver's nominal 10 ms window. Manual rotation did produce nonzero recent counts. No claim is made about the detailed pulse speed response from this sparse display.

After release, subsequent counts shifted slightly (M2 to -96029; M4 to -139003). The active-stop criterion applies to the observation interval, not indefinitely after releasing the motors.

## Evidence and reproducibility

Local captures (generated/ignored by Git) are in `results/20260930T040746Z_config/`: `serial.log`, `manual-restart.log`, `m2-pulse.log`, `m4-pulse.log`, and `final-firmware.log`. The earlier all-zero capture is `results/20260930T040502Z_manual/serial.log`.

Base revision: `35d479d11e58982f0f4331f7b1996ed07f7c3cde`, with uncommitted tester changes. PlatformIO Core 6.1.19, Espressif32 6.12.0, Arduino package 3.20017.241212+sha.dcc1105b. Final uploaded `firmware.bin` SHA256:

```text
8E2C94B890FE297A0BD473DAF7E1A7437620B3DE2B66DAE37526D08E018A7500
```

The pulses used the first build with repeated START selected through the serial menu. The final build changes that mode to the default and updates the help text; its non-motion checks were repeated after upload. No 400 kHz run or physical mm/s calibration was performed.
