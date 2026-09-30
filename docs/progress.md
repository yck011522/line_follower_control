# Testing and development progress

Status vocabulary: **planned**, **in progress**, **blocked**, **complete**. Mark experiments complete only when implementation and recorded hardware evidence support the outcome.

| Stage | Status | Deliverable / evidence |
| --- | --- | --- |
| Repository organization | Complete | Main README, documentation, library/application/experiment folders |
| Requirements capture | Complete | Initial brief captured; unresolved items tracked in open questions |
| Board and interface confirmation | In progress | XIAO ESP32S3 and D-label wiring confirmed: motor D4/D5, line sensor D6/D7; motor reference supplied; read formats pending; E3 sensor deferred |
| PlatformIO and host-runner setup | In progress | Core 6.1.19; E1 builds/uploads on COM4; interactive serial works; automation deferred |
| E0: motor PID | Complete for baseline selection | PID 3/0.375/0.5 adopted; [evidence](../test/E0_motor_tuning/TUNING_RESULTS.md); target 10 and loaded steering unresolved |
| E2: low-speed mapping | In progress | Fixed I2C -100..100 sweep, boot settings, Python capture and plots; revised hardware test pending |
| E3: line sensor | Planned | Driver, firmware, Python runner, CSV and timing summary |
| E1: motor driver (first) | In progress | [Five-second runs](../test/E1_motor_communication/LONG_RUN_RESULTS.md): 100/400 kHz communication and both-motor motion verified; zero-speed hold oscillates; scaling/tuning pending |
| E4: NFC reader | Planned | UID-only reader, present/absent timing and event behavior |
| E5: wireless | Planned | USB bridge and radio tests, then four-robot load measurements |
| Integrated control | Planned | Steering PID, navigation states, command handling and fault behavior |
| Driving validation | Planned | 100 mm/s tracking on 14 mm lines and 30–50 mm turns; higher-rate/speed trials |

## Development log

### 2026-09-30 - E0 baseline and E2 simplification

- Adopted type 1, dead zone 1650, pulse line 2000, ratio 23, diameter 65 mm, and
  PID 3/0.375/0.5. [Project reference](motor-settings.md) distinguishes measured
  improvements from unresolved target-10 hunting and untested loaded steering.
- User chose I2C-only E2 and explicit reliance on saved PID. Reapply the other
  five settings at every ESP32 boot. PID is recorded as assumed, not read back.
- Simplified E2 to start/stop/status/config/help with a fixed -100..100 step-10
  sweep, 1 s baseline, 1 s settling, 2 s measuring, and 1 s zero rest per target.
- Added schema-2 capture, encoder speed analysis, and command-to-speed plots.
  Historical logs/plots are preserved. Revised firmware has not been bench-run.
- Added a root AGENTS.md rule: all Python test ports set RTS off before opening.


### 2026-09-30 — Longer runs at two bus clocks

- Added automatic startup configuration, five-second runs at command 100, both-motor selection, 50 Hz command refresh, and runtime 100/400 kHz selection.
- Built/uploaded and ran both motors at each clock. Measured 4.396/1.411 ms mean feedback reads and 0.970/0.273 ms mean speed writes respectively, with no reported feedback errors.
- Strict zero-speed stationary criterion failed due to small continuing count oscillations. Released outputs with zero PWM; final recent counts were zero. See [results](../test/E1_motor_communication/LONG_RUN_RESULTS.md).

### 2026-09-30 — First hardware tests

- Built/uploaded one interactive firmware and shared motor-driver interface. No automated runner added.
- Verified D4/D5 bus at 100 kHz. Reproduced all-zero feedback with STOP-separated reads and fixed it by using repeated START.
- Observed both encoders during manual rotation; tested M2 and M4 individually at +20 driver units for 250 ms, followed by zero-speed stop observation and zero-PWM release.
- Used the supplied 65 mm wheel diameter and provisional ratio 23/encoder lines 500/type 1. Transport worked; physical speed calibration is still open.
- Final firmware uploaded with repeated START enabled by default. Detailed evidence is in [initial results](../test/E1_motor_communication/INITIAL_RESULTS.md).

### 2026-09-30 — Motor-first plan and host discovery

- Moved E1 ahead of E3 while retaining experiment IDs.
- Read the supplied GT50/motor-driver interface and designed M2/M4 motion, encoder, stop, and 100/400 kHz tests.
- Verified PlatformIO Core 6.1.19, installed Espressif32 6.12.0/XIAO board definition, and one enumerated serial device on COM4.
- Added the E1 PlatformIO configuration using Arduino as the initial framework choice. No firmware build/upload, serial connection, reset, or motor commands performed.
- Corrected wiring to the user's clarified D-label convention: motor SDA D4/SCL D5 (GPIO5/GPIO6), line-sensor SDA D6/SCL D7 (GPIO43/GPIO44). Motor type 1 remains a commissioning candidate; command units, read decoding, and motor scaling need confirmation before motion.
- Verified the supplied pinout image against the installed Arduino board variant. Removed the earlier inferred line-sensor/SPI conflict: D7 and D8 map to distinct GPIO44 and GPIO7.

### 2026-09-30 — Initial structure

- Captured system architecture, pins, message semantics, branch commitment, and performance targets.
- Created reusable-library and per-application/per-experiment directory conventions.
- Defined the proposed automated test and result workflow.
- No hardware tests run; no measured rates or latency claims yet.

## Future experiment entries

For each run worth retaining, add its date, experiment/environment, firmware revision, hardware configuration, result path, outcome, measured timing/error statistics, and next decision. Record failed or incomplete runs when they explain a blocker. Keep design targets separate from measurements.
