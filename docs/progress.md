# Testing and development progress

Status vocabulary: **planned**, **in progress**, **blocked**, **complete**. Mark experiments complete only when implementation and recorded hardware evidence support the outcome.

| Stage | Status | Deliverable / evidence |
| --- | --- | --- |
| Repository organization | Complete | Main README, documentation, library/application/experiment folders |
| Requirements capture | Complete | Initial brief captured; unresolved items tracked in open questions |
| Board and interface confirmation | In progress | XIAO ESP32S3 and D-label wiring confirmed: motor D4/D5, line sensor D6/D7; motor reference supplied; read formats pending; E1 sensor deferred |
| PlatformIO and host-runner setup | In progress | Core 6.1.19 and COM4 enumerated; E2 configuration added; firmware/runner not implemented |
| E1: line sensor | Planned | Driver, firmware, Python runner, CSV and timing summary |
| E2: motor driver (first) | Planned | [Commissioning/timing plan](../test/E2_motor_driver/PLAN.md) prepared; no upload or measurements |
| E3: NFC reader | Planned | UID-only reader, present/absent timing and event behavior |
| E4: wireless | Planned | USB bridge and radio tests, then four-robot load measurements |
| Integrated control | Planned | Steering PID, navigation states, command handling and fault behavior |
| Driving validation | Planned | 100 mm/s tracking on 14 mm lines and 30–50 mm turns; higher-rate/speed trials |

## Development log

### 2026-09-30 — Motor-first plan and host discovery

- Moved E2 ahead of E1 while retaining experiment IDs.
- Read the supplied GT50/motor-driver interface and designed M2/M4 motion, encoder, stop, and 100/400 kHz tests.
- Verified PlatformIO Core 6.1.19, installed Espressif32 6.12.0/XIAO board definition, and one enumerated serial device on COM4.
- Added the E2 PlatformIO configuration using Arduino as the initial framework choice. No firmware build/upload, serial connection, reset, or motor commands performed.
- Corrected wiring to the user's clarified D-label convention: motor SDA D4/SCL D5 (GPIO5/GPIO6), line-sensor SDA D6/SCL D7 (GPIO43/GPIO44). Motor type 1 remains a commissioning candidate; command units, read decoding, and motor scaling need confirmation before motion.
- Verified the supplied pinout image against the installed Arduino board variant. Removed the earlier inferred line-sensor/SPI conflict: D7 and D8 map to distinct GPIO44 and GPIO7.

### 2026-09-30 — Initial structure

- Captured system architecture, pins, message semantics, branch commitment, and performance targets.
- Created reusable-library and per-application/per-experiment directory conventions.
- Defined the proposed automated test and result workflow.
- No hardware tests run; no measured rates or latency claims yet.

## Future experiment entries

For each run worth retaining, add its date, experiment/environment, firmware revision, hardware configuration, result path, outcome, measured timing/error statistics, and next decision. Record failed or incomplete runs when they explain a blocker. Keep design targets separate from measurements.
