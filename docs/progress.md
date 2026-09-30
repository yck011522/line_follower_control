# Testing and development progress

Status vocabulary: **planned**, **in progress**, **blocked**, **complete**. Mark experiments complete only when implementation and recorded hardware evidence support the outcome.

| Stage | Status | Deliverable / evidence |
| --- | --- | --- |
| Repository organization | Complete | Main README, documentation, library/application/experiment folders |
| Requirements capture | Complete | Initial brief captured; unresolved items tracked in open questions |
| Board and E1 interface confirmation | Planned | Board model, pin map, sensor documentation |
| PlatformIO and host-runner setup | Planned | Build environments, serial handshake, repeatable capture |
| E1: line sensor | Planned | Driver, firmware, Python runner, CSV and timing summary |
| E2: motor driver | Planned | Driver, bounded command test, feedback and voltage measurements |
| E3: NFC reader | Planned | UID-only reader, present/absent timing and event behavior |
| E4: wireless | Planned | USB bridge and radio tests, then four-robot load measurements |
| Integrated control | Planned | Steering PID, navigation states, command handling and fault behavior |
| Driving validation | Planned | 100 mm/s tracking on 14 mm lines and 30–50 mm turns; higher-rate/speed trials |

## Development log

### 2026-09-30 — Initial structure

- Captured system architecture, pins, message semantics, branch commitment, and performance targets.
- Created reusable-library and per-application/per-experiment directory conventions.
- Defined the proposed automated test and result workflow.
- No hardware tests run; no measured rates or latency claims yet.

## Future experiment entries

For each run worth retaining, add its date, experiment/environment, firmware revision, hardware configuration, result path, outcome, measured timing/error statistics, and next decision. Record failed or incomplete runs when they explain a blocker. Keep design targets separate from measurements.
