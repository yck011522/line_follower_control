# Documentation index

| Document | Purpose |
| --- | --- |
| [System design](system-design.md) | Architecture, rates, steering behavior, and integration boundaries |
| [Motor settings and tuning](motor-settings.md) | Adopted settings, persistent PID exception, E0 evidence, RTS-off rule |
| [Hardware and pins](hardware.md) | Wiring intent, pin namespaces, buses, power, and missing hardware details |
| [Communication](communication.md) | World-state/telemetry semantics and pending wire protocol decisions |
| [Testing](testing.md) | PlatformIO environment plan, deterministic Python workflow, and measurements |
| [E0 motor PID tuning](../test/E0_motor_tuning/README.md) | Direct USB serial PID candidates, step responses, and Matplotlib plots |
| [E1 motor test plan](../test/E1_motor_communication/PLAN.md) | M2/M4 speed, encoder feedback, stop, and 100/400 kHz comparison |
| [E2 motor speed test](../test/E2_motor_speed/README.md) | Separate 400 kHz speed sweep and log analysis |
| [Development environment](development-environment.md) | Local PlatformIO setup, board selection, and serial-port discovery |
| [Open questions](open-questions.md) | Information needed before implementation and unresolved design choices |
| [Progress](progress.md) | Development sequence, completion status, and evidence |

Requirements capture the project brief. Items marked **proposed**, **assumed**, or **pending** are not validated implementation decisions. Update the relevant document when a decision changes, and link measured evidence from the progress log.
