# Shared firmware libraries

Place reusable peripheral and protocol code here as it is implemented. Planned modules include line-sensor access, motor-driver commands/telemetry, RC522 UID reading, radio messages, and later steering/navigation logic.

Each module should use its own directory with public headers under `include/` and implementation under `src/`, plus PlatformIO library metadata when needed. Keep board pin assignments and experiment scheduling in the application; pass bus/configuration dependencies into drivers. Report timeouts and errors explicitly so experiments can measure them.

Drivers are shared by standalone experiments and the integrated robot. Do not add speculative register definitions before receiving device documentation.
