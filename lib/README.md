# Shared firmware libraries

Place reusable peripheral and protocol code here as it is implemented.

Each module uses its own directory. Small modules may use one header, as [LineSensor](LineSensor/README.md) does; larger modules can split public headers under `include/` and implementation under `src/`. Add PlatformIO metadata only when needed. Keep board pin assignments and experiment scheduling in the application; pass bus/configuration dependencies into drivers. Report timeouts and errors explicitly so experiments can measure them.

Drivers are shared by standalone experiments and the integrated robot. Do not add speculative register definitions before receiving device documentation.
