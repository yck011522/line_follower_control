# Development environment

## Inspected on 2026-09-30

- PlatformIO Core **6.1.19** is available through the VS Code extension installation at `%USERPROFILE%\.platformio\penv\Scripts\platformio.exe`.
- `pio` / `platformio` are not currently on the shell PATH; use the full executable path in PowerShell.
- Installed Espressif32 platform **6.12.0** includes board `seeed_xiao_esp32s3`. Its board definition enables USB CDC on boot and USB mode 1 for Arduino.
- `platformio device list --json-output` detected exactly one serial device: **COM4**, `USB Serial Device (COM4)`, USB VID:PID **303A:1001**.
- Device enumeration confirms visibility, not that test firmware is running or that motor-driver I²C works. No port was opened for serial communication, no reset requested, and no firmware uploaded.

The root `platformio.ini` selects E2, the XIAO board, Arduino, and the installed platform version. It excludes other application folders. This is configuration only: there is no E2 entry point yet, so a firmware build is not ready.

The board ID is also listed in [PlatformIO's board documentation](https://docs.platformio.org/en/latest/boards/espressif32/seeed_xiao_esp32s3.html). Pin aliases must be taken from the [Seeed board reference](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/), not inferred from their label numbers.

## Verified D-pin mapping

The installed `espressif32/boards/seeed_xiao_esp32s3.json` selects Arduino variant `XIAO_ESP32S3`. Its header at `%USERPROFILE%\.platformio\packages\framework-arduinoespressif32\variants\XIAO_ESP32S3\pins_arduino.h` defines D0–D10 and matches the supplied [pinout PNG](../reference/XIAO_ESP32-S3_front_pinout.png).

| Purpose | Arduino aliases | GPIO values |
| --- | --- | --- |
| Motor I²C SDA/SCL | D4 / D5 | 5 / 6 |
| Line-sensor I²C SDA/SCL | D6 / D7 | 43 / 44 |
| RC522 CS/RST | D1 / D0 | 2 / 1 |
| RC522 SCK/MISO/MOSI | D8 / D9 / D10 | 7 / 8 / 9 |

Use these aliases after including `Arduino.h`. For example, the motor bus can use `Wire.begin(D4, D5, 100000)`; the second bus must explicitly select D6/D7. Bare integer pin arguments represent GPIO numbers, not D-label indices. Board selection supplies the mappings; it does not reinterpret a bare `4` as `D4`.

The environment name `[env:e2_motor_driver]` is an application label. `board = seeed_xiao_esp32s3` selects the hardware regardless of that environment name. `platform = espressif32@6.12.0` pins the installed Espressif32 version for repeatability. No custom pin-mapping header is needed.

## Read-only discovery commands

Run from the repository in PowerShell:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" --version
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device list --json-output
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" boards seeed_xiao_esp32s3 --json-output
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" project config --json-output
```

COM numbers can change after USB reconnect/reset or on another computer. Future runners should accept an explicit port and re-enumerate as needed. The current port is an observation, not a shared project setting.

## Python runner environment

No Python runner or new Python environment is needed for this planning/discovery step. When implementing the runner, create repository `.venv/`, install the documented host dependencies there, and invoke `.venv\Scripts\python.exe`. Keep the VS Code PlatformIO environment separate. `.venv/` is already ignored by Git.
