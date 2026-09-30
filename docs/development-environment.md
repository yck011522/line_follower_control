# Development environment

## Inspected on 2026-09-30

- PlatformIO Core **6.1.19** is available through the VS Code extension installation at `%USERPROFILE%\.platformio\penv\Scripts\platformio.exe`.
- `pio` / `platformio` are not currently on the shell PATH; use the full executable path in PowerShell.
- Installed Espressif32 platform **6.12.0** includes board `seeed_xiao_esp32s3`. Its board definition enables USB CDC on boot and USB mode 1 for Arduino.
- `platformio device list --json-output` detected exactly one serial device: **COM4**, `USB Serial Device (COM4)`, USB VID:PID **303A:1001**.
- Initial discovery only enumerated the device. Subsequent E1 commissioning built/uploaded firmware and verified serial plus motor-driver communication; see [bench results](../test/E1_motor_communication/INITIAL_RESULTS.md).

The root `platformio.ini` has separate E1 (`e1_motor_communication`) and E2 (`e2_motor_speed`) environments, with E2 selected by default. Each source filter excludes other applications. Both use the XIAO board, Arduino, and the installed platform version. Monitor DTR/RTS are disabled; the upload tool still resets the board as required for flashing.

The board ID is also listed in [PlatformIO's board documentation](https://docs.platformio.org/en/latest/boards/espressif32/seeed_xiao_esp32s3.html). Pin aliases must be taken from the [Seeed board reference](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/), not inferred from their label numbers.

## Verified D-pin mapping

The installed `espressif32/boards/seeed_xiao_esp32s3.json` selects Arduino variant `XIAO_ESP32S3`. Its header at `%USERPROFILE%\.platformio\packages\framework-arduinoespressif32\variants\XIAO_ESP32S3\pins_arduino.h` defines D0–D10 and matches the supplied [pinout PNG](../reference/XIAO_ESP32-S3_front_pinout.png).

| Purpose | Arduino aliases | GPIO values |
| --- | --- | --- |
| Motor I²C SDA/SCL | D4 / D5 | 5 / 6 |
| Line-sensor I²C SDA/SCL | D6 / D7 | 43 / 44 |
| RC522 CS/RST | D1 / D0 | 2 / 1 |
| RC522 SCK/MISO/MOSI | D8 / D9 / D10 | 7 / 8 / 9 |

Use these aliases after including `Arduino.h`. The motor bus uses `Wire.begin(D4, D5, 400000)`; the second bus must explicitly select D6/D7. Bare integer pin arguments represent GPIO numbers, not D-label indices. Board selection supplies the mappings; it does not reinterpret a bare `4` as `D4`.

The environment name `[env:e1_motor_communication]` is an application label. `board = seeed_xiao_esp32s3` selects the hardware regardless of that environment name. `platform = espressif32@6.12.0` pins the installed Espressif32 version for repeatability. No custom pin-mapping header is needed.

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

A repository `.venv/` was created during commissioning with pyserial 3.5 for plain serial capture and pypdf 6.19.0 for a one-time reference inspection. No automated runner or data parser was added. Interactive use only requires the PlatformIO monitor. Keep the VS Code PlatformIO environment separate; `.venv/` is ignored by Git.
