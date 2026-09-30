# Hardware and pin assignments

## Components

| Component | Known information | Pending |
| --- | --- | --- |
| Robot controller | Seeed Studio XIAO ESP32S3, USB; PlatformIO board `seeed_xiao_esp32s3`; E1 Arduino firmware uploaded | Integrated robot firmware pending |
| Motor controller | Four-channel board, I²C address `0x26` at 5 V through a level converter; M2 left/M4 right; GT50 motors | See [interface reference](../reference/RC_Car_Motor_Driver_Interface.md); command units, read encoding, scaling, timing, and motor type still need verification |
| Line sensor | Eight downward channels; I²C through a level converter | Model, supply/logic voltage, address, protocol, update rate |
| NFC reader | RC522 over SPI; read UID only; IRQ unconnected | Module, supply/logic requirements, compatible tags, library |
| Radio bridge | ESP32 connected to Windows PC over USB | Exact board and USB interface |
| Battery | Lithium-ion pack described as “12.6 V nominal”; voltage read through motor driver | Cell count, actual nominal/full-charge voltages, limits, voltage scaling |

Battery description is recorded as supplied, not validated as an electrical specification. Confirm the pack label/specification before defining thresholds. Motor/logic power distribution, regulator choice, and grounding are not specified yet.

## Robot wiring intent

All pin numbers supplied for this project refer to the board's **D-number labels**. Use these aliases in Arduino firmware; GPIO numbers below describe their chip mapping.

| Interface | Signal | Board pin | Chip mapping |
| --- | --- | --- | --- |
| Motor I²C | SDA | D4 | GPIO5 |
| Motor I²C | SCL | D5 | GPIO6 |
| Line-sensor I²C | SDA | D6 | GPIO43 |
| Line-sensor I²C | SCL | D7 | GPIO44 |
| RC522 SPI | SS / CS | D1 | XIAO GPIO2 |
| RC522 SPI | RST | D0 | XIAO GPIO1 |
| RC522 SPI | SCK | D8 | XIAO GPIO7 |
| RC522 SPI | MISO | D9 | XIAO GPIO8 |
| RC522 SPI | MOSI | D10 | XIAO GPIO9 |
| RC522 | IRQ | Unconnected | Reader must operate without IRQ wiring |

Mappings match the supplied [board pinout image](../reference/XIAO_ESP32-S3_front_pinout.png), the [Seeed XIAO ESP32S3 pin table](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/), and the installed Arduino `XIAO_ESP32S3/pins_arduino.h` variant. The motor bus uses the board's default SDA/SCL pins: D4/D5 (GPIO5/GPIO6). Prefer explicit `D4`/`D5` arguments in application configuration so the wiring is visible.

The line sensor uses the second I²C controller on D6/D7 (GPIO43/GPIO44); RC522 SCK uses D8 (GPIO7). These assignments do not overlap. D6/D7 also carry UART TX/RX aliases, so reserve them for the line-sensor bus rather than using that hardware UART simultaneously. Host communication uses native USB.

In Arduino code, use aliases such as `D4`, not the bare integer `4`: the installed aliases resolve to GPIO numbers. RC522 chip select remains the explicitly assigned `D1`; the board's generic SPI `SS` alias is D7 and must not replace it.

## Bus configuration still required

Motor I²C default is now **400 kHz**, based on E1 bench measurements. E1 retains 100 kHz as a diagnostic option; new motor tests use 400 kHz. The line sensor's supported clock remains unconfirmed.

- Assign primary I²C to the motor driver and a separate I²C controller to the line sensor.
- Confirm device addresses, allowed clock rates, clock stretching, and timeout behavior from the device documentation.
- Confirm level-converter suitability, pull-ups, and logic voltage on each side. The line sensor's voltage has not been supplied.
- Confirm the RC522 module's SPI clock limit and electrical requirements before wiring. Do not infer 5 V compatibility from the motor bus.
- Wheel diameter is 65 mm, supplied by the user. Record sensor channel order, lateral sensor spacing, sensor-to-axle distance, and track width before steering calibration.
