# Hardware and pin assignments

## Components

| Component | Known information | Pending |
| --- | --- | --- |
| Robot controller | ESP32-S3; PlatformIO intended | Seeed Studio XIAO, it has a USB interface |
| Motor controller | I²C at 5 V through a level converter; internal speed PID; encoder servos | Model, address, registers, units, timing, supported telemetry |
| Line sensor | Eight downward channels; I²C through a level converter | Model, supply/logic voltage, address, protocol, update rate |
| NFC reader | RC522 over SPI; read UID only; IRQ unconnected | Module, supply/logic requirements, compatible tags, library |
| Radio bridge | ESP32 connected to Windows PC over USB | Exact board and USB interface |
| Battery | Lithium-ion pack described as “12.6 V nominal”; voltage read through motor driver | Cell count, actual nominal/full-charge voltages, limits, voltage scaling |

Battery description is recorded as supplied, not validated as an electrical specification. Confirm the pack label/specification before defining thresholds. Motor/logic power distribution, regulator choice, and grounding are not specified yet.

## Robot wiring intent

| Interface | Signal | Pin supplied | Interpretation / status |
| --- | --- | --- | --- |
| Motor I²C | SDA | 4 | Assumed GPIO4; SDA/SCL order to confirm |
| Motor I²C | SCL | 5 | Assumed GPIO5; SDA/SCL order to confirm |
| Line-sensor I²C | SDA | 6 | Assumed GPIO6 |
| Line-sensor I²C | SCL | 7 | Assumed GPIO7 |
| RC522 SPI | SS / CS | D1 | Board alias; GPIO mapping pending |
| RC522 SPI | RST | D0 | Board alias; GPIO mapping pending |
| RC522 SPI | SCK | D8 | Board alias; GPIO mapping pending |
| RC522 SPI | MISO | D9 | Board alias; GPIO mapping pending |
| RC522 SPI | MOSI | D10 | Board alias; GPIO mapping pending |
| RC522 | IRQ | Unconnected | Reader must operate without IRQ wiring |

`D0`–`D10` are board-specific aliases and must not be treated as matching GPIO numbers. Confirm all aliases against the chosen board, then check for conflicts with both I²C buses and USB. The requested motor bus is the primary/default I²C interface, but the implementation should explicitly configure the confirmed pins instead of assuming framework defaults.

## Bus configuration still required

- Assign primary I²C to the motor driver and a separate I²C controller to the line sensor.
- Confirm device addresses, allowed clock rates, clock stretching, and timeout behavior from the device documentation.
- Confirm level-converter suitability, pull-ups, and logic voltage on each side. The line sensor's voltage has not been supplied.
- Confirm the RC522 module's SPI clock limit and electrical requirements before wiring. Do not infer 5 V compatibility from the motor bus.
- Record sensor channel order, lateral sensor spacing, sensor-to-axle distance, wheel diameter, and track width before steering calibration.
