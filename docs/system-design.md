# System design

## Architecture

```text
Windows PC (future Python controller)
  └─ USB serial ─ ESP32 radio bridge
                    └─ wireless (presumed ESP-NOW)
                         ├─ Robot 1 (ESP32-S3)
                         ├─ Robot 2 (ESP32-S3)
                         ├─ Robot 3 (ESP32-S3)
                         └─ Robot 4 (ESP32-S3)

Each robot:
  I²C bus 0 ─ level converter ─ motor driver ─ left/right encoder servos
  I²C bus 1 ─ level converter ─ eight-channel downward line sensor
  SPI       ─ RC522 downward NFC reader (IRQ unconnected)
```

The motor driver owns low-level encoder feedback and motor speed PID control. Robot firmware supplies wheel-speed targets and reads available driver telemetry, including battery voltage. Host responsibilities include world-state generation and robot monitoring. The location and distribution of the NFC meaning dictionary remain to be decided.

Separate buses allow independent peripheral configuration and scheduling. They do not by themselves guarantee concurrent transactions or a faster loop: driver blocking time, device conversion time, bus speed, and scheduling must be measured.

## Performance targets

| Quantity | Initial target / design input |
| --- | --- |
| Local control | 50 Hz (20 ms period); investigate 100 Hz (10 ms) and higher |
| World-state transmission | 50 Hz |
| Per-robot telemetry | Approximately 20 Hz |
| Forward speed | 100 mm/s initially; higher if measurements and tracking permit |
| Line width | 14 mm |
| Turn radius | 30–50 mm; definition of radius to confirm |
| Robots | Four |

At 100 mm/s, the robot travels 2 mm per 50 Hz cycle and 1 mm per 100 Hz cycle. These distances alone do not establish tracking performance: wheel spacing, sensor placement, sensing latency, and motor response also matter. Repeated turn geometry could support a later model-based strategy, after a simple steering PID baseline is measured.

Wireless command and telemetry rates are independent of the local steering rate. Proposed integration behavior is to use the newest accepted command without waiting for a fresh wireless packet on every control cycle.

## Line tracking and branch behavior

Normal tracking follows the line center. Line polarity must be configurable so that black-on-white and white-on-black boards have the same logical representation. The sensor reports eight logical channel values; channel order and conversion from raw readings are pending hardware confirmation.

The host sends a turn indicator before the branch decision point. At that point, the robot commits to one choice and keeps it until a branch-complete NFC tag is observed. A later turn-indicator update must not redirect an active branch. A stop command must still stop motion during a branch; the proposed behavior is to retain the latched turn while stopped and resume it when forward is requested.

Proposed branch mapping, to validate on the physical board:

| Latched indicator | Tracking strategy |
| --- | --- |
| Left | Follow left edge |
| Straight | Follow center |
| Right | Follow right edge |

NFC tags identify branch and merge events; the line sensor is not responsible for classifying intersections. Merge behavior is initially ordinary tracking through the merge, with NFC entry/exit events available for state reporting.

## State-machine outline (proposed)

| Navigation state | Entry | Exit |
| --- | --- | --- |
| Tracking | Startup once ready, or branch/merge completion | Branch or merge entry tag |
| Branching | Branch decision tag; latch current turn indicator | Branch completion tag |
| Merging | Merge entry tag | Merge completion tag |

Forward/stop is a separate motion permission, not a replacement navigation state. The exact meaning of “emerging intersection,” tag event debouncing, unexpected tag handling, startup readiness, line loss, and communication loss are open decisions. Fault handling must be specified before integrated driving tests.

The host offers no reverse command. Whether tight curves may use a stationary or reversing inner wheel is unresolved; do not assume wheel-level reverse is allowed.
