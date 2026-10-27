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

The current motor baseline is [type 1, dead zone 1650, pulse line 500, ratio 23,
diameter 65 mm, PID 3/0.375/0.5](motor-settings.md). On ESP32 boot, rewrite the
five I2C-accessible settings, require acknowledgments, and keep motion disabled
until initialization succeeds. Rely on saved driver PID; it cannot be verified or
set through the documented I2C interface. No motor-driver UART is required.
E0 did not resolve smooth target-10 operation or validate loaded cornering.

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

## Controller modes and motion commands

The controller state separates operating mode from line-follower navigation
state. World State bit 0 selects AUTO (`1`) or MANUAL (`0`) on every valid
packet; it is not a toggle. AUTO runs the local line-following controller.
In AUTO, the forward bit selects the configurable run-speed target or zero
without changing controller state. MANUAL directly commands wheel movement:
forward/backward uses a fixed 50 mm/s base speed, and conflicting forward and
backward bits mean STOP. `S<mm/s>` serial commands always select AUTO; bare
`S` and `STOP` stop the car.

In MANUAL, a turn indicator with forward/backward applies a gentle differential
turn using steering magnitude 0.8. A turn indicator without a direction
request commands a zero-radius turn. Right takes precedence if both indicators
are set. Manual movement is independent of line-follower steering, so MANUAL
can be used to reposition a car while its line-follower state is `LINE_LOST`.

The optional startup setting selects AUTO at 100 mm/s after setup. Battery
safety overrides either mode: enter `LOW_BATT` if filtered voltage is below
9.5 V for five seconds, stop both motors, and return to AUTO after filtered
voltage stays above 10.0 V for five seconds.

## Line tracking and branch behavior

Normal tracking follows the line center. Line polarity must be configurable so that black-on-white and white-on-black boards have the same logical representation. The sensor reports eight logical channel values; channel order and conversion from raw readings are pending hardware confirmation.

The host sends turn indicators in World State. In the planned NFC-driven flow,
the robot retains the latest indicators and uses them when leaving an
`APPROACHING_BRANCH` tag. The branch state then remains selected until another
tag is seen. This replaces the earlier proposal that branch completion must
be identified by a dedicated branch-complete tag. Whether turn indicators
should be latched at branch approach rather than read when the tag disappears
remains to be verified; the current protocol draft uses the latest indicator
at tag departure.

Proposed AUTO branch mapping, to validate on the physical board:

| Latest indicator when the branch tag leaves range | Tracking strategy |
| --- | --- |
| Left | Follow left edge |
| Straight | Follow center |
| Right | Follow right edge |

NFC tags identify GO, branch, and merge events; the line sensor is not
responsible for classifying intersections. The planned state transitions are:

- A BRANCH tag enters `APPROACHING_BRANCH`. When the tag age exceeds 50 ms,
  the latest turn indicators select `BRANCHING_CENTER`, `BRANCHING_LEFT`, or
  `BRANCHING_RIGHT`.
- Branch subtypes `BRANCH_LEFT`, `BRANCH_RIGHT`, `BRANCH_LR`, and
  `BRANCH_THREEWAY` share this generic flow initially.
- GO returns to `FOLLOW_LINE`; another BRANCH begins a new approach; MERGE
  enters its matching `MERGING_FROM_*` state. Branch and merge states remain
  selected until another tag is seen.
- The E14 UID lookup table maps UIDs to tag types. Until it exists, unknown
  UIDs are treated as GO.
- A tag is currently considered visible while its last successful read age is
  at most 50 ms. A later successful read after the age exceeds 50 ms is a new
  tag event.

The full tag-driven branch/merge behavior is an E14 implementation target.
E13 implements AUTO/MANUAL mode and the basic FOLLOW_LINE/LINE_LOST states.

## State-machine outline (proposed)

| Navigation state | Entry | Exit |
| --- | --- | --- |
| `FOLLOW_LINE` | Startup/default; GO tag | BRANCH, MERGE, or sustained line loss |
| `APPROACHING_BRANCH` | BRANCH tag while visible | After tag age exceeds 50 ms, select a branching state using the latest turn indicators |
| `BRANCHING_CENTER/LEFT/RIGHT` | Branch tag departure | Next tag: GO, BRANCH, or MERGE |
| `MERGING_FROM_LEFT/RIGHT/CENTER` | Matching MERGE tag | Next tag: GO, BRANCH, or another MERGE |
| `LINE_LOST` | No valid line for over 1.5 s; save prior navigation state | Reacquired line restores the saved state |

`LINE_LOST` is an overlay on the previously selected navigation state. In AUTO
it stops motor targets after the existing 1.5-second grace period. MANUAL may
move the car for recovery. When a line is reacquired, restore the saved
navigation state regardless of controller mode; AUTO resumes line-following,
while MANUAL remains under direct wheel control.

Battery thresholds, line-loss handling, tag age, and unknown-tag behavior are
specified in [remote_control_payload.md](remote_control_payload.md). The E14
UID/type lookup table, tag event behavior in `LINE_LOST`, and whether a turn
indicator is latched at branch approach remain verification items.
