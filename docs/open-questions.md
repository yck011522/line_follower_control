# Open questions

These questions do not block repository organization. Resolve them before the corresponding implementation or hardware test.

## Needed for E1 (now first)

- Board confirmed: Seeed Studio XIAO ESP32S3. All project pin numbers mean board D labels. Motor wiring: **SDA D4 (GPIO5), SCL D5 (GPIO6)**. Arduino is the initial framework choice in `platformio.ini`; its installed board variant supplies the correct D aliases.
- Motor [reference supplied](../reference/RC_Car_Motor_Driver_Interface.md): address `0x26`, M2/M4, big-endian speed/PWM commands. Still need speed-command units/range, read transaction format, feedback/configuration byte order, cumulative word order/coherence, and battery scaling.
- Manual encoder response and short motor type 1 pulses worked. Confirm forward polarity, pulse-count convention, speed scale, and handling of fractional gearing before sustained closed-loop commands. User-supplied wheel diameter is 65 mm.
- Confirm allowed driver supply maximum: supplied board range is 5–12 V versus the project's 12.6 V pack description.
- See the staged [E1 plan](../test/E1_motor_communication/PLAN.md); these gaps do not prevent host setup or planning.

## Needed for E3 (deferred)

1. What is the eight-channel line-sensor model? Supply its protocol/datasheet, I²C address, supply/logic voltage, supported clocks, raw/digital read format, and update behavior.
Line-sensor wiring is SDA D6 (GPIO43), SCL D7 (GPIO44), on the second I²C controller. Pin notation is resolved; it does not conflict with RC522 SCK on D8 (GPIO7).

## Needed for later peripheral tests

- Motor driver exact model/vendor specification, errors, command timeout, and stop behavior; known register summary is now in the supplied reference.
- RC522 module and tag type; board alias mappings are now recorded in hardware.md.
- Battery pack cell count, nominal/full-charge voltage, and intended operating limits. The supplied “12.6 V nominal” description needs confirmation.
- Bridge board model and confirmation that the intended wireless protocol is ESP-NOW.

## Needed before integration

- Track width, sensor span/channel spacing, sensor-to-axle offset, and whether turn radius means line centerline radius. Wheel diameter is 65 mm.
- Whether the no-reverse rule applies to each wheel as well as overall vehicle travel; whether an inner wheel may stop on a sharp curve.
- Exact NFC placement at the decision point, tag dictionary ownership, and its association with topology versions.
- Meaning of “emerging intersection” and final navigation states; behavior on duplicate, missing, or unexpected tags.
- Branch behavior across stop/resume and topology updates; choice when no valid indicator exists at entry.
- Startup readiness, command freshness timeout, line-loss recovery, and topology mismatch response.
- Confirm communication field formats, ID provisioning, USB framing, radio setup, and acknowledgment semantics described in [communication](communication.md).

## Remaining after E0 baseline selection

- PID 3/0.375/0.5 and other adopted settings are recorded in [motor-settings](motor-settings.md).
  PID persistence is deliberately relied on in I2C-only E2; no boot readback is possible.
- Validate revised E2 on the ESP32, including reverse motion and both motors.
- Calibrate physical speed independently; the hand-measured 44998 counts per
  wheel revolution gives a nominal scale, not established odometry.
- Investigate target-10 stop/start feedback and zero-speed hunting; validate
  normal-to-low-speed transitions and unequal wheel speeds under vehicle load.
