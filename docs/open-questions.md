# Open questions

These questions do not block repository organization. Resolve them before the corresponding implementation or hardware test.

## Needed for E1

1. What is the exact ESP32-S3 board model and preferred PlatformIO framework (Arduino or ESP-IDF)? Which physical USB connection will be used?
2. What is the eight-channel line-sensor model? Supply its protocol/datasheet, I²C address, supply/logic voltage, supported clocks, raw/digital read format, and update behavior.
3. Are the two I²C pairs GPIO4/GPIO5 and GPIO6/GPIO7, with SDA first? Does “6, 6, and 7” mean 6 and 7? Confirm these are GPIO numbers rather than board `D` aliases.

## Needed for later peripheral tests

- Motor driver model and I²C command specification, including speed units, speed feedback, battery scaling, errors, command timeout, and stop behavior.
- RC522 module and tag type; mapping of D0/D1/D8/D9/D10 on the chosen controller board.
- Battery pack cell count, nominal/full-charge voltage, and intended operating limits. The supplied “12.6 V nominal” description needs confirmation.
- Bridge board model and confirmation that the intended wireless protocol is ESP-NOW.

## Needed before integration

- Wheel diameter, track width, sensor span/channel spacing, sensor-to-axle offset, and whether turn radius means line centerline radius.
- Whether the no-reverse rule applies to each wheel as well as overall vehicle travel; whether an inner wheel may stop on a sharp curve.
- Exact NFC placement at the decision point, tag dictionary ownership, and its association with topology versions.
- Meaning of “emerging intersection” and final navigation states; behavior on duplicate, missing, or unexpected tags.
- Branch behavior across stop/resume and topology updates; choice when no valid indicator exists at entry.
- Startup readiness, command freshness timeout, line-loss recovery, and topology mismatch response.
- Confirm communication field formats, ID provisioning, USB framing, radio setup, and acknowledgment semantics described in [communication](communication.md).
