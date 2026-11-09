# E16 car firmware

Build/upload `e16_car_1` on COM13. Uses the unchanged E13 modem on COM11
and shared seven-byte World State / 21-byte Robot State protocol.

E14 movement behavior is preserved. Every supported four-byte UID read updates
the retained UID and read time, including repeats, unknown tags and revisits.
There is no tag list, order validation, local topology decision or NFC-read
progress watchdog. State 13 remains reserved; other state numbers are unchanged.

An NFC I2C error stops targets and enters recovery. The existing 20-minute
periodic reinitialization waits until no direction/turn is commanded. Recovery
requires both encoder speeds at or below 5 mm/s for 200 ms, then exposes its
state for at least 150 ms before reader stop/initialization. Failures retry
after 2 seconds. A 10-second total recovery deadline reboots the controller
instead of holding the movement interlock indefinitely. A blocking operation
is additionally covered by the existing 12-second task watchdog. A permanent
hardware fault can still cause repeated reboot/recovery; safety is not bypassed.

After successful recovery, queued commands are discarded and the next World
State can select AUTO or MANUAL, including reverse. There is no fresh-forward
or valid-line recovery latch. Normal AUTO line-loss, battery and 500 ms World
State timeout protections still apply. Boot commands zero speed.

Motor GPIO PWM configuration and encoder calibration are inherited unchanged
from E14/E15: 66.5 mm wheel, 150:1 gearing, 9 pulses per motor revolution.
The separate I2C motor-driver experiments are not this car's motor interface.

Lifted-wheel hardware smoke check (no upload/reset):
`C:\Users\scmuser\.platformio\penv\Scripts\python.exe test/E16_car/bench.py --run`.
This explicitly commands bounded movement through COM11, records both ports,
and attempts stop on completion/errors. Results retain incomplete status on failure.
Speeds are encoder-derived nominal mm/s, not driver-reported speed or measured
ground speed. A lifted-wheel run cannot validate track steering or physical
tag transitions without presenting lines/tags. Fault recovery/reboot also needs
a supervised hardware fault experiment; normal operation alone cannot prove it.
