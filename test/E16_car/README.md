# E16 lifted-wheel hardware check

Run `C:\Users\scmuser\.platformio\penv\Scripts\python.exe test/E16_car/bench.py --run`
with wheels lifted, COM13 car and COM11 unchanged modem. This explicitly starts
bounded movement and attempts stop through both ports during cleanup. No host
unit tests or simulated devices are used.

## 2026-11-09 result

`e16_car_1` built and uploaded; flash hash verified. The real-hardware run
completed and retained [raw evidence and report](results/20261109T091310Z/summary.md).
MANUAL forward/reverse measured approximately +/-51 nominal encoder mm/s for
50 mm/s targets. Left/right pivots, simultaneous-turn right precedence,
opposed-direction stopping, command-loss stopping and reverse resumption passed.
AUTO stopped with LINE_LOST when no line was present; MANUAL reverse remained
usable with LINE_LOST set. Final targets and measured wheel speeds were zero.
Control rate was approximately 100 Hz and battery approximately 11.8 V.

Packet sizes remain seven and 21 bytes. Valid Robot State packets, sequence
echoes, retained zero UID and saturated unread age were observed through COM11.
No tag was presented. Actual UID transitions, AUTO steering over a line,
periodic reinitialization, I2C fault recovery/reboot and battery thresholds
remain unverified on hardware.

The timeout check confirms stopping after World State silence, not a precise
device-side measurement of 500 ms. Speeds use inherited 66.5 mm wheel diameter,
150:1 gearing and 9 pulses/revolution; no ground-speed calibration was performed.
Results directories are Git-ignored but retained locally, including failed runs.

## Circular-track follow-up, USB disconnected

The [5-second check](results/20261109T092844Z_track/summary.md) and
[30-second check](results/20261109T092917Z_track/summary.md) completed through
COM11 alone. The longer run delivered 600 AUTO packets without telemetry
sequence gaps and accepted six distinct four-byte UIDs. After the initial
command transition, controller/line states stayed AUTO/FOLLOW_LINE (0/0).
The last UID remained retained during approximately 23 seconds without a new
read. Explicit stopping ended with both encoder speeds zero. Physical lap
count, UID revisits and I2C recovery/reboot were not established by these runs.

Repeat with `C:\Users\scmuser\.platformio\penv\Scripts\python.exe test/E16_car/track.py --run --seconds 5`.
The modem-only runner requires a visible line and retained UID before motion,
limits each AUTO run to 30 seconds, stops on stale feedback or protection
states, and retains raw/incomplete-run evidence. It sends zero movement in
cleanup and releases the serial port without changing RTS.
