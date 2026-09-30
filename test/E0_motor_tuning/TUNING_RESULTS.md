# E0 tuning observations - 2026-09-30

Provisional candidate left stored: **P=3, I=0.375, D=0.5**. Final run completed
with no reported cleanup errors; zero speed, zero PWM release, and upload off
were sent before closing COM5. This is a working candidate for further testing,
not an optimal PID claim. Very-low-speed operation at target 10 remains poor.

All tests commanded M2 only. Type 1, dead zone 1650, pulse line 2000, ratio 23,
and diameter 65 were verified and unchanged. DTR on / RTS off was used for all
six additional runs, with successful PID resets and reconnects. Wheel load,
battery voltage, and physical motion were not independently measured.

## Runs

Folders below are under `results/`. Each contains raw serial logs, measurements,
metadata, `speed.png`, and a summary. Generated results are Git-ignored.

| Run | PID | Speeds | Seconds at each speed | Purpose |
| --- | --- | --- | --- | --- |
| 20260930T082151_301315Z | 3, 3, 0.5 | 20/40/60/80 | 3 | Original user baseline |
| 20260930T082631_417090Z | 3, 3, 0 | 20/40/60/80 | 3 | User's derivative comparison |
| 20260930T083638_483457Z | 1.5, 3, 0.5 | 20/40/60/80 | 3 | User's lower-P test |
| 20260930T083945_913721Z | 3, 1.5, 0.5 | 20/40/60/80 | 3 | Halve I, restore P |
| 20260930T084110_603904Z | 3, 0.75, 0.5 | 20/40/60/80 | 3 | Halve I again |
| 20260930T084201_675377Z | 3, 0.375, 0.5 | 20/40/60/80 | 3 | Lower I further |
| 20260930T084257_558107Z | 3, 0.75, 0.5 | 20/40/60/80 | 3 | Repeat candidate |
| 20260930T084355_908223Z | 3, 0.75, 0.5 | 10/20/40/80 | 5 | Longer, lower-speed validation |
| 20260930T084453_247285Z | 3, 0.375, 0.5 | 10/20/40/80 | 5 | Matched comparison with lower I |

## Findings

The user's reduction of P to 1.5 worsened the 20-speed result: final-second
standard deviation increased from 3.84 to 5.73, and whole-step peak increased
from 26.86 to 31.85. It did not improve the other speeds consistently either.

Keeping P=3/D=0.5 and reducing I improved sustained 20-speed tracking. With I=1.5,
final-second standard deviation was 2.53. With I=0.75 it was 1.81, and 1.85 on
repeat. These three-second runs support the direction of improvement, but longer
runs showed that the final-second statistics vary.

The matched five-second runs give the most useful final comparison:

| Target | I | Final-second mean | Final-second std dev | Final-second peak-to-peak | Whole-step peak |
| --- | --- | --- | --- | --- | --- |
| 10 | 0.75 | 11.62 | 6.29 | 27.63 | 27.63 |
| 10 | 0.375 | 9.20 | 7.81 | 29.63 | 29.63 |
| 20 | 0.75 | 18.81 | 2.73 | 11.65 | 31.41 |
| 20 | 0.375 | 19.66 | 2.68 | 11.10 | 24.75 |
| 40 | 0.75 | 40.91 | 3.03 | 11.88 | 54.27 |
| 40 | 0.375 | 39.44 | 3.75 | 10.88 | 51.27 |
| 80 | 0.75 | 81.58 | 2.80 | 8.99 | 95.22 |
| 80 | 0.375 | 79.93 | 0.58 | 2.11 | 93.22 |

I=0.375 is the provisional choice because the 20-speed average and startup peak
are better in the matched test, and the 80-speed response is much smoother.
It is not better by every metric: 40-speed standard deviation is worse, and
10-speed ripple is worse. Zero-speed hunting is not reliably eliminated: the
first I=0.375 run ended all rest intervals at zero, but the longer run had hunting
after targets 10, 20, and 40. Do not infer a robust stop from one successful trace.

At target 10, both candidates show near-zero feedback followed by bursts toward
30. Neither is satisfactory for a smoothly turning inner wheel at that command.
The trace alone cannot separate actual stop/start motion from encoder estimation
effects. Investigate physical wheel motion and dead-zone/friction behavior before
assuming more PID changes alone will solve this. Dead zone was not changed here.

For differential steering, the next validation should include M4 and unequal
wheel commands under representative load, including transitions from normal
speed down to low speed. Current tests start each speed from zero and cannot
establish cornering performance. Motor-direction polarity and feedback scaling
also remain distinct from this PID comparison.

## Comparison artifacts

- [All measurements](results/comparison_20260930/comparison.md)
- [CSV](results/comparison_20260930/comparison.csv)
- [20-speed traces with shared axes](results/comparison_20260930/low_speed_comparison.png)
- [Final candidate plot including target 10](results/20260930T084453_247285Z/speed.png)

The comparison was generated with `compare.py`, reading saved runs only. All
statistics use driver-reported speed units and PC receive timestamps. The upload
rate limits visibility of fast dynamics. No statistical significance is claimed
from these few sequential runs; supply/load/temperature and sampling can affect
comparisons, and no simultaneous two-wheel or loaded vehicle test was performed.
