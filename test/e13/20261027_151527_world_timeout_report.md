# E13 World State Timeout Stationary Verification

- Run: 20261027_151527
- Test: MANUAL-stop commands sent for 2.0 s, followed by 2.0 s without World State, then an AUTO-stop command.
- Motion: direction and turn bits remained clear; measured wheel speeds stayed 0/0 mm/s.
- Serial line levels: DTR=true and RTS=false set before opening COM15; RTS was never toggled.
- Robot State packets received: 80.
- World State echoes from this run: 80.
- After the 500 ms deadline, 31 telemetry snapshots continued to report MANUAL, 0/0 mm/s measured speed, and the same last World State sequence.
- Raw timestamped capture: [20261027_151527_world_timeout_raw.txt](./20261027_151527_world_timeout_raw.txt)

An AUTO-stop packet was transmitted at the end of the run; its resulting
controller state is confirmed in the steady-link capture.
