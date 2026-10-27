# E13 Steady BLE Stationary Verification

- Run: 20261027_151116
- Test: World State sent to the E13 car ID 1 through modem COM15; Robot State returned from the car through COM15.
- Schedule: AUTO stop for 0–6 s; MANUAL stop for 6–12 s; AUTO stop for 12–18 s; then pause World State transmission for 1.5 s before a final AUTO-stop packet.
- Motion: no forward, backward, or turn request; startup motion disabled.
- Serial line levels: DTR=true and RTS=false set before opening COM15; RTS was never toggled.
- World State frames sent: 900.
- Robot State frames received: 406, all with robot ID 1.
- Echoed World State sequences from this run: 405 of 406.
- Observed telemetry rate: 19.97 packets/s.
- Telemetry sequence gaps during this capture: 0.
- Controller state counts: AUTO=286, MANUAL=120.
- At least 500 ms after World State transmission paused, telemetry still reported AUTO and measured wheel speeds remained 0/0 mm/s.
- Timestamped raw capture: [20261027_151116_steady_ble_raw.txt](./20261027_151116_steady_ble_raw.txt)

This run verifies steady two-way transport, AUTO/MANUAL selection, the
approximately 20 Hz notification rate, and the zero-motion timeout behavior.
