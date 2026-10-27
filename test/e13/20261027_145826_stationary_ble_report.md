# E13 Initial Stationary BLE Communication Capture

- Run: 20261027_145826
- Firmware: E13 car ID 1 on COM13; Test 14 modem on COM15.
- Flash status: both PlatformIO uploads completed with verified flash hashes.
- Car startup motion: disabled; all transmitted control values requested zero movement.
- Serial line levels: DTR=true and RTS=false set before opening both ports; RTS was never toggled.
- World State frames sent: 700.
- Valid 21-byte Robot State packets received: 494.
- Packets with robot ID 1: 494.
- Packets echoing a World State sequence sent in this run: 494.
- Controller state counts: STOPPED=195, AUTO=219, MANUAL=80.
- Raw serial and World State capture: [20261027_145826_stationary_ble_raw.txt](./20261027_145826_stationary_ble_raw.txt)

This initial discovery capture had one telemetry-sequence discontinuity from
133 to 3258 while the car had not yet echoed a World State. Its cause was not
determined, so this run is retained as exploratory evidence rather than the
steady-state acceptance result. The subsequent steady-link capture had no
telemetry-sequence gaps.
