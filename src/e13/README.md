# E13 BLE Telemetry

## Objective

Integrate the proven Test 14 BLE transport into the E12 line-following firmware so
one modem can receive telemetry from zero to four cars. The existing E12 motor,
line sensor, NFC, serial-control, and 100 Hz control-loop behavior remains
local to each car. E13 adds BLE telemetry, World State selection of AUTO versus
MANUAL, and battery/line-loss state handling. NFC-driven branch and merge
navigation is documented as the next E14 step.

The wire contract is [remote_control_payload.md](../../docs/remote_control_payload.md).
The retained Test 14 evidence is in
[test_14_result.md](../../../esp32_broadcast_research/test/test_14/results/test_14_result.md).

## Scope and non-goals

- Retain the tested Test 14 UUIDs, seven-byte World State, 21-byte Robot
  State, advertising service data, 20 Hz notifications, modem discovery and
  recovery behavior.
- Keep the modem source behavior unchanged apart from its new source path and
  include paths.
- Implement World State mode selection: `auto=1` selects AUTO; `auto=0`
  selects MANUAL.
- Implement AUTO run-target selection and fixed-speed MANUAL wheel commands
  using the movement bits described in the protocol.
- Implement real battery sampling/filtering, LOW_BATT handling, and
  FOLLOW_LINE/LINE_LOST state reporting.
- Retain turn indicators for future branch behavior, but do not implement
  NFC-driven branch/merge transitions in E13.
- Do not add configuration commands; `C` remains reserved as specified.
- Use the four-byte NFC UID and last-seen/age behavior documented in the
  protocol; unknown-tag classification and the UID lookup table are E14 work.

## Implemented layout

```text
src/e13/
  README.md                         This implementation plan
  include/
    ble14_protocol.h                Exact Test 14 packed packets and codec
    ble14_watchdog.h                Exact Test 14 task-watchdog helpers
    ble14_modem_config.h            Exact Test 14 four-link NimBLE setting
  car/
    main.cpp                        E12 application with exposed E13 settings
    motor_control.h/.cpp            E12 motor implementation, initially copied unchanged
    motor_config.h                  E12 motor constants, initially copied unchanged
    ble_telemetry.h/.cpp            Reusable car-side BLE server and telemetry façade
  modem/
    main.cpp                        Test 14 modem, migrated unchanged in behavior
  pc-ui/
    package.json                    Node dependencies and scripts
    server.mjs                      Serial reader, packet decoder, WebSocket host
    public/
      index.html                    Four-car dashboard
      app.js                        Browser rendering and stale-data updates
      styles.css                    Fixed four-card layout
```

`platformio.ini` has one modem environment plus four car environments. Each
car environment sets `BLE_CAR_ID`; car 1 and the modem are mapped to COM13 and
COM15 for the current bench. The car `main.cpp` keeps the settings visible
together near its top:

- `BLE_CAR_ID` (build-time logical ID 1 through 4);
- `kBleTelemetryPeriodMs` (50 ms);
- configurable AUTO run target (initially 100 mm/s);
- MANUAL base speed (50 mm/s) and gentle-turn steering magnitude (0.8);
- optional automatic transition from STOPPED to AUTO after setup (`false`
  initially for safe stationary bring-up);
- initial advertising interval/name behavior;
- development serial diagnostic enablement; and
- battery ADC and EMA settings described in the protocol.

The shared headers retain their Test 14 names and packed layouts. This avoids
an unreviewed protocol translation or accidental structure padding.

## Classes and responsibilities

### `BleTelemetry`

`ble_telemetry.h` exposes the only BLE-facing object used by `car/main.cpp`.
It owns `NimBLEServer`, the World State and Robot State characteristics,
advertising state, connection/subscription state, packet counters, and the
Test 14 world-sequence loss window.

Public API:

```cpp
struct RobotTelemetrySnapshot {
    uint8_t nfcUid[4];
    uint16_t nfcAgeMs;
    uint8_t controllerState;
    uint8_t lineFollowerState;
    int16_t leftSpeedMmS;
    int16_t rightSpeedMmS;
    uint16_t batteryMv;
};

class BleTelemetry {
public:
    bool begin(uint8_t carId);
    void service(uint32_t nowMs);
    void publishIfDue(uint32_t nowMs, uint32_t periodMs,
                      const RobotTelemetrySnapshot& snapshot);
    bool takeWorldState(Ble14::World& world);
    bool hasWorldState() const;
    uint32_t worldStateAgeMs(uint32_t nowMs) const;
};
```

`begin(carId)` creates the tested service, characteristics, advertising
metadata, MTU request, and callbacks. The car main loop owns watchdog
registration. `service()`
restarts advertising after a disconnect and performs only periodic,
non-blocking maintenance. `publishIfDue()` produces the Test 14 20 Hz
notification and fills the packet's telemetry and last received World State
sequence. It must not call `delay`, write USB output, or acquire E12 motor/I2C
resources from BLE callback context.

The migrated `WorldCallbacks`, `ServerCallbacks`, and `RobotCallbacks` stay
private implementation classes in `ble_telemetry.cpp`. Their tested validation,
sequence accounting, connection baseline reset, and subscription handling are
preserved. Shared callback state is guarded by the existing ESP32 critical
section pattern. `main.cpp` reads current E12 state and passes a value snapshot
to `publishIfDue()`, so BLE code never reaches into E12 globals.

### `main.cpp`

The car application is an E12 copy with these integration points:

1. construct `BleTelemetry` and call `begin()` after E12 hardware
   initialization;
2. retain the 100 Hz control tick and asynchronous NFC servicing;
3. read D10 and update the voltage EMA (`k=0.02`) in every 100 Hz control
   cycle, using the divider in `docs/hardware.md`;
4. replace `lineFollowerActive` with explicit controller/line-follower
   states and implement World State and serial mode/speed semantics;
5. create a `RobotTelemetrySnapshot` from a coherent main-loop snapshot;
6. call `service()` and `publishIfDue()` between control ticks; and
7. feed the car watchdog from the live main loop.

The legacy human-readable USB telemetry remains development output and is
guarded so an undrained USB port cannot delay motor control. It is not sent to
the modem. The modem's USB serial output remains binary-packet hex lines only,
as in Test 14. Remove `P`, `L`, and `R` serial commands; `S<mm/s>` always
selects AUTO and bare `S`/`STOP` stops.

### `modem/main.cpp`

This is a direct migration of the proven Test 14 modem. It retains:

- logical ID discovery from `[14, robot_id]` service data;
- one connection per ID and periodic scans for missing cars;
- 64-byte MTU request and 20 ms connection interval;
- one expiring shared latest-World mailbox;
- per-peer eight-submission echo-credit limit;
- bounded callback-to-USB feedback queue; and
- per-task watchdog feeding and recovery deadlines.

No UI-specific protocol, extra status lines, connection counters, or BLE
headers are added to its normal USB output. A missing car therefore has no
line to forward, which makes its age entirely a PC UI concern.

## E12-to-Robot-State mapping

| Robot State field | E13 source | Notes |
| --- | --- | --- |
| `robot_id` | `BLE_CAR_ID` | Must match advertising service data. |
| `telemetry_seq` | `BleTelemetry` | Increment for every connected/subscribed notification attempt. |
| `last_world_seq` | validated World State callback | Zero until this BLE connection receives a valid World State. |
| `world_packets_lost_5s` | migrated callback counters | Keep Test 14 plausibility and saturation rules. |
| `nfc_marker_id` | E12 `latestNfcUid` | Four bytes; retain last successful UID, initially all zero. |
| `nfc_marker_age_ms` | E12 `lastUidScannedAtUs` | Initialize to `UINT16_MAX`; age saturates at that maximum. |
| `controller_state` | E13 controller state machine | AUTO, MANUAL, LOW_BATT, or STOPPED. |
| `line_follower_state` | E13 line state machine | Implement FOLLOW_LINE/LINE_LOST. NFC branch/merge transitions are E14 scope. |
| `left_speed_mm_s` | `getLeftMotorSpeedMmS()` | Round and saturate to `int16_t`. This is encoder-derived physical speed using E12 calibration. |
| `right_speed_mm_s` | `getRightMotorSpeedMmS()` | Round and saturate to `int16_t`. |
| `battery_mv` | D10 voltage divider | Read each control cycle, filter with EMA `k=0.02`, report measured millivolts. |

World State bit 0 selects AUTO or MANUAL on every valid packet. In AUTO,
forward selects the configurable run target or 0 mm/s without changing
controller state. In MANUAL, forward/backward request fixed 50 mm/s motion;
both bits together stop. A direction plus turn indicator uses steering
magnitude 0.8; a turn indicator without direction requests a zero-radius turn.
Right takes precedence if both indicators are set. Serial `S<mm/s>` always
selects AUTO, while bare `S` and `STOP` stop the car.

The car-side World State timeout is 500 ms. It zeros World State-derived
speed targets and turn indicators without changing AUTO/MANUAL mode. Serial
speed commands are not expired by this timer. Battery voltage is sampled from
D10 every 100 Hz tick using the documented 100k/22k divider and EMA `k=0.02`;
LOW_BATT enters below 9.5 V for five seconds and recovers above 10.0 V for
five seconds into AUTO at zero speed.

Enter LOW_BATT after filtered voltage remains below 9.5 V for five seconds;
return to AUTO after it remains above 10.0 V for five seconds. Preserve E12's
1.5-second line-loss grace period. LINE_LOST saves the previously selected
line state and stops AUTO targets; MANUAL remains available for repositioning.
On line reacquisition, restore the saved line state regardless of controller
mode; AUTO resumes line-following and MANUAL remains under direct control.

The E14 line-state design adds NFC tag classification and transitions:
GO returns to FOLLOW_LINE, BRANCH uses APPROACHING_BRANCH then the latest turn
indicator after tag age exceeds 50 ms, and MERGE selects a matching
MERGING_FROM_* state. The UID lookup table is deferred; unknown tags are GO.
Capture this desired behavior in [states.md](../../docs/states.md), but do not
implement NFC-driven branch/merge transitions as part of E13.

## Node dashboard

`pc-ui/server.mjs` opens the modem's USB port and accepts only a newline
terminated 42-hex-character Robot State. It decodes the existing little-endian
21-byte payload, validates IDs 1 through 4, and stores one record per logical
car:

```text
lastValidPacket, lastReceivedAt, decodedRobotState, parseErrorCount
```

It broadcasts those records over WebSocket. The browser owns a fixed card for
each car ID and refreshes age four times per second. It also exposes a
per-car World State movement control and topology ID field; the Node server
transmits the shared World State at 50 Hz. Movement defaults to AUTO with
forward clear, so the initial command is stationary.

- no valid packet since page/server start: `No data`;
- valid packet: render telemetry and `last data age`;
- age above 1,000 ms: retain the last values but label the
  card `Stale`;
- absent, disconnected, or later-returning cars need no special modem message.

This directly supports zero through four present cars and prevents a car's
absence from shifting other cars' panels. When the dashboard is stopped or
disconnected, the car-side 500 ms timeout zeros World State-derived speed and
turn targets.

To install dependencies, open a terminal in `src/e13/pc-ui` and run
`npm install`. Then double-click `start-dashboard.bat` to start the dashboard,
or run `npm start` in that folder. Keep the console window open while using
the dashboard; press Ctrl+C to stop the server. The default modem port is
`COM15`; `MODEM_PORT` and `HTTP_PORT` can override the serial and dashboard
ports. It binds to loopback at `http://127.0.0.1:8080`.

## Migration sequence

1. Copy the Test 14 protocol/watchdog/modem configuration headers and modem
   source into E13 without semantic edits. Add E13 PlatformIO profiles and
   build the modem.
2. Copy E12 car sources into `e13/car`, preserving the existing motor-control
   implementation and hardware calibration.
3. Extract the Test 14 car's BLE server, callback classes, advertising, world
   accounting, notification cadence, and watchdog use into `BleTelemetry`.
   Keep its tests' timing and packet validation intact.
4. Integrate `BleTelemetry` into E13 `main.cpp` without moving the 100 Hz
   control path or asynchronous NFC service into callbacks.
5. Implement the agreed controller, line-loss, battery, timeout, and real
   telemetry mappings; do not substitute Test 14 synthetic constants.
6. Build all five firmware profiles. Flash and assess one stationary,
   wheel-supported car plus modem before introducing additional cars.
7. Re-run the Test 14 communication qualification cases against E13 car
   firmware: nominal four-car, missing start, individual/double reconnect,
   idle, mailbox expiry/coalescing, and watchdog recovery. Keep raw captures
   and mark incomplete runs rather than overwriting historical evidence.
8. Run the Node dashboard against recorded modem hex lines and then against
   the live modem. Verify all five presence states (zero through four) and
   stale-age behavior.
9. Run a bounded live line-following observation only after stationary BLE
   verification succeeds. Motion must be started by an explicit existing
   command, stopped on failure/cancellation, and logged as a distinct
   experiment.

## Acceptance criteria

- All firmware profiles compile with the pinned platform and NimBLE version.
- Modem framing and packet sizes remain exactly 7-byte World State and 21-byte
  Robot State; no normal modem USB lines are added.
- A one-car E13 run reports measured E12 wheel speeds and validates
  `last_world_seq` echo progress without control-loop-rate regression.
- Four E13 cars meet the qualified Test 14 baseline: 50 Hz World State,
  20 Hz telemetry per available car, correct ID routing, automatic missing-car
  rejoin, and no BLE callback/USB backpressure stall.
- The dashboard visibly represents all four logical IDs at all times and
  correctly distinguishes `No data`, current data, and stale retained data.
