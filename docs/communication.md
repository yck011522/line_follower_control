# Communication requirements and pending protocol

This document captures message meaning only. No byte layout, field widths, enums, framing, or executable protocol is defined yet. The user will supply further peripheral and communication details.

## Links

1. Windows PC ↔ USB-connected ESP32 bridge: host framing and baud rate / USB mode pending.
2. Bridge → four robots: broadcast world state, initially 50 Hz.
3. Each robot → central controller through bridge: robot state, approximately 20 Hz per robot; broadcast versus addressed transmission pending.
4. Robot → motor driver / line sensor: separate I²C protocols, pending device documentation.
5. Robot ↔ RC522: SPI, UID acquisition without reading tag payload contents.

“ESP32 protocol” is provisionally interpreted as **ESP-NOW**. Confirm this before selecting radio APIs or defining payload limits.

## World state: requested fields

| Field | Meaning |
| --- | --- |
| Sequence number | Identifies a world-state update |
| Topology version | Identifies the current game-board configuration |
| Commands for four robots | Per-robot forward/stop and left/straight/right turn indicator |

Whether commands are keyed by explicit robot IDs or fixed array positions is pending. Turn indicators express upcoming choices; the committed branch choice is latched locally at the NFC decision point. Forward/stop remains applicable during the branch.

## Robot telemetry: requested fields

| Field | Meaning / open detail |
| --- | --- |
| Robot ID | Explicit identity in each packet; persistent provisioning in non-volatile memory is intended |
| Last received world-state sequence | Application acknowledgment of the latest accepted update; reception does not prove execution |
| Last received topology version | Reports the version received; whether it also means locally applied must be defined |
| Last NFC UID | Tag identity only; representation and variable-length handling pending |
| NFC observation age | Elapsed time since that UID was last read; time unit and never-seen representation pending |
| Navigation state | Tracking, branching/intersection, or merging; final states/enums pending |
| Left/right current wheel speed | Intended measured speed; confirm driver feedback availability, units, and sign |
| Line sensor state | Eight booleans; channel ordering and logical polarity pending |
| Battery status | At least driver-reported pack voltage; scaling and additional status flags pending |
| Uptime | Time since robot startup; unit and rollover behavior pending |

Keep measured wheel speed distinct from commanded speed. Do not silently substitute targets if the driver cannot report actual speed. Likewise, received topology version alone does not establish that a corresponding tag dictionary has been installed or applied.

## NFC tag semantics

The tag dictionary associates UIDs with branch entry, branch completion, merge entry, merge completion, and zone/special meanings. Dictionary ownership, distribution, and versioning are undecided. If meanings depend on board topology, their lookup must use the appropriate topology version.

Repeated observation of the same tag means that the car is resting immediately right above the tag. Specifically for branch-out tags, this is the moment when the user can still choose how to branch out. Once the tags have been passed, then the robot will commit to the last position indicated from the last received turn indicator command.

Repeated observation of the same tag should keep updating the last NFC observation age.

## Decisions before implementation

- Message versioning, byte order, field sizes, units, validity flags, and packet length validation.
- USB framing and handling partial reads, malformed input, and reconnects.
- Robot ID allocation/provisioning, radio channel, peer setup, and packet source validation.
- Sequence rollover, duplicate/out-of-order updates, bridge/robot restarts, and host sequence reset.
- Stale-command timeout, startup motion permission, topology mismatch, and loss-of-link behavior.
- Telemetry scheduling for four robots and whether radio/application acknowledgments are needed beyond reported sequence numbers.
- Timestamp clock domains: measure round trips on one clock unless a synchronization method is explicitly implemented.
