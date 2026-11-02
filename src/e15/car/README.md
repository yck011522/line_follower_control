# E15 car firmware

Build `e15_car_1` with PlatformIO; it uses the E13 modem and shared BLE
protocol. E14 is unchanged. Before any motion, perform the supervised bench,
track and fault checks in the E15 test plan.

The 20-minute timer starts at successful reader initialization. A due periodic
reinitialization waits until no direction or turn is commanded; it does not
interrupt a command. An NFC I2C error or invalid UID transition immediately
stops targets. State codes 11, 12, and 13 identify the respective reasons.
The recovery state is entered after settling and held at least 150 ms; if
settling times out, the state is shown while the car remains stopped and
continues checking. This allows normal 20 Hz notification attempts but cannot
guarantee BLE delivery.

Both encoder-derived wheel speeds must stay at or below 5 mm/s for 200 ms
before initialization. If they do not settle within 3 seconds, the car
remains stopped and continues checking. Reader stop and initialization must
both succeed; failed attempts remain stopped and retry after 2 seconds.
Serial diagnostics include the reason and initializer duration. Initialization
is blocking, so telemetry and control ticks pause during it. The shared
watchdog timeout is 12 seconds; measure real initialization times and watchdog
behavior on hardware, including faults, before the long run.

Commands arriving during recovery or its blocking initializer are discarded.
After success, motion requires a later forward World State packet (or positive
serial speed command), a valid line reading, no LINE_LOST state, and the existing
battery safety conditions. World State packets are continuously retransmitted,
so a forward command still held by the dashboard can qualify after recovery;
this is not a new button press. BLE receipt and actual motor behavior are not
guaranteed by the displayed command.

The track cycle is `706ebee5 -> 40a2bee5 -> 30aabee5 -> 10dcbfe5 ->
7066bee5 -> 10e4bfe5 -> 7034c1e5 -> 10ecbfe5 -> repeat`. The first
known fresh UID establishes a baseline; repeated reads are normal. Unknown or
out-of-order UIDs trigger recovery. After recovery the baseline is cleared;
the UID that caused an invalid-transition recovery is tolerated until a different
UID appears, avoiding an endless recovery on the same tag. A known UID
re-establishes the baseline; an unknown one cannot. This strict cycle is for the
current forward track only; revise it for another topology or reverse.
