# E14 dashboard logging

Start the dashboard with `start-dashboard.bat` or `npm start`. Recording starts
automatically, even if the modem is not connected. Each dashboard run creates
two CSV files in `../pc-ui-logs/` with the same local start-time prefix:
`YYYY-MM-DD_HH-MM-SS_telemetry.csv` and `..._commands.csv`. A numeric suffix
distinguishes runs started in the same second. Existing logs are not overwritten.

The telemetry file contains one row per valid Robot State received from any car.
Its `time_local` is the PC receive time with milliseconds and a UTC offset; the
remaining columns are decoded packet fields. Wheel speeds are the car's
encoder-derived measurements, subject to the firmware's wheel/encoder calibration
and filtering; they are not independent ground-speed measurements.

The commands file records `mode_change` and `movement_change` rows only when the
corresponding bits change. The Stop button can change both, producing one row
of each type with the same resulting movement byte. These rows record dashboard
intent, not proof that a car received or acted on a command. Repeated World
State transmissions and topology changes are not logged. All cars initially
have the AUTO-stop movement byte `01`.

The timestamps use the PC wall clock, so clock adjustments can affect time
differences. An NFC UID is retained after a read and its age is sampled at the
telemetry rate; this log alone cannot prove that every physical tag passage was
detected or that every individual read was observed.

The dashboard displays the recording name or a recording error. If recording
fails, existing files may be incomplete; check the terminal for the error.
Use Ctrl+C in the dashboard console for a clean shutdown and flush. A forced
close, crash, or power loss can leave buffered rows unwritten; the CSV files do
not contain a completion marker.
