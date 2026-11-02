# E15 dashboard and recording

Run `npm ci`, then `npm start` or `start-dashboard.bat` (default modem COM15).
This dashboard uses the existing E13 modem and packet layout. Logs go only to
`../pc-ui-logs/`; E14 recordings are not modified.

Each car's Simulated control starts only when explicitly enabled, with fresh
telemetry, a progressing World State echo, and working recording. It alternates
AUTO forward and AUTO stop; default durations are 60 and 5 seconds. The
server owns each independent schedule and shares it with all browser views.
Each run lasts at most eight hours. Stop, disabling the checkbox, or any manual
command cancels the schedule. Modem errors, stale telemetry or World State
echo, recording failure, or a scheduler delay over two seconds cancel it
without restarting it. A shutdown attempts to transmit a stop. A disconnected
car cannot be guaranteed to receive that stop; the car and modem timeouts are
the fallback. Monitor the car and provide a physical emergency stop.

Recording starts automatically. Timestamped telemetry and command CSVs use the
E14 column layouts. Command changes represent dashboard intent, not confirmed
car action. The telemetry CSV records numeric controller states 11, 12 and 13
for periodic, I2C error, and invalid-transition recovery respectively. Encoder
speeds depend on firmware wheel/encoder calibration and filtering; they are not
independent ground speed.

`<prefix>_schedules.jsonl` records each explicit start, durations in milliseconds,
eight-hour intended duration, and a matching end with a completion flag and
reason. An unmatched start is incomplete (for example after a crash or power
loss). Recording errors can also leave metadata or buffered CSV rows incomplete;
check the dashboard and terminal. Cleanly stop the server with Ctrl+C.
Timestamps use PC wall time and may be affected by clock changes.

For analysis exclude stopped and recovery intervals from lap timing. Compare
transition gaps and recovery counts against E14, keeping the raw logs. These
observations cannot prove that the NFC chip reset or its gain changed.
