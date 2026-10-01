# LineSensor

Owns and automatically configures one I²C controller. Construct it inside Arduino `setup()` (for example as a function-local static), then call `tick()` from the main loop. Do not assign another owner to the same bus controller.

Defaults: bus 1, SDA D6, SCL D7, address `0x12`, register `0x30`, 1 MHz bus clock, 1 ms per-Wire-operation timeout, 50 Hz polling, and five **total attempts** per due poll. Set `Config::requestHz=0` for a request on every tick. Rate and attempt limit can also be changed through setters.

`tick()` returns immediately when not due. A due poll reads up to `maxAttempts` times and stops at the first successful one-byte response. Retries are synchronous, so several timed-out requests can delay the caller; this is not an asynchronous driver. The 1 ms Wire timeout is a requested setting, not a guaranteed latency bound: hardware failures during verification took about one second per physical request. Thus an exhausted five-attempt poll can take seconds. Expired polling slots are skipped rather than serviced in a catch-up burst. No serial messages are printed by the class. Disable Arduino core logging in the application's build flags, as E3 does, to suppress Wire diagnostics too.

`reading()` returns `valid`, `rawMask`, `receivedAtUs`, and dynamically calculated `ageUs`. Before the first successful read, `valid=false` and `ageUs=UINT64_MAX`; inspect `valid` before using the mask. Failure leaves the previous mask/timestamp intact. Repeated successful reads refresh the timestamp even if the byte is unchanged. This age measures time since successful communication, not the sensor's internal conversion age. Callers choose their own stale-data threshold.

The manufacturer example implies bit 7=X1 through bit 0=X8 and active-low on black. `Reading::blackMask()` inverts the raw bits. Spatial filtering, center/edge estimation, and line-polarity configuration are deferred.

`TickResult` supplies quiet per-poll timing, retry, error, and missed-slot information for experiments. A recovered poll can be successful even though some physical requests failed. Use from one task; bus/state access is not synchronized for concurrent callers.
